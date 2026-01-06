#!/usr/bin/env python3
"""
OnShape → glTF Batch Exporter - Optimized for Large Robot Assemblies

Key improvements:
- 3-minute timeout for export initiation (large assemblies need time to queue)
- Ultra-coarse mesh settings to reduce processing time
- 30-minute polling with exponential backoff
- Better error handling and progress reporting
"""

import argparse
import requests
import os
import sys
import time
from urllib.parse import quote_plus
from datetime import datetime, timedelta

# =============================================================================
# CONFIGURATION
# =============================================================================
BASE_URL = "https://cad.onshape.com"
API_VERSION = "v11"
INITIATE_TIMEOUT = 180  # 3 minutes for export initiation (can be slow for large assemblies)
REQUEST_TIMEOUT = 60     # 1 minute for all other requests

def get_auth(access_key, secret_key):
    """Return HTTP Basic Auth tuple"""
    return (access_key, secret_key)

def sanitize_filename(name):
    """Remove invalid filename characters"""
    invalid_chars = '<>:"/\\|?*'
    for char in invalid_chars:
        name = name.replace(char, '_')
    return name.strip() or "Unnamed"

def verify_credentials(auth):
    """Test API credentials"""
    url = f"{BASE_URL}/api/users/sessioninfo"
    try:
        resp = requests.get(url, auth=auth, timeout=REQUEST_TIMEOUT)
        resp.raise_for_status()
        print("✓ API keys validated")
        return True
    except Exception as e:
        print(f"✗ Credential check failed: {e}")
        return False

def list_elements(did, wid, auth):
    """Get all elements in the workspace"""
    url = f"{BASE_URL}/api/documents/d/{did}/w/{wid}/elements"
    try:
        resp = requests.get(url, auth=auth, timeout=REQUEST_TIMEOUT)
        resp.raise_for_status()
        return resp.json()
    except requests.exceptions.HTTPError as e:
        print(f"ERROR fetching elements: {e} (Status: {e.response.status_code})")
        if e.response.status_code == 403:
            print("  → Your API keys do not have access to this document.")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

def initiate_gltf_export(did, wid, eid, element_type, element_name, auth):
    """
    Initiate glTF export with ultra-coarse mesh settings.
    Uses longer timeout since large assemblies take time to even queue.
    """
    if element_type == "Assembly":
        endpoint = f"/api/{API_VERSION}/assemblies/d/{did}/w/{wid}/e/{eid}/export/gltf"
    elif element_type == "PartStudio":
        endpoint = f"/api/{API_VERSION}/partstudios/d/{did}/w/{wid}/e/{eid}/export/gltf"
    else:
        return None

    url = f"{BASE_URL}{endpoint}"
    headers = {
        'Accept': 'application/json;charset=UTF-8; qs=0.09',
        'Content-Type': 'application/json;charset=UTF-8; qs=0.09'
    }

    # Ultra-coarse settings for large robot assemblies
    body = {
        "grouping": True,
        "meshParams": {
            "angularTolerance": 0.35,     # ~20 degrees - very coarse curves
            "distanceTolerance": 0.05,    # 5 cm tolerance - huge reduction in triangles
            "maximumChordLength": 0.2,    # 20 cm chords
            "resolution": "COARSE",
            "unit": "METER"
        },
        "storeInDocument": True
    }

    try:
        print(f"  Initiating ultra-coarse glTF export for: {element_name}")
        print(f"  (Using {INITIATE_TIMEOUT}s timeout for initiation...)")
        
        # Use longer timeout for initiation - large assemblies need it
        resp = requests.post(
            url, 
            json=body, 
            auth=auth, 
            headers=headers, 
            timeout=INITIATE_TIMEOUT
        )
        resp.raise_for_status()
        
        translation_id = resp.json().get("id")
        if translation_id:
            print(f"  Export queued successfully (ID: {translation_id})")
            return translation_id
        else:
            print("  WARNING: No translation ID returned")
            return None
            
    except requests.exceptions.Timeout:
        print(f"  ERROR: Export initiation timed out after {INITIATE_TIMEOUT} seconds")
        print(f"  TIP: Full robot assembly is very large. Try exporting sub-assemblies individually:")
        print(f"       - Drivetrain")
        print(f"       - Arm/Manipulator")
        print(f"       - Intake")
        print(f"       Each will be faster and more reliable.")
        return None
    except Exception as e:
        print(f"  ERROR initiating export: {e}")
        return None

def poll_translation_status(translation_id, auth, element_name="Unknown"):
    """
    Poll translation status with exponential backoff.
    Waits up to 30 minutes for completion.
    """
    url = f"{BASE_URL}/api/{API_VERSION}/translations/{translation_id}"
    headers = {'Accept': 'application/json;charset=UTF-8; qs=0.09'}

    max_duration_seconds = 1800  # 30 minutes
    poll_interval = 15.0
    max_poll_interval = 60.0
    
    start_time = time.time()
    print(f"  Polling status (max wait: 30 minutes)...")

    while (time.time() - start_time) < max_duration_seconds:
        elapsed = time.time() - start_time
        try:
            resp = requests.get(url, auth=auth, headers=headers, timeout=REQUEST_TIMEOUT)
            resp.raise_for_status()
            data = resp.json()
            state = data.get("requestState")

            if state == "DONE":
                result_eid = data.get("resultElementIds", [None])[0]
                print(f"  ✓ Export completed in {str(timedelta(seconds=int(elapsed)))}")
                return ("DONE", result_eid)
            elif state == "FAILED":
                reason = data.get("failureReason", "Unknown")
                print(f"  ✗ FAILED: {reason}")
                return (None, None)
            else:
                print(f"  [{str(timedelta(seconds=int(elapsed)))} elapsed] State: {state}")
                time.sleep(poll_interval)
                poll_interval = min(poll_interval * 1.5, max_poll_interval)

        except Exception as e:
            print(f"  Poll error: {e}")
            time.sleep(poll_interval)

    print(f"  ⏱ TIMEOUT after 30 minutes")
    print("  TIP: Check your OnShape document - the export may complete later.")
    print("       For faster exports, try individual sub-assemblies instead of full robot.")
    return (None, None)

def download_gltf_blob(did, wid, result_eid, element_name, output_dir, auth):
    """Download the exported glTF file"""
    url = f"{BASE_URL}/api/v6/blobelements/d/{did}/w/{wid}/e/{result_eid}"
    headers = {'Accept': 'application/octet-stream'}

    try:
        print(f"  Downloading glTF for: {element_name}")
        resp = requests.get(url, auth=auth, headers=headers, stream=True, timeout=REQUEST_TIMEOUT)
        resp.raise_for_status()

        safe_name = sanitize_filename(element_name)
        filename = f"{safe_name}.zip" if 'zip' in resp.headers.get('content-type', '') else f"{safe_name}.gltf"
        filepath = os.path.join(output_dir, filename)

        with open(filepath, 'wb') as f:
            for chunk in resp.iter_content(chunk_size=8192):
                f.write(chunk)

        size_mb = os.path.getsize(filepath) / (1024*1024)
        print(f"  ✓ Saved: {filename} ({size_mb:.1f} MB)")
        return True
    except Exception as e:
        print(f"  ERROR downloading: {e}")
        return False

def export_gltf(did, wid, eid, element_name, element_type, output_dir, auth):
    """Complete export workflow for one element"""
    print(f"\n{'='*70}")
    print(f"Exporting: {element_name} ({element_type})")
    print(f"{'='*70}")

    translation_id = initiate_gltf_export(did, wid, eid, element_type, element_name, auth)
    if not translation_id:
        return False

    status, result_eid = poll_translation_status(translation_id, auth, element_name)
    if status != "DONE" or not result_eid:
        return False

    return download_gltf_blob(did, wid, result_eid, element_name, output_dir, auth)

def main():
    parser = argparse.ArgumentParser(
        description="OnShape → glTF Batch Exporter (Optimized for Large Robot Assemblies)"
    )
    parser.add_argument("--access_key", required=True, help="OnShape API Access Key")
    parser.add_argument("--secret_key", required=True, help="OnShape API Secret Key")
    parser.add_argument("--did", required=True, help="Document ID")
    parser.add_argument("--wid", required=True, help="Workspace ID")
    parser.add_argument("--output", default="OnShape_glTF_Export", help="Output directory")
    parser.add_argument("--skip-assemblies", action="store_true", help="Skip assembly exports")
    parser.add_argument("--skip-partstudios", action="store_true", help="Skip part studio exports")
    args = parser.parse_args()

    auth = get_auth(args.access_key, args.secret_key)
    os.makedirs(args.output, exist_ok=True)

    print("\n" + "="*70)
    print("OnShape → glTF Exporter (Optimized for Large Robot Assemblies)")
    print("="*70)

    if not verify_credentials(auth):
        sys.exit(1)

    elements = list_elements(args.did, args.wid, auth)

    exportable = [e for e in elements if
                  (e.get("type") == "Assembly" and not args.skip_assemblies) or
                  (e.get("type") == "PartStudio" and not args.skip_partstudios)]

    if not exportable:
        print("No elements to export")
        return

    print(f"\nFound {len(exportable)} element(s) to export:")
    for elem in exportable:
        print(f"  - {elem.get('name')} ({elem.get('type')})")
    print()

    success = 0
    failed = []
    
    for i, elem in enumerate(exportable, 1):
        name = elem.get("name", "Unnamed")
        eid = elem.get("id")
        etype = elem.get("type")
        
        print(f"[{i}/{len(exportable)}] {name}")
        if export_gltf(args.did, args.wid, eid, name, etype, args.output, auth):
            success += 1
        else:
            failed.append(name)

    print("\n" + "="*70)
    print(f"EXPORT COMPLETE - {success}/{len(exportable)} succeeded")
    if failed:
        print(f"\nFailed exports:")
        for name in failed:
            print(f"  ✗ {name}")
    print("="*70)
    
    print("\nUnity Import Tips:")
    print("• Install glTFast package (Package Manager → Add by name → com.atteneder.gltfast)")
    print("• Drag .gltf files or extract .zip → Assets folder")
    print("• For runtime loading:")
    print("   var gltf = new GltfImport();")
    print("   await gltf.Load(\"file:///\" + fullPath);")
    print("   await gltf.InstantiateMainSceneAsync(transform);")
    print("="*70)
    
    if failed:
        print("\n⚠ RECOMMENDATION FOR FAILED EXPORTS:")
        print("Large assemblies often timeout. Try exporting sub-assemblies individually:")
        print("  1. Open OnShape document")
        print("  2. Export each major sub-assembly separately (Drivetrain, Arm, Intake, etc.)")
        print("  3. Import to Unity and reassemble there")
        print("="*70)

    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()