#!/usr/bin/env python3
"""
OnShape Part Name Auditor

Lists all part names from every Part Studio and instance names from every Assembly
in the specified document/workspace. This allows your team to audit naming conventions.

Outputs:
- Hierarchical list to console
- Optional CSV export: one row per part/instance with tab name and part name
"""

import argparse
import requests
import os
import sys
import csv
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================
BASE_URL = "https://cad.onshape.com"
API_VERSION = "v11"  # Current as of 2026
REQUEST_TIMEOUT = 60

def get_auth(access_key, secret_key):
    return (access_key, secret_key)

def verify_credentials(auth):
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
    url = f"{BASE_URL}/api/documents/d/{did}/w/{wid}/elements"
    try:
        resp = requests.get(url, auth=auth, timeout=REQUEST_TIMEOUT)
        resp.raise_for_status()
        return resp.json()
    except Exception as e:
        print(f"ERROR fetching elements: {e}")
        sys.exit(1)

def get_partstudio_parts(did, wid, eid, auth):
    """
    Get list of parts in a Part Studio using the /parts endpoint (simple and gives names directly)
    """
    url = f"{BASE_URL}/api/parts/d/{did}/w/{wid}/e/{eid}"
    try:
        resp = requests.get(url, auth=auth, timeout=REQUEST_TIMEOUT)
        resp.raise_for_status()
        parts = resp.json()
        # Each part: {"partId": "JHD...", "name": "My Bracket", ...}
        return [p["name"] for p in parts if p.get("name")]
    except Exception as e:
        print(f"  ERROR fetching parts: {e}")
        return []

def get_assembly_instances(did, wid, eid, auth):
    """
    Get assembly definition and extract instance names
    """
    url = f"{BASE_URL}/api/assemblies/d/{did}/w/{wid}/e/{eid}"
    print(f"  DEBUG: Fetching from: {url}")
    try:
        resp = requests.get(url, auth=auth, timeout=REQUEST_TIMEOUT)
        print(f"  DEBUG: Response status: {resp.status_code}")
        resp.raise_for_status()
        data = resp.json()
        instances = data.get("rootAssembly", {}).get("instances", [])
        # Each instance: {"id": "...", "name": "My Bracket <1>", "partId": "...", ...}
        return [inst["name"] for inst in instances if inst.get("name")]
    except Exception as e:
        print(f"  ERROR fetching assembly definition: {e}")
        print(f"  TIP: Check element ID is valid and element type is 'Assembly'")
        return []

def main():
    parser = argparse.ArgumentParser(
        description="OnShape Part Name Auditor - List all part/instance names for naming convention audit"
    )
    parser.add_argument("--access_key", required=True, help="OnShape API Access Key")
    parser.add_argument("--secret_key", required=True, help="OnShape API Secret Key")
    parser.add_argument("--did", required=True, help="Document ID")
    parser.add_argument("--wid", required=True, help="Workspace ID")
    parser.add_argument("--csv", action="store_true", help="Export results to CSV file")
    args = parser.parse_args()

    auth = get_auth(args.access_key, args.secret_key)

    print("\n" + "="*70)
    print("OnShape Part Name Auditor")
    print("="*70)

    if not verify_credentials(auth):
        sys.exit(1)

    elements = list_elements(args.did, args.wid, auth)

    part_studios = [e for e in elements if e.get("type") == "PartStudio"]
    assemblies = [e for e in elements if e.get("type") == "Assembly"]

    all_parts = []  # For CSV: list of (tab_name, part_name)

    print(f"\nFound {len(part_studios)} Part Studio(s) and {len(assemblies)} Assembly(s)\n")

    # Process Part Studios
    if part_studios:
        print("PART STUDIOS")
        print("-"*40)
        for elem in part_studios:
            name = elem.get("name", "Unnamed")
            eid = elem.get("id")
            print(f"\n→ {name}")
            parts = get_partstudio_parts(args.did, args.wid, eid, auth)
            if parts:
                for p in sorted(parts):
                    print(f"   • {p}")
                    all_parts.append((name, p))
            else:
                print("   (No parts found)")

    # Process Assemblies
    if assemblies:
        print("\n\nASSEMBLIES (instance names)")
        print("-"*40)
        for elem in assemblies:
            name = elem.get("name", "Unnamed")
            eid = elem.get("id")
            print(f"\n→ {name}")
            instances = get_assembly_instances(args.did, args.wid, eid, auth)
            if instances:
                for i in sorted(instances):
                    print(f"   • {i}")
                    all_parts.append((name, i))
            else:
                print("   (No instances found)")

    print("\n" + "="*70)
    print("AUDIT COMPLETE")
    print(f"Total unique part/instance names collected: {len(all_parts)}")
    print("="*70)

    # Optional CSV export
    if args.csv:
        if all_parts:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"OnShape_PartAudit_{timestamp}.csv"
            with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Tab Name", "Part/Instance Name"])
                writer.writerows(all_parts)
            print(f"\nCSV exported: {csv_filename}")
        else:
            print("\n⚠ CSV export skipped: No parts/instances were successfully retrieved.")

    print("\nReview the list above and ensure all names follow your team's naming convention.")
    print("For Unity import: Consistent, descriptive names here will make glTF files easier to identify later.")
    print("="*70)

    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()