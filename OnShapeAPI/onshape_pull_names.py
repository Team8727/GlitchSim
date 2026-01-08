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
import re
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================
BASE_URL = "https://cad.onshape.com"
API_VERSION = "v11"  # Current as of 2026
REQUEST_TIMEOUT = 60

def extract_quantity_from_name(name):
    """
    Extract quantity from instance name (e.g., "My Part <3>" -> ("My Part", 3)).
    Returns tuple of (cleaned_name, quantity).
    If no quantity found, returns (original_name, 1).
    """
    match = re.search(r'\s*<(\d+)>\s*$', name)
    if match:
        quantity = int(match.group(1))
        cleaned_name = name[:match.start()].strip()
        return (cleaned_name, quantity)
    return (name, 1)

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

def get_assembly_definition(did, wid, eid, auth):
    """
    Get full assembly definition including all instances and subassemblies.
    
    The OnShape API returns:
    - rootAssembly: The top-level assembly with instances, occurrences, patterns
    - subAssemblies: Array of nested assembly definitions  
    - parts: Detailed part information for parts used in assembly
    - partStudioFeatures: Part Studio feature references
    
    Instance types: "Part", "Assembly", "Feature", "Unknown"
    """
    url = f"{BASE_URL}/api/assemblies/d/{did}/w/{wid}/e/{eid}"
    print(f"  Fetching assembly definition...")
    try:
        resp = requests.get(url, auth=auth, timeout=REQUEST_TIMEOUT)
        resp.raise_for_status()
        return resp.json()
    except Exception as e:
        print(f"  ERROR fetching assembly definition: {e}")
        return None

def extract_assembly_items(assembly_data, include_subassemblies=True):
    """
    Extract all part and subassembly names from assembly definition with hierarchy info.
    
    Returns list of dicts with:
    - name: Instance name (e.g., "My Bracket <1>")
    - type: "Part", "Assembly", "Feature", etc.
    - parent: Name of parent assembly (None for root items)
    - depth: Nesting level (0 for root, 1 for subassembly contents, etc.)
    """
    items = []
    
    if not assembly_data:
        return items
    
    # Build a lookup of subassembly elementId -> subassembly info
    # This helps us find the name of each subassembly
    subassembly_lookup = {}
    for sub in assembly_data.get("subAssemblies", []):
        elem_id = sub.get("elementId")
        if elem_id:
            subassembly_lookup[elem_id] = sub
    
    # Process root assembly instances
    root = assembly_data.get("rootAssembly", {})
    root_element_id = root.get("elementId", "root")
    
    for inst in root.get("instances", []):
        name = inst.get("name", "Unnamed")
        inst_type = inst.get("type", "Unknown")
        suppressed = inst.get("suppressed", False)
        if not suppressed:
            items.append({
                "name": name,
                "type": inst_type,
                "parent": None,  # Root level - no parent
                "depth": 0
            })
    
    # Process subassemblies (these are the definitions of nested assemblies)
    if include_subassemblies:
        for sub in assembly_data.get("subAssemblies", []):
            # Find the name of this subassembly from root instances
            sub_element_id = sub.get("elementId")
            sub_name = None
            
            # Look for the instance in root that references this subassembly
            for inst in root.get("instances", []):
                if inst.get("elementId") == sub_element_id and inst.get("type") == "Assembly":
                    # Strip the instance counter (e.g., " <1>") to get clean name
                    sub_name = inst.get("name", "Unknown Subassembly")
                    break
            
            if not sub_name:
                sub_name = f"Subassembly ({sub_element_id[:8]}...)" if sub_element_id else "Unknown Subassembly"
            
            # Each subassembly has its own instances
            for inst in sub.get("instances", []):
                name = inst.get("name", "Unnamed")
                inst_type = inst.get("type", "Unknown")
                suppressed = inst.get("suppressed", False)
                if not suppressed:
                    items.append({
                        "name": name,
                        "type": inst_type,
                        "parent": sub_name,
                        "depth": 1
                    })
    
    return items

def get_assembly_instances(did, wid, eid, auth):
    """
    Get assembly definition and extract instance names (legacy wrapper).
    Returns list of instance names for backward compatibility.
    """
    data = get_assembly_definition(did, wid, eid, auth)
    if not data:
        return []
    
    items = extract_assembly_items(data, include_subassemblies=True)
    # Return just names for backward compatibility
    return [item["name"] for item in items]

def get_assembly_items_detailed(did, wid, eid, auth):
    """
    Get detailed assembly items with type and hierarchy information.
    
    Returns list of dicts with:
    - name: Instance name
    - type: "Part", "Assembly", "Feature" 
    - parent: Parent assembly name (None for root items)
    - depth: Nesting level
    """
    data = get_assembly_definition(did, wid, eid, auth)
    if not data:
        return []
    
    return extract_assembly_items(data, include_subassemblies=True)

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

    all_parts = []  # For CSV: list of (tab_name, parent, part_name, item_type, quantity)

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
                    clean_name, qty = extract_quantity_from_name(p)
                    print(f"   • {clean_name}" + (f" (qty: {qty})" if qty > 1 else ""))
                    all_parts.append((name, "(Part Studio)", clean_name, "Part", qty))
            else:
                print("   (No parts found)")

    # Process Assemblies
    if assemblies:
        print("\n\nASSEMBLIES")
        print("-"*40)
        for elem in assemblies:
            tab_name = elem.get("name", "Unnamed")
            eid = elem.get("id")
            print(f"\n→ {tab_name}")
            
            # Get detailed items with hierarchy info
            items = get_assembly_items_detailed(args.did, args.wid, eid, auth)
            
            if items:
                # Group items by parent for display
                root_items = [i for i in items if i["parent"] is None]
                
                # Get unique parent names for subassembly grouping
                parent_names = sorted(set(i["parent"] for i in items if i["parent"] is not None))
                
                # Display and collect root items
                if root_items:
                    print("   Root assembly instances:")
                    for item in sorted(root_items, key=lambda x: x["name"]):
                        clean_name, qty = extract_quantity_from_name(item['name'])
                        type_indicator = "[ASM]" if item["type"] == "Assembly" else "[PRT]"
                        qty_str = f" (qty: {qty})" if qty > 1 else ""
                        print(f"     {type_indicator} {clean_name}{qty_str}")
                        all_parts.append((tab_name, "(Root)", clean_name, item['type'], qty))
                
                # Display and collect items grouped by their parent subassembly
                for parent_name in parent_names:
                    sub_items = [i for i in items if i["parent"] == parent_name]
                    if sub_items:
                        print(f"   └─ {parent_name}:")
                        for item in sorted(sub_items, key=lambda x: x["name"]):
                            clean_name, qty = extract_quantity_from_name(item['name'])
                            type_indicator = "[ASM]" if item["type"] == "Assembly" else "[PRT]"
                            qty_str = f" (qty: {qty})" if qty > 1 else ""
                            print(f"        {type_indicator} {clean_name}{qty_str}")
                            all_parts.append((tab_name, parent_name, clean_name, item['type'], qty))
            else:
                print("   (No instances found)")

    print("\n" + "="*70)
    print("AUDIT COMPLETE")
    print(f"Total unique part/instance names collected: {len(all_parts)}")
    print("="*70)

    # Optional CSV export
    if args.csv:
        if all_parts:
            csv_filename = "OnShape_PartNameAudit.csv"
            with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Tab Name", "Parent Assembly", "Part/Instance Name", "Type", "Quantity"])
                writer.writerows(all_parts)
            print(f"\nCSV exported: {csv_filename}")
        else:
            print("\n⚠ CSV export skipped: No parts/instances were successfully retrieved.")

    print("\nReview the list above and ensure all names follow your team's naming convention.")
    print("="*70)

    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()