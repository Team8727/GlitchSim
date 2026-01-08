import pandas as pd
import re
import sys

# Separate schema for easy modification
# Allowed locations (as set for fast lookup)
ALLOWED_LOCATIONS = {
    'CHA', 'STR', 'INT', 'ARM', 'LEV', 'SHO', 'TUR', 'IND', 'END', 'HOP',
    'CLB', 'DRV', 'SEN', 'BAT', 'ELE'
}

# Custom types with regex for qualifiers (case upper, no spaces)
CUSTOM_TYPES = {
    'TU': r'\d+(?:\.\d+)?(?:X\d+(?:\.\d+)?){1,2}',
    'STF': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?',
    'HX': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?',
    'CH': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?(?:MM)?',
    'LB': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?X\d+(?:\.\d+)?\-\d+(?:\.\d+)?',
    'ALS': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?\-\d+(?:\.\d+)?\-\d{2}',
    'STL': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?\-\d+(?:\.\d+)?\-\d{2}',
    'HKA': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?\-\d+(?:\.\d+)?\-\d{2}',
    'HKS': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?\-\d+(?:\.\d+)?\-\d{2}',
    'PC': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?\-\d+(?:\.\d+)?\-\d{2}',
    '3DP': r'[A-Z]+\-\d+(?:\.\d+)?(?:X\d+(?:\.\d+)?){0,3}\-\d{2}',
    'SP': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?',
    'BT': r'\d+\-\d+\-\d+(?:MM)?',
    'GR': r'\d+\-[0-9A-Z]+\-\d+(?:\.\d+)?(?:MM)?',
    'BR': r'\d+(?:\.\d+)?(?:MM)?X\d+(?:\.\d+)?(?:MM)?X\d+(?:\.\d+)?(?:MM)?',
    'CHN': r'\d+\-\d+',
    'PNC': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?',
    'MM': r'[A-Z0-9]+\-\d{2}',
}

# Premade types with regex for qualifiers
PREMADE_TYPES = {
    'BOLT-1032': r'\d+(?:\.\d+)?(?:\-[A-Z]+)?',
    'BOLT-14': r'\d+(?:\.\d+)?(?:\-[A-Z]+)?',
    'NUT-HEX': r'(?:\d+)?',
    'WSH': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?X\d+(?:\.\d+)?',
    'RIV': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?',
    'SRV': r'[A-Z0-9]+\-\d{2}',
    'MTR': r'[A-Z0-9]+\-\d+(?:\.\d+)?',
    'SNS': r'[A-Z0-9]+\-\d{2}',
    'WHL': r'\d+(?:\.\d+)?X\d+(?:\.\d+)?\-[A-Z]+',
    'BAT-PART': r'\d+\-\d+(?:AH)?',
    'WIR': r'\d+\-\d+\-[A-Z]+',
}

def clean_part_name(name):
    # Remove instance marker like <1> and trim
    name = re.sub(r'\s*<\d+>$', '', name).strip()
    return name

def normalize_for_matching(name):
    # Remove spaces, convert to upper for matching
    return name.upper().replace(' ', '').replace('"', '')

def is_compliant(part_name):
    cleaned = clean_part_name(part_name)
    normalized = normalize_for_matching(cleaned)
    
    if not re.match(r'^[A-Z]{3}\-', normalized):
        return False
    
    parts = normalized.split('-')
    loc = parts[0]
    if loc not in ALLOWED_LOCATIONS:
        return False
    
    # Check premade types (compound keys)
    for key in sorted(PREMADE_TYPES, key=len, reverse=True):  # Longer keys first
        key_upper = key.upper()
        if normalized.startswith(loc + '-' + key_upper):
            qual_start = len(loc + '-' + key_upper)
            qual = normalized[qual_start + 1:] if qual_start < len(normalized) - 1 else ''
            if re.match(PREMADE_TYPES[key] + r'$', qual):
                return True
    
    # Check custom types
    if len(parts) > 1 and parts[1] in CUSTOM_TYPES:
        qual = '-'.join(parts[2:])
        if re.match(CUSTOM_TYPES[parts[1]] + r'$', qual):
            return True
    
    return False

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python script.py <csv_file_path>")
        sys.exit(1)
    
    csv_path = sys.argv[1]
    df = pd.read_csv(csv_path)
    
    # Add compliant column
    df['Compliant'] = df['Part/Instance Name'].apply(lambda x: 'Yes' if is_compliant(x) else 'No')
    
    # Get flagged rows
    flagged_df = df[df['Compliant'] == 'No']
    
    # Output to console
    print("Flagged non-compliant rows:")
    print(flagged_df.to_string(index=False))
    
    # Optionally save to new CSV
    output_path = 'flagged_' + csv_path.split('/')[-1]
    flagged_df.to_csv(output_path, index=False)
    print(f"\nFlagged rows saved to {output_path}")