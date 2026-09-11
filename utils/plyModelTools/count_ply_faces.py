#!/usr/bin/env python3
"""
Script to count the number of faces and vertices in a .ply 3D model file.
Default file: powerTransmissionTower.ply in the same directory.
"""

import sys
import os

def parse_ply_header(file_path):
    """
    Parse PLY file header to extract format and element counts (faces, vertices, etc.).
    Supports both ASCII and Binary PLY formats without external dependencies.
    """
    header_info = {
        'format': 'unknown',
        'elements': {},
        'element_order': []
    }
    
    current_element = None

    with open(file_path, 'rb') as f:
        # Read lines in binary mode to handle line breaks robustly
        line = f.readline()
        if not line.strip().startswith(b'ply'):
            raise ValueError(f"File '{file_path}' is not a valid PLY file (missing 'ply' header magic signature).")
            
        while True:
            line = f.readline()
            if not line:
                break
                
            line_str = line.decode('ascii', errors='ignore').strip()
            if not line_str:
                continue
                
            tokens = line_str.split()
            if tokens[0] == 'format':
                if len(tokens) >= 2:
                    header_info['format'] = ' '.join(tokens[1:])
            elif tokens[0] == 'element':
                if len(tokens) >= 3:
                    elem_name = tokens[1]
                    elem_count = int(tokens[2])
                    header_info['elements'][elem_name] = elem_count
                    header_info['element_order'].append(elem_name)
                    current_element = elem_name
            elif tokens[0] == 'end_header':
                break

    return header_info

def count_ply_faces(file_path):
    """
    Counts and prints face details of a .ply file.
    """
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        sys.exit(1)
        
    print(f"Reading PLY file: {file_path}")
    
    # 1. Read header using native parser
    try:
        header = parse_ply_header(file_path)
    except Exception as e:
        print(f"Error reading PLY header: {e}")
        sys.exit(1)
        
    face_count = header['elements'].get('face', 0)
    vertex_count = header['elements'].get('vertex', 0)
    
    print("\n" + "=" * 50)
    print(f" PLY 3D Model Details: {os.path.basename(file_path)}")
    print("=" * 50)
    print(f" File Format   : {header['format']}")
    print(f" Vertex Count  : {vertex_count:,}")
    print(f" Face Count    : {face_count:,}")
    
    # Print any other elements found in header (e.g. edge, camera, etc.)
    other_elements = {k: v for k, v in header['elements'].items() if k not in ('vertex', 'face')}
    if other_elements:
        print(" Other Elements:")
        for elem, count in other_elements.items():
            print(f"   - {elem}: {count:,}")
            
    print("=" * 50)
    
    # 2. Try validating with plyfile if available
    try:
        from plyfile import PlyData
        plydata = PlyData.read(file_path)
        if 'face' in plydata:
            plyfile_face_count = len(plydata['face'])
            print(f"[Verified with plyfile library] Face Count: {plyfile_face_count:,}")
    except ImportError:
        pass
    except Exception as e:
        print(f"[plyfile validation skipped]: {e}")
        
    return face_count

def main():
    # If a file path is provided via command line argument, use it; otherwise default to powerTransmissionTower.ply in script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_ply = os.path.join(script_dir, "powerTransmissionTower.ply")
    
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
    else:
        target_file = default_ply

    count_ply_faces(target_file)

if __name__ == "__main__":
    main()
