#!/usr/bin/env python3
"""
3D Mesh Optimization & Decimation Script for Power Transmission Tower Models.
Reduces redundant face and vertex counts while preserving tower geometry and silhouette.
"""

import sys
import os
import argparse
import numpy as np
import trimesh
import fast_simplification

def optimize_mesh(input_path, output_path, target_reduction=0.5, weld_only=False):
    """
    Optimizes a 3D PLY mesh by:
    1. Welding duplicate vertices (merging shared points with 0.1mm tolerance)
    2. Cleaning up degenerate / zero-area faces and duplicate faces
    3. Applying Quadric Edge Collapse Decimation (QEM) to simplify mesh triangles
    """
    if not os.path.exists(input_path):
        print(f"Error: Input file '{input_path}' not found.")
        sys.exit(1)

    print("=" * 60)
    print(f" 3D Mesh Optimization Pipeline")
    print("=" * 60)
    print(f" Input File       : {input_path}")
    print(f" Target Output   : {output_path}")
    print(f" Reduction Ratio : {target_reduction * 100:.1f}% reduction target")

    orig_size = os.path.getsize(input_path)

    # 1. Load mesh
    print("\n[Step 1/3] Loading mesh and welding duplicate vertices...")
    raw_mesh = trimesh.load(input_path, process=False, skip_materials=True)
    orig_verts_count = len(raw_mesh.vertices)
    orig_faces_count = len(raw_mesh.faces)
    print(f"  -> Original Mesh : {orig_verts_count:,} vertices, {orig_faces_count:,} faces")

    # Vertex Welding: round coordinates to 4 decimals to merge split duplicate vertices
    v_rounded = np.round(raw_mesh.vertices, 4)
    unique_v_idx, inverse_idx = trimesh.grouping.unique_rows(v_rounded)
    welded_verts = v_rounded[unique_v_idx]
    welded_faces = inverse_idx[raw_mesh.faces]

    mesh = trimesh.Trimesh(vertices=welded_verts, faces=welded_faces, process=False)
    welded_verts_count = len(mesh.vertices)
    print(f"  -> After Welding : {welded_verts_count:,} vertices ({((orig_verts_count - welded_verts_count)/orig_verts_count)*100:.1f}% merged)")

    # 2. Cleanup Degenerate & Duplicate Faces
    print("\n[Step 2/3] Cleaning degenerate & duplicate faces...")
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    clean_verts_count = len(mesh.vertices)
    clean_faces_count = len(mesh.faces)
    print(f"  -> Cleaned Mesh  : {clean_verts_count:,} vertices, {clean_faces_count:,} faces")

    # 3. Decimation (Quadric Edge Collapse)
    if weld_only:
        print("\n[Step 3/3] Skipping decimation (--weld-only selected).")
        opt_mesh = mesh
    else:
        print(f"\n[Step 3/3] Applying Quadric Edge Collapse decimation (Target: {target_reduction*100:.0f}% face reduction)...")
        v_opt, f_opt = fast_simplification.simplify(mesh.vertices, mesh.faces, target_reduction=target_reduction)
        opt_mesh = trimesh.Trimesh(vertices=v_opt, faces=f_opt, process=True)

    final_verts_count = len(opt_mesh.vertices)
    final_faces_count = len(opt_mesh.faces)

    # 4. Export Optimized Mesh
    print(f"\nExporting optimized mesh to '{output_path}'...")
    opt_mesh.export(output_path)
    opt_size = os.path.getsize(output_path)

    # 5. Print Optimization Summary Report
    print("\n" + "=" * 60)
    print(" OPTIMIZATION SUMMARY REPORT")
    print("=" * 60)
    print(f" Metrics              | Original      | Optimized     | Reduction (%)")
    print("-" * 60)
    print(f" Polygon Face Count   | {orig_faces_count:>11,} | {final_faces_count:>11,} | -{(1 - final_faces_count/orig_faces_count)*100:>6.2f}%")
    print(f" Vertex Count         | {orig_verts_count:>11,} | {final_verts_count:>11,} | -{(1 - final_verts_count/orig_verts_count)*100:>6.2f}%")
    print(f" File Size            | {orig_size/1024:>9.1f} KB | {opt_size/1024:>9.1f} KB | -{(1 - opt_size/orig_size)*100:>6.2f}%")
    print("=" * 60)
    print(f"SUCCESS: Optimized 3D model saved to [ {output_path} ]\n")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_input = os.path.join(script_dir, "powerTransmissionTower.ply")
    default_output = os.path.join(script_dir, "powerTransmissionTower_optimized.ply")

    parser = argparse.ArgumentParser(description="Optimize 3D PLY mesh by removing redundant faces and vertices.")
    parser.add_argument("-i", "--input", default=default_input, help=f"Input PLY file (default: {os.path.basename(default_input)})")
    parser.add_argument("-o", "--output", default=default_output, help=f"Output PLY file (default: {os.path.basename(default_output)})")
    parser.add_argument("-r", "--reduction", type=float, default=0.5, help="Target face reduction ratio between 0.0 and 0.9 (default: 0.5 -> 50%% face reduction)")
    parser.add_argument("--weld-only", action="store_true", help="Only perform vertex welding and face deduplication without decimation")

    args = parser.parse_args()
    optimize_mesh(args.input, args.output, args.reduction, args.weld_only)

if __name__ == "__main__":
    main()
