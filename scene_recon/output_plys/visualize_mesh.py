#!/usr/bin/env python3
"""Quick visualization of a mesh"""
from pathlib import Path

import trimesh

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Quick visualization of a mesh.\n\t')
    parser.add_argument(
        'mesh_path', type=str, help="Path to the mesh")
    args = parser.parse_args()

    # Verify mesh_path
    args.mesh_path = Path(args.mesh_path).resolve()
    assert args.mesh_path.is_file(), f"Path {args.mesh_path} does not exist"

    mesh = trimesh.load(args.mesh_path)
    mesh.show(flags={'cull': False})
