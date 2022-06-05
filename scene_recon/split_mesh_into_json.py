#!/usr/bin/env python3
"""Split a mesh file into chunks and save as a JSON representation
Usage: python3 split_mesh_into_json.py ./output_plys/d435i_corridor1.ply --box-size 3.0
"""
import json
from pathlib import Path

import numpy as np
import pyvista as pv  # pip3 install pyvista pythreejs  # last one for notebook visualization
from tqdm import tqdm

from typing import Dict

def crop_mesh_with_box(m: pv.PolyData, origin=[0.0, 0.0, 0.0],
                       extents=[1.0, 1.0, 1.0]) -> pv.PolyData:
    """Crop a mesh with an AABB at given origin and extents"""
    origin, extents = np.asanyarray(origin), np.asanyarray(extents)
    # bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
    bounds = np.vstack([origin - extents, origin + extents]).T.flatten()
    return m.clip_box(bounds, invert=False).extract_surface()


def _slice_mesh_chunks(m: pv.PolyData, box_size=1.0) -> Dict[str, pv.PolyData]:
    """Slice a mesh with AABBs into chunks with given box_size
    Ignore any empty mesh chunk.
    """
    mesh_chunks = {}  # {coord_str: mesh}

    mins, maxs = np.asanyarray(m.bounds).reshape(3, 2).T
    id_mins = np.floor(mins / box_size + 0.5).astype(int)
    id_maxs = np.ceil(maxs / box_size - 0.5).astype(int)

    xx, yy, zz = np.mgrid[id_mins[0]:id_maxs[0]+1,
                          id_mins[1]:id_maxs[1]+1,
                          id_mins[2]:id_maxs[2]+1]

    for x, y, z in zip(tqdm(xx.flatten(), desc='mesh slicing'), yy.flatten(), zz.flatten()):
        name = f"({x},{y},{z})"
        m_chunk = crop_mesh_with_box(
            m, np.array([x, y, z]) * box_size, [box_size] * 3
        )
        if m_chunk.n_points == 0:
            continue

        mesh_chunks[name] = m_chunk

    return mesh_chunks


def slice_mesh_chunks(mesh_path: Path, save_mesh_chunks_dir: Path, box_size=1.0):
    """Slice a mesh with AABBs into chunks with given box_size
    Ignore any empty mesh chunk.

    :param mesh_path : Path to the mesh file to slice
    :param save_mesh_chunks_dir : Path to a directory that stores sliced meshes.
                                  JSON file will be saved to its parent folder.
                                  Example: Path('./test_ply_split/meshes')
    :param box_size : Chunk cube side length in float
    """
    mesh = pv.read(mesh_path)
    print(f'Loaded a TriangleMesh with {mesh.n_points} points and {mesh.n_faces} triangles')

    print('\nBegin mesh slicing .....')
    mesh_chunks = _slice_mesh_chunks(mesh, box_size=box_size)
    print(f'Sliced mesh into {len(mesh_chunks)} chunks')

    mesh_dict = {
        "models": [],
        "name": mesh_path.stem,
        "scale": 1.0
    }

    print('\nBegin saving mesh chunks .....')
    for chunk_coord_str, m_chunk in tqdm(mesh_chunks.items(), desc='mesh saving'):
        save_m_chunk_path = save_mesh_chunks_dir / f"chunk_{chunk_coord_str}.ply"

        m_chunk.save(save_m_chunk_path, binary=False, texture='RGB')
        mesh_dict["models"].append({
            "name": '/'.join(save_m_chunk_path.parts[-2:])
        })

    save_json_path = save_mesh_chunks_dir.parent / 'config.json'
    with save_json_path.open('w') as f:
        json.dump(mesh_dict, f, indent="    ")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Split a given mesh into chunks and saved as a JSON.\n\t')
    parser.add_argument(
        'mesh_path', type=str, help="Path to the mesh for splitting")
    parser.add_argument(
        "--save-dir", type=str, default=None,
        help="Path to saving directory. If not provided, save to same directory as mesh_path.")
    parser.add_argument(
        "--box-size", type=float, default=1.0, help="Chunk cube side length")
    args = parser.parse_args()

    # Verify mesh_path
    args.mesh_path = Path(args.mesh_path).resolve()
    assert args.mesh_path.is_file(), f"Path {args.mesh_path} does not exist"

    if args.save_dir is None:
        args.save_dir = args.mesh_path.with_name(args.mesh_path.stem + '_split')
    else:
        args.save_dir = Path(args.save_dir).resolve()

    save_mesh_chunks_dir = args.save_dir / 'meshes'
    save_mesh_chunks_dir.mkdir(parents=True, exist_ok=False)

    print(f'\nUsing box_size of {args.box_size}')

    slice_mesh_chunks(args.mesh_path, save_mesh_chunks_dir, args.box_size)

    print(f'\nSuccessfully split "{args.mesh_path.name}" and saved in "{args.save_dir}"\n')
