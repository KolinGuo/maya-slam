#!/usr/bin/env python3
"""Quick visualization of a mesh
Usage examples:
    python3 visualize_mesh.py ../scene_recon/output_plys/d435i_corridor1.ply

    python3 visualize_mesh.py ../scene_recon/output_plys/d435i_corridor1_split/meshes

    python3 visualize_mesh.py ../scene_recon/output_plys/d435i_corridor1.ply \
            ../scene_recon/output_plys/d435i_corridor1_pose_refine.ply

    python3 visualize_mesh.py ../scene_recon/output_plys/d435i_corridor1_split/meshes \
            ../scene_recon/output_plys/d435i_corridor1_pose_refine.ply \
            ../scene_recon/output_plys/d435i_corridor2_split/meshes \
            ../scene_recon/output_plys/d435i_corridor2_pose_refine.ply
"""
from pathlib import Path

from tqdm import tqdm
import pyvista as pv

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Quick visualization of a mesh.\n\t')
    parser.add_argument(
        'mesh_paths', nargs='+', type=str, help="Paths to the meshes (at most 4)")
    args = parser.parse_args()

    # Check number of mesh paths
    assert len(args.mesh_paths) <= 4, f'Only supports less than 4 meshes but gets {len(args.mesh_paths)} meshes'

    # Verify mesh_path
    args.mesh_paths = [Path(mesh_path).resolve() for mesh_path in args.mesh_paths]
    for mesh_path in args.mesh_paths:
        assert (mesh_path.is_file() and mesh_path.suffix == '.ply') or \
               (mesh_path.is_dir() and len(list(mesh_path.glob('*.ply'))) > 0), \
            f'Path "{mesh_path}" does not exist, is not a mesh file, or does not contain mesh files'

    # Load all meshes
    meshes = {}  # {mesh_path: mesh}
    for mesh_path in args.mesh_paths:
        if mesh_path.is_file():
            print(f'Loading mesh "{mesh_path.name}" .....')
            meshes[mesh_path.name] = pv.read(mesh_path)
        else:
            mesh_path_postfix = '/'.join(mesh_path.parts[-2:])
            print(f'Loading meshes from "{mesh_path_postfix}" .....')

            split_mesh = pv.PolyData()
            for m_chunk_path in tqdm(list(mesh_path.glob('*.ply')), desc='Chunks'):
                # Loading mesh chunks is fast (~10 seconds for ~1 GB mesh)
                # For +=, by default will merge equivalent points which is very slow if lots of points
                # += does perform inplace updates.
                #split_mesh += pv.read(m_chunk_path)

                # Turn off merge_points and enable inplace merging
                split_mesh = split_mesh.merge(pv.read(m_chunk_path), merge_points=False, inplace=True)
            meshes[mesh_path.parent.name] = split_mesh

    # Plotter window shape
    window_shapes = {
        1: (1, 1),
        2: (1, 2),
        3: (2, 2),
        4: (2, 2),
    }
    window_shape = window_shapes[len(args.mesh_paths)]
    # Plotter subplot indices
    subplot_indices = [(0, 0), (0, 1), (1, 0), (1, 1)]

    plotter = pv.Plotter(shape=window_shape)
    for i, (mesh_name, mesh) in enumerate(meshes.items()):
        subplot_index = subplot_indices[i]
        plotter.subplot(*subplot_index)

        plotter.add_text(mesh_name, font_size=18)
        plotter.add_mesh(mesh, rgb=True)
        plotter.add_axes(interactive=True)
        plotter.add_bounding_box(color='orange')

    plotter.show()
