#include <iostream>  // NOLINT

#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_tango_interface/core/tango_layer_interface.h"
#include "voxblox_tango_interface/io/tango_layer_io.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc != 3) {
    throw std::runtime_error(
        "Args: filename to load, followed by filename to save to");
  }

  const std::string file = argv[1];

  TangoLayerInterface::Ptr layer_from_file;
  io::TangoLoadLayer(file, &layer_from_file);

  std::cout << "Layer memory size: " << layer_from_file->getMemorySize()
            << "\n";

  // Mesh accessories.
  MeshIntegrator<TsdfVoxel>::Config mesh_config;
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel> > mesh_integrator_;

  mesh_layer_.reset(new MeshLayer(layer_from_file->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, layer_from_file.get(), mesh_layer_.get()));

  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  std::cout << "Number of meshes: " << mesh_layer_->getNumberOfAllocatedMeshes()
            << "\n";
  bool meshSuccess = outputMeshLayerAsPly(argv[2], *mesh_layer_);

  if (meshSuccess == false) {
    throw std::runtime_error("Failed to save mesh");
  }

  return 0;
}
