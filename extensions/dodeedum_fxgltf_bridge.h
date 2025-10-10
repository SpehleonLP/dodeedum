#ifndef DODEEDUM_FXGLTF_BRIDGE_H
#define DODEEDUM_FXGLTF_BRIDGE_H
#include "../include/dodeedum_mesh.h"

namespace fx { namespace gltf { struct Document; struct Primitive; } }

namespace DoDeeDum
{

std::vector<Mesh> GetMeshes(fx::gltf::Document const& doc);
Mesh GetMesh(fx::gltf::Document const& doc, int32_t mesh_id);

}

#endif // DODEEDUM_FXGLTF_BRIDGE_H
