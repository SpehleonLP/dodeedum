#ifndef DODEEDUM_FXGLTF_BRIDGE_H
#define DODEEDUM_FXGLTF_BRIDGE_H
#include "dodeedum_mesh.h"

namespace fx { namespace gltf { struct Document; struct Primitive; } }

namespace DoDeeDum
{

std::vector<Mesh> GetMeshes(fx::gltf::Document const& doc);

}

#endif // DODEEDUM_FXGLTF_BRIDGE_H
