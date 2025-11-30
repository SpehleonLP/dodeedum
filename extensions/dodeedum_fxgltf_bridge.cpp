#include "dodeedum_fxgltf_bridge.h"
#include "fx/gltf.h"
#include <vector>

namespace DoDeeDum
{
// Helper structure to hold glTF vertex attribute data for rintintin processing
struct GLTFAttributeData {
using ComponentType = fx::gltf::Accessor::ComponentType;
using Type = fx::gltf::Accessor::Type;
    const fx::gltf::Document* document;
    const fx::gltf::Accessor* accessor;
    const fx::gltf::BufferView* bufferView;
    const uint8_t* data;
    
    ComponentType componentType{};
    Type type{};
    uint32_t count{};
    
    
    GLTFAttributeData(const fx::gltf::Document* doc, int32_t accessorIdx) 
        : document(doc), accessor(nullptr), bufferView(nullptr), data(nullptr) {
        if (accessorIdx >= 0 && accessorIdx < static_cast<int32_t>(doc->accessors.size())) {
            accessor = &doc->accessors[accessorIdx];
            componentType = accessor->componentType;
            type = accessor->type;
            count = accessor->count;
            
            GetTypeSizeInBytes();
            
            if (accessor->bufferView >= 0 && accessor->bufferView < static_cast<int32_t>(doc->bufferViews.size())) {
                bufferView = &doc->bufferViews[accessor->bufferView];
                if (bufferView->buffer >= 0 && bufferView->buffer < static_cast<int32_t>(doc->buffers.size())) {
                    const auto& buffer = doc->buffers[bufferView->buffer];
                    data = buffer.data.data() + bufferView->byteOffset + accessor->byteOffset;
                }
            }
        }
    }
    
    bool isValid() const {
        return accessor && bufferView && data;
    }
    

	static int32_t GetComponentSizeInBytes(ComponentType componentType)
	{
		static auto constexpr SignedInt = ComponentType(int(ComponentType::UnsignedInt)-1);
		
		switch(componentType)
		{
		default:							throw std::invalid_argument("componentType");
		case ComponentType::Byte:			return 1;
		case ComponentType::UnsignedByte:	return 1;
		case ComponentType::Short:			return 2;
		case ComponentType::UnsignedShort:	return 2;
		case SignedInt: 	                return 4;
		case ComponentType::UnsignedInt:	return 4;
		case ComponentType::Float:			return 4;
		}
	}
	static int32_t GetNoComponentsInType(Type type)
	{
		switch(type)
		{
		default:			throw std::invalid_argument("type");
		case Type::Scalar:	return 1;
		case Type::Vec2:	return 2;
		case Type::Vec3:	return 3;
		case Type::Vec4:	return 4;
		case Type::Mat2:	return 4;
		case Type::Mat3:	return 9;
		case Type::Mat4:	return 16;
		}
	}
	static int32_t GetTypeSizeInBytes(Type type, ComponentType componentType)  { return GetNoComponentsInType(type) * GetComponentSizeInBytes(componentType); }

	inline int32_t GetComponentSizeInBytes() const { return GetComponentSizeInBytes(componentType); }
	inline int32_t GetNoComponentsInType() const { return GetNoComponentsInType(type); }
	inline int32_t GetTypeSizeInBytes() const { return GetTypeSizeInBytes(type, componentType); }
	inline uint32_t byteLength() const { return GetTypeSizeInBytes(type, componentType) * count; }
};

}

static DoDeeDum::MeshAttrib MeshAttrib_Factory(fx::gltf::Document const& doc, uint32_t accessor)
{
	DoDeeDum::GLTFAttributeData gltfData(&doc, accessor);
    if (!gltfData.isValid()) {
        throw std::runtime_error("Invalid glTF attribute data");
    }
    
	DoDeeDum::MeshAttrib attrib = {};
    attrib.src = const_cast<void*>(static_cast<const void*>(gltfData.data));
    attrib.byteLength = gltfData.bufferView->byteLength;
    attrib.type = DoDeeDum::AttribType(gltfData.accessor->componentType);
    attrib.size = gltfData.GetNoComponentsInType();
    attrib.normalized = gltfData.accessor->normalized ? 1 : 0;
    attrib.stride = static_cast<unsigned short>(gltfData.bufferView->byteStride);
    if (attrib.stride == 0) {
        // If stride is 0, it means tightly packed
        attrib.stride = static_cast<unsigned short>(
            gltfData.GetComponentSizeInBytes() * attrib.size
        );
    }
    attrib.offset = 0; // Already accounted for in the data pointer
    
    return attrib;
}

// Convert glTF primitive mode to rintintin geometry type
static DoDeeDum::GeomType gltfToUtilityGeometry(fx::gltf::Primitive::Mode mode) {
    switch (mode) {
        case fx::gltf::Primitive::Mode::Triangles: return DoDeeDum::GeomType::TRIANGLES;
        case fx::gltf::Primitive::Mode::TriangleStrip: return DoDeeDum::GeomType::TRIANGLE_STRIP;
        case fx::gltf::Primitive::Mode::TriangleFan: return DoDeeDum::GeomType::TRIANGLE_FAN;
        default: throw std::runtime_error("Unsupported primitive mode for rintintin");
    }
}

static DoDeeDum::Primitive Primitive_Factory(const fx::gltf::Document& document, fx::gltf::Primitive const& primitive)
{
    // Get required vertex attributes
    auto positionIter = primitive.attributes.find("POSITION");
    auto jointsIter = primitive.attributes.find("JOINTS_0");
    auto weightsIter = primitive.attributes.find("WEIGHTS_0");
    
    if (positionIter == primitive.attributes.end() ||
        jointsIter == primitive.attributes.end() ||
        weightsIter == primitive.attributes.end()) {
        throw std::runtime_error("Required vertex attributes (POSITION, JOINTS_0, WEIGHTS_0) not found");
    }
    
    auto position = MeshAttrib_Factory(document, positionIter->second);
    
    DoDeeDum::Primitive p = {};
    
     // Set up rintintin mesh structure
    p.position = ([position](uint32_t i) { return position.readf(i); });
    
    int counter = 0;
    while(jointsIter != primitive.attributes.end() 
		&& weightsIter != primitive.attributes.end() )
    {
		auto joints = MeshAttrib_Factory(document, jointsIter->second);
		auto weights = MeshAttrib_Factory(document, weightsIter->second);
    
		p.skin.push_back({
			.joints=([joints](uint32_t i) { return joints.readi(i); }),
			.weights=([weights](uint32_t i) { return weights.readf(i); })
		});
		
		++counter;
		jointsIter = primitive.attributes.find("JOINTS_" + std::to_string(counter));
		weightsIter = primitive.attributes.find("WEIGHTS_" + std::to_string(counter));
    }
    
    
	p.indices.no_verts = document.accessors[positionIter->second].count;
    p.indices.geometry_type = gltfToUtilityGeometry(primitive.mode);
    
    // Handle indices
    if (primitive.indices >= 0) {
        auto indexData =  DoDeeDum::GLTFAttributeData(&document, primitive.indices);
        if (indexData.isValid()) {
            p.indices.index_array_buffer = indexData.data;
            p.indices.index_type = DoDeeDum::AttribType(indexData.accessor->componentType);
            p.indices.no_indices = indexData.accessor->count;
        }
    } else {
        p.indices.index_array_buffer = nullptr;
        p.indices.index_type = DoDeeDum::AttribType(0);
        p.indices.no_indices = 0;
    }
    
    if(uint32_t(primitive.material) < document.materials.size())
    {
		auto & mat = document.materials[primitive.material];
		p.isDoubleSided = mat.doubleSided;
		p.hasAlpha = mat.alphaMode != fx::gltf::Material::AlphaMode::Opaque;
    }
    
    return p;
}

std::vector<DoDeeDum::Mesh> DoDeeDum::GetMeshes(fx::gltf::Document const& doc)
{
	std::vector<DoDeeDum::Mesh> r(doc.meshes.size());
	
	for(auto i = 0u; i < r.size(); ++i)
	{
		r[i] = GetMesh(doc, i);
	}
	
	return r;
}

DoDeeDum::Mesh DoDeeDum::GetMesh(fx::gltf::Document const& doc, int32_t i)
{
	Mesh mesh;
	
	mesh.reserve(doc.meshes[i].primitives.size());

	for(auto & primitive : doc.meshes[i].primitives)
	{
		switch(primitive.mode)
		{
		default:
			break;
			
		case fx::gltf::Primitive::Mode::Triangles:
		case fx::gltf::Primitive::Mode::TriangleStrip:
		case fx::gltf::Primitive::Mode::TriangleFan:
			mesh.push_back(Primitive_Factory(doc, primitive));
			break;
		}
	}
	
	return mesh;
}
