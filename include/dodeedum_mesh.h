#ifndef DoDeeDum_MESH_H
#define DoDeeDum_MESH_H
#include "glm/gtc/type_precision.hpp"
#include <functional>
#include <vector>
#include <array>

namespace DoDeeDum
{

struct Tri
{
	std::array<glm::dvec3, 3> positions;
	glm::vec3 weights;
	uint32_t joint;
};

struct Vert
{
	glm::dvec3 position;
	glm::uvec4 joints;
	glm::vec4  weights;
};


enum class AttribType : uint16_t {
	BYTE           = 0x1400,  ///< Signed 8-bit integer
	UNSIGNED_BYTE  = 0x1401,  ///< Unsigned 8-bit integer
	SHORT          = 0x1402,  ///< Signed 16-bit integer
	UNSIGNED_SHORT = 0x1403,  ///< Unsigned 16-bit integer
	INT            = 0x1404,  ///< Signed 32-bit integer
	UNSIGNED_INT   = 0x1405,  ///< Unsigned 32-bit integer
	FLOAT          = 0x1406,  ///< 32-bit floating point
	DOUBLE         = 0x140A,  ///< 64-bit floating point
	HALF_FLOAT     = 0x140B,  ///< 16-bit floating point
	FIXED          = 0x140C,  ///< Fixed-point number
	INT_2_10_10_10_REV = 0x8D9F,            ///< Packed signed 2.10.10.10 format
	UNSIGNED_INT_2_10_10_10_REV = 0x8368,   ///< Packed unsigned 2.10.10.10 format
	UNSIGNED_INT_10F_11F_11F_REV = 0x8C3B   ///< Packed 10.11.11 floating point format
};

enum class GeomType : uint16_t {
	TRIANGLES,
	TRIANGLE_STRIP,
	TRIANGLE_FAN
};


template<typename T> struct GetAttribType { enum { value = 0 }; };
template<AttribType> struct GetAttribValue { using type = void; }; 

#define DECLARE_ATTRIBUTE(x, y) \
template<> struct GetAttribType<x> { enum { value = (int)AttribType::y }; }; \
template<> struct GetAttribValue<AttribType::y> { using type = x; }; 

DECLARE_ATTRIBUTE(int8_t, BYTE);
DECLARE_ATTRIBUTE(uint8_t, UNSIGNED_BYTE);
DECLARE_ATTRIBUTE(int16_t, SHORT);
DECLARE_ATTRIBUTE(uint16_t, UNSIGNED_SHORT);
DECLARE_ATTRIBUTE(int32_t, INT);
DECLARE_ATTRIBUTE(uint32_t, UNSIGNED_INT);
DECLARE_ATTRIBUTE(float, FLOAT);
DECLARE_ATTRIBUTE(double, DOUBLE);

#undef DECLARE_ATTRIBUTE

struct MeshAttrib
{		
	void * src;
	uint64_t byteLength;
    uint32_t offset;
	AttribType type;
    uint16_t stride;
	uint8_t size;
    bool    normalized;
    
    size_t		 get_type_size() const;
	glm::dvec4   readf(uint32_t index) const;
	glm::i64vec4 readi(uint32_t index) const;
};

struct MeshIndices
{
    void const* index_array_buffer;    ///< Index buffer for indexed geometry
    AttribType index_type;             ///< Data type of indices
    GeomType   geometry_type;			///< Type of geometry primitives
	
    uint64_t no_verts;					///< Number of vertices
    uint64_t no_indices;				///< Number of indices
    
    uint32_t get_triangle_count() const;
    uint32_t read_index(uint32_t i) const;
    glm::uvec3 get_tri(uint32_t index) const;
    
    template<typename F>
    void for_each_tri(F const& func, uint32_t thread_id = 0, uint32_t no_threads = 1) const
    {
		uint64_t N = get_triangle_count();
		uint64_t begin = (thread_id * N) / no_threads;
		uint64_t end = ((thread_id+1) * N) / no_threads;
		
		for(uint64_t i = begin; i < end; ++i)
		{
			if(func(get_tri(i)))
				return;
		}
    }
    
    template<typename F>
    void for_each_index(F const& func, uint32_t thread_id = 0, uint32_t no_threads = 1) const
    {		
		if(index_array_buffer)
		{
			uint64_t N = no_indices; 
			uint64_t begin = (thread_id * N) / no_threads;
			uint64_t end = ((thread_id+1) * N) / no_threads;
	
			for(uint64_t i = begin; i < end; ++i)
			{
				if(func(read_index(i)))
					return;
			}
		}
		else
		{
			uint64_t N = no_verts; 
			uint64_t begin = (thread_id * N) / no_threads;
			uint64_t end = ((thread_id+1) * N) / no_threads;
	
			for(uint64_t i = begin; i < end; ++i)
			{
				if(func(i))
					return;
			}
		}
    }
};

struct Primitive
{
	std::function<glm::dvec4(uint32_t)>   position;
	std::function<glm::i64vec4(uint32_t)> joints;
	std::function<glm::dvec4(uint32_t)>   weights;
	MeshIndices indices;
	bool        flipNormals{false};
	bool        isDoubleSided{true};
	bool        hasAlpha{false};
	
    template<typename F>
    void for_each_tri(F & func, uint32_t thread_id = 0, uint32_t no_threads = 1) const;
    template<typename F>
    void for_each_vertex(F & func, uint32_t thread_id = 0, uint32_t no_threads = 1) const;
};

using Mesh = std::vector<Primitive>;

template<typename F>
void Primitive::for_each_tri(F & func, uint32_t thread_id, uint32_t no_threads) const
{
	indices.for_each_index([&](glm::uvec3 tri) -> bool
	{
		if(flipNormals) std::swap(tri.x, tri.y);
		
		Tri t;
		
		std::array<glm::uvec3, 3> joints;
		std::array<glm::vec4, 3> weights;
		
		t.positions[0] = this->position(tri.x); joints[0] = this->joints(tri.x); weights[0] = this->joints(tri.x);
		t.positions[1] = this->position(tri.y); joints[1] = this->joints(tri.y); weights[1] = this->joints(tri.y);
		t.positions[2] = this->position(tri.z); joints[2] = this->joints(tri.z); weights[2] = this->joints(tri.z);
		
		for(uint32_t v = 0; v < 3; ++v)
		{
			for(uint32_t j = 0; j < 3; ++j)
			{
				if(weights[v][j] == 0)
					continue;
					
				t.weights[v] = weights[v][j];
				t.joint = joints[v][j];
				
				for(uint32_t v1 = v+1; v1 < 3; ++v1)
				{							
					for(uint32_t j1 = 0; j1 < 3; ++j1)
					{
						if(joints[v1][j1] == t.joint)
						{
							t.weights[v1] = weights[v1][j1];
							// don't process same joint again.
							weights[v1][j1] = 0;
							break;
						}
					}
				}
			
				if(func(t))
					return true;
			}
		}
		
		return false;
	}, thread_id, no_threads);
}

template<typename F>
void Primitive::for_each_vertex(F & func, uint32_t thread_id, uint32_t no_threads) const
{
	indices.for_each_index([&](uint32_t index) -> bool
	{
		Vert v;
		v.position = this->position(index);
		v.joints = this->joints(index);
		v.weights = this->weights(index);
	
		return func(v);
	}, thread_id, no_threads);
}

   
}

#endif // UTILITY_MESHREADER_H

