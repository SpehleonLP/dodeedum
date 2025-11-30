#ifndef DODEEDUM_PROJECT2D_H
#define DODEEDUM_PROJECT2D_H
#include <filesystem>
#include <functional>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/ext/vector_float2.hpp>
#include <span>
#include <string>
#include <vector>

namespace DoDeeDum
{

struct AABB2D;

struct DebugOut
{
	int id_no{};
	const char * directory{};
	const char * name{};
	
	bool empty() const { return directory == 0L || name == 0L; };
	std::filesystem::path file(const char * tag) const;
};

template<typename T> 
inline std::array<T, 3> sort3(std::array<T, 3> const& values)
{
	int _min = values[0] < values[1]? 0 : 1;
	int _max = 1 - _min;
	
	_min = values[_min] <= values[2]? _min : 2;
	_max = values[_max] > values[2]? _max : 2;
	//the three indices must sum to 0+1+2=3.
	int _mid = 3 - _min - _max;
		
	return {values[_min], values[_mid], values[_max]};	
}
	
struct Primitive;
using Mesh = std::vector<Primitive>;

// get the weights for the mesh as if the joints were a single joint
// O(verts+J)
// if all 1.0 then will be empty.
// if defined but sub-array empty then all in primitive is 0.
std::vector<std::vector<float>>	 GetSubsetWeights(Mesh const& mesh, std::span<const uint32_t> joints = {});
// O(tris)
std::vector<std::vector<bool>>	 GetSubsetMarks(Mesh const& mesh, std::vector<std::vector<float>> const& weights);
float GetAverageWeight(glm::vec3 w);

bool GetIntersection(glm::vec2 & dst, const glm::dvec2 &a0,  const glm::dvec2 &a1, const glm::dvec2 &b0,  const glm::dvec2 &b1);

struct ProjectedMesh
{
	static ProjectedMesh Factory(Mesh const& mesh, glm::mat4 const& projection, std::function<glm::mat4(Primitive const&, int index)> const& GetSkinningTransform, DebugOut const& out, float cutoff = 0.5, std::span<const uint32_t> joints = {}) 
		{ return ProjectedMesh(ProjectedMesh(mesh, projection, GetSkinningTransform, out, joints), out, cutoff); };

	ProjectedMesh() = default;
	ProjectedMesh(ProjectedMesh const&, DebugOut const& out, float cutoff);
	ProjectedMesh(Mesh const& mesh, glm::mat4 const& projection, std::function<glm::mat4(Primitive const&, int index)>  const& GetSkinningTransform, DebugOut const& out, std::span<const uint32_t> joints = {});
//	ProjectedMesh(Mesh const& mesh, glm::vec3 point, glm::vec3 normal, std::span<const uint32_t> joints = {});
	~ProjectedMesh();
	
	// w is weight. 
	// sorted by y then x
	std::vector<glm::vec4>  points;
	// each tri is sorted so z is its largest index.
	// then they are sorted by least maximum. 
	std::vector<glm::uvec3> tris;	
	
	glm::vec2 min{0}, max{0};
	glm::vec2 translated_by{0};
	
	template<typename F>
	void for_each_intersection(uint32_t A, uint32_t B, F const& f) const;
	
	float get_value_at_point(glm::vec2 const& point, uint32_t tri) const;
	
	glm::vec2 center_projection();
	
	// Serialization functions
	bool serialize(std::string const& filepath) const;
	void export_debug_OBJ(std::filesystem::path const&) const;
	
	static ProjectedMesh deserialize(std::filesystem::path const&);

	bool   polygons_share_edge(uint32_t a, uint32_t b) const;
	bool   polygons_intersect(uint32_t a, uint32_t b) const;
	AABB2D get_polygon_bounds(uint32_t a) const;
	bool   point_in_polygon(glm::vec2 const& p, uint32_t a) const;
	size_t get_polygon_count() const { return tris.size(); }	
		
private:	
	void sort();
};

template<typename F>
void ProjectedMesh::for_each_intersection(uint32_t A, uint32_t B, F const& f) const
{
	for(int i = 0; i < 3; ++i)
	{
		auto a0 = tris[A][i];
		auto a1 = tris[A][(i+1)%3];
		
		for(int j = 0; j < 3; ++j)
		{
			auto b0 = tris[B][j];
			auto b1 = tris[B][(j+1)%3];
			
			glm::vec2 dst;
			if(GetIntersection(dst, (glm::vec2&)points[a0], (glm::vec2&)points[a1], (glm::vec2&)points[b0], (glm::vec2&)points[b1]))
			{
				if(f(dst)) return;
			}
		}
	}
}

}


#endif // DODEEDUM_PROJECT2D_H
