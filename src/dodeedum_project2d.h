#ifndef DODEEDUM_PROJECT2D_H
#define DODEEDUM_PROJECT2D_H
#include <filesystem>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/ext/vector_float2.hpp>
#include <span>
#include <string>
#include <vector>

namespace DoDeeDum
{

struct Primitive;
using Mesh = std::vector<Primitive>;

// get the weights for the mesh as if the joints were a single joint
// O(verts+J)
// if all 1.0 then will be empty.
// if defined but sub-array empty then all in primitive is 0.
std::vector<std::vector<float>>	 GetSubsetWeights(Mesh const& mesh, std::span<uint32_t> joints = {});
// O(tris)
std::vector<std::vector<bool>>	 GetSubsetMarks(Mesh const& mesh, std::vector<std::vector<float>> const& weights);
float GetAverageWeight(glm::vec3 w);

bool GetIntersection(glm::vec2 & dst, glm::vec2 const& a0,  glm::vec2 const& a1, glm::vec2 const& b0,  glm::vec2 const& b1);

struct ProjectedMesh
{
	ProjectedMesh(Mesh const& mesh, glm::mat4 const& projection, std::span<uint32_t> joints = {});
	ProjectedMesh(Mesh const& mesh, glm::vec3 point, glm::vec3 normal, std::span<uint32_t> joints = {});
	~ProjectedMesh();
	
	// z is weight. 
	// sorted by y then x
	std::vector<glm::vec3>  points;
	// each tri is sorted so {min, mid, max} indices
	// then they are sorted by least maximum. 
	std::vector<glm::uvec3> tris;	
	
	glm::vec2 min, max;
	
	template<typename F>
	void for_each_intersection(uint32_t A, uint32_t B, F const& f) const;
	
	float get_value_at_point(glm::vec2 const& point, uint32_t tri) const;
	
	// Serialization functions
	bool serialize(std::string const& filepath) const;
	ProjectedMesh(std::filesystem::path const&);
	
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
