#ifndef DODEEDUM_H
#define DODEEDUM_H
#include "dodeedum_mesh.h"
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <span>
#include <vector>

namespace DoDeeDum
{

struct FastTri;
struct Primitive;
using Mesh = std::vector<Primitive>;

// if something is marked not-to-get it may still be gotten as a consequence of another rule. 
enum Options
{
	GET_DELAUNY_PERIMITER = 1 << 0,
	GET_OUTLINE = 1 << 1,
	GET_AREA = 1 << 2,
	GET_WEIGHTED_AREA = 1 << 3,
	GET_AREA_2ND_MOMENT = 1 << 4,
	GET_WEIGHTED_AREA_2ND_MOMENT = 1 << 5,
	GET_CENTER_OF_AREA = 1 << 6,
	
	// add a point representing the joint, this will be at 0,0 
	INCLUDE_CENTER = 1 << 7,
	
	GET_ALL = ~0u
};

struct Silhouette
{
	std::vector<glm::vec2>  points;
	std::vector<uint32_t>   outline;
	std::vector<uint32_t> delauny_triangulation; 
	
	double area{};
	double weightedArea{}; // skinning weights
	
	struct Moment2
	{
		float xx, yy, xy;
	};
	
	Moment2 area2ndMoment{};
	Moment2 weightedArea2ndMoment{};
	glm::vec2 center{};
};

// joints is what skinning weights we're interested in. 
std::vector<Silhouette> GetSilhouettes(Mesh const& mesh, glm::mat4 const& projection, Options = GET_ALL, std::span<uint32_t> joints = {});

}


#endif // DODEEDUM_H
