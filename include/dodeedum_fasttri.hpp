#ifndef DODEEDUM_FASTTRI_HPP
#define DODEEDUM_FASTTRI_HPP
#include <algorithm>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <vector>

namespace TonTon
{

/// Optimized for fast tests of if a point is in the set. 
struct FastTri
{
	FastTri(glm::vec2 const& v0, glm::vec2 const& v1, glm::vec2 const& v2)
	{
        auto denom = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
	
        edges[0] = {v1.y - v2.y, v2.x - v1.x, v1.x*v2.y - v2.x*v1.y};
        edges[1] = {v2.y - v0.y, v0.x - v2.x, v2.x*v0.y - v0.x*v2.y};
     //   edges[2] = {v0.y - v1.y, v1.x - v0.x, v0.x*v1.y - v1.x*v0.y};
	
		edges[0] *= 1.f / denom;
		edges[1] *= 1.f / denom;
		
        // Add bounding box for sorting/culling
        max_y = std::max({v0.y, v1.y, v2.y});
	}
	
	glm::vec3 edges[2];
	// min y has no meaning b/c we don't sort by it... 
    float     max_y;
	
    inline bool contains(glm::vec2 p) const {
        // Fast barycentric test, its fast enough that the bb test gets us very little
        float u = ((edges[0].x * p.x + edges[0].y * p.y + edges[0].z));
        float v = ((edges[1].x * p.x + edges[1].y * p.y + edges[1].z));
        
        return u >= 0 && v >= 0 && (u + v) <= 1;
    }
    
    inline bool operator<(FastTri const& it) const
    {
		return max_y < it.max_y;
    }
};
    
inline bool IsContained(std::vector<FastTri> const& fastTris, glm::vec2 point)
{    // Find first triangle where max_y >= point.y
    auto start = std::lower_bound(fastTris.begin(), fastTris.end(), point.y, 
        [](const FastTri& info, float value) {
            return info.max_y < value;  // This is correct
        });
        
    for(auto itr = start; itr < fastTris.end(); ++itr) 
	{
		if(itr->contains(point))		
			return true;
	}

	return false;
}

}

#endif // DODEEDUM_FASTTRI_HPP
