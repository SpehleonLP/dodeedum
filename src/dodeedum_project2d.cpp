#include "dodeedum_project2d.h"
#include "dodeedum_mesh.h"
#include <cstring>
#include <fstream>
#include <cfloat>

namespace DoDeeDum
{
	template<typename T> 
	std::array<T, 3> sort3(std::array<T, 3> const& values)
	{
		int _min = values[0] < values[1]? 0 : 1;
		int _max = 1 - _min;
		
		_min = values[_min] <= values[2]? _min : 2;
		_max = values[_max] > values[2]? _max : 2;
		//the three indices must sum to 0+1+2=3.
		int _mid = 3 - _min - _max;
			
		return {values[_min], values[_mid], values[_max]};	
	}
}

DoDeeDum::ProjectedMesh::~ProjectedMesh() = default;

DoDeeDum::ProjectedMesh::ProjectedMesh(Mesh const& mesh, glm::mat4 const& projection, std::span<uint32_t> joints)
{
	min = glm::vec2(FLT_MAX);
	max = glm::vec2(-FLT_MAX);
	
	auto weights = GetSubsetWeights(mesh, joints);
	auto marks   = GetSubsetMarks(mesh, weights);

	std::vector<int> vertex_mapping;
	
	for(auto i = 0u; i < mesh.size(); ++i)
	{
		auto * w = weights.size()? weights[i].data() : nullptr;
		auto & p = mesh[i];
		
		if(marks.size() && marks[i].empty()) continue;
		
		vertex_mapping.clear();
		vertex_mapping.resize(p.indices.no_verts, -1);
		
		for(auto v = 0u; v < vertex_mapping.size(); ++v)
		{
			if(marks.size()	&& !marks[i][v]) continue;
		
			float weight = weights.empty()? 1.0 : (w? w[v] : 0.0);
			
			glm::vec3 pos = p.position(v);
			glm::vec4 vert = projection * glm::vec4(pos, 1.f);
			vert *= vert.w? 1.0 / vert.w : 1.0;
			
			vertex_mapping[v] = points.size();
			points.push_back({vert.x, vert.y, weight});	
			
			min = glm::min(min, (glm::vec2&)vert);	
			max = glm::max(max, (glm::vec2&)vert);			
		}
		
		p.indices.for_each_tri([&](glm::uvec3 const& tri) -> bool
		{
			glm::uvec3 new_tri = { vertex_mapping[tri.x], vertex_mapping[tri.y], vertex_mapping[tri.z] };
			
			if(new_tri.x != ~0u && new_tri.y != ~0u && new_tri.z != ~0u)
			{
				tris.push_back(new_tri);
			}
			
			return false;
		});
	}
	
	if(min.x > max.x)
		min = max = glm::vec2(0);
}
std::vector<std::vector<float>>	 DoDeeDum::GetSubsetWeights(Mesh const& mesh, std::span<uint32_t> joints)
{
	std::vector<std::vector<float>>	 r;
	if(joints.empty()) return {};

	int32_t max_joint = -1;
	
	for(auto i = 0u; i < joints.size(); ++i)
	{
		max_joint = std::max<int32_t>(max_joint, (int32_t)joints[i]);
	}	

	std::vector<bool> joint_mask;
	if(max_joint+1 > 0)
	{
		joint_mask.resize(max_joint + 1);
		
		for(auto j : joints)
			joint_mask[j] = true;
	}
	
	r.resize(mesh.size());
	bool all_one = true;
	
	for(auto & p : mesh)
	{
		std::vector<float> weights(p.indices.no_verts);
		bool non_zero = false;
		
		for(auto i = 0u; i < weights.size(); ++i)
		{
			glm::uvec4 joints0 = p.joints(i);
			glm::vec4  weights0 = p.weights(i);
			
			float total_weight = 0.f;
			
			if(joint_mask.empty())
				total_weight = 1.0;
			else
			{
				if(joints0.x < joint_mask.size() && joint_mask[joints0.x])	total_weight += weights0.x; 
				if(joints0.y < joint_mask.size() && joint_mask[joints0.y])	total_weight += weights0.y; 
				if(joints0.z < joint_mask.size() && joint_mask[joints0.z])	total_weight += weights0.z; 
				if(joints0.w < joint_mask.size() && joint_mask[joints0.w])	total_weight += weights0.w; 
			}
			
			weights[i] = total_weight;
			non_zero |= (total_weight != 0);
			all_one &= (total_weight == 1.0);
		}
		
		if(non_zero)
			r[&p - mesh.data()] = std::move(weights);
	}
	
	if(all_one) return {};
	
	return r;
}

std::vector<std::vector<bool>>  DoDeeDum::GetSubsetMarks(Mesh const& mesh, std::vector<std::vector<float>> const& weights)
{
	std::vector<std::vector<bool>>	 r;
	
	if(weights.empty()) return {};
	
	r.resize(mesh.size());
	bool set_everything = true;
	
	for(auto i = 0u; i < mesh.size(); ++i)
	{
		auto & p = mesh[i];
		
		if(weights[i].size() == 0)
			continue;
			
		std::vector<bool> marks(p.indices.no_verts);
		auto & w = weights[i];
		bool set_something = false;
		
		p.indices.for_each_tri([&](glm::uvec3 const& tri) -> bool
		{
			if(w[tri.x] || w[tri.y] || w[tri.z])
			{
				marks[tri.x] = true;
				marks[tri.y] = true;
				marks[tri.z] = true;
				set_something = true;
			}
			else
			{
				set_everything = false;
			}
			
			return false;
		});	
		
		if(set_something)
			r[i] = std::move(marks);
	}

	if(set_everything) return {};
	
	return r;
}


void DoDeeDum::ProjectedMesh::sort()
{
	std::vector<std::pair<int, glm::vec2>> memo;
	memo.resize(points.size());
	
	for(auto i = 0u; i < points.size(); ++i)
	{
		memo[i].first = i;
		memo[i].second = points[i];
	}
	
	std::sort(memo.begin(), memo.end(), [](auto & a, auto & b) 
	{ 
		return (a.second.y < b.second.y)? 
					(a.second.y < b.second.y)
				:	(a.second.x < b.second.x);
	});
	std::vector<glm::vec3> new_points(points.size());
	
	std::vector<int> vertex_mapping;
	vertex_mapping.resize(points.size());
	
	for(auto i = 0u; i < points.size(); ++i)
	{
		new_points[i] = points[memo[i].first];
		vertex_mapping[memo[i].first] = i;
	}
	
	points = std::move(new_points);
	
	for(auto & t : tris)
	{
		t.x = vertex_mapping[t.x];
		t.y = vertex_mapping[t.y];
		t.z = vertex_mapping[t.z];
		
		// sort indices
		(std::array<uint32_t, 3>&)t = sort3((std::array<uint32_t, 3>&)t);
	}
	
	std::sort(tris.begin(), tris.end(), [](auto & a, auto & b) { return a.z < b.z; });
}


bool DoDeeDum::GetIntersection(glm::vec2 & dst, glm::vec2 const& a0, glm::vec2 const& a1, glm::vec2 const& b0, glm::vec2 const& b1)
{
    glm::vec2 s1 = a1 - a0;
    glm::vec2 s2 = b1 - b0;

    float denominator = (-s2.x * s1.y + s1.x * s2.y);

    // Check if lines are parallel (denominator is zero)
    if (std::abs(denominator) < 1e-6f)
        return false;

    float s = (-s1.y * (a0.x - b0.x) + s1.x * (a0.y - b0.y)) / denominator;
    float t = ( s2.x * (a0.y - b0.y) - s2.y * (a0.x - b0.x)) / denominator;

    // Check if intersection point lies on both line segments
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Intersection detected
        dst.x = a0.x + (t * s1.x);
        dst.y = a0.y + (t * s1.y);
        return true;
    }

    return false; // No intersection
}

float DoDeeDum::ProjectedMesh::get_value_at_point(glm::vec2 const& point, uint32_t tri) const
{
	if (tri >= tris.size())
		return 0.0f;

	auto const& t = tris[tri];
	glm::vec2 v0 = {points[t.x].x, points[t.x].y};
	glm::vec2 v1 = {points[t.y].x, points[t.y].y};
	glm::vec2 v2 = {points[t.z].x, points[t.z].y};

	// Compute barycentric coordinates
	float denom = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);

	if (std::abs(denom) < 1e-10f)
		return 0.0f; // Degenerate triangle

	float a = ((v1.y - v2.y) * (point.x - v2.x) + (v2.x - v1.x) * (point.y - v2.y)) / denom;
	float b = ((v2.y - v0.y) * (point.x - v2.x) + (v0.x - v2.x) * (point.y - v2.y)) / denom;
	float c = 1.0f - a - b;

	// Check if point is inside triangle
	if (a < 0.0f || a > 1.0f || b < 0.0f || b > 1.0f || c < 0.0f || c > 1.0f)
		return 0.0f;

	// Interpolate weights (z components)
	return a * points[t.x].z + b * points[t.y].z + c * points[t.z].z;
}

using vec3 = glm::vec3;

// Clip a polygon against the plane: dot(barycentric, w) = 1.0
// Keeps the region where the weight <= 1
static int clipPolygonAt1(vec3 poly[10], int count, vec3 w) {
    vec3 outPoly[10];
    int outCount = 0;
    
    for(int i = 0; i < count; i++) {
        vec3 v0 = poly[i];
        vec3 v1 = poly[(i + 1) % count];
        
        float val0 = dot(v0, w);
        float val1 = dot(v1, w);
        
        bool inside0 = val0 <= 1.0;
        bool inside1 = val1 <= 1.0;
        
        if(inside0) {
            outPoly[outCount++] = v0;
        }
        
        if(inside0 != inside1) {
            // Edge crosses w=1 plane, compute intersection
            float t = (1.0 - val0) / (val1 - val0);
            outPoly[outCount++] = mix(v0, v1, t);
        }
    }
    
    for(int i = 0; i < outCount; i++) {
        poly[i] = outPoly[i];
    }
    
    return outCount;
}

float DoDeeDum::GetAverageWeight(vec3 w) {
    // All weights >= 1: fully saturated
    if(w.x >= 1.0 && w.y >= 1.0 && w.z >= 1.0) return 1.0;
    
    // All weights < 1: simple average
    if(w.x < 1.0 && w.y < 1.0 && w.z < 1.0) {
        return (w.x + w.y + w.z) / 3.0;
    }
    
    // Mixed case: some weights >= 1, some < 1
    // Need to clip triangle against w=1 plane
    
    // Start with triangle in barycentric coordinates
    vec3 poly[10];
    poly[0] = vec3(1.0, 0.0, 0.0);  // weight = w.x
    poly[1] = vec3(0.0, 1.0, 0.0);  // weight = w.y
    poly[2] = vec3(0.0, 0.0, 1.0);  // weight = w.z
    int count = 3;
    
    // Clip against w <= 1 plane
    int clippedCount = clipPolygonAt1(poly, count, w);
    
    if(clippedCount == 0) return 1.0;  // Everything was clipped (shouldn't happen with our check above)
    
    // Compute area-weighted average over the clipped polygon
    // Split polygon into triangles and sum contributions
    
    float totalArea = 0.0;
    float weightedSum = 0.0;
    
    for(int i = 1; i < clippedCount - 1; i++) {
        vec3 v0 = poly[0];
        vec3 v1 = poly[i];
        vec3 v2 = poly[i + 1];
        
        vec3 e1 = v1 - v0;
        vec3 e2 = v2 - v0;
        float triArea = length(cross(e1, e2)) * 0.5;
        
        // Evaluate clamped weight at each vertex
        float val0 = glm::min(glm::dot(v0, w), 1.f);
        float val1 = glm::min(glm::dot(v1, w), 1.f);
        float val2 = glm::min(glm::dot(v2, w), 1.f);
        
        // Average value over triangle with linear interpolation
        float triAvg = (val0 + val1 + val2) / 3.0;
        
        totalArea += triArea;
        weightedSum += triArea * triAvg;
    }
    
    return weightedSum / totalArea;
}

bool DoDeeDum::ProjectedMesh::serialize(std::string const& filepath) const
{
	std::ofstream file(filepath, std::ios::binary);
	if (!file.is_open())
		return false;

	// Write header
	char header[8] = "PMESH01"; // Version 01
	file.write(header, 8);

	// Write min/max bounds
	file.write(reinterpret_cast<char const*>(&min), sizeof(glm::vec2));
	file.write(reinterpret_cast<char const*>(&max), sizeof(glm::vec2));

	// Write point count and points
	uint64_t point_count = points.size();
	file.write(reinterpret_cast<char const*>(&point_count), sizeof(uint64_t));
	file.write(reinterpret_cast<char const*>(points.data()), point_count * sizeof(glm::vec3));

	// Write triangle count and triangles
	uint64_t tri_count = tris.size();
	file.write(reinterpret_cast<char const*>(&tri_count), sizeof(uint64_t));
	file.write(reinterpret_cast<char const*>(tris.data()), tri_count * sizeof(glm::uvec3));

	return file.good();
}

DoDeeDum::ProjectedMesh::ProjectedMesh(std::filesystem::path const& filepath)
{
	std::ifstream file(filepath, std::ios::binary);
	if (!file.is_open())
		return;

	// Read and verify header
	char header[8];
	file.read(header, 8);
	if (std::strncmp(header, "PMESH01", 7) != 0)
		return;

	// Read min/max bounds
	file.read(reinterpret_cast<char*>(&min), sizeof(glm::vec2));
	file.read(reinterpret_cast<char*>(&max), sizeof(glm::vec2));

	// Read point count and points
	uint64_t point_count;
	file.read(reinterpret_cast<char*>(&point_count), sizeof(uint64_t));
	points.resize(point_count);
	file.read(reinterpret_cast<char*>(points.data()), point_count * sizeof(glm::vec3));

	// Read triangle count and triangles
	uint64_t tri_count;
	file.read(reinterpret_cast<char*>(&tri_count), sizeof(uint64_t));
	tris.resize(tri_count);
	file.read(reinterpret_cast<char*>(tris.data()), tri_count * sizeof(glm::uvec3));
}
