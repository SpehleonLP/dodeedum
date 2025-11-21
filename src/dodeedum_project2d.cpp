#include "dodeedum_project2d.h"
#include "dodeedum_vertex_remapping.hpp"
#include "dodeedum_mesh.h"
#include "../include/dodeedum.h"
#include "dodeedum_quadtree.h"
#include <cstring>
#include <fstream>
#include <cfloat>


std::filesystem::path DoDeeDum::DebugOut::file(const char * tag) const 
{ 
	char buffer[64];
	if(snprintf(buffer, sizeof(buffer), "%s-%s-%d.obj", name, tag, id_no) > 0)
	{
		return std::filesystem::path(directory) / buffer;			
	}
	
	return std::filesystem::path{};
}
	
DoDeeDum::ProjectedMesh::~ProjectedMesh() = default;

DoDeeDum::ProjectedMesh::ProjectedMesh(ProjectedMesh const& it, DebugOut const& out, float cutoff)
{	
	min = glm::vec2(FLT_MAX);
	max = glm::vec2(-FLT_MAX);
	
	points.reserve(it.points.size()*1.5);
	tris.reserve(it.tris.size()*1.5);
	std::vector<int> point_index(it.points.size(), -1);
	
	auto copy_vertex = [&](uint32_t id) -> uint32_t
	{
		if(point_index[id] < 0)
		{
			point_index[id] = points.size();
			points.push_back(it.points[id]);
			
			min = glm::min(min, (glm::vec2&)points.back());
			max = glm::max(max, (glm::vec2&)points.back());
		}
		
		return point_index[id];
	};
	
	auto add_new_vertex = [&](const glm::vec4& v) -> uint32_t
	{
		uint32_t idx = points.size();
		points.push_back(v);
		
		min = glm::min(min, (glm::vec2&)points.back());
		max = glm::max(max, (glm::vec2&)points.back());
		
		return idx;
	};
	
	auto interpolate_at_cutoff = [&](const glm::vec4& below, const glm::vec4& above) -> glm::vec4
	{
		float t = (cutoff - below.z) / (above.z - below.z);
		return glm::mix(below, above, t);
	};
	
	std::vector<std::pair<glm::vec4, int>> above_vertices; // vertex and original index (-1 for new)
	std::vector<std::pair<glm::vec4, int>> below_vertices;
	
	for(auto i = 0u; i < it.tris.size(); ++i)
	{
		std::array<uint32_t, 3>& indices = (std::array<uint32_t, 3>&)it.tris[i];
		
		std::array<glm::vec4, 3> tri =
		{
			it.points[it.tris[i].x],
			it.points[it.tris[i].y],
			it.points[it.tris[i].z]
		};
	
		if(tri[0].w < cutoff && tri[1].w < cutoff && tri[2].w < cutoff)
			continue;
			
		if(tri[0].w >= cutoff && tri[1].w >= cutoff && tri[2].w >= cutoff)
		{
			tris.push_back({
				copy_vertex(it.tris[i].x),
				copy_vertex(it.tris[i].y),
				copy_vertex(it.tris[i].z)
			});
			continue;
		}
		
		// Clip triangle at cutoff plane
		above_vertices.clear(); 
		below_vertices.clear();
		
		for(int j = 0; j < 3; ++j)
		{
			if(tri[j].w >= cutoff)
				above_vertices.push_back({tri[j], (int)indices[j]});
			else
				below_vertices.push_back({tri[j], (int)indices[j]});
		}
		
		// Case 1: One vertex above, two below - creates one triangle
		if(above_vertices.size() == 1)
		{
			auto v_above  = above_vertices[0].first;
			int idx_above = above_vertices[0].second;
			
			auto v_below0 = below_vertices[0].first;
			auto v_below1 = below_vertices[1].first;
			
			auto intersect0 = interpolate_at_cutoff(v_below0, v_above);
			auto intersect1 = interpolate_at_cutoff(v_below1, v_above);
			
			uint32_t id_above = copy_vertex(idx_above);
			uint32_t id_int0 = add_new_vertex(intersect0);
			uint32_t id_int1 = add_new_vertex(intersect1);
			
			tris.push_back({id_above, id_int0, id_int1});
		}
		// Case 2: Two vertices above, one below - creates two triangles (a quad)
		else if(above_vertices.size() == 2)
		{
			auto v_above0 = above_vertices[0].first;
			auto v_above1 = above_vertices[1].first;
			int idx_above0 = above_vertices[0].second;
			int idx_above1 = above_vertices[1].second;
			
			auto v_below = below_vertices[0].first;
			
			auto intersect0 = interpolate_at_cutoff(v_below, v_above0);
			auto intersect1 = interpolate_at_cutoff(v_below, v_above1);
			
			uint32_t id_above0 = copy_vertex(idx_above0);
			uint32_t id_above1 = copy_vertex(idx_above1);
			uint32_t id_int0 = add_new_vertex(intersect0);
			uint32_t id_int1 = add_new_vertex(intersect1);
			
			// Form a quad: v_above0, intersect0, intersect1, v_above1
			// Split into two triangles
			tris.push_back({id_above0, id_int0, id_int1});
			tris.push_back({id_above0, id_int1, id_above1});
		}
	}
	
	sort();
	
	if(out.empty() == false)
		export_debug_OBJ(out.file("projection-cutoff"));
}
	
DoDeeDum::ProjectedMesh::ProjectedMesh(Mesh const& mesh, glm::mat4 const& projection, DebugOut const& out, std::span<const uint32_t> joints)
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
			points.push_back({vert.x, vert.y, vert.z, weight});	
			
			min = glm::min(min, (glm::vec2&)vert);	
			max = glm::max(max, (glm::vec2&)vert);			
		}
		
		p.indices.for_each_tri([&](glm::uvec3 const& tri) -> bool
		{
			glm::uvec3 new_tri = { vertex_mapping[tri.x], vertex_mapping[tri.y], vertex_mapping[tri.z] };
			
			if(p.flipNormals)
				std::swap(new_tri.x, new_tri.y);
			
			if(new_tri.x != ~0u && new_tri.y != ~0u && new_tri.z != ~0u)
			{
				tris.push_back(new_tri);
			}
			
			return false;
		});
	}
	
	if(min.x > max.x)
		min = max = glm::vec2(0);
		
	translated_by = center_projection();;
	sort();
	
	if(out.empty() == false)
		export_debug_OBJ(out.file("projection"));
}
std::vector<std::vector<float>>	 DoDeeDum::GetSubsetWeights(Mesh const& mesh, std::span<const uint32_t> joints)
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
	std::vector<int> vertex_mapping = sort_vertices(points);
	std::vector<int> deduplicate = deduplicate_vertices(points, glm::vec4(max - min, 0, 0));
	
	for(auto & t : tris)
	{
		t.x = deduplicate[vertex_mapping[t.x]];
		t.y = deduplicate[vertex_mapping[t.y]];
		t.z = deduplicate[vertex_mapping[t.z]];
				
		// Rotate indices so that t.z holds the largest value while preserving winding order
		if (t.x > t.y && t.x > t.z) {
			// t.x is largest, rotate right: (x,y,z) -> (z,x,y)
			uint32_t temp = t.z;
			t.z = t.x;
			t.x = t.y;
			t.y = temp;
		} else if (t.y > t.z) {
			// t.y is largest, rotate left: (x,y,z) -> (y,z,x)
			uint32_t temp = t.x;
			t.x = t.y;
			t.y = t.z;
			t.z = temp;
		}
	}
	
	std::sort(tris.begin(), tris.end(), [](auto & a, auto & b) { return a.z < b.z; });
}

bool DoDeeDum::GetIntersection(glm::vec2 & dst, glm::dvec2 const& a0, glm::dvec2 const& a1, glm::dvec2 const& b0, glm::dvec2 const& b1)
{
    glm::dvec2 s1 = glm::normalize(a1 - a0);
	glm::dvec2 s2 = glm::normalize(b1 - b0);
	double len1 = glm::length(a1 - a0);
	double len2 = glm::length(b1 - b0);
	
	double denominator = (-s2.x * s1.y + s1.x * s2.y);
	
	if (!std::isnormal(denominator))
		return false;
	
	double s = (-s1.y * (a0.x - b0.x) + s1.x * (a0.y - b0.y)) / denominator;
	double t = ( s2.x * (a0.y - b0.y) - s2.y * (a0.x - b0.x)) / denominator;
	
	// Now t and s are in world units, so check against actual lengths
	if (s >= 0 && s <= len2 && t >= 0 && t <= len1)
	{
		dst.x = a0.x + (t * s1.x);
		dst.y = a0.y + (t * s1.y);
		return true;
	}
	
	return false;
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

	// Interpolate weights (w components)
	return a * points[t.x].w + b * points[t.y].w + c * points[t.z].w;
}

using vec3 = glm::vec3;
using vec4 = glm::vec3;

// Clip a polygon against the plane: dot(barycentric, w) = 1.0
// Keeps the region where the weight <= 1
static int clipPolygonAt1(std::span<vec3> poly, vec4 w) {
    vec4 outPoly[10];
    int outCount = 0;
    
    for(uint32_t i = 0; i < poly.size(); i++) {
        vec4 v0 = poly[i];
        vec4 v1 = poly[(i + 1) % poly.size()];
        
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
    std::array<vec3, 3> poly;
    poly[0] = vec3(1.0, 0.0, 0.0);  // weight = w.x
    poly[1] = vec3(0.0, 1.0, 0.0);  // weight = w.y
    poly[2] = vec3(0.0, 0.0, 1.0);  // weight = w.z
    
    // Clip against w <= 1 plane
    int clippedCount = clipPolygonAt1(std::span{poly}, w);
    
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

void DoDeeDum::ProjectedMesh::export_debug_OBJ(std::filesystem::path const& path) const
{
	std::ofstream file(path);
	if (!file.is_open())
	{
		throw std::system_error(errno, std::generic_category(), 
		                        "Failed to open file: " + path.string());
	}
	
	// Write vertices
	for (const auto& point : points)
	{
		file << "v " << point.x << " " << point.y << " " << point.z << "\n";
	}
	
	// Write edges as lines (OBJ line elements)
	for (const auto& tri : tris)
	{
		// OBJ indices are 1-based
		file << "f " << (tri.x + 1) << " " << (tri.y + 1) << " " << (tri.z + 1) << "\n";
	}
	
	file.close();
	
	if (file.fail())
	{
		throw std::system_error(errno, std::generic_category(),
		                        "Failed to write to file: " + path.string());
	}
}

DoDeeDum::ProjectedMesh DoDeeDum::ProjectedMesh::deserialize(std::filesystem::path const& filepath)
{
	ProjectedMesh r;
	
	std::ifstream file(filepath, std::ios::binary);
	if (!file.is_open())
		return r;

	// Read and verify header
	char header[8];
	file.read(header, 8);
	if (std::strncmp(header, "PMESH01", 7) != 0)
		return r;

	// Read min/max bounds
	file.read(reinterpret_cast<char*>(&r.min), sizeof(glm::vec2));
	file.read(reinterpret_cast<char*>(&r.max), sizeof(glm::vec2));

	// Read point count and points
	uint64_t point_count;
	file.read(reinterpret_cast<char*>(&point_count), sizeof(uint64_t));
	r.points.resize(point_count);
	file.read(reinterpret_cast<char*>(r.points.data()), point_count * sizeof(glm::vec3));

	// Read triangle count and triangles
	uint64_t tri_count;
	file.read(reinterpret_cast<char*>(&tri_count), sizeof(uint64_t));
	r.tris.resize(tri_count);
	file.read(reinterpret_cast<char*>(r.tris.data()), tri_count * sizeof(glm::uvec3));
	
	return r;
}

bool DoDeeDum::ProjectedMesh::polygons_share_edge(uint32_t tri_a, uint32_t tri_b) const
{
	auto const& t_a = tris[tri_a];
	auto const& t_b = tris[tri_b];

	// Get vertices
	std::array<uint32_t, 3> verts_a = {t_a.x, t_a.y, t_a.z};
	std::array<uint32_t, 3> verts_b = {t_b.x, t_b.y, t_b.z};

	// Count shared vertices
	int shared = 0;
	for (uint32_t va : verts_a)
	{
		for (uint32_t vb : verts_b)
		{
			if (va == vb)
			{
				++shared;
				break;
			}
		}
	}

	// Two triangles share an edge if they have exactly 2 shared vertices
	return shared == 2;
}

bool DoDeeDum::ProjectedMesh::polygons_intersect(uint32_t tri_a, uint32_t tri_b) const
{
	auto const& t_a = tris[tri_a];
	auto const& t_b = tris[tri_b];

	glm::vec2 const& a0 = points[t_a.x];
	glm::vec2 const& a1 = points[t_a.y];
	glm::vec2 const& a2 = points[t_a.z];

	glm::vec2 const& b0 = points[t_b.x];
	glm::vec2 const& b1 = points[t_b.y];
	glm::vec2 const& b2 = points[t_b.z];

	// Test if any vertex of one triangle is inside the other
	if (point_in_triangle(a0, b0, b1, b2) ||
	    point_in_triangle(a1, b0, b1, b2) ||
	    point_in_triangle(a2, b0, b1, b2) ||
	    point_in_triangle(b0, a0, a1, a2) ||
	    point_in_triangle(b1, a0, a1, a2) ||
	    point_in_triangle(b2, a0, a1, a2))
	{
		return true;
	}

	// Test if any edges intersect
	std::array<std::pair<glm::vec2, glm::vec2>, 3> edges_a = {
		{{a0, a1}, {a1, a2}, {a2, a0}}
	};
	std::array<std::pair<glm::vec2, glm::vec2>, 3> edges_b = {
		{{b0, b1}, {b1, b2}, {b2, b0}}
	};

	for (auto const& edge_a : edges_a)
	{
		for (auto const& edge_b : edges_b)
		{
			if (segments_intersect(edge_a.first, edge_a.second, edge_b.first, edge_b.second))
			{
				return true;
			}
		}
	}

	return false;
}


bool DoDeeDum::ProjectedMesh::point_in_polygon(glm::vec2 const& p, uint32_t tri) const
{
	auto const& t_a = tris[tri];
	
	glm::vec2 const&  a0 = points[t_a.x];
	glm::vec2 const&  a1 = points[t_a.y];
	glm::vec2 const&  a2 = points[t_a.z];

	return point_in_triangle(p, a0, a1, a2);
}

DoDeeDum::AABB2D DoDeeDum::ProjectedMesh::get_polygon_bounds(uint32_t tri_index) const
{
	auto const& tri = tris[tri_index];

	AABB2D bounds;
	bounds.min = {
		std::min({points[tri.x].x, points[tri.y].x, points[tri.z].x}),
		std::min({points[tri.x].y, points[tri.y].y, points[tri.z].y})
	};
	bounds.max = {
		std::max({points[tri.x].x, points[tri.y].x, points[tri.z].x}),
		std::max({points[tri.x].y, points[tri.y].y, points[tri.z].y})
	};

	return bounds;
}

glm::vec2 DoDeeDum::ProjectedMesh::center_projection()
{
	glm::vec2 translation = (min + max)/2.f;
	
	for(auto i = 0u; i < points.size(); ++i)
	{
		(glm::vec2&)(points[i].x) -= translation;
	}
	
	min -= translation;
	max -= translation;
	return -translation;
}
