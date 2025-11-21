#include "dodeedum_meshedges.h"
#include "dodeedum_project2d.h"
#include "dodeedum_quadtree.h"
#include "dodeedum_vertex_remapping.hpp"
#include <algorithm>
#include <cfloat>
#include <glm/ext/scalar_constants.hpp>
#include "../include/dodeedum.h"
#include <unordered_map>


static bool segment_segment_intersect(glm::dvec2 p1, glm::dvec2 p2, 
							glm::dvec2 p3, glm::dvec2 p4,
							float& t1, float& t2);
							
template<typename T, glm::qualifier q>
static bool are_verts_same(glm::vec<2, T, q> const& a, glm::vec<2, T, q> const& b) 
{	
	static constexpr auto EPS = std::numeric_limits<T>::epsilon();
	return (std::fabs(a.x - b.x) < EPS && std::fabs(a.y - b.y) < EPS);
};
				
struct MeshEdgeHelper
{
	MeshEdgeHelper()
	{
		intersections.reserve(8);
	}
	
    void clip_segment_outside_triangle(glm::vec2 p1, glm::vec2 p2, 
                               const std::array<glm::vec2, 3>& tri);
                                
	void AddVertex(glm::vec2 const& a, glm::vec2 const& b)
	{
		if(!are_verts_same(a, b))
			result.push_back({a, b});
	};
	
    std::vector<std::pair<glm::vec2, glm::vec2>> result;      
	std::vector<std::pair<float, glm::vec2>> intersections; // t parameter and point               

};

DoDeeDum::MeshEdges::MeshEdges(ProjectedMesh const& mesh, DebugOut const& out) :
	min(mesh.min),
	max(mesh.max)
{
	std::unordered_map<uint64_t, int> memo;

	for(auto i = 0u; i < mesh.tris.size(); ++i)
	{
		std::array<uint32_t, 3>& indices = (std::array<uint32_t, 3>&)mesh.tris[i];		
		std::array<glm::vec3, 3> tri =
		{
			mesh.points[mesh.tris[i].x],
			mesh.points[mesh.tris[i].y],
			mesh.points[mesh.tris[i].z]
		};
		
		// is this the same as gl winding?
		glm::vec2 normal = glm::cross(tri[1] - tri[0], tri[2] - tri[0]);
		glm::vec2 centroid = (tri[0] + tri[1] + tri[2]) / 3.f;
		
		for(auto j = 0u; j < 3; ++j)
		{
			std::pair<uint32_t, uint32_t> edge{
				indices[j], indices[(j+1)%3]
			};
			
			if(edge.first > edge.second)
			{
				std::swap(edge.first, edge.second);
			}
			
			glm::vec2 edge_vector = glm::normalize((glm::vec2&)mesh.points[edge.second] - (glm::vec2&)mesh.points[edge.first]);
			glm::vec2 ortho = glm::vec2(-edge_vector.y, edge_vector.x);
			if(std::isnan(edge_vector.x)) continue;
			
			int flags = 
				  (1 << int(glm::dot(ortho, centroid - (glm::vec2&)mesh.points[edge.first]) > 0))
				| (4 << int(glm::dot(ortho, normal) < 0));
				
			memo[(uint64_t&)edge] |= flags;		
		}
	}
	
	std::vector<int> point_index(mesh.points.size(), -1);
	auto copy_vertex = [&](uint32_t id) -> uint32_t
	{
		assert(id < point_index.size());
		
		if(point_index[id] < 0)
		{
			point_index[id] = points.size();
			points.push_back(mesh.points[id]);
		}
		
		return point_index[id];
	};
	
	points.reserve(std::max<int>(32, std::sqrt(mesh.points.size())));
	for(auto item : memo)
	{
		std::pair<uint32_t, uint32_t> e = (std::pair<uint32_t, uint32_t>&)item.first;		
		
		// skip internal edges
		if((item.second & 0x03) != 0x03) 
			edges.push_back(Edge{copy_vertex(e.first), copy_vertex(e.second), uint8_t(item.second)});		
	}
	
	if(out.empty() == false)
		export_debug_OBJ(out.file("pruned"));
}

namespace DoDeeDum
{

void check(ProjectedMesh const& mesh, MeshEdges const& edges)
{
	QuadTree tree(mesh);
	tree.build();

	std::vector<uint32_t> indices;
	std::vector<uint32_t> indices2;
	for(auto i = 0u; i < edges.edges.size(); ++i)
	{	
		auto bounds = edges.get_polygon_bounds(i);
		indices.clear();
		indices2.clear();
		tree.query_region(edges.get_polygon_bounds(i), [&](uint32_t tri_index)
		{
			indices.push_back(tri_index);
			return false;
		});
		
		for(auto j = 0u; j < tree.get_polygon_count(); ++j)
		{
			if(bounds.intersects(tree.get_polygon_bounds(j)))
			{
				indices2.push_back(j);
			}
		}
		
		std::sort(indices.begin(), indices.end());
		std::sort(indices2.begin(), indices2.end());
		
		assert(indices == indices2);
	}

};

}

DoDeeDum::MeshEdges::MeshEdges(ProjectedMesh const& mesh, MeshEdges const& edges, DebugOut const& out) :
	min(mesh.min),
	max(mesh.max)
{	
	QuadTree tree(mesh);
	tree.build();
	
	MeshEdgeHelper helper;
	// reuse buffer to avoid malloc
	std::vector<std::pair<glm::vec2, glm::vec2>> edge_segments;
			
	for(auto i = 0u; i < edges.edges.size(); ++i)
	{
		glm::vec2 start = (glm::vec2&)edges.points[edges.edges[i].first ];
		glm::vec2 end   = (glm::vec2&)edges.points[edges.edges[i].second];
		
		auto CountInCommon = [&](std::array<glm::vec2, 3> & tri_verts)
		{
			bool count_end = 0;
			bool count_start = 0;
			
			for(int j = 0; j < 3; ++j)
			{
				auto distance_start = std::max(
					std::fabs(tri_verts[j].x - start.x), 
					std::fabs(tri_verts[j].y - start.y));
					
				auto distance_end = std::max(
					std::fabs(tri_verts[j].x - end.x), 
					std::fabs(tri_verts[j].y - end.y));
					
				count_start |= (distance_start < DoDeeDum::EPS);
				count_end |= (distance_end < DoDeeDum::EPS);
			}
		
			return int(count_start)+int(count_end);
		};
		
		edge_segments.clear();
		edge_segments.push_back({start, end});
		
		// it looks like the reason this fails is that de-duplication failed
		tree.query_region(edges.get_polygon_bounds(i), [&](uint32_t tri_index)
		{
			const auto& tri = mesh.tris[tri_index];
			std::array<glm::vec2, 3> tri_verts = {
				(glm::vec2&)mesh.points[tri.x],
				(glm::vec2&)mesh.points[tri.y],
				(glm::vec2&)mesh.points[tri.z]
			};
			
			if(CountInCommon(tri_verts) == 2)
				return false;
				
			
			helper.result.clear();
			
			for(const auto& seg : edge_segments)
			{
				// Clip segment against triangle, keeping parts OUTSIDE
				helper.clip_segment_outside_triangle(seg.first, seg.second, tri_verts);
			}
			
			edge_segments.swap(helper.result);
			return false;
		});
		
		uint8_t flags = edges.edges[i].flags;
		
		for(const auto& seg : edge_segments)
		{
			// Add segment to final edge list
			uint32_t idx_start = points.size();
			points.push_back(seg.first);
			points.push_back(seg.second);
			
			this->edges.push_back(Edge{idx_start, idx_start+1, flags});
		}
	}	
	
	sort();
	
	if(out.empty() == false)
		export_debug_OBJ(out.file("edges"));
}
	
DoDeeDum::AABB2D DoDeeDum::MeshEdges::get_polygon_bounds(uint32_t a) const
{
	auto & v0 = points[edges[a].first];
	auto & v1 = points[edges[a].second];
	
	return {glm::min(v0, v1), glm::max(v0, v1)};
}

bool   DoDeeDum::MeshEdges::polygons_intersect(uint32_t a, uint32_t b) const
{
	auto & a0 = points[edges[a].first];
	auto & a1 = points[edges[a].second];
	auto & b0 = points[edges[b].first];
	auto & b1 = points[edges[b].second];
	
	return segments_intersect(a0, a1, b0, b1);
}

template<typename T, typename F1, typename F2>
void erase(std::vector<T> & vec, F1 & equals, F2 & should_keep)
{
	uint32_t read, write, next;
	for(read = 0, write = 0; read < vec.size(); read=next)
	{
		auto & t = vec[read];
		auto next = read+1;
		for(; next < vec.size(); ++next)
		{
			if(!equals(t, vec[next]))
				break;
		}
		
		if(should_keep)
			vec[write++] = vec[read];
	}
	
	vec.resize(write);
}

void DoDeeDum::MeshEdges::sort()
{
	std::vector<int> vertex_mapping = sort_vertices(points);
	std::vector<int> deduplicate = deduplicate_vertices(points, max - min);
	
	for(auto & t : edges)
	{
		t.first = deduplicate[vertex_mapping[t.first]];
		t.second = deduplicate[vertex_mapping[t.second]];
		
		if(t.first > t.second)
		{
			t = t.flip_around();
		}
	}
	
	std::sort(edges.begin(), edges.end());
	
	uint32_t read, write, next;
	for(read = 0, write = 0; read < edges.size(); read=next)
	{
		auto & t = edges[read];
		for(next = read+1; next < edges.size(); ++next)
		{
			if(t.first != edges[next].first
			|| t.second != edges[next].second)
				break;
		}
		
		if(t.first != t.second)
			edges[write++] = edges[read];
	}
	
	edges.resize(write);
}



// Helper: Clip a line segment, keeping parts OUTSIDE the triangle
void MeshEdgeHelper::clip_segment_outside_triangle(glm::vec2 p1, glm::vec2 p2, 
                               const std::array<glm::vec2, 3>& tri)
{	
//	float p1_distance = DoDeeDum::distance_from_triangle(p1, tri[0], tri[1], tri[2]);
//	float p2_distance = DoDeeDum::distance_from_triangle(p2, tri[0], tri[1], tri[2]);
	
	// Check if endpoints are inside triangle
	bool p1_inside = DoDeeDum::point_in_triangle(p1, tri[0], tri[1], tri[2]);
	bool p2_inside = DoDeeDum::point_in_triangle(p2, tri[0], tri[1], tri[2]);
	
	if(p1_inside && p2_inside)
		return;
		
	intersections.clear();
	
	if(p1_inside == false)
		intersections.push_back({0.f, p1});
		
	auto begin = intersections.size();
	
	for(int i = 0; i < 3; ++i)
	{
		glm::vec2 edge_start = tri[i];
		glm::vec2 edge_end = tri[(i+1)%3];
		
		float t1, t2;
		if(segment_segment_intersect(p1, p2, edge_start, edge_end, t1, t2))
		{
			if(t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f)
			{
				intersections.push_back({t1, glm::mix(p1, p2, t1)});
			}
		}
	}

	std::sort(intersections.begin()+begin, intersections.end(), 
			  [](auto& a, auto& b) { return a.first < b.first; });
			
	if(p2_inside == false) 
		intersections.push_back({1.f, p2});  
	
	// Keep alternating segments (outside, inside, outside, ...)
	// Start is outside, so keep [p1, intersections[0]]
	for(uint32_t i = 1; i < intersections.size(); ++i)
	{  
		glm::vec2 midpoint = (intersections[i-1].second + intersections[i].second) * 0.5f;
		bool mid_inside = DoDeeDum::point_in_triangle(midpoint, tri[0], tri[1], tri[2]);
    
		if(!mid_inside)
			AddVertex(intersections[i-1].second, intersections[i].second);
	}
}

static bool segment_segment_intersect(glm::dvec2 p1, glm::dvec2 p2, 
                                glm::dvec2 p3, glm::dvec2 p4,
                                float& t1, float& t2)
{
	auto d1 = p2 - p1;
	auto d2 = p4 - p3;
	
	auto l1 = glm::length(d1);
	auto l2 = glm::length(d2);
	
	auto n1 = d1/l1;
	auto n2 = d2/l2;
	
	float cross = n1.x * n2.y - n1.y * n2.x;
	
	if(std::isnormal(cross) == false) return false; // Parallel
	
	auto d3 = p3 - p1;
	
	// Compute t values using normalized directions
	double t1_norm = (d3.x * n2.y - d3.y * n2.x) / cross;
	double t2_norm = (d3.x * n1.y - d3.y * n1.x) / cross;
	
	// Scale back to original parameterization [0, 1]
	t1 = t1_norm / l1;
	t2 = t2_norm / l2;
	
	return true;
}

std::vector<std::vector<uint32_t>> DoDeeDum::MeshEdges::to_edge_list() const
{
	std::vector<std::vector<uint32_t>> r(points.size());

	for(auto const& e : edges)
	{
		r[e.first].push_back(e.second);
		r[e.second].push_back(e.first);
	}
	
// trim wires	
	auto RemoveRedundantEdges = [&r](int i )
	{
		bool removed_something = false;
		
		for(auto j = 0u; j < r[i].size(); ++j)
		{
			for(auto k = j+1; k < r[i].size(); ++k)
			{
				if(r[i][k] == r[i][j])
				{
					r[i].erase(r[i].begin()+k);
					removed_something = true;
					--k;
				}
			}	
		}
		
		return removed_something;
	};

	for(auto i = 0u; i < r.size(); ++i)
	{
		if(RemoveRedundantEdges(i) == false)
			continue;
		
		auto op = i;
		while(r[op].size() == 1)
		{				
			auto other = r[op].back();
			r[op].pop_back();
			
			for(auto j = 0u; j < r[other].size(); ++j)
			{
				if(r[other][j] == op)
				{
					r[other].erase(r[other].begin()+j);
					--j;
				}
			}
			
			if(other < i && r[other].size() == 1)
				op = other;
			else
				break;
		}
	}	//*/
	
	return r;
}

std::unordered_map<uint64_t, std::pair<uint32_t, float>> DoDeeDum::MeshEdges::to_turn_list(std::vector<std::vector<uint32_t>> const& edge_list) const
{
	std::unordered_map<uint64_t, std::pair<uint32_t, float>> turn_list;

	auto AddEdges = [&](Edge e)
	{
		uint64_t key = (uint64_t(e.first) << 32) | e.second;
		
		auto itr = turn_list.find(key);
		if(itr != turn_list.end()) 
			return;
			
		auto & list = edge_list[e.second];
		// viewing alpha card on edge, so just turn around! 
		if(list.size() <= 1)
			return;
				
		auto start = points[e.first];
		auto pivot = points[e.second];
		auto normal = glm::normalize(pivot - start);
		auto ortho = glm::vec2(normal.y, normal.x);
		
		// first is vertex, second is flags of pivot -> vertex
		uint32_t min_edge{};
		uint32_t max_edge{};
		double min_angle =  FLT_MAX;
		double max_angle = -FLT_MAX;
		
		for(auto const& edge : list)
		{
			if(edge == e.first)
				continue;
				
			auto vec = points[edge] - pivot;
			
			auto y = glm::dot(ortho, vec);
			auto x = glm::dot(normal, vec);
			
			auto angle = std::atan2(y, x);
			
			if(angle < min_angle)
			{
				min_angle = angle;
				min_edge = edge;
			}
			
			if(angle > max_angle)
			{
				max_angle = angle;
				max_edge = edge;
			}
		}
		
		assert(min_angle != FLT_MAX);
		
		// start → pivot → min_found (turn right)
		turn_list[key] = {min_edge, min_angle};
	
		// min_found → pivot → start (the reverse, turn left from opposite direction)
		uint64_t reverse_key = (uint64_t(max_edge) << 32) | e.second;
		turn_list[reverse_key] = {e.first, -max_angle};  // Angle is negated
	};

	for(auto e : edges)
	{
		AddEdges(e);
	}
	
	for(auto e : edges)
	{
		AddEdges(e.flip_around());
	}
	
	return turn_list;
}


struct DoDeeDum::MeshEdges::Perimiter DoDeeDum::MeshEdges::GetPerimiter()
{
    struct Perimiter result;
    
    // Build the data structures we need
    auto edge_list = to_edge_list();
    auto turn_list = to_turn_list(edge_list);
    
    // Track which directed edges we've visited
    std::vector<bool> visited(points.size(), false);
    
    // Try to start a loop from each edge (in both directions)
    for(const auto& edge : edges)
    {
        for(int dir = 0; dir < 2; ++dir)
        {
            Edge e = (dir == 0) ? edge : edge.flip_around();
            
			bool inside_on_left = (e.flags & 0x05) != 0;  // Bit 2: inside is on left/positive side
		
            if(inside_on_left)    continue;
            if(visited[e.second]) continue;
                        
            // Start a new loop
            uint32_t loop_start = result.indices.size();
            
            uint32_t current = e.first;
            uint32_t next = e.second;
            uint32_t start_vertex = current;
            uint32_t second_vertex = next;
            
			// Walk around the perimeter
			bool broke = false;
            do {
                uint64_t key = (uint64_t(current) << 32) | next;
                
				if(visited[next])
				{
					broke = true;
					break;
				}
                
                // Mark this directed edge as visited
                visited[next] = true;
                
                // Add the current vertex to the path
                result.indices.push_back(current);
                               
                // Look up where to turn next
                auto it = turn_list.find(key);
                if(it == turn_list.end())
                {
                    // Shouldn't happen for closed loops, but handle gracefully
                    broke = true;
                    break;
                }
                
                // Follow the turn: current → next → found
                uint32_t found = it->second.first;
                current = next;
                next = found;
                
            } while(current != start_vertex || next != second_vertex);
            
            if(!broke)
            {
				// Record this loop
				uint32_t loop_count = result.indices.size() - loop_start;
				if(loop_count > 0)
				{
					result.loops.push_back({loop_start, loop_count});
				}
            }
        }
    }
    
    return result;
}

#include <fstream>

void DoDeeDum::MeshEdges::export_debug_OBJ(std::filesystem::path const& path) const
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
		file << "v " << point.x << " " << point.y << " " << 0 << "\n";
	}
	
	// Write edges as lines (OBJ line elements)
	for (const auto& edge : edges)
	{
		// OBJ indices are 1-based
		file << "l " << (edge.first + 1) << " " << (edge.second + 1) << "\n";
	}
	
	file.close();
	
	if (file.fail())
	{
		throw std::system_error(errno, std::generic_category(),
		                        "Failed to write to file: " + path.string());
	}
}
