#ifndef DODEEDUM_MESHEDGES_H
#define DODEEDUM_MESHEDGES_H
#include <filesystem>
#include <glm/vec2.hpp>
#include <unordered_map>
#include <vector>

namespace DoDeeDum
{

struct AABB2D;
struct ProjectedMesh;

class MeshEdges
{
public:
	static MeshEdges Perimiter(ProjectedMesh const& mesh) { return MeshEdges(mesh, MeshEdges(mesh)); }
	
	~MeshEdges() = default;

	struct Edge
	{
		uint32_t first;
		uint32_t second;
		uint8_t  flags;
		
		Edge flip_around() const 
		{
			auto flags = this->flags;
			flags = (1 << (flags & 0x5)) | ((flags >> 1) & 0x05);
			return {second, first, flags};
		}
		
		inline bool operator==(const Edge& b) const = default;
		inline bool operator<(const Edge& b) const
		{
			return second != b.second?
				  second < b.second
				: first < b.first;
		}
	};	
	
	struct Perimiter
	{
		std::vector<uint32_t> indices;
		std::vector<std::pair<uint32_t, uint32_t>> loops;
	};
	
	struct Perimiter GetPerimiter();	
	
	AABB2D get_polygon_bounds(uint32_t a) const;
	bool   polygons_intersect(uint32_t a, uint32_t b) const;
	bool   polygons_share_edge(uint32_t, uint32_t) const { return false; }
	bool   point_in_polygon(glm::vec2 const&, uint32_t) const { return false; }
	size_t get_polygon_count() const { return edges.size(); }	
	
	void export_debug_OBJ(std::filesystem::path const&) const;
	
	std::vector<glm::vec2> points;
	std::vector<Edge>		edges;
	
// get just the edges that are 
	MeshEdges(ProjectedMesh const& mesh);
// trim overlaps, get just perimiters
	MeshEdges(ProjectedMesh const& mesh, MeshEdges const& edges);
	

	std::vector<std::vector<uint32_t>> to_edge_list() const;
	std::unordered_map<uint64_t, std::pair<uint32_t, float>> to_turn_list(std::vector<std::vector<uint32_t>> const& edge_list) const;
	
	void sort();	
	void trim_colinear(std::vector<std::vector<std::pair<uint32_t, uint8_t>>> const& edge_list);	
};

}

#endif // DODEEDUM_MESHEDGES_H
