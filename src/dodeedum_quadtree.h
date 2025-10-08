#ifndef DODEEDUM_QUADTREE_H
#define DODEEDUM_QUADTREE_H

#include "Spehleon/dodeedum/src/dodeedum_project2d.h"
#include <glm/vec2.hpp>
#include <vector>
#include <array>
#include <span>
#include <cstdint>

namespace DoDeeDum
{

struct AABB2D
{
	glm::vec2 min;
	glm::vec2 max;

	bool contains(glm::vec2 const& point) const;
	bool intersects(AABB2D const& other) const;
	bool contains_triangle(glm::vec2 const& v0, glm::vec2 const& v1, glm::vec2 const& v2) const;
};

struct TriangleQuery
{
	uint32_t triangle_index;
	AABB2D bounds;
};

class QuadTree
{
public:
	QuadTree(ProjectedMesh const& mesh, uint32_t max_depth = 8);
	~QuadTree() = default;

	// Find all triangles that share edges with the given triangle
	void find_edge_neighbors(std::vector<uint32_t> & dst, uint32_t tri_index) const;

	// Find all triangles that overlap or intersect with the given triangle
	void find_overlapping(std::vector<uint32_t> & dst, uint32_t tri_index) const;

	// Find all triangles within a bounding box
	void query_region(std::vector<uint32_t> & dst, AABB2D const& region) const;

	// Find all triangles containing or near a point
	void query_point(std::vector<uint32_t> & dst, glm::vec2 const& point, float radius = 0.0f) const;

	// Get AABB for a specific triangle
	AABB2D get_triangle_bounds(uint32_t tri_index) const;
	
	template<typename F>
	void query_region(AABB2D const& region, F const& func) const;
	
	template<typename F>
	void query_point(glm::vec2 const& point, F const& func) const;
	
	bool IsContained(glm::vec2 const& point) const;
	
private:
	static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF;
	static constexpr uint32_t MAX_TRIS_PER_NODE = 4;

	struct Node
	{
		AABB2D bounds;
		// For leaf nodes (is_leaf=true): stores up to 4 triangle indices
		// For internal nodes (is_leaf=false): stores 4 child node indices (NW, NE, SW, SE)
		std::array<uint32_t, 4> data;
		bool is_leaf;

		Node() : data{INVALID_INDEX, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX}, is_leaf(true) {}
	};

	ProjectedMesh const& mesh_;
	std::vector<Node> nodes_;
	uint32_t root_idx_;
	uint32_t max_depth_;

	uint32_t allocate_node();
	void build_tree(uint32_t node_idx, std::vector<uint32_t> const& tri_indices, uint32_t depth);
	void subdivide(uint32_t node_idx);
	void query_region_recursive(uint32_t node_idx, AABB2D const& region, std::vector<uint32_t>& results) const;

	// Helper functions for geometric tests
	bool triangles_share_edge(uint32_t tri_a, uint32_t tri_b) const;
	bool triangles_intersect(uint32_t tri_a, uint32_t tri_b) const;
	bool point_in_triangle(glm::vec2 const& p, glm::vec2 const& v0, glm::vec2 const& v1, glm::vec2 const& v2) const;
	bool point_in_triangle(glm::vec2 const& p, uint32_t tri) const;
	bool segments_intersect(glm::vec2 const& a0, glm::vec2 const& a1, glm::vec2 const& b0, glm::vec2 const& b1) const;

	AABB2D compute_bounds(std::span<uint32_t const> tri_indices) const;
	AABB2D compute_triangle_bounds(uint32_t tri_index) const;
};

template<typename F>
inline void QuadTree::query_region(AABB2D const& region, F const& func) const
{
	std::vector<int> stack;
	stack.reserve(max_depth_*4);
	
	stack.push_back(root_idx_);
	
	while(stack.size())
	{
		uint32_t node_idx = stack.back();
		stack.pop_back();
	
		Node const& node = nodes_[node_idx];
	
		if (!node.bounds.intersects(region))
			continue;
		
		if(node.is_leaf == false)
		{
			// Recurse into children
			for (uint32_t i = 0; i < 4; ++i)
			{
				if(node.data[i] != INVALID_INDEX)
					stack.push_back(node.data[i]);
			}
		}
		else
		{
			// Check each triangle in this leaf
			for (uint32_t i = 0; i < MAX_TRIS_PER_NODE; ++i)
			{
				uint32_t tri_idx = node.data[i];
				if (tri_idx == INVALID_INDEX)
					break; // No more triangles in this leaf
	
				AABB2D tri_bounds = compute_triangle_bounds(tri_idx);
				if (tri_bounds.intersects(region))
				{
					if(func(tri_idx) == true)
						return;
				}
			}	
		}
	}
}

template<typename F>
inline void QuadTree::query_point(glm::vec2 const& point, F const& func) const
{
	std::vector<int> stack;
	stack.reserve(max_depth_*4);
	
	stack.push_back(root_idx_);
	
	while(stack.size())
	{
		uint32_t node_idx = stack.back();
		stack.pop_back();
	
		Node const& node = nodes_[node_idx];
	
		if (!node.bounds.contains(point))
			continue;
		
		if(node.is_leaf == false)
		{
			// Recurse into children
			for (uint32_t i = 0; i < 4; ++i)
			{
				if(node.data[i] != INVALID_INDEX)
					stack.push_back(node.data[i]);
			}
		}
		else
		{
			// Check each triangle in this leaf
			for (uint32_t i = 0; i < MAX_TRIS_PER_NODE; ++i)
			{
				uint32_t tri_idx = node.data[i];
				if (tri_idx == INVALID_INDEX)
					break; // No more triangles in this leaf
	
				if (point_in_triangle(point, tri_idx))
				{
					if(func(tri_idx) == true)
						return;
				}
			}	
		}
	}
}

inline bool QuadTree::IsContained(glm::vec2 const& point) const
{
	bool found = false;
	query_point(point, [&](uint32_t)
	{
		found = true;
		return true;
	});
	
	return found;
}

}

#endif // DODEEDUM_QUADTREE_H
