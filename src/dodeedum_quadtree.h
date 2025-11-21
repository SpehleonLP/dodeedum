#ifndef DODEEDUM_QUADTREE_H
#define DODEEDUM_QUADTREE_H

#include "dodeedum_project2d.h"
#include <glm/vec2.hpp>
#include <vector>
#include <array>
#include <span>
#include <cstdint>

namespace DoDeeDum
{

float distance_from_triangle(glm::dvec2 const& p, glm::dvec2 const& v0, glm::dvec2 const& v1, glm::dvec2 const& v2);

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

struct QuadTreePolicy
{
private:
	struct iQuadTreePolicy
	{
		virtual ~iQuadTreePolicy() = default;
		
		virtual bool   polygons_share_edge(uint32_t a, uint32_t b) const = 0;
		virtual bool   polygons_intersect(uint32_t, uint32_t) const = 0;
		virtual AABB2D get_polygon_bounds(uint32_t it) const = 0;
		virtual bool point_in_polygon(glm::vec2 const& p, uint32_t a) const = 0;
		virtual size_t get_polygon_count() const = 0;
	};
	
	template<typename T>
	struct QuadTreePolicyNode : public iQuadTreePolicy
	{
		QuadTreePolicyNode(T * ptr) : it(ptr) {};
		
		bool   polygons_share_edge(uint32_t a, uint32_t b) const override { return it->polygons_share_edge(a, b); }	
		bool   polygons_intersect(uint32_t a, uint32_t b) const override { return it->polygons_intersect(a, b); }	
		AABB2D get_polygon_bounds(uint32_t a) const override { return it->get_polygon_bounds(a); }	
		bool   point_in_polygon(glm::vec2 const& p, uint32_t a) const override { return it->point_in_polygon(p, a); }	
		size_t get_polygon_count() const override { return it->get_polygon_count(); }	
		
		T * it;
	};

public:
	template<typename T>
	QuadTreePolicy(T * it)
	{
		static_assert(sizeof(QuadTreePolicyNode<T>) == sizeof(_policy));
		new(&_policy) QuadTreePolicyNode<T>(it);
	}
	template<typename T>
	QuadTreePolicy(T & it)
	{
		static_assert(sizeof(QuadTreePolicyNode<T>) == sizeof(_policy));
		new(&_policy) QuadTreePolicyNode<T>(&it);
	}

	inline bool   polygons_share_edge(uint32_t a, uint32_t b) const { return ((iQuadTreePolicy*)&_policy)->polygons_share_edge(a, b);  }
	inline bool   polygons_intersect(uint32_t a, uint32_t b) const { return ((iQuadTreePolicy*)&_policy)->polygons_intersect(a, b);  }
	inline AABB2D get_polygon_bounds(uint32_t a) const { return ((iQuadTreePolicy*)&_policy)->get_polygon_bounds(a);  }
	inline bool   point_in_polygon(glm::vec2 const& p, uint32_t a) const { return ((iQuadTreePolicy*)&_policy)->point_in_polygon(p, a);  }
	inline size_t get_polygon_count() const { return ((iQuadTreePolicy*)&_policy)->get_polygon_count();  }
	
private:
	std::pair<void*, void*> _policy;
};

class QuadTree : public QuadTreePolicy
{
public:	
	template<typename T>
	QuadTree(T const& it) : QuadTreePolicy(it) {}
	
	void build();

	template<typename F> void query_region(AABB2D const& region, F const& func) const;
	template<typename F> void query_point(glm::vec2 const& point, F const& func) const;
	
	void find_edge_neighbors(std::vector<uint32_t> & dst, uint32_t tri_index) const;
	void find_overlapping(std::vector<uint32_t> & dst, uint32_t tri_index) const;
	void query_region(std::vector<uint32_t> & dst, AABB2D const& region) const;
	void query_point(std::vector<uint32_t> & dst, glm::vec2 const& point, float radius = 0.0f) const;
	bool IsContained(glm::vec2 const& point) const;
	
protected:
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

	std::vector<Node> nodes_;
	uint32_t root_idx_{INVALID_INDEX};
	uint32_t max_depth_{};
	uint32_t total_items_{};

	uint32_t allocate_node();
	AABB2D compute_bounds(std::span<uint32_t const> tri_indices) const;
	void build_tree(uint32_t node_idx, const std::vector<glm::vec2> &x_pos_memo, std::vector<std::pair<glm::vec2, uint32_t> > &x_pos_scratch, std::span<const uint32_t> tri_indices, uint32_t depth);
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
	
				AABB2D tri_bounds = get_polygon_bounds(tri_idx);
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
	
				if (point_in_polygon(point, tri_idx))
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

inline void QuadTree::query_region(std::vector<uint32_t> & dst, AABB2D const& region) const
{
	query_region(region, [&](uint32_t it)
	{
		dst.push_back(it);
		return false;
	});
}

	// Find all triangles containing or near a point
inline void QuadTree::query_point(std::vector<uint32_t> & dst, glm::vec2 const& point, float radius) const
{
	AABB2D region;
	region.min = {point.x - radius, point.y - radius};
	region.max = {point.x + radius, point.y + radius};
	return query_region(dst, region);
}

inline void QuadTree::find_edge_neighbors(std::vector<uint32_t> & neighbors, uint32_t tri_index) const
{
	if (tri_index >= total_items_)
		return;

	AABB2D tri_bounds = get_polygon_bounds(tri_index);

	// Query a slightly expanded region to catch edge-adjacent triangles
	float epsilon = 0.0001f;
	tri_bounds.min.x -= epsilon;
	tri_bounds.min.y -= epsilon;
	tri_bounds.max.x += epsilon;
	tri_bounds.max.y += epsilon;

	query_region(tri_bounds, [&](uint32_t candidate)
	{
		if (candidate != tri_index && polygons_share_edge(tri_index, candidate))
		{
			neighbors.push_back(candidate);
		}
		
		return false;
	});
}

inline void QuadTree::find_overlapping(std::vector<uint32_t> & overlapping, uint32_t tri_index) const
{
	if (tri_index >= total_items_)
		return;

	AABB2D tri_bounds = get_polygon_bounds(tri_index);

	query_region(tri_bounds, [&](uint32_t candidate)
	{
		if (candidate != tri_index && polygons_intersect(tri_index, candidate))
		{
			overlapping.push_back(candidate);
		}
		
		return false;
	});
}



}

#endif // DODEEDUM_QUADTREE_H
