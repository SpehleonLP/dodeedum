#include "dodeedum_quadtree.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace DoDeeDum
{

// AABB2D implementation
bool AABB2D::contains(glm::vec2 const& point) const
{
	return point.x >= min.x && point.x <= max.x &&
	       point.y >= min.y && point.y <= max.y;
}

bool AABB2D::intersects(AABB2D const& other) const
{
	return !(max.x < other.min.x || min.x > other.max.x ||
	         max.y < other.min.y || min.y > other.max.y);
}

bool AABB2D::contains_triangle(glm::vec2 const& v0, glm::vec2 const& v1, glm::vec2 const& v2) const
{
	// Check if triangle AABB intersects this AABB
	float tri_min_x = std::min({v0.x, v1.x, v2.x});
	float tri_max_x = std::max({v0.x, v1.x, v2.x});
	float tri_min_y = std::min({v0.y, v1.y, v2.y});
	float tri_max_y = std::max({v0.y, v1.y, v2.y});

	return !(tri_max_x < min.x || tri_min_x > max.x ||
	         tri_max_y < min.y || tri_min_y > max.y);
}

// QuadTree implementation
QuadTree::QuadTree(ProjectedMesh const& mesh, uint32_t max_depth)
	: mesh_(mesh)
	, root_idx_(INVALID_INDEX)
	, max_depth_(max_depth)
{
	if (mesh_.tris.empty())
		return;

	// Estimate node count: worst case is (4^(depth+1) - 1) / 3 for a full quadtree
	// More conservative: 4^(depth+1) to allow for some overhead
	size_t estimated_nodes = 1;
	for (uint32_t i = 0; i <= max_depth_; ++i)
	{
		estimated_nodes *= 4;
	}
	estimated_nodes = std::min(estimated_nodes, mesh_.tris.size() * 2); // Cap by triangle count
	nodes_.reserve(estimated_nodes);

	// Build triangle index list
	// Since tris are sorted by least maximum, we can pass them in order
	std::vector<uint32_t> all_indices(mesh_.tris.size());
	for (uint32_t i = 0; i < mesh_.tris.size(); ++i)
		all_indices[i] = i;

	// Compute root bounds
	AABB2D root_bounds = compute_bounds(all_indices);

	root_idx_ = allocate_node();
	nodes_[root_idx_].bounds = root_bounds;

	build_tree(root_idx_, all_indices, 0);
}

uint32_t QuadTree::allocate_node()
{
	uint32_t idx = static_cast<uint32_t>(nodes_.size());
	nodes_.emplace_back();
	return idx;
}

void QuadTree::build_tree(uint32_t node_idx, std::vector<uint32_t> const& tri_indices, uint32_t depth)
{
	// Stop subdividing if we've reached max depth or have few enough triangles
	if (depth >= max_depth_ || tri_indices.size() <= MAX_TRIS_PER_NODE)
	{
		nodes_[node_idx].is_leaf = true;
		// Store up to 4 triangle indices
		for (size_t i = 0; i < std::min(tri_indices.size(), size_t(MAX_TRIS_PER_NODE)); ++i)
		{
			nodes_[node_idx].data[i] = tri_indices[i];
		}
		return;
	}

	// Create children
	subdivide(node_idx);

	// Distribute triangles to children
	std::array<std::vector<uint32_t>, 4> child_tris;

	// Get child indices (must do this AFTER subdivide, and avoid keeping node reference)
	std::array<uint32_t, 4> child_indices;
	for (int i = 0; i < 4; ++i)
	{
		child_indices[i] = nodes_[node_idx].data[i];
	}

	// Exploit sorted order: triangles are sorted by least maximum vertex index
	// This means spatially coherent triangles tend to be grouped together
	for (uint32_t tri_idx : tri_indices)
	{
		AABB2D tri_bounds = compute_triangle_bounds(tri_idx);

		// Check which quadrants this triangle intersects
		for (int i = 0; i < 4; ++i)
		{
			uint32_t child_idx = child_indices[i];
			if (child_idx != INVALID_INDEX && nodes_[child_idx].bounds.intersects(tri_bounds))
			{
				child_tris[i].push_back(tri_idx);
			}
		}
	}

	// Check if subdivision made progress: if all/most children have the same triangles,
	// subdivision isn't helping (degenerate case with overlapping/identical triangles)
	size_t max_child_size = 0;
	for (int i = 0; i < 4; ++i)
	{
		max_child_size = std::max(max_child_size, child_tris[i].size());
	}

	// If the largest child has nearly all the triangles, subdivision didn't help
	// Just make this a leaf and distribute triangles randomly among the 4 slots
	if (max_child_size >= tri_indices.size() * 0.9)
	{
		// Degenerate case: triangles are overlapping/identical, can't subdivide effectively
		nodes_[node_idx].data.fill(INVALID_INDEX);
		nodes_[node_idx].is_leaf = true;

		// Store up to 4 triangles randomly
		for (size_t i = 0; i < std::min(tri_indices.size(), size_t(MAX_TRIS_PER_NODE)); ++i)
		{
			nodes_[node_idx].data[i] = tri_indices[i];
		}
		return;
	}

	// Recursively build children
	bool all_empty = true;
	for (int i = 0; i < 4; ++i)
	{
		if (!child_tris[i].empty())
		{
			build_tree(child_indices[i], child_tris[i], depth + 1);
			all_empty = false;
		}
	}

	// If all children are empty, this should remain a leaf
	if (all_empty)
	{
		nodes_[node_idx].data.fill(INVALID_INDEX);
		nodes_[node_idx].is_leaf = true;
		// Store original triangles if any
		for (size_t i = 0; i < std::min(tri_indices.size(), size_t(MAX_TRIS_PER_NODE)); ++i)
		{
			nodes_[node_idx].data[i] = tri_indices[i];
		}
	}
	else
	{
		nodes_[node_idx].is_leaf = false;
	}
}

void QuadTree::subdivide(uint32_t node_idx)
{
	// Copy bounds before allocating nodes (allocation can invalidate node reference)
	AABB2D bounds = nodes_[node_idx].bounds;
	glm::vec2 center = (bounds.min + bounds.max) * 0.5f;

	// NW
	uint32_t nw_idx = allocate_node();
	nodes_[nw_idx].bounds = {{bounds.min.x, center.y}, {center.x, bounds.max.y}};

	// NE
	uint32_t ne_idx = allocate_node();
	nodes_[ne_idx].bounds = {{center.x, center.y}, bounds.max};

	// SW
	uint32_t sw_idx = allocate_node();
	nodes_[sw_idx].bounds = {bounds.min, center};

	// SE
	uint32_t se_idx = allocate_node();
	nodes_[se_idx].bounds = {{center.x, bounds.min.y}, {bounds.max.x, center.y}};

	// Update parent node after all allocations (to avoid dangling reference)
	nodes_[node_idx].data[0] = nw_idx;
	nodes_[node_idx].data[1] = ne_idx;
	nodes_[node_idx].data[2] = sw_idx;
	nodes_[node_idx].data[3] = se_idx;
}

void QuadTree::find_edge_neighbors(std::vector<uint32_t> & neighbors, uint32_t tri_index) const
{
	if (tri_index >= mesh_.tris.size())
		return;

	AABB2D tri_bounds = get_triangle_bounds(tri_index);

	// Query a slightly expanded region to catch edge-adjacent triangles
	float epsilon = 0.0001f;
	tri_bounds.min.x -= epsilon;
	tri_bounds.min.y -= epsilon;
	tri_bounds.max.x += epsilon;
	tri_bounds.max.y += epsilon;

	std::vector<uint32_t> candidates;
	candidates.reserve(neighbors.size());
	query_region(candidates, tri_bounds);

	for (uint32_t candidate : candidates)
	{
		if (candidate != tri_index && triangles_share_edge(tri_index, candidate))
		{
			neighbors.push_back(candidate);
		}
	}
}

void QuadTree::find_overlapping(std::vector<uint32_t> & overlapping, uint32_t tri_index) const
{
	if (tri_index >= mesh_.tris.size())
		return;

	AABB2D tri_bounds = get_triangle_bounds(tri_index);
	std::vector<uint32_t> candidates;
	candidates.reserve(overlapping.size());
	query_region(candidates, tri_bounds);

	for (uint32_t candidate : candidates)
	{
		if (candidate != tri_index && triangles_intersect(tri_index, candidate))
		{
			overlapping.push_back(candidate);
		}
	}
}

void QuadTree::query_region(std::vector<uint32_t> & dst, AABB2D const& region) const
{
	if (root_idx_ != INVALID_INDEX)
		query_region_recursive(root_idx_, region, dst);
}

void QuadTree::query_point(std::vector<uint32_t> & dst, glm::vec2 const& point, float radius) const
{
	AABB2D region;
	region.min = {point.x - radius, point.y - radius};
	region.max = {point.x + radius, point.y + radius};
	return query_region(dst, region);
}


AABB2D QuadTree::get_triangle_bounds(uint32_t tri_index) const
{
	return compute_triangle_bounds(tri_index);
}

void QuadTree::query_region_recursive(uint32_t node_idx, AABB2D const& region, std::vector<uint32_t>& results) const
{
	if (node_idx == INVALID_INDEX)
		return;

	std::vector<int> stack;
	stack.reserve(max_depth_*4);
	
	stack.push_back(node_idx);
	
	while(stack.size())
	{
		node_idx = stack.back();
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
					results.push_back(tri_idx);
				}
			}	
		}
	}
}

bool QuadTree::triangles_share_edge(uint32_t tri_a, uint32_t tri_b) const
{
	auto const& t_a = mesh_.tris[tri_a];
	auto const& t_b = mesh_.tris[tri_b];

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

bool QuadTree::triangles_intersect(uint32_t tri_a, uint32_t tri_b) const
{
	auto const& t_a = mesh_.tris[tri_a];
	auto const& t_b = mesh_.tris[tri_b];

	glm::vec2 const& a0 = mesh_.points[t_a.x];
	glm::vec2 const& a1 = mesh_.points[t_a.y];
	glm::vec2 const& a2 = mesh_.points[t_a.z];

	glm::vec2 const& b0 = mesh_.points[t_b.x];
	glm::vec2 const& b1 = mesh_.points[t_b.y];
	glm::vec2 const& b2 = mesh_.points[t_b.z];

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

bool QuadTree::point_in_triangle(glm::vec2 const& p, glm::vec2 const& v0, glm::vec2 const& v1, glm::vec2 const& v2) const
{
	// Barycentric coordinate method
	float denom = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
	if (std::abs(denom) < 1e-10f)
		return false;

	float a = ((v1.y - v2.y) * (p.x - v2.x) + (v2.x - v1.x) * (p.y - v2.y)) / denom;
	float b = ((v2.y - v0.y) * (p.x - v2.x) + (v0.x - v2.x) * (p.y - v2.y)) / denom;
	float c = 1.0f - a - b;

	return a >= 0.0f && a <= 1.0f && b >= 0.0f && b <= 1.0f && c >= 0.0f && c <= 1.0f;
}

bool QuadTree::point_in_triangle(glm::vec2 const& p, uint32_t tri) const
{
	auto const& t_a = mesh_.tris[tri];
	
	glm::vec2 const&  a0 = mesh_.points[t_a.x];
	glm::vec2 const&  a1 = mesh_.points[t_a.y];
	glm::vec2 const&  a2 = mesh_.points[t_a.z];

	return point_in_triangle(p, a0, a1, a2);
}

bool QuadTree::segments_intersect(glm::vec2 const& a0, glm::vec2 const& a1, glm::vec2 const& b0, glm::vec2 const& b1) const
{
	auto sign = [](float x) { return (x > 0.0f) - (x < 0.0f); };

	auto ccw = [](glm::vec2 const& A, glm::vec2 const& B, glm::vec2 const& C) {
		return (C.y - A.y) * (B.x - A.x) - (B.y - A.y) * (C.x - A.x);
	};

	float d1 = ccw(b0, b1, a0);
	float d2 = ccw(b0, b1, a1);
	float d3 = ccw(a0, a1, b0);
	float d4 = ccw(a0, a1, b1);

	if (sign(d1) != sign(d2) && sign(d3) != sign(d4))
		return true;

	// Check for collinear overlap
	if (std::abs(d1) < 1e-10f && std::abs(d2) < 1e-10f)
	{
		// Both segments on same line, check if they overlap
		float min_ax = std::min(a0.x, a1.x);
		float max_ax = std::max(a0.x, a1.x);
		float min_bx = std::min(b0.x, b1.x);
		float max_bx = std::max(b0.x, b1.x);

		return max_ax >= min_bx && max_bx >= min_ax;
	}

	return false;
}

AABB2D QuadTree::compute_bounds(std::span<uint32_t const> tri_indices) const
{
	AABB2D bounds;
	bounds.min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
	bounds.max = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

	for (uint32_t tri_idx : tri_indices)
	{
		auto const& tri = mesh_.tris[tri_idx];

		for (uint32_t vert_idx : {tri.x, tri.y, tri.z})
		{
			auto const& point = mesh_.points[vert_idx];
			bounds.min.x = std::min(bounds.min.x, point.x);
			bounds.min.y = std::min(bounds.min.y, point.y);
			bounds.max.x = std::max(bounds.max.x, point.x);
			bounds.max.y = std::max(bounds.max.y, point.y);
		}
	}

	return bounds;
}

AABB2D QuadTree::compute_triangle_bounds(uint32_t tri_index) const
{
	auto const& tri = mesh_.tris[tri_index];

	AABB2D bounds;
	bounds.min = {
		std::min({mesh_.points[tri.x].x, mesh_.points[tri.y].x, mesh_.points[tri.z].x}),
		std::min({mesh_.points[tri.x].y, mesh_.points[tri.y].y, mesh_.points[tri.z].y})
	};
	bounds.max = {
		std::max({mesh_.points[tri.x].x, mesh_.points[tri.y].x, mesh_.points[tri.z].x}),
		std::max({mesh_.points[tri.x].y, mesh_.points[tri.y].y, mesh_.points[tri.z].y})
	};

	return bounds;
}

}
