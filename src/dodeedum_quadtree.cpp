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

bool point_in_triangle(glm::dvec2 const& p, glm::dvec2 const& v0, glm::dvec2 const& v1, glm::dvec2 const& v2)
{
	auto denom = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
	
	if (denom == 0)
		return false;  // Degenerate triangle
	
	auto a = ((v1.y - v2.y) * (p.x - v2.x) + (v2.x - v1.x) * (p.y - v2.y)) / denom;
	auto b = ((v2.y - v0.y) * (p.x - v2.x) + (v0.x - v2.x) * (p.y - v2.y)) / denom;
	auto c = 1.0 - a - b;
	
	// Use epsilon tolerance for boundary cases
	return a >= 0 && a <= 1.0 && 
	       b >= 0 && b <= 1.0 && 
	       c >= 0 && c <= 1.0;
}

bool segments_intersect(glm::dvec2 const& a0, glm::dvec2 const& a1, glm::dvec2 const& b0, glm::dvec2 const& b1)
{
	auto sign = [](float x) { return (x > 0.0f) - (x < 0.0f); };

	auto ccw = [](glm::dvec2 const& A, glm::dvec2 const& B, glm::dvec2 const& C) {
		return (C.y - A.y) * (B.x - A.x) - (B.y - A.y) * (C.x - A.x);
	};

	auto d1 = ccw(b0, b1, a0);
	auto d2 = ccw(b0, b1, a1);
	auto d3 = ccw(a0, a1, b0);
	auto d4 = ccw(a0, a1, b1);

	if (sign(d1) != sign(d2) && sign(d3) != sign(d4))
		return true;

	// Check for collinear overlap
	if (std::abs(d1) < 1e-10 && std::abs(d2) < 1e-10)
	{
		// Both segments on same line, check if they overlap
		auto min_ax = std::min(a0.x, a1.x);
		auto max_ax = std::max(a0.x, a1.x);
		auto min_bx = std::min(b0.x, b1.x);
		auto max_bx = std::max(b0.x, b1.x);

		return max_ax >= min_bx && max_bx >= min_ax;
	}

	return false;
}



void QuadTree::build()
{
	if(root_idx_ != INVALID_INDEX)
		return;

	total_items_ = get_polygon_count();
	if (total_items_ == 0)
		return;

	// Estimate node count: worst case is (4^(depth+1) - 1) / 3 for a full quadtree
	// More conservative: 4^(depth+1) to allow for some overhead
	uint32_t probable_depth = std::ceil(log2(total_items_)/2);
	size_t estimated_nodes = 1;
	for (uint32_t i = 0; i <= probable_depth; ++i)
	{
		estimated_nodes *= 4;
	}
	nodes_.reserve(estimated_nodes/2.8);

	// Build triangle index list
	// Since tris are sorted by least maximum, we can pass them in order
	std::vector<uint32_t> all_indices(total_items_);
	std::vector<glm::vec2> max_size_memo(total_items_);
	
	for (uint32_t i = 0; i < total_items_; ++i)
	{
		all_indices[i] = i;
		max_size_memo[i] = get_polygon_bounds(i).max;
	}

	root_idx_ = allocate_node();
	
	std::vector<std::pair<glm::vec2, uint32_t>> scratch;
	build_tree(root_idx_, max_size_memo, scratch, all_indices, 0);
}

uint32_t QuadTree::allocate_node()
{
	uint32_t idx = static_cast<uint32_t>(nodes_.size());
	nodes_.emplace_back();
	return idx;
}
	
void QuadTree::build_tree(uint32_t node_idx, std::vector<glm::vec2> const& max_size_memo, std::vector<std::pair<glm::vec2, uint32_t>> & scratch, std::span<const uint32_t> tri_indices, uint32_t depth)
{
	if(tri_indices.size() <= 4)
	{
		auto & n = nodes_[node_idx];
		n.data.fill(INVALID_INDEX);
		n.is_leaf = true;
	
		for(auto i = 0u; i < tri_indices.size(); ++i)
		{
			n.data[i] = tri_indices[i];
		}
	
		n.bounds = compute_bounds(tri_indices);
        max_depth_ = std::max(max_depth_, depth);
        return;
	}

	auto CompareX_Dominant = [](std::pair<glm::vec2, uint32_t> & a, std::pair<glm::vec2, uint32_t> & b)
	{
		return a.first != b.first? 
			(a.first.x != b.first.x?
				a.first.x < a.first.x 
			  : a.first.y < b.first.y)
			: a.second < b.second;
	};

	auto CompareY_Dominant = [](std::pair<glm::vec2, uint32_t> & a, std::pair<glm::vec2, uint32_t> & b)
	{
		return a.first != b.first? 
			(a.first.y != b.first.y?
				a.first.y < a.first.y 
			  : a.first.x < b.first.x)
			: a.second < b.second;
	};
	
	auto Split = [&](std::span<const uint32_t> span) -> std::vector<uint32_t>
	{
	// reuse because malloc = expensive.
		scratch.resize(span.size());
		
		for(auto i = 0u; i < span.size(); ++i)
		{
			scratch[i] = {max_size_memo[span[i]], span[i]};
		}
		
		std::sort(scratch.begin(), scratch.end(), CompareX_Dominant);
		
		auto top = std::span{ scratch.data(), scratch.size()/2 };
		auto bottom = std::span{ top.end(), scratch.size() - top.size() };
	
		std::sort(top.begin(), top.end(), CompareY_Dominant);
		std::sort(bottom.begin(), bottom.end(), CompareY_Dominant);
		
		std::vector<uint32_t> indices(span.size());
		for(auto i = 0u; i < span.size(); ++i)
		{
			indices[i] = scratch[i].second;
		}
		
		return indices;
	};
	
	// Exploit sorted order: triangles are sorted by least maximum vertex index
	// AND: verticies are sorted by Y 
	auto top = Split({ tri_indices.data(), tri_indices.size()/2 });
	auto bottom = Split({ tri_indices.data() + tri_indices.size()/2, tri_indices.size() - tri_indices.size()/2 });
	
	std::array<std::span<uint32_t>, 4> children;
	children[0] = { top.data(), top.size()/2 };
	children[1] = { children[0].end(), top.size() - children[0].size() };
	children[2] = { bottom.data(), bottom.size()/2 };
	children[3] = { children[2].end(), bottom.size() - children[2].size() };
	
	for(auto i = 0u; i < 4; ++i)
	{
		auto idx = allocate_node();
		nodes_[node_idx].data[i] = idx;
		
		build_tree(idx, max_size_memo, scratch, children[i], depth+1);
		
		if(i == 0)
		{
			nodes_[node_idx].bounds = nodes_[idx].bounds;
			nodes_[node_idx].is_leaf = false;
		}
		else
		{
			nodes_[node_idx].bounds.min = glm::min(nodes_[node_idx].bounds.min, nodes_[idx].bounds.min);
			nodes_[node_idx].bounds.max = glm::max(nodes_[node_idx].bounds.max, nodes_[idx].bounds.max);
		}
	}
}


AABB2D QuadTree::compute_bounds(std::span<uint32_t const> tri_indices) const
{
	AABB2D bounds;
	bounds.min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
	bounds.max = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

	for (uint32_t tri_idx : tri_indices)
	{
		auto b = get_polygon_bounds(tri_idx);

		bounds.min = glm::min(bounds.min, b.min);
		bounds.max = glm::max(bounds.max, b.max);
	}

	return bounds;
}


}
