#include "dodeedum.h"
#include "Spehleon/dodeedum/src/dodeedum_project2d.h"
#include "dodeedum_quadtree.h"
#include <algorithm>

#ifndef DelaunatorPrecision
#define DelaunatorPrecision 32
#endif

#include "delaunator-cpp/include/delaunator.hpp"

using vec2 = glm::vec<2, delaunator::d_fp, glm::precision::highp>;
static auto constexpr EPS = std::numeric_limits<delaunator::d_fp>::epsilon();

static std::vector<vec2> GetAllPoints(DoDeeDum::ProjectedMesh const& d, DoDeeDum::QuadTree const& tree);
static std::vector<float> GetAllWeights(std::vector<vec2> const& points, DoDeeDum::ProjectedMesh const& d, DoDeeDum::QuadTree const& tree);
static std::vector<uint32_t> GetTriangulation(std::vector<vec2> & points, DoDeeDum::QuadTree const& tree, bool did_insert);
static std::vector<std::vector<uint32_t>> ToEdgeLists(std::vector<uint32_t> const& triangles, uint32_t no_verts, bool only_outside);
static std::vector<std::vector<uint32_t>> ToCliques(std::vector<std::vector<uint32_t>> const& edge_lists);
static std::vector<DoDeeDum::Silhouette> GetSilhouettes(std::vector<glm::vec2> const& points, std::vector<float> const& weights, std::vector<uint32_t> const& triangles, std::vector<std::vector<uint32_t>> const& cliques, DoDeeDum::Options options);

std::vector<DoDeeDum::Silhouette> DoDeeDum::GetSilhouettes(Mesh const& mesh, glm::mat4 const& projection, Options options, std::span<uint32_t> joints)
{
	ProjectedMesh proj(mesh, projection, joints);
	DoDeeDum::QuadTree quadtree(proj);
	
	std::vector<DoDeeDum::Silhouette> r;
	
	{
		auto points = GetAllPoints(proj, quadtree);
		bool added_center = false;
		
		if(options & INCLUDE_CENTER)
		{
			added_center = !quadtree.IsContained({0, 0});
			
			if(added_center)
				points.push_back({0, 0});
		}
	
		std::vector<float> weights;
		
		if(options & (GET_WEIGHTED_AREA|GET_WEIGHTED_AREA_2ND_MOMENT))
			weights = GetAllWeights(points, proj, quadtree);
		auto triangles = GetTriangulation(points, quadtree, added_center);
		auto edge_list = ToEdgeLists(triangles, points.size()/2, false);
		r = ::GetSilhouettes(points, weights, triangles, ToCliques(edge_list), options);
	}
	
	if(options & GET_DELAUNY_PERIMITER)
	{
		for(auto & s : r)
		{
			s.delauny_triangulation = GetTriangulation(s.points, quadtree, false);
		}
	}
	
	return r;
}

static std::vector<vec2> GetAllPoints(DoDeeDum::ProjectedMesh const& d, DoDeeDum::QuadTree const& tree)
{
	std::vector<vec2> r;
	
	r.reserve(d.points.size()*2);
	
	for(auto i = 0u; i < d.points.size(); ++i)
	{
		r.push_back((glm::vec2&)d.points[i]);
	}
	
	std::vector<uint32_t> p;
	for(auto i = 0u; i < d.tris.size(); ++i)
	{
		p.clear();
		tree.find_overlapping(p, i);
		
		for(auto & tri : p)
		{
			if(tri < i) continue;
			
			d.for_each_intersection(i, tri, [&](glm::vec2 point) -> bool
			{
				r.push_back(point);
				return false;
			});
		}
	}

	float scale = std::min(d.max.x - d.min.x, d.max.y - d.min.y);
	// make reasonable big so eps test is useful.
	scale = scale == 0.0? 1.0 : (scale < 1.0? 1.0 / scale : 1.0);
	
	auto nan = std::nanf("");
	
	std::sort(r.begin(), r.end(), [&](auto &a, auto & b) { return a.y < b.y; });
	 
	uint32_t read = 0, write = 0;
	for(; read < r.size(); ++read)
	{
		if(std::isnan(r[read].x)) continue;
		
		for(uint32_t j = read; j < r.size(); ++read)
		{
			if(std::fabs(r[read].y - r[j].y) > EPS)
				break;
			
			// mark as duplicate.
			if(std::fabs(r[read].x - r[j].x) < EPS)
				r[j].x = nan;
		}
		
		r[write++] = r[read];
	}
	
	r.resize(write);
	
	return r;
}

static std::vector<float> GetAllWeights(std::vector<vec2> const& points, DoDeeDum::ProjectedMesh const& d, DoDeeDum::QuadTree const& tree)
{
	std::vector<float> r(points.size(), 0);
	std::vector<uint32_t> p;
	
	for(auto i = 0u; i < points.size(); ++i)
	{
		p.clear();
		tree.query_point(p, points[i], EPS);
		
		for(auto tri : p)
			r[i] += d.get_value_at_point(points[i], tri);
	}
	
	return r;
}

struct Triangulation
{
	double area;
	double weightedArea;
	glm::vec2 centroid;
	std::vector<uint32_t> tris;
};

static std::vector<uint32_t> GetTriangulation(std::vector<vec2> & points, DoDeeDum::QuadTree const& tree, bool did_insert)
{
	if(points.empty()) return {};
	auto span = std::span<const delaunator::d_fp>(&points[0].x, points.size()*2);
	auto triangles = std::move(delaunator::Delaunator(span).triangles);
	
	bool check_edge_lists = false;
	
	uint32_t N = points.size()-int(did_insert);
	uint32_t write = 0u;
	for(auto read = 0u; read < triangles.size(); read += 3)
	{
		glm::uvec3 tri{triangles[read+0], triangles[read+1], triangles[read+2] };
			
		vec2 const& v0 = points[tri.x*2];
		vec2 const& v1 = points[tri.y*2];
		vec2 const& v2 = points[tri.z*2];
			
		auto centroid = (v0 + v1 + v2) / 3.f;
		bool contains_centroid = tree.IsContained(centroid);
		bool contains_added_point = false;
		
		if(!contains_centroid)
		{
			contains_added_point = (tri.x == N || tri.y == N || tri.z == N);
			check_edge_lists |= contains_added_point;
		}
			
	// basically if this test is bad then the mesh is ill formed.
		if(!contains_added_point && !tree.IsContained(centroid))
		{
			if(read != write)
			{
				triangles[write+0] = triangles[read+0];
				triangles[write+1] = triangles[read+1];
				triangles[write+2] = triangles[read+2];
			}
			
			write += 3;
		}
	}
	
	triangles.resize(write);	
	
	if(!check_edge_lists)
		return triangles;
		
	std::unordered_map<uint64_t, std::vector<int>> edge_lists;
	
	for(auto read = 0u; read < triangles.size(); read += 3)
	{
		glm::uvec3 tri{triangles[read+0], triangles[read+1], triangles[read+2] };
		if(!(tri.x == N || tri.y == N || tri.z == N)) continue;
		
		for(int e = 0; e < 3; ++e)
		{
			uint32_t a = tri[e];
			uint32_t b = tri[e%3];
			
			if(a > b) std::swap(a, b);
		
			if(b == N)
			{
				uint64_t key = (uint64_t(a) << 32) | b;
				edge_lists[key].push_back(read/3);
			}
		}
	}
	
	bool has_perimiter_edge = 0;
	for(auto & itr : edge_lists)
	{
		if(itr.second.size() == 1)
		{
			has_perimiter_edge = true;
			break;
		}
	}
	
	if(!has_perimiter_edge)
		return triangles;
		
	points.resize(points.size()-2);
	return GetTriangulation(points, tree, false);
}

static std::vector<std::vector<uint32_t>> ToEdgeLists(std::vector<uint32_t> const& triangles, uint32_t no_verts, bool only_outside)
{	
	std::vector<std::vector<uint32_t>> v;
	v.resize(no_verts);
	
	for(auto &a : v)
		a.reserve(4);
		
	for(auto read = 0u; read < triangles.size(); read += 3)
	{
		glm::uvec3 tri{triangles[read+0], triangles[read+1], triangles[read+2] };
		
		v[tri.x].push_back(tri.y);
		v[tri.x].push_back(tri.z);
		v[tri.y].push_back(tri.x);
		v[tri.y].push_back(tri.z);
		v[tri.z].push_back(tri.x);
		v[tri.z].push_back(tri.y);
	}
	
	for(auto & vec : v)
	{
		std::sort(vec.begin(), vec.end());
		
		if(only_outside == false)
		{
			auto itr = std::unique(vec.begin(), vec.end());
			vec.resize(itr - vec.begin());
		}
		else
		{
			uint32_t read = 0, write = 0;
			
			for(read = 0; read < vec.size(); ++read)
			{
				if(read+1 < vec.size())
				{
					if(vec[read] == vec[read+1])
					{
						++read;
						continue;
					}
				}
				
				vec[write++] = vec[read];
			}
			
			vec.resize(write);
		}
	}
	
	return v;
}

static std::vector<std::vector<uint32_t>> ToCliques(std::vector<std::vector<uint32_t>> const& edge_lists)
{
	std::vector<std::vector<uint32_t>> r;
	
	std::vector<bool> marks(edge_lists.size(), false);
	std::vector<uint32_t> stack;
	stack.reserve(edge_lists.size());
	
	for(auto i = 0u; i < marks.size(); ++i)
	{
		if(marks[i]) continue;
		
		stack.push_back(i);
		std::vector<uint32_t> list;
		list.reserve(edge_lists.size());
		
		while(stack.size())
		{
			auto item = stack.back();
			stack.pop_back();
			
			marks[item] = true;	
			list.push_back(item);	
				
			for(auto v : edge_lists[item])
			{
				if(!marks[v])
					stack.push_back(v);
			}		
		}
		
		r.push_back(std::move(list));
	}

	return r;
}

static std::vector<DoDeeDum::Silhouette> GetSilhouettes(std::vector<glm::vec2> const& points, std::vector<float> const& weights, std::vector<uint32_t> const& triangles, std::vector<std::vector<uint32_t>> const& cliques, DoDeeDum::Options options)
{
	std::vector<uint32_t> index_to_clique;
	std::vector<DoDeeDum::Silhouette> r(cliques.size());
	index_to_clique.resize(points.size(), 0);
	
	for(auto i = 1.0; i < cliques.size(); ++i)
	{
		for(auto c : cliques[i])
			index_to_clique[c] = i;
	}	
		
	for(auto read = 0u; read < triangles.size(); read += 3)
	{
		glm::uvec3 tri{triangles[read+0], triangles[read+1], triangles[read+2] };
			
		vec2 const& v0 = points[tri.x];
		vec2 const& v1 = points[tri.y];
		vec2 const& v2 = points[tri.z];
			
		auto edge0 = v1 - v0;
		auto edge1 = v2 - v0;
		auto area = 0.5f * glm::abs(edge0.x * edge1.y - edge0.y * edge1.x);
		glm::vec3 w = weights.empty()? glm::vec3(1.0) : glm::vec3{weights[tri.x], weights[tri.y], weights[tri.z]};
		
		auto centroid = (v0*w.x + v1*w.y + v2*w.z) / (w.x+w.y+w.z);
		float weight = weights.empty()? 1.0 : DoDeeDum::GetAverageWeight(w);
		
		auto which = index_to_clique[tri.x];
		assert(index_to_clique[tri.z] == which);
		assert(index_to_clique[tri.y] == which);
		
		auto & silhouette = r[which];
		
		silhouette.area += area;
		silhouette.weightedArea += weight*area;
		
		silhouette.center += (area*weight)*centroid;
	}

	for(auto & silhouette : r)
	{
		silhouette.center /= silhouette.weightedArea? silhouette.weightedArea : 1.0;

		if(weights.empty())
			silhouette.weightedArea = 0;
	}
	
	if(options & (DoDeeDum::GET_WEIGHTED_AREA_2ND_MOMENT|DoDeeDum::GET_AREA_2ND_MOMENT))
	{
		for(auto read = 0u; read < triangles.size(); read += 3)
		{
			glm::uvec3 tri{triangles[read+0], triangles[read+1], triangles[read+2]};
				
			vec2 const& v0 = points[tri.x];
			vec2 const& v1 = points[tri.y];
			vec2 const& v2 = points[tri.z];
			
			auto which = index_to_clique[tri.x];
			auto& silhouette = r[which];
			vec2 center = silhouette.center;
			
			// Translate vertices to centroid coordinate system
			vec2 p0 = v0 - center;
			vec2 p1 = v1 - center;
			vec2 p2 = v2 - center;
			
			auto edge0 = p1 - p0;
			auto edge1 = p2 - p0;
			float cross = edge0.x * edge1.y - edge0.y * edge1.x;
			
			glm::vec3 w = weights.empty()? glm::vec3(1.0) : glm::vec3{weights[tri.x], weights[tri.y], weights[tri.z]};
			float weight = weights.empty()? 1.0 : DoDeeDum::GetAverageWeight(w);
			
			// Second moment of area about centroid
			float Ixx = (1.0f/12.0f) * cross * (p0.y*p0.y + p1.y*p1.y + p2.y*p2.y + 
												  p0.y*p1.y + p1.y*p2.y + p2.y*p0.y);
			float Iyy = (1.0f/12.0f) * cross * (p0.x*p0.x + p1.x*p1.x + p2.x*p2.x + 
												  p0.x*p1.x + p1.x*p2.x + p2.x*p0.x);
			float Ixy = (1.0f/24.0f) * cross * (v0.x*v0.y + v1.x*v1.y + v2.x*v2.y + 
												  v0.x*v1.y + v0.y*v1.x + v1.x*v2.y + 
												  v1.y*v2.x + v2.x*v0.y + v2.y*v0.x);
					
			silhouette.area2ndMoment.xx += Ixx;
			silhouette.area2ndMoment.yy += Iyy;
			silhouette.area2ndMoment.xy += Ixy;
			
			if(weights.size())
			{
				silhouette.weightedArea2ndMoment.xx += weight * Ixx;
				silhouette.weightedArea2ndMoment.yy += weight * Iyy;
				silhouette.weightedArea2ndMoment.xy += weight * Ixy;				
			}
		}
	}
	
	if(options & (DoDeeDum::GET_OUTLINE|DoDeeDum::GET_DELAUNY_PERIMITER))
	{
		auto edge_lists = ToEdgeLists(triangles, points.size(), true);
		auto cliques = ToCliques(edge_lists);
		
		for(auto i = 0u; i < cliques.size(); ++i)
		{
			r[i].points.reserve(cliques[i].size());
			
			for(auto & p : cliques[i])
			{
				r[i].points.push_back(points[p]);
			}
			
			r[i].outline = std::move(cliques[i]);
		}
	}
	
	return r;
}
