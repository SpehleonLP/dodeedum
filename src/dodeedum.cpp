#include "dodeedum.h"
#include "dodeedum_mesh.h"
#include "dodeedum_meshedges.h"
#include "dodeedum_project2d.h"
#include "dodeedum_quadtree.h"
#include <fstream>
#include <algorithm>
#include <atomic>

#include "../thirdparty/CDT/CDT/include/CDT.h"

#ifndef __unused__
#if defined(__GNUC__) || defined(__clang__)
#define __unused __attribute__((unused))
#else
#define __unused
#endif
#endif


static DoDeeDum::Silhouette GetSilhouette(std::vector<glm::vec2> && points, std::span<uint32_t> edges, std::span<std::pair<uint32_t, uint32_t> > loops, bool get_second_moment);
static DoDeeDum::Silhouette GetSilhouette(std::vector<glm::vec2> && points, std::vector<glm::uvec3> && triangulation, bool get_second_moment);

static void __unused Export_Obj(std::filesystem::path const& p, std::span<glm::vec2> points, std::span<uint32_t> edges, std::span<std::pair<uint32_t, uint32_t> > loops);

/// Get edge first vertex
inline auto edge_get_v1(const std::pair<uint32_t, uint32_t>& e) { return std::min(e.first, e.second); }
inline auto edge_get_v2(const std::pair<uint32_t, uint32_t>& e) { return std::max(e.first, e.second); }

DoDeeDum::Silhouette DoDeeDum::GetSilhouette(Input const& in, const char * path, const char * name)
{	
	static std::atomic<int> counter{0};
	auto id = counter++;
	
	auto functional_projection = in.projection * 
			glm::mat4(
				in.scale.x, 0, 0, 0, 
				0, in.scale.y, 0, 0, 
				0, 0, in.scale.z, 0,
				0, 0, 0, 1);
	
	// functional_projection * scale * in.projection			
	auto scale = functional_projection * glm::inverse(in.projection);
	
	ProjectedMesh projected(in.mesh, in.projection, in.joints);
	MeshEdges edges = MeshEdges::Perimiter(projected);
	
	for(auto & p : edges.points)
	{
		glm::vec4 v = scale * glm::vec4(p, 0, 1);
		p = (glm::vec2&)v * (v.w? 1.f / v.w : 1.f);
	}
	
	if(path && name)
	{
		char buffer[64];
		if(snprintf(buffer, sizeof(buffer), "%s-edges-%d.obj", name, id) > 0)
		{
			edges.export_debug_OBJ(std::filesystem::path(path) / buffer);			
		}
	}	
	
	
	auto perimiter  = edges.GetPerimiter();	
	
	if(path && name)
	{
		char buffer[64];
		if(snprintf(buffer, sizeof(buffer), "%s-perimiter-%d.obj", name, id) > 0)
		{
			::Export_Obj(std::filesystem::path(path) / buffer, edges.points, perimiter.indices, perimiter.loops);			
		}
	}	
	
	auto s = ::GetSilhouette(std::move(edges.points), perimiter.indices, perimiter.loops, in.getSecondMoment);	
	s.projection = functional_projection;
	
	return s;
}

static DoDeeDum::Silhouette GetSilhouette(std::vector<glm::vec2> && points, std::span<uint32_t> edges,  std::span<std::pair<uint32_t, uint32_t>> loops, bool get_second_moment)
{
	std::vector<glm::uvec3> triangulation;
	{
		CDT::Triangulation<float> cdt;
		
		static_assert(sizeof(glm::vec2) == sizeof(CDT::V2d<float>));
		cdt.insertVertices((std::vector<CDT::V2d<float>>&)points);
		
		for(auto loop : loops)
		{
			auto offset = edges.data()+loop.first;
			cdt.insertEdges(
				offset, 
				offset + loop.second,
				[](uint32_t& it)  { return it; },
				[offset=offset,length=loop.second](uint32_t& it) { return offset[((&it - offset)+1)%length]; }
			);
		}
		cdt.eraseOuterTrianglesAndHoles();
	//	cdt.eraseSuperTriangle();
		
		
		triangulation.reserve(cdt.triangles.size());
		
		for(auto i = 0u; i < cdt.triangles.size(); ++i)
		{
			triangulation.push_back({ 
				cdt.triangles[i].vertices[0], 
				cdt.triangles[i].vertices[1], 
				cdt.triangles[i].vertices[2]
			});
			
			assert(cdt.triangles[i].vertices[0] < points.size());
			assert(cdt.triangles[i].vertices[1] < points.size());
			assert(cdt.triangles[i].vertices[2] < points.size());
		}
	}
	
	return GetSilhouette(std::move(points), std::move(triangulation), get_second_moment);
}

static DoDeeDum::Silhouette GetSilhouette(std::vector<glm::vec2> && points, std::vector<glm::uvec3> && triangulation, bool get_second_moment)
{
	DoDeeDum::Silhouette s;
	s.points = std::move(points);
	s.delauny_triangulation = std::move(triangulation);
	
	// just assume the triangulation field was filled somehow.. 

    s.area = 0.0;
    glm::vec2 weighted_center(0.0f, 0.0f);
    
    for (const auto& tri : s.delauny_triangulation)
    {
        const glm::vec2& p0 = s.points[tri.x];
        const glm::vec2& p1 = s.points[tri.y];
        const glm::vec2& p2 = s.points[tri.z];
        
        // Signed area of triangle
        double triangle_area = 0.5 * ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y));
        s.area += triangle_area;
        
        glm::vec2 triangle_centroid = (p0 + p1 + p2) / 3.0f;
        
        weighted_center += triangle_centroid * static_cast<float>(triangle_area);
    }
    
    s.area = std::abs(s.area);
    s.center = weighted_center / static_cast<float>(s.area);
        
    // Calculate second moment if requested
    if (get_second_moment)
    {
        s.area2ndMoment = {0.0f, 0.0f, 0.0f};
        
        for (const auto& tri : s.delauny_triangulation)
        {
            const glm::vec2& p0 = s.points[tri.x];
            const glm::vec2& p1 = s.points[tri.y];
            const glm::vec2& p2 = s.points[tri.z];
            
            double triangle_area = 0.5 * ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y));
            
            // Translate to center
            glm::vec2 q0 = p0 - s.center;
            glm::vec2 q1 = p1 - s.center;
            glm::vec2 q2 = p2 - s.center;
            
            // Second moment integrals for a triangle (using standard formulas)
            float Ixx = (q0.y*q0.y + q1.y*q1.y + q2.y*q2.y + q0.y*q1.y + q1.y*q2.y + q2.y*q0.y) / 6.0f;
            float Iyy = (q0.x*q0.x + q1.x*q1.x + q2.x*q2.x + q0.x*q1.x + q1.x*q2.x + q2.x*q0.x) / 6.0f;
            float Ixy = (2*q0.x*q0.y + 2*q1.x*q1.y + 2*q2.x*q2.y + q0.x*q1.y + q1.x*q0.y + 
                         q1.x*q2.y + q2.x*q1.y + q2.x*q0.y + q0.x*q2.y) / 12.0f;
            
            s.area2ndMoment.xx += Ixx * static_cast<float>(triangle_area);
            s.area2ndMoment.yy += Iyy * static_cast<float>(triangle_area);
            s.area2ndMoment.xy += Ixy * static_cast<float>(triangle_area);
        }
    }
    
    return s;
}
    

void DoDeeDum::Export_Obj(std::filesystem::path const& path, std::span<const glm::vec2> points, std::span<const glm::uvec3> tris)
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


// measure the longest distance along the silhouette that crosses the given line segment. 
DoDeeDum::Silhouette::Measurement DoDeeDum::Silhouette::MeasureWidth(glm::vec2 const& axis, glm::vec2 const& point, glm::vec2 const& dir, double min, double max) const
{
    Measurement result;
    result.length = 0.0;
    
    for(auto i = 0u; i < points.size(); ++i)
    {
        for(auto j = i+1; j < points.size(); ++j)
        {
            // Find where the line segment between points[i] and points[j] 
            // crosses the line defined by point and dir
            
            glm::dvec2 segDir = glm::dvec2(points[j]) - glm::dvec2(points[i]);
            glm::dvec2 toStart = glm::dvec2(points[i]) - glm::dvec2(point);
            
            // Solve for intersection: point + t*dir = points[i] + s*segDir
            // This gives us: toStart + s*segDir = t*dir
            // Using cross product to solve: s = (toStart × dir) / (segDir × dir)
            
            double denom = segDir.x * dir.y - segDir.y * dir.x; // cross product
            
            // stuff is very small, rely on the s check to deal with things that are weird.
            if(std::abs(denom) == 0.0) continue;
            
            double s = (toStart.x * dir.y - toStart.y * dir.x) / denom;
            
            // Check if intersection is on the segment between i and j
            if(s < 0.0 || s > 1.0)
                continue;
            
            // Find t (position along the dir line)
            double t = (toStart.x * segDir.y - toStart.y * segDir.x) / (-denom);
            
            // Check if intersection point is within [min, max] range
            if(t < min || t > max)
                continue;
            
            // Calculate distance between the two points along the axis direction
            double distance = std::abs(glm::dot(points[i] - points[j], axis));
            
            // Update if this is the longest distance found
            if(distance > result.length)
            {
                result.length = distance;
                result.points = {i, j};
            }
        }
    }
    
    return result;
}


static void __unused Export_Obj(std::filesystem::path const& path, std::span<glm::vec2> points, std::span<uint32_t> edges, std::span<std::pair<uint32_t, uint32_t> > loops)
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
	
	for(auto i = 0u; i < loops.size(); ++i)
	{
		file << "g " << i << "\n";
		
		auto & l = loops[i];
		for(auto i = 0u; i < l.second; ++i)
		{
			file << "l " << edges[l.first+i]+1 << " " << edges[l.first+((i+1)%l.second)]+1 << "\n";
		}
	}
		
	file.close();
	
	if (file.fail())
	{
		throw std::system_error(errno, std::generic_category(),
		                        "Failed to write to file: " + path.string());
	}
}

