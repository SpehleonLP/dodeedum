#ifndef DODEEDUM_H
#define DODEEDUM_H
#include <filesystem>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <span>
#include <vector>

namespace DoDeeDum
{

struct Primitive;
using Mesh = std::vector<Primitive>;

struct Silhouette
{
	glm::mat4x4 projection;
	
	std::vector<glm::vec2>  points;
	// CDT
	std::vector<glm::uvec3> delauny_triangulation; 
	
	double area{};
	
	struct Moment2
	{
		float xx, yy, xy;
	};
	
	Moment2 area2ndMoment{};
	glm::vec2 center{};
	
	struct Measurement
	{
		std::pair<uint32_t, uint32_t> points{0, 0};
		double length{};
	};
	
	inline glm::vec2 Project (glm::vec3 const& in) const;
	inline glm::vec2 ProjectRay (glm::vec3 const& in) const;
	
	// measure the longest distance along the silhouette that crosses the given line. 
	inline Measurement MeasureWidth_PointRay(glm::vec3 const& axis, glm::vec3 const& point, glm::vec3 const& ray) const;
	inline Measurement MeasureWidth_Segment(glm::vec3 const& axis, glm::vec3 const& begin, glm::vec3 const& end) const;
	
	// perpendicular to the line.
	inline Measurement MeasureWidth_PointRay(glm::vec3 const& point, glm::vec3 const& ray) const;
	inline Measurement MeasureWidth_Segment(glm::vec3 const& begin, glm::vec3 const& end) const;
	
	inline Measurement MeasureWidth_PointRay(glm::vec2 const& point, glm::vec2 const& ray) const;
	inline Measurement MeasureWidth_Segment(glm::vec2 const& begin, glm::vec2 const& end) const;
	
	inline Measurement MeasureWidth_PointRay(glm::vec2 const& axis, glm::vec2 const& point, glm::vec2 const& ray) const;
	inline Measurement MeasureWidth_Segment(glm::vec2 const& axis, glm::vec2 const& begin, glm::vec2 const& end) const;
	
	Measurement MeasureWidth(glm::vec2 const& axis, glm::vec2 const& point, glm::vec2 const& dir, double min = -1e200, double max = 1e200) const;
	
};

struct Input
{
	Mesh const& mesh;
	glm::mat4 projection;
	glm::vec3 scale{1.0, 1.0, 1.0};
	// skinning weights below this are cut off. 
	float   cutoff{0.5};
	bool     getSecondMoment{true};
	// joints is what skinning weights we're interested in. 
	std::span<const uint32_t> joints{};
};

Silhouette GetSilhouette(Input const& in, const char * path = nullptr, const char * name = nullptr);

// export delauny triangulation for debugging
void Export_Obj(std::filesystem::path const& path, std::span<const glm::vec2> points, std::span<const glm::uvec3> tris);

// utility functions exposed cause they may be useful.
bool point_in_triangle(glm::dvec2 const& p, glm::dvec2 const& v0, glm::dvec2 const& v1, glm::dvec2 const& v2);
bool segments_intersect(glm::dvec2 const& a0, glm::dvec2 const& a1, glm::dvec2 const& b0, glm::dvec2 const& b1);


inline glm::vec2 Silhouette::Project (glm::vec3 const& in) const
{
	auto v4 = projection * glm::vec4(in, 1.0);
	if(v4.w) v4 /= v4.w;
	return glm::vec2(v4);
};

inline glm::vec2 Silhouette::ProjectRay (glm::vec3 const& in) const
{
// may not be equivalent to just taking the last row if FOV etc.
	auto v4 = projection * glm::vec4(0, 0, 0, 1.0);
	if(v4.w) v4 /= v4.w;
	return glm::vec2(glm::normalize(Project(in) - glm::vec2(v4)));
};

// measure the longest distance along the silhouette that crosses the given line. 
inline Silhouette::Measurement Silhouette::MeasureWidth_PointRay(glm::vec3 const& axis, glm::vec3 const& point, glm::vec3 const& ray) const { return MeasureWidth(ProjectRay(axis), Project(point), ProjectRay(ray)); }
inline Silhouette::Measurement Silhouette::MeasureWidth_PointRay(glm::vec3 const& point, glm::vec3 const& ray) const { auto r2 = ProjectRay(ray); return MeasureWidth({-r2.y, r2.x}, Project(point), r2); }
inline Silhouette::Measurement Silhouette::MeasureWidth_PointRay(glm::vec2 const& point, glm::vec2 const& ray) const { return MeasureWidth({-ray.y, ray.x}, point, ray); }
inline Silhouette::Measurement Silhouette::MeasureWidth_PointRay(glm::vec2 const& axis, glm::vec2 const& point, glm::vec2 const& ray) const { return MeasureWidth(axis, point, ray); }

inline Silhouette::Measurement Silhouette::MeasureWidth_Segment(glm::vec3 const& axis, glm::vec3 const& begin, glm::vec3 const& end) const { return MeasureWidth(ProjectRay(axis), Project(begin), Project(end)); }
inline Silhouette::Measurement Silhouette::MeasureWidth_Segment(glm::vec3 const& begin, glm::vec3 const& end) const { return MeasureWidth_Segment(Project(begin), Project(end)); }

inline Silhouette::Measurement Silhouette::MeasureWidth_Segment(glm::vec2 const& b, glm::vec2 const& e) const 
{ 
	auto v = e - b;
	auto l = glm::length(v);
	
	if(l == 0) return {};	
	auto ray = v / l;
	
	return MeasureWidth({-ray.y, ray.x}, b, ray, 0, l); 
}

inline Silhouette::Measurement Silhouette::MeasureWidth_Segment(glm::vec2 const& axis, glm::vec2 const& b, glm::vec2 const& e) const
{
	auto v = e - b;
	auto l = glm::length(v);
	
	if(l == 0) return {};	
	auto ray = v / l;
	
	return MeasureWidth(axis, b, ray, 0, l); 

}

}

#endif // DODEEDUM_H
