#ifndef DODEEDUM_VERTEX_REMAPPING_HPP
#define DODEEDUM_VERTEX_REMAPPING_HPP
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <glm/ext/vector_float2.hpp>
#include <vector>
	
template<int N, typename T, glm::qualifier Q>
std::vector<int> sort_vertices(std::vector<glm::vec<N, T, Q>> & points)
{	
	std::vector<std::pair<int, glm::vec2>> memo;
	memo.resize(points.size());
	
	for(auto i = 0u; i < points.size(); ++i)
	{
		memo[i].first = i;
		// we only compare x and y, and we don't read from this to repopulate points so its fine. 
		memo[i].second = points[i];
	}
	
	std::sort(memo.begin(), memo.end(), [](auto & a, auto & b) 
	{ 
		return (a.second.y != b.second.y)? 
					(a.second.y < b.second.y)
				:	(a.second.x < b.second.x);
	});
	std::vector<glm::vec<N, T, Q>> new_points(points.size());
	
	std::vector<int> vertex_mapping;
	vertex_mapping.resize(points.size());
	
	for(auto i = 0u; i < points.size(); ++i)
	{
		new_points[i] = points[memo[i].first];
		vertex_mapping[memo[i].first] = i;
	}
	
	points = std::move(new_points);	
	return vertex_mapping;
}

template <typename T>
typename std::enable_if<sizeof(T) == 4, int>::type
ulp_diff(T a, T b) {
    int32_t ia, ib;
    std::memcpy(&ia, &a, sizeof(a));
    std::memcpy(&ib, &b, sizeof(b));

    if (ia < 0) ia = 0x80000000 - ia;
    if (ib < 0) ib = 0x80000000 - ib;

    return std::abs(ia - ib);
}

template <typename T>
typename std::enable_if<sizeof(T) == 8, long long>::type
ulp_diff(T a, T b) {
    int64_t ia, ib;
    std::memcpy(&ia, &a, sizeof(a));
    std::memcpy(&ib, &b, sizeof(b));

    if (ia < 0) ia = 0x8000000000000000LL - ia;
    if (ib < 0) ib = 0x8000000000000000LL - ib;

    return std::llabs(ia - ib);
}


template<int N, typename T, glm::qualifier Q>
std::vector<int> deduplicate_vertices(std::vector<glm::vec<N, T, Q>> & points, glm::vec<N, T, Q> scale)
{
	(void)scale;
	std::vector<int> deduplicated(points.size(), -1);
	
	auto IsSame = [](T a, T b) -> bool
	{
		static T constexpr EPS = 1e-5;//std::numeric_limits<T>::epsilon();
		static int constexpr MAX_ULP = 4;
		auto delta = std::abs(a - b);
		return (delta < EPS);// return false;
	//	auto diff = ulp_diff(a, b);
	//	return diff < MAX_ULP;
	};
	
	uint32_t read, write;
	for(read = 0, write = 0; read < points.size(); ++read)
	{
		if(deduplicated[read] != -1)
			continue;
			
		auto & t = points[read];
		
		if(std::fabs(t.y - 0.202458 ) < 0.01)
		{
			int break_point = 0;
			++break_point;
		
		}
		
		for(uint32_t next = read+1; next < points.size(); ++next)
		{	
			if(IsSame(t.y, points[next].y) == false)
			{
				break;
			}
			
			for(auto i = 0; i < N; ++i)
			{
				if(IsSame((&t.x)[i], (&points[next].x)[i]) == false)
				{
					goto not_same;
				}
			}
			
			deduplicated[next] = write;
			
			not_same:
				continue;
		}
		
		deduplicated[read] = write;
		points[write++] = points[read];
	}
	
	points.resize(write);
	return deduplicated;
}


#endif // DODEEDUM_VERTEX_REMAPPING_HPP
