#pragma once

#include <vector>
#include <algorithm>
#include <numeric>
#include <math.h>

template <class T>
inline T
clamp(T a, T l, T h)
{
	return (a < l) ? l : ((a > h) ? h : a);
}

template <class T>
inline float
rmse(const std::vector<T> &v1, const std::vector<T>& v2, const std::vector<bool> mask)
{
    assert(v1.size() == v2.size() && v1.size() != 0);
    float result = 0.f;
	int count = 0;
    for (int i = 0; i < v1.size(); ++i)
		if (mask[i]) {
			result += (v1[i] - v2[i])*(v1[i] - v2[i]);
			count++;
		}
    result /= count;
    return sqrtf(result);
}

void bilinear(
	std::vector<int>&	idx,
	std::vector<float>& weight,
	const float			x,
	const float			y,
	const int			width,
	const int			height);


//typedef float FLOAT;
//
//template <typename T>
//class myBilateralFilter {
//private:
//	std::vector<float> _weight;
//
//public:
//	const int	gaussian_width;
//	const T sigma_d;
//	const T sigma_r;
//
//	myBilateralFilter(const int gaussian_width, const float sigma_d, const float sigma_r)
//		: gaussian_width(gaussian_width), sigma_d(sigma_d), sigma_r(sigma_r) 
//	{
//		const T inv_2_sigma_d_sqr = 1.0f / (sigma_d * sigma_d);
//		const T inv_2_sigma_r_sqr = 1.0f / (sigma_r * sigma_r);
//
//		const int pivot = gaussian_width / 2
//
//		FLOAT sum_weight = 0.0;
//		for (int x = 0; x < gaussian_width; ++x)
//			for (int y = 0; y < gaussian_width; ++y) {
//				FLOAT dist_coor_sqr = (x - pivot)*(x - pivot) + (y - pivot)*(y - pivot);
//				FLOAT dist_value_sqr = (local_area_value[x][y] - image_frame[idx])*(local_area_value[x][y] - image_frame[idx]);
//				FLOAT exponential = -(dist_coor_sqr * inv_2_sigma_d_sqr + dist_value_sqr * inv_2_sigma_r_sqr);
//				FLOAT local_weight = VALID_DEPTH_TEST(local_area_value[x][y]) && mask_in[idx] ? exp(exponential) : 0.0f;
//
//				if (local_weight != 0.0) nvalid++;
//				sum_weight += local_weight;
//				p.mu += local_weight * local_area_value[x][y];
//			}
//		p.mu /= sum_weight;
//	}
//
//};

