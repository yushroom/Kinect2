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

class myBilateralFilter {
private:
	std::vector<float> _weight;

public:
	const int	gaussian_width;
	const float sigma_d;
	const float sigma_r;

	myBilateralFilter(const int gaussian_width, const float sigma_d, const float sigma_r) 
		: gaussian_width(gaussian_width), sigma_d(sigma_d), sigma_r(sigma_r) {
	}
};