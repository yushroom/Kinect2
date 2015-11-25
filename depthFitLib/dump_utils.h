#pragma once
#include "math_utils.h"

#include <codex_basic.h>
#include <codex_math.h>
#include <vector>	

#include <atlimage.h>

typedef codex::math::vector::vector3<float>				vector3f;
typedef codex::math::vector::vector3<int>				vector3i;
typedef codex::math::vector::vector3<unsigned char>		vector3b;

void	dump_point_cloud(
	const std::vector<vector3f>&	points, 
	const char*						path, 
	const vector3b					rgb = vector3b(255, 255, 255));

void    dump_image(
    const std::vector<vector3b>&    image,
    const char*                     path,
    const int                       width,
    const int                       height);

template <typename T>
void dump_normalized_image(
	const std::vector<T>&			image,
	const char*						path,
	const int						width,
	const int						height)
{
	T lower_bound = *std::min_element(image.begin(), image.end());
	T upper_bound = *std::max_element(image.begin(), image.end());
	dump_normalized_image(
		image,
		path,
		width,
		height,
		lower_bound,
		upper_bound);
}

template <typename T>
void dump_normalized_image(
	const std::vector<T>&			image,
	const char*						path,
	const int						width,
	const int						height,
	const T							lower_bound,
	const T							upper_bound)
{
	const T range = upper_bound - lower_bound;
	CImage cimage;
	cimage.Create(width, height, 32);
	for (int i = 0; i < height; ++i)
		for (int j = 0; j < width; ++j) {
		int idx = j + i*width;
		T value = clamp(image[idx], lower_bound, upper_bound);
		value = range==0?1.f:((value - lower_bound) / range);
		BYTE rgb = static_cast<BYTE>(value * 255);
		cimage.SetPixelRGB(j, i, rgb, rgb, rgb);
		}
	cimage.Save(path);
}

