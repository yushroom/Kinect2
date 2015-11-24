#pragma once

#include <codex_basic.h>
#include <codex_math.h>
#include <vector>

typedef codex::math::vector::vector3<float>				vector3f;
typedef codex::math::vector::vector3<int>				vector3i;
typedef codex::math::vector::vector3<unsigned char>		vector3b;

void	dump_point_cloud(
	const std::vector<vector3f>&	points, 
	const char*						path, 
	const vector3b					rgb = vector3b(255, 255, 255));

template <typename T>
void dump_normalized_image(
	const std::vector<T>&			image,
	const char*						path,
	const int						width,
	const int						height);

template <typename T>
void dump_normalized_image(
	const std::vector<T>&			image,
	const char*						path,
	const int						width,
	const int						height,
	const T							lower_bound,
	const T							upper_bound);

