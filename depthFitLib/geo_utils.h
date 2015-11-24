#pragma once

#include <vector>

#include <codex_math.h>

typedef codex::math::vector::vector3<float>				vector3f;

std::vector<vector3f> calc_points_from_depth_image(
	const std::vector<float>&	depth,
	const int					width,
	const int					height,
	const float					fx,
	const float					fy,
	const float					cx,
	const float					cy);