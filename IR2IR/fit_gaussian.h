#pragma once

#include <vector>
#include <utility>
typedef unsigned short DEPTH_TYPE;
typedef std::pair<double, double> GAUSSIAN_TYPE;
std::vector<GAUSSIAN_TYPE> fit_gaussian(
	const std::vector<DEPTH_TYPE>& depth_frame_v1,
	const std::vector<DEPTH_TYPE>& depth_frame_v2,
	const int depth_width,
	const int depth_height,
	const int gaussian_width,
	const double sigma_i,
	const double sigma_v);