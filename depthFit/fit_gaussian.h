#pragma once

#include <vector>
#include <utility>
#ifndef DEPTH_TYPE
typedef float DEPTH_TYPE;
#endif
std::vector<DEPTH_TYPE> fit_depth(
	const std::vector<DEPTH_TYPE>& depth_frame_v1,
	const std::vector<DEPTH_TYPE>& depth_frame_v2,
	const int depth_width,
	const int depth_height,
	const int gaussian_width,
	const double sigma_i,
	const double sigma_v,
	const double beta_v1,
	const double beta_v2,
	const int niter,
	const char *pre);