#pragma once

#include <vector>
#include <utility>
#include <functional>
#ifndef DEPTH_TYPE
typedef float DEPTH_TYPE;
#endif

#define V1_FX (587.088)
#define V1_FY (585.688)
#define V1_CX (321.198)
#define V1_CY (232.202)
#define V2_RESIZED_FX (415.035)
#define V2_RESIZED_FY (413.996)
#define V2_RESIZED_CX (325.237)
#define V2_RESIZED_CY (234.963)

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
    const char* pre
    );

std::vector<DEPTH_TYPE> fit_depth(
	const std::vector<DEPTH_TYPE>& depth_frame_v1,
	const std::vector<DEPTH_TYPE>& depth_frame_v2,
	const int depth_width,
	const int depth_height,
	const int gaussian_width,
	const double sigma_i,
	const double sigma_v,
	std::function<float(float)> beta_v1,
	std::function<float(float)> beta_v2,
	const int niter,
    const char* pre
    );