#pragma once

#include <vector>
#include <utility>
#include <functional>
#ifndef DEPTH_TYPE
typedef float DEPTH_TYPE;
#endif

typedef float FLOAT;
//typedef double FLOAT;

#define V1_FX (587.088f)
#define V1_FY (585.688f)
#define V1_CX (321.198f)
#define V1_CY (232.202f)
#define V2_RESIZED_FX (415.035f)
#define V2_RESIZED_FY (413.996f)
#define V2_RESIZED_CX (325.237f)
#define V2_RESIZED_CY (234.963f)

std::vector<DEPTH_TYPE> fit_depth(
	const std::vector<DEPTH_TYPE>& depth_frame_v1,
	const std::vector<DEPTH_TYPE>& depth_frame_v2,
	const int depth_width,
	const int depth_height,
	const int gaussian_width,
	const FLOAT sigma_i,
	const FLOAT sigma_v,
	const FLOAT beta_v1,
	const FLOAT beta_v2,
	const int niter,
    const char* pre
    );

std::vector<DEPTH_TYPE> fit_depth(
	const std::vector<DEPTH_TYPE>& depth_frame_v1,
	const std::vector<DEPTH_TYPE>& depth_frame_v2,
	const int depth_width,
	const int depth_height,
	const int gaussian_width,
	const FLOAT sigma_i,
	const FLOAT sigma_v,
	std::function<float(float)> beta_v1,
	std::function<float(float)> beta_v2,
	const int niter,
    const char* pre
    );