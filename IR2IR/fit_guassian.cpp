#include "fit_gaussian.h"

#include <cassert>
#include <math.h>

typedef std::vector<DEPTH_TYPE> DEPTH_IMAGE;

#ifndef VALID_DEPTH_TEST
const float DEPTH_INVALID = 1e30f;
#define VALID_DEPTH_TEST(a) (a < DEPTH_INVALID)
#endif

inline void diff(
	const DEPTH_IMAGE&	image1,
	const DEPTH_IMAGE&	image2,
	DEPTH_IMAGE&		result)
{
	assert(image1.size() == image2.size());
	result.resize(image1.size());
	for (int i = 0; i < image1.size(); ++i)
		result[i] = image1[i] - image2[i];
}

std::vector<GAUSSIAN_TYPE> fit_gaussian(
	const std::vector<DEPTH_TYPE>& depth_frame_v1,
	const std::vector<DEPTH_TYPE>& depth_frame_v2,
	const int depth_width,
	const int depth_height,
	const int gaussian_width,
	const double sigma_i,
	const double sigma_v)
{
	assert(depth_frame_v1.size() == depth_frame_v2.size());
	assert(depth_frame_v1.size() == depth_width*depth_height);
	DEPTH_IMAGE real_depth = depth_frame_v1;
	DEPTH_IMAGE bias_depth;
	diff(depth_frame_v1, depth_frame_v2, bias_depth);
	
	std::vector<GAUSSIAN_TYPE> result(depth_frame_v1.size());

	const int half_gaussian_width = (gaussian_width - 1) / 2;
	for (int i = 0; i < depth_height; ++i)
		for (int j = 0; j < depth_width; ++j) {
			int idx = j + i*depth_width;
			auto &p = result[idx];
			//outliers
			if (i - half_gaussian_width < 0 || i + half_gaussian_width >= depth_height
				|| j - half_gaussian_width < 0 || j + half_gaussian_width >= depth_width) {
				p = std::make_pair(0.0, 0.0);
			}
			else {
				std::vector<std::vector<double>>
					local_area_depth(gaussian_width, std::vector<double>(gaussian_width, 0.0));
				double sum_depth = 0.0;
				for (int k = i - half_gaussian_width,x=0; k <= i + half_gaussian_width; ++k,++x)
					for (int l = j - half_gaussian_width,y=0; j <= j + half_gaussian_width; ++j,++y){
						int local_idx = k + l*depth_width;
						local_area_depth[x][y] = depth_frame_v2[local_idx];
						sum_depth += local_area_depth[x][y];
					}
				double ave_depth = sum_depth / (gaussian_width*gaussian_width); 
				double sum_weight = 0.0;
				p = std::make_pair(0.0, 0.0);
				for (int x = 0, pivot=gaussian_width/2; x < gaussian_width; ++x)
					for (int y = 0; y < gaussian_width; ++y) {
						double dist_coor_sqr = (x - pivot)*(x - pivot) + (y - pivot)*(y - pivot);
						double dist_value_sqr = (local_area_depth[x][y] - ave_depth)*(local_area_depth[x][y] - ave_depth);
						double exponential = -(dist_coor_sqr / (2 * sigma_i*sigma_i) + dist_value_sqr / (2 * sigma_v*sigma_v));
						double local_weight = VALID_DEPTH_TEST(local_area_depth[x][y])?exp(exponential):0.0;
						sum_weight += local_weight;
						p.first += local_weight * local_area_depth[x][y];
						p.second = local_weight*(local_area_depth[x][y] - ave_depth)*(local_area_depth[x][y] - ave_depth);
					}
				p.first /= sum_weight;
				p.second = sqrt(p.second / sum_weight);
			}
		}
	return result;
}