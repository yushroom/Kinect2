#include "fit_gaussian.h"

#include <cassert>
#include <math.h>

typedef std::vector<DEPTH_TYPE> DEPTH_IMAGE;
struct gaussian_pair{
	double mu, sigma;
};
typedef std::vector<gaussian_pair> GAUSSIAN_IMAGE;

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

inline void fit_gaussian_helper(
	GAUSSIAN_IMAGE& result,
	const std::vector<DEPTH_TYPE>& image_frame,
	const int image_width,
	const int image_height,
	const int gaussian_width,
	const double sigma_i,
	const double sigma_v
	)
{
	assert(image_frame.size() == image_width*image_height);
	result.resize(image_frame.size());

	const int half_gaussian_width = (gaussian_width - 1) / 2;
	for (int i = 0; i < image_height; ++i)
		for (int j = 0; j < image_width; ++j) {
			int idx = j + i*image_width;
			auto &p = result[idx];
			//outliers
			if (i - half_gaussian_width < 0 || i + half_gaussian_width >= image_height
				|| j - half_gaussian_width < 0 || j + half_gaussian_width >= image_width) {
				p = { 0.0, 0.0 };
			}
			else {
				std::vector<std::vector<double>>
					local_area_value(gaussian_width, std::vector<double>(gaussian_width, 0.0));
				double sum_value = 0.0;
				for (int k = i - half_gaussian_width, x = 0; k <= i + half_gaussian_width; ++k, ++x)
					for (int l = j - half_gaussian_width, y = 0; l <= j + half_gaussian_width; ++l, ++y){
					int local_idx = l + k*image_width;
					local_area_value[x][y] = image_frame[local_idx];
					sum_value += local_area_value[x][y];
					}
				double ave_value = sum_value / (gaussian_width*gaussian_width);
				double sum_weight = 0.0;
				p = { 0.0, 0.0 };
				for (int x = 0, pivot = gaussian_width / 2; x < gaussian_width; ++x)
					for (int y = 0; y < gaussian_width; ++y) {
					double dist_coor_sqr = (x - pivot)*(x - pivot) + (y - pivot)*(y - pivot);
					double dist_value_sqr = (local_area_value[x][y] - ave_value)*(local_area_value[x][y] - ave_value);
					double exponential = -(dist_coor_sqr / (2 * sigma_i*sigma_i) + dist_value_sqr / (2 * sigma_v*sigma_v));
					double local_weight = VALID_DEPTH_TEST(local_area_value[x][y]) ? exp(exponential) : 0.0;
					sum_weight += local_weight;
					p.mu += local_weight * local_area_value[x][y];
					p.sigma = local_weight*(local_area_value[x][y] - ave_value)*(local_area_value[x][y] - ave_value);
					}
				p.mu /= sum_weight;
				p.sigma = sqrt(p.sigma / sum_weight);
			}
		}
}

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
	const double beta_d,
	int niter)
{
	assert(depth_frame_v1.size() == depth_frame_v2.size());
	assert(depth_frame_v1.size() == depth_width*depth_height);
	DEPTH_IMAGE result = depth_frame_v1;
	DEPTH_IMAGE bias;
	GAUSSIAN_IMAGE gd, gb;
	while (niter--) {
		diff(depth_frame_v2, result, bias);
		fit_gaussian_helper(
			gd,
			result,
			depth_width,
			depth_height,
			gaussian_width,
			sigma_i,
			sigma_v
			);
		fit_gaussian_helper(
			gb,
			bias,
			depth_width,
			depth_height,
			gaussian_width,
			sigma_i,
			sigma_v
			);

		for (int i = 0; i < depth_height; ++i)
			for (int j = 0; j < depth_width; ++j) {
			int idx = j + i*depth_width;
			result[idx] = 
				(
				beta_v1*depth_frame_v1[idx]
				+beta_v2*depth_frame_v2[idx]
				+beta_d*gd[idx].mu
				-beta_v2*beta_d/(beta_v2+beta_d)*gb[idx].mu
				-beta_v2*beta_v2/(beta_v2+beta_d)*depth_frame_v2[idx]
				) 
				/ (beta_v1 + beta_v2 + beta_d - beta_v2*beta_v2 / (beta_v2 + beta_d));
			}
	}
	return result;
}
/*
std::vector<GAUSSIAN_TYPE> fit_gaussian(
	const std::vector<DEPTH_TYPE>& depth_frame_v1,
	const std::vector<DEPTH_TYPE>& depth_frame_v2,
	const int depth_width,
	const int depth_height,
	const int gaussian_width,
	const double sigma_i,
	const double sigma_v,
	const int niter)
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
					for (int l = j - half_gaussian_width,y=0; l <= j + half_gaussian_width; ++l,++y){
						int local_idx = l + k*depth_width;
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
*/