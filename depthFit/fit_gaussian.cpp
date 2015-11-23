#include "fit_gaussian.h"

#include <cassert>
#include <math.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

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

inline std::vector<float> test_gaussian_sigma(
	const float sigma,
	const int half_gaussian_width) {
	std::vector<float> result;
	for (int i = 0; i < half_gaussian_width; ++i) {
		float ex = -i*i / (2 * sigma*sigma);
		result.push_back(exp(ex));
	}
	return result;
}

inline void fit_gaussian_helper(
	GAUSSIAN_IMAGE& result,
	std::vector<bool>& mask,
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
	mask.assign(image_frame.size(), true);

	const int half_gaussian_width = (gaussian_width - 1) / 2;
	for (int i = 0; i < image_height; ++i)
		for (int j = 0; j < image_width; ++j) {
			int idx = j + i*image_width;
			auto &p = result[idx];
			int nvalid = 0;
			//outliers
			if (i - half_gaussian_width < 0 || i + half_gaussian_width >= image_height
				|| j - half_gaussian_width < 0 || j + half_gaussian_width >= image_width) {
				p.mu = 0.0;
				p.sigma = 0.0;
			}
			else {
				std::vector<std::vector<double>>
					local_area_value(gaussian_width, std::vector<double>(gaussian_width, 0.0));
				for (int k = i - half_gaussian_width, x = 0; k <= i + half_gaussian_width; ++k, ++x)
					for (int l = j - half_gaussian_width, y = 0; l <= j + half_gaussian_width; ++l, ++y){
					int local_idx = l + k*image_width;
					local_area_value[x][y] = image_frame[local_idx];
					}
				double sum_weight = 0.0;
				p.mu = 0.0;
				for (int x = 0, pivot = gaussian_width / 2; x < gaussian_width; ++x)
					for (int y = 0; y < gaussian_width; ++y) {
					double dist_coor_sqr = (x - pivot)*(x - pivot) + (y - pivot)*(y - pivot);
					double dist_value_sqr = (local_area_value[x][y] - image_frame[idx])*(local_area_value[x][y] - image_frame[idx]);
					double exponential = -(dist_coor_sqr / (2 * sigma_i*sigma_i) + dist_value_sqr / (2 * sigma_v*sigma_v));
					double local_weight = VALID_DEPTH_TEST(local_area_value[x][y]) ? exp(exponential) : 0.0;
					if (local_weight != 0.0) nvalid++;
					sum_weight += local_weight;
					p.mu += local_weight * local_area_value[x][y];
					}
				p.mu /= sum_weight;

				p.sigma = 0.0;
				sum_weight = 0.0;
				for (int x = 0, pivot = gaussian_width / 2; x < gaussian_width; ++x)
					for (int y = 0; y < gaussian_width; ++y) {
					double dist_coor_sqr = (x - pivot)*(x - pivot) + (y - pivot)*(y - pivot);
					double dist_value_sqr = (local_area_value[x][y] - image_frame[idx])*(local_area_value[x][y] - image_frame[idx]);
					double exponential = -(dist_coor_sqr / (2 * sigma_i*sigma_i) + dist_value_sqr / (2 * sigma_v*sigma_v));
					double local_weight = VALID_DEPTH_TEST(local_area_value[x][y]) ? exp(exponential) : 0.0;
					sum_weight += local_weight;
					//if (i == 3 && j == 490) {
					//	printf("%g %g %g %g\n", local_area_value[x][y], local_weight, p.mu, p.sigma);
					//}
					p.sigma += local_weight*(local_area_value[x][y] - p.mu)*(local_area_value[x][y] - p.mu);
					}
				p.sigma = sqrt(p.sigma / sum_weight); 
				if (nvalid < gaussian_width*gaussian_width / 2 || p.sigma == 0.0) mask[idx] = false;
				//if (p.sigma == 0.0 && false)	{
				//	printf("%d %d %g\n", i, j, sum_weight);
				//	for (int x = 0; x < local_area_value.size(); ++x)
				//		for (int y = 0; y < local_area_value[0].size(); ++y)
				//			printf("%g%c", local_area_value[x][y], y == local_area_value[0].size() - 1 ? '\n' : ' ');
				//	getchar();

				//}
			}
		}
}

void write_normalized_vector(const std::vector<DEPTH_TYPE> &img, const std::string& filename, int w, int h)
{
	//auto vmax = *std::max_element(img.begin(), img.end());
	//auto vmin = *std::min_element(img.begin(), img.end());
	float vmax = -1, vmin = DEPTH_INVALID;
	for (int i = 1; i < w * h; ++i) {
		if (img[i] != 0 && img[i] < vmin) vmin = img[i];
		if (VALID_DEPTH_TEST(img[i]) && img[i] > vmax) vmax = img[i];
	}

	//std::cout << "[write_normalized_vector] max = " << vmax << "  min = " << vmin << std::endl;

	Mat output = Mat::zeros(h, w, CV_8U);
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
		int idx = x + y * w;
		float v = float(img[idx] - vmin) / float(vmax - vmin);
		output.at<unsigned char>(y, x) = unsigned char(v * 255);
		}

	imwrite(filename, output);
}

inline void seperate_gaussian(const GAUSSIAN_IMAGE& image, std::vector<float> &mu, std::vector<float> &sigma) {
	mu.clear();
	sigma.clear();
	for (auto x : image)
		mu.push_back(x.mu), sigma.push_back(x.sigma);
}

inline double l2norm(const DEPTH_IMAGE& image) {
	double sum_sqr = 0.0;
	for (auto x : image) sum_sqr += x*x;
	printf("L2NORM:%g\n", sum_sqr);
	return sqrt(sum_sqr);
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
	const int niter,
	const char* pre)
{
/*
	auto x = test_gaussian_sigma(1, 5);
	for (int i = 0; i < x.size(); ++i)
		printf("%g%c", x[i], i == x.size() - 1 ? '\n' : ' ');
	getchar();
	*/
	assert(depth_frame_v1.size() == depth_frame_v2.size());
	assert(depth_frame_v1.size() == depth_width*depth_height);
	DEPTH_IMAGE result = depth_frame_v1;
	DEPTH_IMAGE bias, last;
	std::vector<bool> maskd, maskb;
	GAUSSIAN_IMAGE gd, gb;
#ifdef DEBUG_DIFF
	FILE *rfp = nullptr;
	if (pre) {
		char path[_MAX_PATH];
		sprintf_s(path, "%s//diff.txt", pre);
		fopen_s(&rfp, path, "w");
	}
#endif
	for (int it = 1; it <= niter; ++it) {
		printf("%d\n", it);
		diff(depth_frame_v2, result, bias);
		last = result;
		fit_gaussian_helper(
			gd,
			maskd,
			result,
			depth_width,
			depth_height,
			gaussian_width,
			sigma_i,
			sigma_v
			);
		fit_gaussian_helper(
			gb,
			maskb,
			bias,
			depth_width,
			depth_height,
			gaussian_width,
			sigma_i,
			sigma_v
			);
#ifdef DEBUG_DIFF
		if (pre) {
			char d_path[_MAX_PATH], b_path[_MAX_PATH];
			char mud_path[_MAX_PATH], mub_path[_MAX_PATH];
			char sigmad_path[_MAX_PATH], sigmab_path[_MAX_PATH];
			sprintf_s(d_path, "%s//%d-d.bmp", pre, it);
			sprintf_s(b_path, "%s//%d-b.bmp", pre, it);

			sprintf_s(mud_path, "%s//%d-mud.bmp", pre, it);
			sprintf_s(sigmad_path, "%s//%d-sigmad.bmp", pre, it);

			sprintf_s(mub_path, "%s//%d-mub.bmp", pre, it);
			sprintf_s(sigmab_path, "%s//%d-sigmab.bmp", pre, it);

			write_normalized_vector(result, std::string(d_path), depth_width, depth_height);
			printf_s("max: %g\n", *std::max_element(result.begin(), result.end()));
			write_normalized_vector(bias, std::string(b_path), depth_width, depth_height);

			std::vector<float> mud, sigmad, mub, sigmab;
			seperate_gaussian(gd, mud, sigmad);
			seperate_gaussian(gb, mub, sigmab);

			write_normalized_vector(mud, std::string(mud_path), depth_width, depth_height);
			write_normalized_vector(sigmad, std::string(sigmad_path), depth_width, depth_height);

			write_normalized_vector(mub, std::string(mub_path), depth_width, depth_height);
			write_normalized_vector(sigmab, std::string(sigmab_path), depth_width, depth_height);
		}
#endif
		const int half_gaussian_width = (gaussian_width - 1) / 2;
		for (int i = 0; i < depth_height; ++i)
			for (int j = 0; j < depth_width; ++j) {
			int idx = j + i*depth_width;
			if (i - half_gaussian_width < 0 || i + half_gaussian_width >= depth_height
				|| j - half_gaussian_width < 0 || j + half_gaussian_width >= depth_width)
				continue;
			const double beta_d = 1.0 / (gd[idx].sigma*gd[idx].sigma);
			if (maskb[idx] && maskd[idx])
			result[idx] =
				(beta_v1*depth_frame_v1[idx] + beta_d*gd[idx].mu + beta_v2*beta_d / (beta_v2 + beta_d)*(depth_frame_v2[idx] - gb[idx].mu))
				/ (beta_v1 + beta_d + beta_v2*beta_d / (beta_v2+beta_d));
			}
#ifdef DEBUG_DIFF
		DEPTH_IMAGE delta;
		diff(last, result, delta);

		printf_s("min delta:%g max delta:%g\n", *std::min_element(delta.begin(), delta.end()), *std::max_element(delta.begin(), delta.end()));
		std::cout << l2norm(delta) << std::endl;

		if (rfp)
			fprintf_s(rfp, "%f\n", l2norm(delta));
#endif	
	}
#ifdef DEBUG_DIFF
	if (rfp) fclose(rfp);
#endif
	return result;
}
