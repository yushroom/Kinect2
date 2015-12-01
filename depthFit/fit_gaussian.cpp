#include "fit_gaussian.h"

#include <cassert>
#include <math.h>
#include <iostream>
#include <algorithm>

#include <dump_utils.h>
#include <gaussian_utils.h>	  
#include <geo_utils.h>


#define V2_RESIZED_FX (415.035)
#define V2_RESIZED_FY (413.996)
#define V2_RESIZED_CX (325.237)
#define V2_RESIZED_CY (234.963)

typedef std::vector<DEPTH_TYPE> DEPTH_IMAGE;
typedef std::vector<gaussian_pair> GAUSSIAN_IMAGE;

#ifndef VALID_DEPTH_TEST
const float DEPTH_INVALID = 1e30f;
#define VALID_DEPTH_TEST(a) (a < DEPTH_INVALID && a != 0.f)
#endif

inline void diff(
	const DEPTH_IMAGE&	        image1,
	const DEPTH_IMAGE&	        image2,
    const std::vector<bool>&    mask,
	DEPTH_IMAGE&		        result)
{
	assert(image1.size() == image2.size());
	result.resize(image1.size());
	for (int i = 0; i < image1.size(); ++i)
		result[i] = mask[i]?(image1[i] - image2[i]):static_cast<DEPTH_TYPE>(0);
}

inline void get_mask(
    const DEPTH_IMAGE&  image,
    std::vector<bool>& mask)
{
    mask.resize(image.size(), true);
    for (int i=0; i<image.size(); ++i) 
        if (!VALID_DEPTH_TEST(image[i]))
            mask[i] = false;
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
	GAUSSIAN_IMAGE&                 result,
	std::vector<bool>&              mask,
	const std::vector<DEPTH_TYPE>&  image_frame,
    const std::vector<bool>&        mask_in,
	const int                       image_width,
	const int                       image_height,
	const int                       gaussian_width,
	const double                    sigma_i,
	const double                    sigma_v
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
					double local_weight = VALID_DEPTH_TEST(local_area_value[x][y]) && mask_in[idx] ? exp(exponential) : 0.0;
				    //if (j == 216 && i == 203) {
        //                printf("%g %g %d\n", exponential, local_weight, mask_in[idx]);
        //                getchar();
        //            }
                    
                    if (local_weight != 0.0) nvalid++;
					sum_weight += local_weight;
					p.mu += local_weight * local_area_value[x][y];
					}
				p.mu /= sum_weight;
                //if (j == 216 && i == 203) {
                //    printf("%g %g\n", p.mu, sum_weight);
                //    getchar();
                //}
				p.sigma = 0.0;
				sum_weight = 0.0;
				for (int x = 0, pivot = gaussian_width / 2; x < gaussian_width; ++x)
					for (int y = 0; y < gaussian_width; ++y) {
					double dist_coor_sqr = (x - pivot)*(x - pivot) + (y - pivot)*(y - pivot);
					double dist_value_sqr = (local_area_value[x][y] - image_frame[idx])*(local_area_value[x][y] - image_frame[idx]);
					double exponential = -(dist_coor_sqr / (2 * sigma_i*sigma_i) + dist_value_sqr / (2 * sigma_v*sigma_v));
					double local_weight = VALID_DEPTH_TEST(local_area_value[x][y]) ? exp(exponential) : 0.0;
					sum_weight += local_weight;
					p.sigma += local_weight*(local_area_value[x][y] - p.mu)*(local_area_value[x][y] - p.mu);
					}
				p.sigma = sqrt(p.sigma / sum_weight); 
				if (nvalid < gaussian_width*gaussian_width / 2 || p.sigma == 0.0) mask[idx] = false;
			}
		}
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
	std::function<float(float)> beta_v1,
	std::function<float(float)> beta_v2,
	const int niter,
    const char* pre
    )
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
	std::vector<bool> maskd, maskb, mask;
	GAUSSIAN_IMAGE gd, gb;
    get_mask(depth_frame_v2, mask);
#define DEBUG_DIFF
#ifdef DEBUG_DIFF         
    FILE *rfp = nullptr;
	if (pre) {
		char path[_MAX_PATH];
		sprintf_s(path, "%s//diff.txt", pre);
		fopen_s(&rfp, path, "w");
	}
#endif
	for (int it = 1; it <= niter; ++it) {
		printf("%dth iter\r", it);
        get_mask(result, mask);
		diff(depth_frame_v2, result, mask, bias);
		last = result;
		fit_gaussian_helper(
			gd,
			maskd,
			result,
            mask,
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
            mask,
			depth_width,
			depth_height,
			gaussian_width,
			sigma_i,
			sigma_v
			);
#ifdef DEBUG_DIFF
        const float lb = 800.f, ub = 1200.f;
		DEPTH_IMAGE w1_image, w2_image, w3_image;
        DEPTH_IMAGE i2_mub;
        std::vector<vector3b> w_image;
        w_image.resize(depth_width*depth_height);
		w1_image.resize(depth_width*depth_height);
		w2_image.resize(depth_width*depth_height);
		w3_image.resize(depth_width*depth_height);
        i2_mub.resize(depth_width*depth_height);
        char w1_path[_MAX_PATH], w2_path[_MAX_PATH], w3_path[_MAX_PATH];
        char i2_mub_path[_MAX_PATH];
        char i1_path[_MAX_PATH];
        char i2_path[_MAX_PATH];
		if (pre) {
			char d_path[_MAX_PATH], b_path[_MAX_PATH], d_bin_path[_MAX_PATH];
			char mud_path[_MAX_PATH], mub_path[_MAX_PATH];
			char sigmad_path[_MAX_PATH], sigmab_path[_MAX_PATH];
			char result_ply[_MAX_PATH];
			
			sprintf_s(i2_mub_path, "%s//%d-i2_mub.png", pre, it);
            sprintf_s(i1_path, "%s//%d-i1.png", pre, it);
            sprintf_s(i2_path, "%s//%d-i2.png", pre, it);

			sprintf_s(d_path, "%s//%d-d.png", pre, it);    
            sprintf_s(d_bin_path, "%s//%d-d.bin", pre, it);
			sprintf_s(b_path, "%s//%d-b.png", pre, it);

			sprintf_s(mud_path, "%s//%d-mud.png", pre, it);
			sprintf_s(sigmad_path, "%s//%d-sigmad.png", pre, it);

			sprintf_s(mub_path, "%s//%d-mub.png", pre, it);
			sprintf_s(sigmab_path, "%s//%d-sigmab.png", pre, it);

			sprintf_s(result_ply, "%s//%d.ply", pre, it);

			sprintf_s(w1_path, "%s//%d-w1.png", pre, it);
			sprintf_s(w2_path, "%s//%d-w2.png", pre, it);
			sprintf_s(w3_path, "%s//%d-w3.png", pre, it);
			{
				auto x = calc_points_from_depth_image(result, depth_width, depth_height,
					V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
				dump_point_cloud(x, result_ply);
			}

			dump_normalized_image(result, d_path, depth_width, depth_height, lb, ub);
            std::vector<unsigned short> result_bin;
            for (auto x:result) result_bin.push_back(x);
            dump_raw_vector(result_bin, d_bin_path);
			printf_s("max: %g\n", *std::max_element(result.begin(), result.end()));
			dump_normalized_image(bias, b_path, depth_width, depth_height, 0.f, 80.f);

			std::vector<float> mud, sigmad, mub, sigmab;
			seperate_gaussian(gd, mud, sigmad);
			seperate_gaussian(gb, mub, sigmab);

            float iter_rmse = rmse(mud, depth_frame_v1);
            //fprintf(rfp, "%g,%g\n", iter_rmse, initial_rmse);
            fprintf(rfp, "%g\n", iter_rmse);
            fflush(rfp);
            //float iter_rmse = rmse(mud, depth_frame_v1);
            //float iter_rmse_true = rmse(mud, depth_frame_v1_true);
            //fprintf(rfp, "%d,%g,%g\n", it, iter_rmse, iter_rmse_true);
            //fflush(rfp);
			
            dump_normalized_image(mud, mud_path, depth_width, depth_height, lb, ub);
			dump_normalized_image(sigmad, sigmad_path, depth_width, depth_height);

			dump_normalized_image(mub, mub_path, depth_width, depth_height, 0.f, 80.f);
			dump_normalized_image(sigmab, sigmab_path, depth_width, depth_height);
		}
#endif
		const int half_gaussian_width = (gaussian_width - 1) / 2;

#pragma omp parallel for
		for (int i = 0; i < depth_height; ++i)
			for (int j = 0; j < depth_width; ++j) {
			    int idx = j + i*depth_width;
			    if (i - half_gaussian_width < 0 || i + half_gaussian_width >= depth_height
				    || j - half_gaussian_width < 0 || j + half_gaussian_width >= depth_width)
				    continue;
			        const float beta_d = 1.0 / (gd[idx].sigma*gd[idx].sigma);
			        if (maskb[idx] && maskd[idx]) {
                        float b2 = beta_v2(depth_frame_v2[idx]);
				        float w1 = beta_v1(depth_frame_v1[idx]), w2 = beta_d, w3 = b2*beta_d / (b2 + beta_d);
				        float sum_w = w1 + w2 + w3;
				        float v1 = depth_frame_v1[idx], v2 = gd[idx].mu, v3 = depth_frame_v2[idx] - gb[idx].mu;
#ifdef DEBUG_DIFF
				        w1_image[idx] = w1/sum_w;
				        w2_image[idx] = w2/sum_w;
				        w3_image[idx] = w3/sum_w;
                        w_image[idx] = vector3b(w1/sum_w*255, w2/sum_w*255, w3/sum_w*255);
                        i2_mub[idx] = v3;
#endif
                        float result_value = (w1*v1 + w2*v2 + w3*v3) / sum_w;
                        if (!_isnanf(result_value))
				            result[idx] = result_value;
					        //(beta_v1*depth_frame_v1[idx] + beta_d*gd[idx].mu + beta_v2*beta_d / (beta_v2 + beta_d)*(depth_frame_v2[idx] - gb[idx].mu))
					        /// (beta_v1 + beta_d + beta_v2*beta_d / (beta_v2 + beta_d));
			        }

			}
#ifdef DEBUG_DIFF
		DEPTH_IMAGE delta;
		diff(last, result, mask, delta);
        //dump_normalized_image(w1_image, w1_path, 512, 424, 0.f, 1.f);
        //dump_normalized_image(w2_image, w2_path, 512, 424, 0.f, 1.f);
        //dump_normalized_image(w3_image, w3_path, 512, 424, 0.f, 1.f);
        //dump_image(w_image, w1_path, 512, 424);
        dump_normalized_image(i2_mub, i2_mub_path, depth_width, depth_height, lb, ub);
        dump_normalized_image(depth_frame_v1, i1_path, depth_width, depth_height, lb, ub);
        dump_normalized_image(depth_frame_v2, i2_path, depth_width, depth_height, lb, ub);
		printf_s("min delta:%g max delta:%g\n", *std::min_element(delta.begin(), delta.end()), *std::max_element(delta.begin(), delta.end()));
		std::cout << l2norm(delta) << std::endl;

#endif	
	}
	printf("\n");
#ifdef DEBUG_DIFF
	if (rfp) fclose(rfp);
#endif
	return result;
}
