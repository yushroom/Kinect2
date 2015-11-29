#pragma once
#include <vector>
#include <codex_math.h>
#include <functional>
#include <math.h>

typedef codex::math::prob::random_number_generator<float> RNG;
struct gaussian_pair{
	float mu, sigma;
	gaussian_pair(){}
	gaussian_pair(float Mu, float Sigma) :mu(Mu), sigma(Sigma){}
};


template <typename T>
T inline gaussian_blur(
	const std::vector<T>&	image,
	const int				x,
	const int				y,
	const int				width,
	const int				height,
	const int				gaussian_width,
	const T					sigma
	)
{
	const int half_gaussian_width = (gaussian_width - 1) / 2;
	T sum = static_cast<T>(0);
	T weight = static_cast<T>(0);
	for (int i = 0, px = x - half_gaussian_width; i < gaussian_width; ++i, ++px)
		for (int j = 0, py = y - half_gaussian_width; j < gaussian_width; ++j, ++py) {
			if (px < 0 || px >= width || py < 0 || py >= height) continue;
			double dist_coor_sqr = (px - x)*(px - x) + (py - y)*(py - y);
			double exponential = -dist_coor_sqr / (2 * sigma*sigma);
			double local_weight = exp(exponential);
			weight += local_weight;
			sum += local_weight * image[px+py*width];
		}
	return static_cast<T>(sum / weight);
}


/*
generate a depthmap where depth[x, y] = base + gaussian(sigma) + x*bias
*/

template <typename T>
void inline gen_gaussian_image(
    std::vector<T>&     result,
	RNG&				rng,
	const float			sigma,
	const int			gaussian_width,
	const float			gaussian_sigma,
    const int           width,
    const int           height,
	std::function<T(int, int)> original_pixel_at) 
{
    result.resize(height * width);
	std::vector<float> gaussian(width*height, 0.f), gaussian_copy(width*height, 0.f);
	for (size_t i = 0; i < width*height; i += 2) {
		float a, b;
		codex::math::prob::normal_rng_pair(a, b, rng);
		int x = i%width;
		gaussian[i] = a;
		if (i + 1 < width*height)
			gaussian[i + 1] = b;
	}
	for (int i = 0; i < height; ++i)
		for (int j = 0; j < width; ++j) {
			gaussian_copy[i*width + j] = gaussian_blur(
				gaussian,
				j, i,
				width, height,
				gaussian_width,
				gaussian_sigma);
		}

    for (size_t i = 0; i < width*height; ++i) {
        int x = i % width;
		int y = i / width;
		result[i] = static_cast<T>(gaussian_copy[i]*sigma + original_pixel_at(x, y));
    }
}