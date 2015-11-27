#pragma once
#include <vector>
#include <codex_math.h>
#include <functional>

typedef codex::math::prob::random_number_generator<float> RNG;
struct gaussian_pair{
	float mu, sigma;
	gaussian_pair(){}
	gaussian_pair(float Mu, float Sigma) :mu(Mu), sigma(Sigma){}
};

/*
generate a depthmap where depth[x, y] = base + gaussian(sigma) + x*bias
*/


template <typename T>
void inline gen_gaussian_image(
    std::vector<T>&     result,
	RNG&				rng,
	const float			sigma,
    const int           width,
    const int           height,
	std::function<T(int, int)> original_pixel_at) 
{
    result.resize(height * width);
    for (size_t i = 0; i < width*height; i+=2) {
        float a, b;
        codex::math::prob::normal_rng_pair(a, b, rng);
        int x = i%width;
		result[i] = static_cast<T>(sigma*a + original_pixel_at(i%width, i / width));
        if (i+1 < width*height)
            result[i+1] = static_cast<T>(sigma*b + original_pixel_at((i+1)%width, (i+1)/width));
    }
}