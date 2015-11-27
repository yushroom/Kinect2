#pragma once
#include <vector>
#include <codex_math.h>

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
    const T             base,
    const float         bias,
	const float			sigma,
    const int           width,
    const int           height) 
{
    codex::math::prob::random_number_generator<float> rng;
    result.resize(height * width);
    for (size_t i = 0; i < width*height; i+=2) {
        float a, b;
        codex::math::prob::normal_rng_pair(a, b, rng);
        int x = i%width;
        result[i] = base + sigma*a + x*bias;
        if (i+1 < width*height)
            result[i+1] = static_cast<T>(base + sigma*b + (i+1)%width*bias);
    }
}