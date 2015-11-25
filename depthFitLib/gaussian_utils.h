#pragma once

struct gaussian_pair{
	float mu, sigma;
	gaussian_pair(){}
	gaussian_pair(float Mu, float Sigma) :mu(Mu), sigma(Sigma){}
};