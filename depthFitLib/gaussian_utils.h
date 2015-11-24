#pragma once

struct gaussian_pair{
	double mu, sigma;
	gaussian_pair(){}
	gaussian_pair(double Mu, double Sigma) :mu(Mu), sigma(Sigma){}
};