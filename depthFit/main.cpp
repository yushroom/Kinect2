#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

#include "auxiliar.h"
#include "fit_gaussian.h"

struct Parameters_T{
	int width, height,frames;
	std::string v1_source_path;
	std::string v2_source_path;
	std::string output_folder_path;

};


Parameters_T *get_arguments(int argc, char **argv) {
	if (argc != 7) {
		printf("usage: depthFit.exe $v1_sources_path &v2_sources_path $output_folder_path $width $height frames\n");
		exit(-1);
	}
	Parameters_T *params = new Parameters_T();
	params->v1_source_path = std::string(argv[1]);
	params->v2_source_path = std::string(argv[2]);
	params->output_folder_path = std::string(argv[3]);
	params->width = atoi(argv[4]);
	params->height = atoi(argv[5]);
	params->frames = atoi(argv[6]);
	return params;
}



int main(int argc,char **argv){
	
	Parameters_T *params = get_arguments(argc,argv);
	
	std::vector<std::vector<DEPTH_TYPE>>v1_depths;
	std::vector<std::vector<DEPTH_TYPE>>v2_depths;
	std::vector<std::vector<DEPTH_TYPE>>ret_depths;

	getBinFromefile(params->v1_source_path,v1_depths,params->width,params->height,params->frames);
	getBinFromefile(params->v2_source_path, v2_depths, params->width, params->height, params->frames);

	const int		gaussian_width = 5;
	const double	sigma_i = 2.0;
	const double	sigma_v = 1.0;
	const double	simga1 = 4;
	const double	sigma2 = 15;
	const double	beta1 = 1.0 / simga1 / simga1;
	const double	beta2 = 1.0 / sigma2 / sigma2;
	const int		niter = 100;
	const char		pre[] = "D:\\cvpr\\desk\\fit_gaussian";
	
	ret_depths.resize(params->frames);
	for (int i = 0; i < params->frames; i++) {
		ret_depths[i] = fit_depth(
			v1_depths[i],
			v2_depths[i],
			params->width,
			params->height,
			gaussian_width,
			sigma_i,
			sigma_v,
			beta1,
			beta2,
			niter,
			pre
			);
	}
	MergeBinFromFile(params->output_folder_path + "\\ret.bin",ret_depths);
	return 0;
}