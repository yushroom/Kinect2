#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

#include "auxiliar.h"
#include "fit_gaussian.h"
#include <cassert>

int main(int argc,char **argv)
{    
    assert(argc == 3);
	printf("%d\n",sizeof(TCHAR));
	//Parameters_T *params = readIniFile("../depthFit.ini");
    Parameters_T *params = readIniFile(argv[1]);

	std::vector<std::vector<DEPTH_TYPE>>v1_depths;
	std::vector<std::vector<DEPTH_TYPE>>v2_depths;
	std::vector<std::vector<DEPTH_TYPE>>ret_depths;

	getBinFromefile(params->v1_source_path,v1_depths,params->width,params->height,params->frames);
	getBinFromefile(params->v2_source_path, v2_depths, params->width, params->height, params->frames);

	const int		gaussian_width = 5;
	const double	sigma_i = 2.0;
	const double	sigma_v = 10.0;
	const double	simga1 = 4;
	const double	sigma2 = 1.4;
	//const double	beta1 = 1.0 / simga1 / simga1;
	//const double	beta2 = 1.0 / sigma2 / sigma2;
    std::function<float(float)> beta1 = [](float depth_mm) ->float { float sigma = 0.0046f*depth_mm-2.1476f; return 1.0f / sigma / sigma; };
    std::function<float(float)> beta2 = [](float depth_mm) ->float { float sigma = 0.0006f*depth_mm+1.0001f; return 1.0f / sigma / sigma; }; 
	const int		niter = 10;
	//const char		pre[] = "D:\\cvpr\\desk\\fit_gaussian";
    //const char		pre[] = R"(D:\cvpr\synthetic_depth_image\fit_gaussian)";
    const char* pre = argv[2];
	
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


	MergeBinFromFile(std::string(pre) + "\\ret.bin",ret_depths);
	return 0;
}