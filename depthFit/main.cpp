#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

#include "auxiliar.h"
#include "fit_gaussian.h"
#include <cassert>
#include "dump_utils.h"
#include "geo_utils.h"

int main(int argc,char **argv) 
{    
    //assert(argc >= 3);
	//Parameters_T *params = readIniFile("../depthFit.ini");
	Parameters_T *params = argc >= 3 ? readIniFile(argv[1]) : readIniFile(R"(D:\cvpr\10Objects\depthFit_0.ini)");
	const char* pre = argc >=3 ?  argv[2] : R"(D:\cvpr\10Objects\testOut)";

	std::vector<std::vector<DEPTH_TYPE>>v1_depths;
	std::vector<std::vector<DEPTH_TYPE>>v2_depths;
	std::vector<std::vector<DEPTH_TYPE>>ret_depths;

	getBinFromefile(params->v1_source_path,v1_depths,params->width,params->height,params->frames);
	getBinFromefile(params->v2_source_path, v2_depths, params->width, params->height, params->frames);

	int	gaussian_width = 5;
	FLOAT	sigma_i = 2.0f;
	FLOAT	sigma_v = 10.0f;
	//const FLOAT	simga1 = 4.f;
	//const FLOAT	sigma2 = 1.4f;

	if (argc >= 6) {
		gaussian_width = atoi(argv[3]);
		sigma_i = atof(argv[4]);
		sigma_v = atof(argv[5]);
	}

	//const double	beta1 = 1.0 / simga1 / simga1;
	//const double	beta2 = 1.0 / sigma2 / sigma2;
    std::function<float(float)> beta1 = [](float depth_mm) ->float { float sigma = 0.0046f*depth_mm-2.1476f; return 1.0f / sigma / sigma; };
    std::function<float(float)> beta2 = [](float depth_mm) ->float { float sigma = 0.0006f*depth_mm+1.0001f; return 1.0f / sigma / sigma; }; 
	const int		niter = 10;
	//const char		pre[] = "D:\\cvpr\\desk\\fit_gaussian";
    //const char		pre[] = R"(D:\cvpr\synthetic_depth_image\fit_gaussian)";
    
	
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
	//auto points = calc_points_from_depth_image(ret_depths[0], params->width, params->height, V1_FX, V1_FY, V1_CX, V1_CY);
	//auto normals = calc_normal_map(points, params->width, params->height, V1_FX, V1_FY, V1_CX, V1_CY);
	//dump_normal_map(normals, params->width, params->height, std::string(pre) + "\\ret-normal.png");
	//dump_shading(normals, points, params->width, params->height, std::string(pre) + "\\ret-shading.png");
	return 0;
}