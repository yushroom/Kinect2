#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "utils.hpp"
#include "geo_utils.h"
#include "dump_utils.h"

using namespace std;


#define WIDTH 640
#define HEIGHT 480

#define V1_FX (587.088)
#define V1_FY (585.688)
#define V1_CX (321.198)
#define V1_CY (232.202)

#define V2_RESIZED_FX (415.035)
#define V2_RESIZED_FY (413.996)
#define V2_RESIZED_CX (325.237)
#define V2_RESIZED_CY (234.963)

#if 0
#define EXTRINSICS_FILE_PATH ".\\extrinsic_IR1IR2_1202.yml"
#define INTRINSICS_FILE_PATH ".\\intrinsics_IR1IR2_1202.yml"
#define TABLE1 ".\\table1_zfx.txt"
#define TABLE2 ".\\table2_zfx.txt"
#else
#define EXTRINSICS_FILE_PATH "D:\\yyk\\image\\Calibration_1202\\extrinsic_IR1IR2_1202.yml"
#define INTRINSICS_FILE_PATH "D:\\yyk\\image\\Calibration_1202\\intrinsics_IR1IR2_1202.yml"
#define TABLE1 "D:\\yyk\\image\\table1_zfx.txt"
#define TABLE2 "D:\\yyk\\image\\table2_zfx.txt"
#endif 

struct GlobalParm {
	cv::Mat R;
	cv::Mat T;
	UndistortionTable undistortLookupTable[2];
};

GlobalParm globalParam;



void process(
	const string& depth1_path,
	const string& depth2_path,
	const string& prefix,
	const bool save_temp_file)
{
	RawDepth depth_pixels[2];
	FloatDepth depth_und[2];
	readRawDepth(depth1_path, depth_pixels[0], WIDTH, HEIGHT);
	readRawDepth(depth2_path, depth_pixels[1], 512, 424);

	//calc_and_dump_normal_map_and_shading(depth_pixels[0], WIDTH, HEIGHT, V1_FX, V1_FY, V1_CX, V1_CY);

	// dump normal map
	{
		auto position = calc_points_from_depth_image(depth_pixels[0], WIDTH, HEIGHT, V1_FX, V1_FY, V1_CX, V1_CY);
		auto normal_map = calc_normal_map(position, WIDTH, HEIGHT, V1_FX, V1_FY, V1_CX, V1_CY);
		dump_normal_map(normal_map, WIDTH, HEIGHT, prefix + "-V1-normal.bmp");
		dump_shading(normal_map, position, WIDTH, HEIGHT, prefix + "-V1-shading.bmp");

		dump_point_cloud(position, (prefix + "-V1.ply").c_str());
	}
	{
		int w = 512, h = 424;
		auto position = calc_points_from_depth_image(depth_pixels[1], w, h, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
		auto normal_map = calc_normal_map(position, w, h, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
		dump_normal_map(normal_map, w, h, prefix + "-V2-normal.bmp");
		dump_shading(normal_map, position, w, h, prefix + "-V2-shading.bmp");

		dump_point_cloud(position, (prefix + "-V2.ply").c_str());
	}

	shiftDepth(depth_pixels[0], WIDTH, HEIGHT);

	cv::Mat img_depth[2];
	//img_depth[0] = ToCVImage(depth_pixels[0], WIDTH, HEIGHT);
	//img_depth[1] = ResizeAndToCVImage(depth_pixels[1], WIDTH, HEIGHT);
	//depth_pixels[1] = matToVector<UINT16>(img_depth[1]);
	FloatDepth temp = resize(depth_pixels[1], 512, 424, WIDTH, HEIGHT);

	//write depth_pixels[1] as bmp to see if 

	myUndistort(depth_pixels[0], depth_und[0], globalParam.undistortLookupTable[0], WIDTH, HEIGHT);
	myUndistort(temp,			 depth_und[1], globalParam.undistortLookupTable[1], WIDTH, HEIGHT);

	{
		int w = WIDTH, h = HEIGHT;
		auto position = calc_points_from_depth_image(depth_und[0], w, h, V1_FX, V1_FY, V1_CX, V1_CY);
		auto normal_map = calc_normal_map(position, w, h, V1_FX, V1_FY, V1_CX, V1_CY);
		dump_normal_map(normal_map, w, h, prefix + "-V2-und-normal.bmp");
		dump_shading(normal_map, position, w, h, prefix + "-V2-und-shading.bmp");
		dump_point_cloud(position, (prefix + "-V1-und.ply").c_str());
	}
	{
		int w = WIDTH, h = HEIGHT;
		auto position = calc_points_from_depth_image(depth_und[1], WIDTH, HEIGHT, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
		auto normal_map = calc_normal_map(position, w, h, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
		dump_normal_map(normal_map, w, h, prefix + "-V2-und-normal.bmp");
		dump_shading(normal_map, position, w, h, prefix + "-V2-und-shading.bmp");
		dump_point_cloud(position, (prefix + "-V2-und.ply").c_str());
	}

	Mat points_1 = calcPointsFromDepthMap(depth_und[0], WIDTH, HEIGHT, V1_FX, V1_FY, V1_CX, V1_CY);

	// V2 -> V1
	float fx, fy, cx, cy;
	fx = V2_RESIZED_FX;
	fy = V2_RESIZED_FY;
	cx = V2_RESIZED_CX;
	cy = V2_RESIZED_CY;

	// pack R, T
	Mat trans = Mat::zeros(3, 3, CV_32F);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			trans.at<float>(i, j) = globalParam.R.at<double>(i, j);
	Mat points_1_to_2 = trans * points_1;
	const double tx = globalParam.T.at<double>(0) * 10;
	const double ty = globalParam.T.at<double>(1) * 10;
	const double tz = globalParam.T.at<double>(2) * 10;

	FloatDepth depth_2_to_1(WIDTH * HEIGHT, DEPTH_INVALID);

	for (int j = 0; j < HEIGHT; ++j)
	{
		for (int i = 0; i < WIDTH; ++i)
		{
			int idx = j * WIDTH + i;

			// (x y z) in camera2
			float x1, y1, z1;
			x1 = points_1.at<float>(0, idx);
			y1 = points_1.at<float>(1, idx);
			z1 = points_1.at<float>(2, idx);

			// (x y z) in camera1
			float x2, y2, z2;
			x2 = points_1_to_2.at<float>(0, idx) + tx;
			y2 = points_1_to_2.at<float>(1, idx) + ty;
			z2 = points_1_to_2.at<float>(2, idx) + tz;

			if (!VALID_DEPTH_TEST(z2)) {
				depth_2_to_1[idx] = DEPTH_INVALID;
			}

			float u2, v2;
			u2 = (fx * x2 / z2 + cx - 0.5f);
			v2 = (fy * y2 / z2 + cy - 0.5f);

			depth_2_to_1[idx] = bilinear(u2, v2, depth_und[1], WIDTH, HEIGHT, true, true);
		}
	}

	// dump result

	cv::imwrite(prefix + "-1-diff-(2to1).bmp", diffDepth(depth_und[0], depth_2_to_1, WIDTH, HEIGHT, 200));

	saveToBin(depth_2_to_1, prefix + "-2to1.bin", WIDTH, HEIGHT);
	auto points = calc_points_from_depth_image(depth_2_to_1, WIDTH, HEIGHT, V1_FX, V1_FY, V1_CX, V1_CY);
	auto normals = calc_normal_map(points, WIDTH, HEIGHT, V1_FX, V1_FY, V1_CX, V1_CY);
	dump_normal_map(normals, WIDTH, HEIGHT, prefix + "-2to1-normal.bmp");
	dump_shading(normals, points, WIDTH, HEIGHT, prefix + "-2to1-shaing.bmp");

	dump_normalized_image<float>(depth_2_to_1, (prefix + "-2to1.bmp").c_str(), WIDTH, HEIGHT, 1000, 1250);
	dump_point_cloud(points, (prefix + "-2to1.ply").c_str());
}


int main(int argc, char* argv[])
{
	string path_file = argc >= 2 ? argv[1] : R"(D:\cvpr\10Objects\10-selected\path.txt)";
	//string path_file = argc >= 2 ? argv[1] : R"(D:\cvpr\synthetic\path.txt)";
	//string extrinsic_filename = argc >= 3 ? argv[2] : EXTRINSICS_FILE_PATH;

	{
		const char * extrinsic_filename = EXTRINSICS_FILE_PATH;
		cv::FileStorage fs;
		fs.open(extrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
		}
		fs["R"] >> globalParam.R;
		fs["T"] >> globalParam.T;
	}

	ReadCalibratedUndistortionTable(globalParam.undistortLookupTable[0], WIDTH, HEIGHT, TABLE1);
	ReadCalibratedUndistortionTable(globalParam.undistortLookupTable[1], WIDTH, HEIGHT, TABLE2);

	ifstream fin(path_file);
	string str;
	vector<string> image_path_list;
	while (fin >> str) {
		image_path_list.push_back(str);
	}
	while (image_path_list.size() % 2 != 0) {
		image_path_list.pop_back();
	}
	cout << "image path list: " << image_path_list.size() << endl;

	bool save_temp_file = false;

	for (int i = 0; i < image_path_list.size() / 2; ++i) {
		const string& depth1_path = image_path_list[i * 2];
		const string& depth2_path = image_path_list[i * 2 + 1];
		string prefix = depth1_path.substr(0, depth1_path.find("-a.bin"));
		process(depth1_path, depth2_path, prefix, save_temp_file);
	}

	return 0;
}