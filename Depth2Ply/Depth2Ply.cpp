#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
using namespace std;

typedef unsigned short UINT16;
typedef UINT16 DepthType;
typedef unsigned char BYTE;

static void load_raw_depth_from_bin_file(string depth_bin_path, vector<UINT16>& out_raw_depth, int width, int height)
{
	out_raw_depth.resize(width * height, 0);
	cout << depth_bin_path << endl;
	ifstream is(depth_bin_path, ios::binary);
	if (!is) {
		cout << "Error: file " << depth_bin_path << " not found.\n";
	}
	is.read((char*)&out_raw_depth[0], width * height * sizeof(UINT16));
	is.close();
}

static void calc_points(const vector<DepthType>& depth_pixels, vector<cv::Point3f>& point_cloud, int width, int height,
	const float fovx, const float fovy)
	//inline void calc_points(const UINT16* depth_pixels, const UINT16* infrared_pixels, vector<vector3f>& point_cloud, int width, int height,
	//	const float fx, const float fy, const float cx, const float cy)
{
	//Assert(depth_pixels.size() == width * height);
	//point_cloud.resize(width*height);
	//for (int i = 0; i < depth_pixels.size(); i++) {
	//	int w = i % width;
	//	int h = i / width;
	//	h = height - 1 - h;
	//	point_cloud[i] = /*unproject(w, h, fx, fy, cx, cy) **/ depth_pixels[i];
	//}

	point_cloud.resize(width*height);
	const float DegreesToRadians = 3.14159265359f / 180.0f;
	const float xScale = tanf(fovx * DegreesToRadians * 0.5f) * 2.0f / width;
	const float yScale = tanf(fovy * DegreesToRadians * 0.5f) * 2.0f / height;
	int	half_width = width / 2;
	int	half_height = height / 2;
	for (int j = 0; j < height; j++){
		for (int i = 0; i < width; i++){
			int idx = j*width + i;
			unsigned short pixel_depth = depth_pixels[idx];
			float	depth = -pixel_depth * 0.001;	//	unit in meters

			//auto pos = NuiTransformDepthImageToSkeleton(i, j, pixel_depth, NUI_IMAGE_RESOLUTION_640x480);
			//point_cloud[idx].x = pos.x / pos.w;
			//point_cloud[idx].y = pos.y / pos.w;
			//point_cloud[idx].z = pos.z / pos.w;

			//point_cloud[idx].x= -(i + 0.5 - half_width) * xyScale * depth;
			//point_cloud[idx].y = (j + 0.5 - half_height) * xyScale * depth;
			//point_cloud[idx].z = depth;		//	in OpenGL coordinate

			point_cloud[idx].x = -(i + 0.5f - half_width) * xScale * depth;
			point_cloud[idx].y = (j + 0.5f - half_height) * yScale * depth;
			point_cloud[idx].z = depth;
		}
	}

	return;
}

static void writePly(const vector<cv::Point3f>& point_cloud, const vector<BYTE>& infrared_pixels, const char *filename)
//inline void writePly(const vector<vector3f>& point_cloud, const RGBQUAD* infrared_pixels, const char *filename)
{
	FILE *fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", point_cloud.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");

	for (int i = 0; i < point_cloud.size(); i++) {
		fprintf_s(fp, "%f %f %f %d %d %d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, infrared_pixels[i], infrared_pixels[i], infrared_pixels[i]);
		//fprintf_s(fp, "%f %f %f %d %d %d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, 255, 255, 255);
	}
	fclose(fp);
}

#define IR_FX (387.56565394355408)
#define IR_FY (387.43950296593346)
#define IR_CX (259.15112583019447)
#define IR_CY (206.03282313805542)

int main()
{
	int width = 512, height = 424;
	string depth_path = "D:\\yyk\\image\\NoiseModel_1119\\1447901478-331-b.bin";
	string IR_path = "D:\\yyk\\image\\NoiseModel_1119\\1447901478-331-b.bmp";

	cv::Mat image = cv::imread(IR_path, 0);
	vector<BYTE> IR_vector(width * height);
	for (int j = 0; j < height; ++j) {
		for (int i = 0; i < width; ++i) {
			int idx = j * width + i;
			IR_vector[idx] = image.at<BYTE>(j, i);
		}
	}

	vector<UINT16> depth_map;
	load_raw_depth_from_bin_file(depth_path, depth_map, width, height);
	vector<cv::Point3f> points;
	calc_points(depth_map, points, width, height, IR_FX, IR_FY);
	writePly(points, IR_vector, "D:\\1447901478-331-b.ply");
	return 0;
}