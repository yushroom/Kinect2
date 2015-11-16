#pragma once
#include <iostream>
#include <vector>
#include <utility>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <advmath.h>
#include <cmath>

#include "Error.hpp"

using codex::math::vector3f;
using codex::math::vector2f;

using namespace std;
using namespace cv;
typedef unsigned short UINT16;
typedef UINT16 DepthType;

#define NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV                 (58.5f)
#define NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV                   (45.6f)
#define V1_FOV_X NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV
#define V1_FOV_Y NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

template<typename T>
static Mat vector_to_img_uc(const vector<T>& vec, int w, int h, float scale = 1) {
	Mat img(h, w, CV_8U);
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img.at<unsigned char>(j, i) = unsigned char(vec[idx] * scale);
		}
	}
	return img;
}

inline float lerp(float a, float b, float t) {
	return (1-t) * a + t *b;
}

template<typename T>
static float calc_STD(const vector<T>& v);

// corners: left_top, left_bottom, right_top, right_bottom
static float calc_lateral_noise(const Mat& depth_gray, const Point2f corners[4], float* out_x_left, float* out_x_right);

static void calc_points(const vector<UINT16>& depth_pixels, cv::Rect roi , vector<vector3f>& point_cloud, const int width, const int height,
						const float fovx, const float fovy);

vector3f pca(const std::vector<vector3f> &points);

/**linear fit for a set of 2D points, 
   points on the resulting line can be expressed as p = o + d*t,  
* */
void linear_regression(const vector<vector2f>& points, vector2f& o, vector2f& d);

//void linear_regression(const vector<vector2f>& points, float* k, float *b) {
//	float sum_a = 0.f, sum_b = 0.f, sum_xy = 0.f, sum_xx = 0.f;
//	float sum_x_square = 0.f;
//	*k 
//}

class KinectNoiseModel {
protected:
	int width;
	int height;
	string m_IR_image_path;
	string m_depth_bin_path;

	virtual bool find_rect_in_IR(Mat& image, vector<Point>& out_points) = 0;

	static void load_raw_depth_from_bin_file(string depth_bin_path, vector<UINT16>& out_raw_depth, int width, int height);
	
	bool get_noise(vector<Point2f> rect_points);

public:
	KinectNoiseModel(const string depth_bin_path, const string& IR_image_path)
		: m_IR_image_path(IR_image_path), m_depth_bin_path(depth_bin_path) {
			m_lateral_noise = m_axial_noise = m_angle = m_distance = 0;
	}

	float m_lateral_noise;
	float m_axial_noise;

	float m_distance;
	float m_angle;

	virtual bool process() = 0;
};

class KinectNoiseModel1 : public KinectNoiseModel {
protected:
	virtual bool find_rect_in_IR(Mat& image, vector<Point>& out_points) override;

public:
	KinectNoiseModel1(const string depth_bin_path, const string& IR_image_path) 
		: KinectNoiseModel(depth_bin_path, IR_image_path) {
		width = 640;
		height = 480;
	}

	virtual bool process() override {
		Mat ir_rgb = imread(m_IR_image_path);
		assert(ir_rgb.rows == height && ir_rgb.cols == width);

		vector<Point> points;
		if (!find_rect_in_IR(ir_rgb, points)) {
			//cout << "rect not found\n";
			error("rect not found\n");
			return false;
		}
		cv::RotatedRect minRect = minAreaRect(cv::Mat(points));
		cv::Point2f rect_points[4];
		minRect.points(rect_points);
		vector<Point2f> points2(rect_points, rect_points + 4);
		return get_noise(points2);
	}
};

class KinectNoiseModel2 : public KinectNoiseModel {
protected:
	virtual bool find_rect_in_IR(Mat& image, vector<Point>& out_points) override;

public:
	KinectNoiseModel2(const string depth_bin_path, const string& IR_image_path) 
		: KinectNoiseModel(depth_bin_path, IR_image_path) {
		width = 512;
		height = 424;
	}

	virtual bool process() override {
		Mat IR_gray = imread(m_IR_image_path, CV_LOAD_IMAGE_GRAYSCALE);
		assert(IR_gray.rows == height && IR_gray.cols == width);

		vector<Point> points;
		if (!find_rect_in_IR(IR_gray, points)) {
			//cout << "rect not found\n";
			error("rect not found\n");
			return false;
		}
		cv::RotatedRect minRect = minAreaRect(cv::Mat(points));
		cv::Point2f rect_points[4];
		minRect.points(rect_points);
		vector<Point2f> points2(rect_points, rect_points + 4);
		return get_noise(points2);
	}
};