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
#include <io.h>
#include <algorithm>

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
static Mat vector_to_img_uc(const vector<T>& vec, int w, int h) {
	Mat img(h, w, CV_8U);
	auto max_ele = *std::max_element(vec.begin(), vec.end());
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img.at<unsigned char>(j, i) = unsigned char(float(vec[idx]) / max_ele * 255.f);
		}
	}
	return img;
}

inline float lerp(float a, float b, float t) {
	return (1-t) * a + t *b;
}

static cv::Mat debugSquares(const std::vector<std::vector<cv::Point> >& squares, cv::Mat& image)
{
	for (int i = 0; i < squares.size(); i++) {
		// draw contour
		//cv::drawContours(image, squares, i, cv::Scalar(255, 0, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

		// draw bounding rect
		//cv::Rect rect = boundingRect(cv::Mat(squares[i]));
		//cv::rectangle(image, rect.tl(), rect.br(), cv::Scalar(0, 255, 0), 2, 8, 0);

		// draw rotated rect
		cv::RotatedRect minRect = minAreaRect(cv::Mat(squares[i]));
		cv::Point2f rect_points[4];
		minRect.points(rect_points);
		for (int j = 0; j < 4; j++) {
			cv::line(image, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 0, 255), 1, 8); // blue
		}
	}

	return image;
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
	float m_threshold;
	float m_scale;

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

	virtual bool process() {
		string corners_file_name = m_IR_image_path.substr(0, m_IR_image_path.size() - 3) + "txt";
		if (_access_s(corners_file_name.c_str(), 0) != 0) {
			warning("corner file %s not exists. skip this image.\n", corners_file_name.c_str());
			return false;
		}
		
		vector<Point2f> corners(4);

		ifstream fin(corners_file_name);
		vector<int> c;
		int t;
		while (fin >> t)
			c.push_back(t);
		if (c.size() != 8) {
			error("corners in file not match!\n");
			return false;
		}
		else {
			for (int i = 0; i < 4; ++i) {
				corners[i].x = c[i * 2];
				corners[i].y = c[i * 2 + 1]; 
			}
			cout << "    load corners from file.\n";
		}

		return get_noise(corners);
	}
};

class KinectNoiseModel1 : public KinectNoiseModel {
protected:
	virtual bool find_rect_in_IR(Mat& image, vector<Point>& out_points) override;

public:
	KinectNoiseModel1(const string depth_bin_path, const string& IR_image_path) 
		: KinectNoiseModel(depth_bin_path, IR_image_path) {
		width = 640;
		height = 480;
		m_threshold = 0.7f;
		m_scale = 3.0f;
	}

	//virtual bool process() override {
	//	Mat ir_rgb = imread(m_IR_image_path);
	//	assert(ir_rgb.rows == height && ir_rgb.cols == width);

	//	vector<Point> points;
	//	if (!find_rect_in_IR(ir_rgb, points)) {
	//		//cout << "rect not found\n";
	//		error("rect not found\n");
	//		return false;
	//	}
	//	cv::RotatedRect minRect = minAreaRect(cv::Mat(points));
	//	cv::Point2f rect_points[4];
	//	minRect.points(rect_points);
	//	vector<Point2f> points2(rect_points, rect_points + 4);
	//	return get_noise(points2);
	//}
};

class KinectNoiseModel2 : public KinectNoiseModel {
protected:
	virtual bool find_rect_in_IR(Mat& image, vector<Point>& out_points) override;

public:
	KinectNoiseModel2(const string depth_bin_path, const string& IR_image_path) 
		: KinectNoiseModel(depth_bin_path, IR_image_path) {
		width = 512;
		height = 424;
		m_threshold = 0.2f;
		m_scale = 5.f;
	}

	//virtual bool process() override {
	//	Mat IR_gray = imread(m_IR_image_path, CV_LOAD_IMAGE_GRAYSCALE);
	//	assert(IR_gray.rows == height && IR_gray.cols == width);

	//	vector<Point> points;
	//	if (!find_rect_in_IR(IR_gray, points)) {
	//		//cout << "rect not found\n";
	//		error("rect not found\n");
	//		return false;
	//	}
	//	cv::RotatedRect minRect = minAreaRect(cv::Mat(points));
	//	cv::Point2f rect_points[4];
	//	minRect.points(rect_points);
	//	vector<Point2f> points2(rect_points, rect_points + 4);
	//	return get_noise(points2);
	//}
};