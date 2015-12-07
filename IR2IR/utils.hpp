#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <advmath.h>
#include <cassert>

using std::vector;
using std::string;
using codex::math::vector3f;
using codex::math::vector2f;

const float DEPTH_INVALID = 1e30f;
#define VALID_DEPTH_TEST(a) (a < DEPTH_INVALID)

//template<typename T>
//T clamp(T val, const T low, const T high)
//{
//	if (val < low)
//		return low;
//	else if (val > high)
//		return high;
//	return val;
//}

template <typename T>
static float bilinear(
	float x, float y, 
	const std::vector<T>& src, 
	int width, int height, 
	bool ignore_zero = false, 
	bool ignore_invalid = false) 
{
	static int dx[] = { 0, 0, 1, 1 };
	static int dy[] = { 0, 1, 0, 1 };
	float weight = 0.f, value = 0.f;
	for (int i = 0; i < 4; ++i) {
		int ix = int(x) + dx[i], iy = int(y) + dy[i];
		if (ix < 0 || ix >= width || iy < 0 || iy >= height) continue;
		int idx = ix + iy*width;
		if (ignore_zero && src[idx] == 0) continue;
		if (ignore_invalid && !VALID_DEPTH_TEST(src[idx])) continue;
		float wx = abs(ix - x), wy = abs(iy - y);
		float w = (1.f - wx)*(1.f - wy);
		value += src[idx] * w;
		weight += w;
	}

	return (weight == 0.f ? DEPTH_INVALID : value / weight);
	/*
	int ix0 = x, iy0 = y, ix1 = ix0 + 1, iy1 = iy0 + 1;
	float qx1 = x - ix0, qy1 = y - iy0, qx0 = 1.f - qx1, qy0 = 1.f - qy1;

	float weight = 0.f, value = 0.f;
	float w[] = { qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1 };
	//pair<int, int> xy[] = { { ix0, iy0 }, { ix1, iy0 }, { ix0, iy1 }, { ix1, iy1 } };
	int xx[] = {ix0, ix1, ix0, ix1};
	int yy[] = {iy0, iy0, iy1, iy1};

	for (int j = 0; j<4; j++) {
		if (xx[j] < 0 || xx[j] >= width || yy[j] < 0 || yy[j] >= height)
			continue;
		int idx = xx[j] + yy[j] * width;
		if (ignore_zero && src[idx] == 0) continue;
		if (ignore_invalid && !VALID_DEPTH_TEST(src[idx])) continue;
		value += src[idx] * w[j];
		weight += w[j];
	}

	return (weight == 0.f ? DEPTH_INVALID : value / weight);
	*/
	//return static_cast<T>(weight == 0.f ? DEPTH_INVALID : value / weight);
}

template <typename T>
static float bilinear(
	float x, float y,
	const std::function<T(const int x, const int y)> getPixel,
	int width, int height, 
	bool ignore_zero = false) 
{
	int ix0 = x, iy0 = y, ix1 = ix0 + 1, iy1 = iy0 + 1;
	float qx1 = x - ix0, qy1 = y - iy0, qx0 = 1.f - qx1, qy0 = 1.f - qy1;

	float weight = 0.f, value = 0.f;
	float w[] = { qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1 };
	//pair<int, int> xy[] = { { ix0, iy0 }, { ix1, iy0 }, { ix0, iy1 }, { ix1, iy1 } };
	int xx[] = { ix0, ix1, ix0, ix1 };
	int yy[] = { iy0, iy0, iy1, iy1 };

	for (int j = 0; j < 4; j++) {
		if (xx[j] < 0 || xx[j] >= width || yy[j] < 0 || yy[j] >= height)
			continue;
		//int idx = xx[j] + yy[j] * width;
		T pixel = getPixel(xx[i], yy[j]);
		if (ignore_zero && pixel == 0) continue;
		value += pixel * w[j];
		weight += w[j];
	}

	return (weight == 0.f ? DEPTH_INVALID : value / weight);
	//return static_cast<T>(weight == 0.f ? DEPTH_INVALID : value / weight);
}

template<typename T>
cv::Mat diff_image(	const std::vector<T>& image1, 
					const std::vector<T>& image2,
					const int width,
					const int height,
					const float threshold) 
{
	cv::Mat ret = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Vec3b yellow(0, 1, 1);
	cv::Vec3b blue(1, 0, 0);
	cv::Vec3b white(255, 255, 255);
	for (int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x)
		{
			int idx = y * width + x;
			if (image1[idx] > 1e20 || image2[idx] > 1e20) {
				ret.at<cv::Vec3b>(y, x) = white / 2;
				continue;
			}
			float diff = image1[idx] - image2[idx];
			//std::cout << diff << endl;
			diff = clamp<float>(diff, -threshold, threshold);
			if (diff >= 0) {
				uchar c = diff / threshold * 255;
				ret.at<cv::Vec3b>(y, x) = Vec3b(0, 0, c);
			}
			else {
				uchar c = -diff / threshold * 255;
				ret.at<cv::Vec3b>(y, x) = Vec3b(c, 0, 0);	// red
			}
			//ret.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
		}

	return ret;
}


//static vector<vector3f> calc_points(const vector<float>& depth, int width, int height, float fx, float fy, float cx, float cy) 
//{
//	assert(depth.size() == width * height);
//	float inv_fx = 1.f / fx;
//	float inv_fy = 1.f / fy;
//	vector<vector3f> ret(width * height);
//	for (int y = 0; y < height; ++y)
//		for (int x = 0; x < width; ++x) 
//		{
//			int idx = y * width + x;
//			float d = depth[idx];
//			if (!VALID_DEPTH_TEST(d)) 
//			{
//				ret[idx] = vector3f(x, y, -1);	// invalid
//				continue;
//			}
//			float xx = (x + 0.5f - cx) * inv_fx * d;
//			float yy = (y + 0.5f - cy) * inv_fy * d;
//			ret[idx] = vector3f(xx, yy, d);
//		}
//
//	return ret;
//}
//
//static vector3f pca(const std::vector<vector3f> &points)
//{
//	vector3f ave_pos;
//	for (auto &p : points)
//		ave_pos += p / (float)points.size();
//	advmath::la_matrix<float> A(3, points.size()), C;
//	for (int i = 0; i < points.size(); i++) {
//		vector3f p = points[i] - ave_pos;
//		for (int j = 0; j < 3; j++)
//			A.m[i * 3 + j] = p.v[j];
//	}
//	advmath::smmmul(C, A, A.transpose());
//	advmath::la_matrix<float> U, Vt;
//	advmath::la_vector<float> sigma;
//	advmath::ssvd(U, Vt, sigma, C, 1.f);
//	return vector3f(U.m[6], U.m[7], U.m[8]);
//}
//
//static const vector3f invalid_normal(-1, -1, -1);
//inline bool normal_vaild(const vector3f& v) {
//	return v.x != -1 && v.y != -1 && v.z != -1;
//}
//
//static std::vector<vector3f> my_calc_normal_map(const vector<vector3f>& position, int width, int height, float fx, float fy, float cx, float cy)
//{
//	assert(position.size() == width * height);
//	vector<vector3f> ret;
//	for (int y = 0; y < height; ++y) 
//	{
//		for (int x = 0; x < width; ++x)
//		{
//			vector<vector3f> points_in_3x3;
//			for (int j = -1; j <= 1; ++j)
//				for (int i = -1; i <= 1; ++i) 
//				{
//					int xx = x+i;
//					int yy = y+j;
//					if (xx >= 0 && xx<width && yy >= 0 && yy<height) 
//					{
//						auto& p = position[yy * width + xx];
//						if (p.z > 0)
//							points_in_3x3.push_back(p);
//					}
//				}
//			vector3f normal(-1, -1, -1);	// invalid normal
//			if (points_in_3x3.size() > 0)
//				normal = pca(points_in_3x3);
//			ret.push_back(normal);
//		}
//	}
//
//	return ret;
//}

//void dump_normal_map(const vector<vector3f>& normal_map, const int width, const int height, const string& file_path)
//{
//	assert(normal_map.size() == width * height);
//	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
//	for (int y = 0; y < height; ++y) {
//		for (int x = 0; x < width; ++x) {
//			vector3f normal = normal_map[y * width + x];
//			normal = (normal + vector3f(1.f, 1.f, 1.f)) / 2.0f * 255;
//			image.at<cv::Vec3b>(y, x) = cv::Vec3b(normal.x, normal.y, normal.z);
//		}
//	}
//	cv::imwrite(file_path, image);
//}

//void dump_shading(const vector<vector3f>& normal_map, const vector<vector3f>& position, const int width, const int height, const string& file_path) 
//{
//	assert(normal_map.size() == width*height);
//	assert(position.size() == width*height);
//
//	vector3f light_dir(10, 10, 10);
//	light_dir = light_dir.normalized_vector();
//
//	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
//	for (int y = 0; y < height; ++y) 
//	{
//		for (int x = 0; x < width; ++x) 
//		{
//			int idx = y * width + x;
//			const vector3f& normal = normal_map[idx];
//			float c = 0;
//			if (normal_vaild(normal)) 
//			{
//				const vector3f& pos = position[idx];
//				c = normal * light_dir;
//			}
//			c *= 255;
// 			image.at<cv::Vec3b>(y, x) = cv::Vec3b(c, c, c);
//		}
//	}
//	cv::imwrite(file_path, image);
//
//}