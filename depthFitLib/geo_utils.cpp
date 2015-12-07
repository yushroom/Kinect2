#include "geo_utils.h"
#include <cassert>
#include <advmath.h>
#include <vector>
#include "dump_utils.h"

using std::vector;

const float DEPTH_INVALID = 1e30f;
#define VALID_DEPTH_TEST(a) (a < DEPTH_INVALID)

std::vector<vector3f> calc_points_from_depth_image(
	const std::vector<float>&	depth,
	const int					width,
	const int					height,
	const float					fx,
	const float					fy,
	const float					cx,
	const float					cy)
{
	assert(depth.size() == width * height);
	const float inv_fx = 1.f / fx;
	const float inv_fy = 1.f / fy;
	std::vector<vector3f> points(width*height);
#pragma omp parallel for
	for (int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x) {
			int idx = y * width + x;
			float z = depth[idx];
			if (!VALID_DEPTH_TEST(z))
			{
				points[idx] = vector3f(x, y, -1);	// invalid
			}
			else {
				float xx = (x + 0.5f - cx) * inv_fx * z;
				float yy = (y + 0.5f - cy) * inv_fy * z;
				points[idx] = vector3f(xx, yy, z);
			}
		}
	return points;
}

std::vector<vector3f> calc_points_from_depth_image(
	const std::vector<unsigned short>&	depth,
	const int					width,
	const int					height,
	const float					fx,
	const float					fy,
	const float					cx,
	const float					cy)
{
	assert(depth.size() == width * height);
	float inv_fx = 1.f / fx;
	float inv_fy = 1.f / fy;
	std::vector<vector3f> points(width*height);
#pragma omp parallel for
	for (int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x) {
			int idx = y * width + x;
			unsigned short d = depth[idx];
			if (d == 0)
			{
				points[idx] = vector3f(x, y, -1);	// invalid
			}
			else {
				float z = d;
				float xx = (x + 0.5f - cx) * inv_fx * z;
				float yy = (y + 0.5f - cy) * inv_fy * z;
				points[idx] = vector3f(xx, yy, z);
			}
		}

	return points;
}


vector3f pca(const std::vector<vector3f> &points)
{
	vector3f ave_pos;
	for (auto &p : points)
		ave_pos += p / (float)points.size();
	advmath::la_matrix<float> A(3, points.size()), C;
	for (int i = 0; i < points.size(); i++) {
		vector3f p = points[i] - ave_pos;
		for (int j = 0; j < 3; j++)
			A.m[i * 3 + j] = p.v[j];
	}
	advmath::smmmul(C, A, A.transpose());
	advmath::la_matrix<float> U, Vt;
	advmath::la_vector<float> sigma;
	advmath::ssvd(U, Vt, sigma, C, 1.f);
	return vector3f(U.m[6], U.m[7], U.m[8]);
}


static const vector3f invalid_normal(-1, -1, -1);
inline bool normal_vaild(const vector3f& v) {
	return v.x != -1 && v.y != -1 && v.z != -1;
}

std::vector<vector3f> calc_normal_map(
	const std::vector<vector3f>&	position,
	const int						width,
	const int						height,
	const float						fx,
	const float						fy,
	const float						cx,
	const float						cy)
{
	assert(position.size() == width * height);
	vector<vector3f> ret;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			vector<vector3f> points_in_3x3;
			for (int j = -1; j <= 1; ++j)
				for (int i = -1; i <= 1; ++i)
				{
					int xx = x + i;
					int yy = y + j;
					if (xx >= 0 && xx < width && yy >= 0 && yy < height)
					{
						auto& p = position[yy * width + xx];
						if (p.z > 0)
							points_in_3x3.push_back(p);
					}
				}
			vector3f normal(-1, -1, -1);	// invalid normal
			if (points_in_3x3.size() >= 3)
				normal = pca(points_in_3x3);
			ret.push_back(normal);
		}
	}

	return ret;
}

std::vector<vector3f> clac_shading(
	const vector<vector3f>& normal_map, 
	const vector<vector3f>& position, 
	const int width,
	const int height, 
	const std::string& file_path)
{
	assert(normal_map.size() == width*height);
	assert(position.size() == width*height);

	vector3f light_dir(10, 10, 10);
	light_dir = light_dir.normalized_vector();

	//cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	std::vector<vector3f> ret(width * height);
	//for (int y = 0; y < height; ++y)
	//{
	//	for (int x = 0; x < width; ++x)
#pragma omp parallel for
	for (int idx = 0; idx < width * height; ++idx)
	{
		//int idx = y * width + x;
		const vector3f& normal = normal_map[idx];
		float c = 0;
		if (normal_vaild(normal))
		{
			const vector3f& pos = position[idx];
			c = normal * light_dir;
		}
		c *= 255;
		//image.at<cv::Vec3b>(y, x) = cv::Vec3b(c, c, c);
		ret[idx] = vector3f(c, c, c);
	}
	return ret;
	//cv::imwrite(file_path, image);
}

void calc_and_dump_normal_map_and_shading (
	const std::vector<float>& depth, 
	const std::string& normal_file_path,
	const std::string& shading_file_path,
	const int width, const int height,
	const float fx, const float fy,
	const float cx, const float cy)
{
	auto points = calc_points_from_depth_image(depth, width, height, fx, fy, cx, cy);
	auto normals = calc_points_from_depth_image(depth, width, height, fx, fy, cx, cy);
	dump_normal_map(normals, width, height, normal_file_path);
	dump_shading(normals, points, width, height, shading_file_path);
}
