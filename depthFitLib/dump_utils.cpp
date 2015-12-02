#include "dump_utils.h"
#include "math_utils.h"

#include <algorithm>
#include <atlimage.h>


void	dump_point_cloud(
	const std::vector<vector3f>& points,
	const char *path,
	const vector3b rgb)
{
	FILE *fp;
	fopen_s(&fp, path, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", points.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");

	for (int i = 0; i < points.size(); ++i) 
		fprintf_s(fp, "%g %g %g %d %d %d\n", 
			points[i].x, points[i].y, points[i].z, 
			rgb.x, rgb.y, rgb.z);
	fclose(fp);

}

void    dump_image(
    const std::vector<vector3b>&    image,
    const char*                     path,
    const int                       width,
    const int                       height)
{
    CImage cimage;
	cimage.Create(width, height, 32);
	for (int i = 0; i < height; ++i)
		for (int j = 0; j < width; ++j) {
		int idx = j + i*width;
		cimage.SetPixelRGB(j, i, image[idx].x, image[idx].y, image[idx].z);
		}
	cimage.Save(path);
}

void dump_normal_map(const std::vector<vector3f>& normal_map, const int width, const int height, const std::string& file_path)
{
	assert(normal_map.size() == width * height);
	//cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	CImage cimage;
	cimage.Create(width, height, 32);
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			vector3f normal = normal_map[y * width + x];
			normal = (normal + vector3f(1.f, 1.f, 1.f)) / 2.0f * 255;
			//image.at<cv::Vec3b>(y, x) = cv::Vec3b(normal.x, normal.y, normal.z);
			cimage.SetPixelRGB(x, y, normal.x, normal.y, normal.z);
		}
	}
	//cv::imwrite(file_path, image);
	cimage.Save(file_path.c_str());
}

static const vector3f invalid_normal(-1, -1, -1);
inline bool normal_vaild(const vector3f& v) {
	return v.x != -1 && v.y != -1 && v.z != -1;
}

void dump_shading(const std::vector<vector3f>& normal_map, const std::vector<vector3f>& position, const int width, const int height, const std::string& file_path)
{
	assert(normal_map.size() == width*height);
	assert(position.size() == width*height);

	vector3f light_dir(10, 10, 10);
	light_dir = light_dir.normalized_vector();

	//cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	CImage cimage;
	cimage.Create(width, height, 32);
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			int idx = y * width + x;
			const vector3f& normal = normal_map[idx];
			float c = 0;
			if (normal_vaild(normal))
			{
				const vector3f& pos = position[idx];
				c = normal * light_dir;
			}
			c *= 255;
			//image.at<cv::Vec3b>(y, x) = cv::Vec3b(c, c, c);
			cimage.SetPixelRGB(x, y, c, c, c);
		}
	}
	//cv::imwrite(file_path, image);
	cimage.Save(file_path.c_str());
}
