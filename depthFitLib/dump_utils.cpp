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

template <typename T>
void dump_normalized_image(
	const std::vector<T>&			image,
	const char*						path,
	const int						width,
	const int						height)
{
	T lower_bound = *std::_Find_elem(image.begin(), image.end());
	T upper_bound = *std::_Find_elem(image.begin(), image.end());
	dump_normalized_image(
		image,
		path,
		width,
		height,
		lower_bound,
		upper_bound);
}

template <typename T>
void dump_normalized_image(
	const std::vector<T>&			image,
	const char*						path,
	const int						width,
	const int						height,
	const T							lower_bound,
	const T							upper_bound)
{
	const T range = upper_bound - lower_bound;
	CImage cimage;
	cimage.Create(width, height, 32);
	for (int i = 0; i < height; ++i)
		for (int j = 0; j < width; ++j) {
			int idx = j + i*width;
			T value = clamp(image[idx], lower_bound, upper_bound);
			value = (value - lower_bound) / range;
			BYTE rgb = static_cast<BYTE>(value * 255);
			cimage.SetPixelRGB(j, i, rgb, rgb, rgb);
		}
	cimage.Save(path);
}