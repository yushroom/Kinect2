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
