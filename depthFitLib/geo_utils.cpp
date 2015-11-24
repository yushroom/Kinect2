#include "geo_utils.h"


std::vector<vector3f> calc_points_from_depth_image(
	const std::vector<float>&	depth,
	const int					width,
	const int					height,
	const float					fx,
	const float					fy,
	const float					cx,
	const float					cy)
{
	 float inv_fx = 1.f / fx;
	 float inv_fy = 1.f / fy;
	 std::vector<vector3f> points(width*height);
	 for (int j = 0; j < height; ++j)
		 for (int i = 0; i < width; ++i) {
			 int idx = j * width + i;
			 float z = depth[idx];
			 float x = (i + 0.5f - cx) * inv_fx * z;
			 float y = (j + 0.5f - cy) * inv_fy * z;
			 points[idx] = vector3f(x, y, z);
		 }

	 return points;
}