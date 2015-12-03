#pragma once

#include <vector>
#include <codex_math.h>

typedef codex::math::vector::vector3<float>				vector3f;

std::vector<vector3f> calc_points_from_depth_image(
	const std::vector<float>&	depth,
	const int					width,
	const int					height,
	const float					fx,
	const float					fy,
	const float					cx,
	const float					cy);

std::vector<vector3f> calc_points_from_depth_image(
	const std::vector<unsigned short>&	depth,
	const int					width,
	const int					height,
	const float					fx,
	const float					fy,
	const float					cx,
	const float					cy);


vector3f pca(const std::vector<vector3f> &points);

std::vector<vector3f> calc_normal_map(
	const std::vector<vector3f>&	position,
	const int						width,
	const int						height,
	const float						fx, 
	const float						fy,
	const float						cx, 
	const float						cy 
);

void calc_and_dump_normal_map(
	const std::vector<float>&		depth,
	const std::string&				file_path,
	const int						width,
	const int						height,
	const float						fx,
	const float						fy,
	const float						cx,
	const float						cy
	);

std::vector<vector3f> clac_shading(
	const std::vector<vector3f>& normal_map, 
	const std::vector<vector3f>& position, 
	const int width, 
	const int height, 
	const std::string& file_path);