#include "KinectNoiseModel.h"

void KinectNoiseModel::load_raw_depth_from_bin_file( string depth_bin_path, vector<UINT16>& out_raw_depth, int width, int height )
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

bool KinectNoiseModel::get_noise(vector<Point2f> rect_points)
{
	//vector<Point2f> rect_points;
	//find_rect_in_IR(rect_points);

	assert(rect_points.size() == 4);
	vector<UINT16> raw_depth;
	load_raw_depth_from_bin_file(m_depth_bin_path, raw_depth, width, height);

	Mat depth_gray, depth_rgb;
	depth_gray = vector_to_img_uc(raw_depth, width, height, 0.1f);
	cvtColor(depth_gray, depth_rgb, CV_GRAY2BGR);

	Point2f corners[4];
	// 0 1 2 3
	// left_top left_bottom right_top right_bottom
	auto mean_p2 = std::accumulate(rect_points.begin(), rect_points.end(), Point2f(0, 0)) / 4.0f;
	for (auto& p : rect_points) {
		int i = 0, j = 0;
		if (p.x > mean_p2.x) {	// left
			i = 1;
		}
		if (p.y > mean_p2.y) { // bottom
			j = 1;
		}
		corners[i*2+j] = p;
	}

	float top_y    = max(corners[0].y, corners[2].y);
	float bottom_y = min(corners[1].y, corners[3].y);
	const float h = bottom_y - top_y;
	assert(h > 0);
	float y_offset_top = (h * 0.1f);
	float y_offset_bottom = (h * 0.3f);

	corners[2].y = corners[0].y = top_y + y_offset_top;		// top
	corners[3].y = corners[1].y = bottom_y - y_offset_bottom;			// bottom

	float left_x = -1.f, right_x = -1.f;
	float lateral_noise = calc_lateral_noise(depth_gray, corners, &left_x, &right_x);
	if (lateral_noise <= 0) {
		return false;
	}
	assert(lateral_noise >= 0);
	assert(left_x > 0);
	
	float x_offset = (3.0f * lateral_noise);
	//cout << x_offset << endl;
	left_x += x_offset;
	right_x -= x_offset;
	
	//assert(left_x < right_x);
	if (left_x >= right_x) {
		error("left_x >= right_x\n");
		return false;
	}

	// get axial noise
	//float left_x  = max(corners[0].x, corners[1].x) + x_offset;
	//float right_x = min(corners[2].x, corners[3].x) - x_offset;
	corners[0].x = corners[1].x = left_x;
	corners[2].x = corners[3].x = right_x;
	Scalar white(255, 255, 255);
	Mat depth_rgb_rect2;
	depth_rgb.copyTo(depth_rgb_rect2);
	cv::line(depth_rgb_rect2, corners[0], corners[1], white);
	cv::line(depth_rgb_rect2, corners[1], corners[3], white);
	cv::line(depth_rgb_rect2, corners[3], corners[2], white);
	cv::line(depth_rgb_rect2, corners[2], corners[0], white);

	//vector<vector<Point> > points2;
	//points2.push_back(new_points);
	//debugSquares(points2, depth_rgb);
	cv::imshow("draw rect 2", depth_rgb_rect2);
	top_y += y_offset_top;
	bottom_y -= y_offset_bottom;

	cv::Rect roi(left_x, top_y, right_x - left_x, bottom_y - top_y);
	imshow("roi", depth_rgb(roi));
	vector<vector3f> proj_points;
	calc_points(raw_depth, roi, proj_points, width, height, V1_FOV_X, V1_FOV_Y);

	auto normal = pca(proj_points);
	//cout << "normal of fitted plane: [" << normal.x << ' ' << normal.y << ' ' << normal.z << ']' << endl;

	vector3f mean_p3 = std::accumulate(proj_points.begin(), proj_points.end(), vector3f(0, 0, 0)) / proj_points.size();
	std::cout << "mean point: [" << mean_p3.x << ' ' << mean_p3.y << ' ' << mean_p3.z << ']' << endl;

	float A = normal.x, B = normal.y, C = normal.z;
	float D = -(A * mean_p3.x + B * mean_p3.y + C * mean_p3.z);
	std::cout << "fitted plane: A = " << A << ", B = " << B << ", C = " << C << ", D = " << D << endl;

	m_angle = acosf(normal.z / sqrtf(normal.x * normal.x + normal.z * normal.z));
	m_angle *= 180.0f / M_PI;
	std::cout << "angle = " << m_angle << endl;

	int mid_x = (left_x + right_x) / 2;
	float average_depth = 0.f, count = 0;
	for (int x = mid_x - 5; x <= mid_x + 5; ++x) {
		for (int y = top_y; y < bottom_y; ++ y) {
			int idx = y * width + x;
			average_depth += raw_depth[idx] * 0.001f;
			count ++;
		}
	}
	average_depth /= (float)count;
	m_distance = average_depth;
	std::cout << "distance = " << m_distance << endl;

	vector<float> depth_diff;
	depth_diff.reserve(proj_points.size());
	float t = 1.0f / sqrtf(A*A+B*B+C*C);
	for (auto& p : proj_points) {
		float fitted_z = (A * p.x + B * p.y + D) / (-C);
		depth_diff.push_back(p.z - fitted_z);
		//float dist_to_plane = fabsf(A*p.x + B*p.y + C*p.z + D) * t;
		//depth_diff.push_back(dist_to_plane);
		//cout << fitted_z << ' ' << p.z << ' ' << p.z - fitted_z << endl;
	}

	float axial_noise = calc_STD(depth_diff);
	//cout << "axial noise: " << axial_noise;
	warning("axial noise: %lf\n", axial_noise);

	m_lateral_noise = lateral_noise;
	m_axial_noise = axial_noise;

	return true;
}


bool KinectNoiseModel1::find_rect_in_IR( Mat& image, vector<Point>& out_points )
{
	// blur will enhance edge detection
	Mat blurred(image);
	medianBlur(image, blurred, 9);

	Mat gray0(blurred.size(), CV_8U), gray;
	vector<vector<Point> > contours;
	vector<vector<Point>> squares;

	// find squares in every color plane of the image
	for (int c = 0; c < 3; c++)
	{
		int ch[] = { c, 0 };
		mixChannels(&blurred, 1, &gray0, 1, ch, 1);

		// try several threshold levels
		const int threshold_level = 2;
		for (int l = 0; l < threshold_level; l++)
		{
			// Use Canny instead of zero threshold level!
			// Canny helps to catch squares with gradient shading
			if (l == 0)
			{
				Canny(gray0, gray, 10, 20, 3); // 

				// Dilate helps to remove potential holes between edge segments
				dilate(gray, gray, Mat(), Point(-1, -1));
			}
			else
			{
				gray = gray0 >= (l + 1) * 255 / threshold_level;
			}

			// Find contours and store them in a list
			findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

			// Test contours
			vector<Point> approx;
			
			for (size_t i = 0; i < contours.size(); i++)
			{
				// approximate contour with accuracy proportional
				// to the contour perimeter
				approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				if (approx.size() == 4 &&
					fabs(contourArea(Mat(approx))) > 1000 &&
					isContourConvex(Mat(approx)))
				{
					double maxCosine = 0;

					for (int j = 2; j < 5; j++)
					{
						double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					if (maxCosine < 0.3)
						squares.push_back(approx);
				}
			}	
		}
	}
	//cout << squares.size() << endl;
	//assert(squares.size() >= 1);
	if (squares.size() == 0) {
		return false;
	}
	out_points = squares[0];
	return true;
}

bool KinectNoiseModel2::find_rect_in_IR( Mat& image, vector<Point>& out_points )
{
	threshold(image, image, 100, 255, CV_THRESH_BINARY);
	medianBlur(image, image, 5);

	//imshow("after threshold&&blur", IR_gray);

	vector<vector<Point>> contours;
	findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//drawContours(src, contours, -1, 255);
	//imshow("draw contours", src);

	double area, max_area = 100;
	int max_idx = -1;
	for (int i = 0;  i < contours.size(); ++i) {
		area = abs(contourArea(contours[i]));
		if (area > max_area) {
			max_idx = i;
			max_area = area;
		}
	}
	//cout << max_idx << endl;
	if (max_idx < 0) {
		return false;
	}

	//vector<Point> poly;
	auto& poly = out_points;
	approxPolyDP(contours[max_idx], poly, arcLength(contours[max_idx], true) * 0.02, true);
	//cout << poly.size();

	Mat result;
	cvtColor(image, result, CV_GRAY2BGR);
	for (int i = 0; i < poly.size(); ++i) {
		cv::line(result, poly[i], poly[(i+1)%poly.size()], Scalar(255, 0, 0));
	}
	imshow("result", result);
	return true;
}


template<typename T>
float calc_STD( const vector<T>& v )
{
	float sum = std::accumulate(v.begin(), v.end(), 0.0f);
	float mean = float(sum) / v.size();
	float ret_std = 0.f;
	for (auto i : v) {
		ret_std += (i - mean) * (i - mean);
	}
	ret_std = sqrtf(ret_std / float(v.size()-1));
	return ret_std;
}

// corners: left_top, left_bottom, right_top, right_bottom
float calc_lateral_noise( const Mat& depth_gray, const Point2f corners[4], float* out_x_left, float* out_x_right)
{
	//cv::line(depth_gray, p1, p2, cv::Scalar(0, 0, 255), 1, 8); // blue
	//imshow("draw line", depth_gray);
	assert(corners[0].y < corners[1].y);
	assert(corners[2].y < corners[3].y);
	assert(corners[0].x < corners[2].x); 
	assert(corners[1].x < corners[3].x);

	Mat depth_threshold;
	const int threshold_value = 30;
	threshold(depth_gray, depth_threshold, threshold_value, 255, THRESH_BINARY);
	//imshow("depth_threshold", depth_threshold);
	//depth_gray = depth_threshold;
	depth_threshold.copyTo(depth_gray);
	
	Mat depth_rgb;
	cvtColor(depth_gray, depth_rgb, CV_GRAY2RGB);
	{
		Scalar red(0, 0, 255);
		cv::line(depth_rgb, corners[0], corners[1], red);
		cv::line(depth_rgb, corners[1], corners[3], red);
		cv::line(depth_rgb, corners[3], corners[2], red);
		cv::line(depth_rgb, corners[2], corners[0], red);
	}

	vector<vector2f> edge_point2ds;
	vector<float> dist_to_line;

	// left edge and right edge
	for (int edge_index = 0; edge_index < 2; ++edge_index)
	{
		auto& top = corners[edge_index*2];
		auto& bottom = corners[edge_index*2+1];

		edge_point2ds.clear();
		edge_point2ds.reserve(int(bottom.y - top.y) + 1);

		for (int y = top.y; y <= (int)bottom.y; y++) {
			int xx = (int)lerp(top.x, bottom.x, float(y - top.y + 1) / (bottom.y - top.y + 1));
			int x = 10;
			bool found = false;
			const int offset = 10;
			if (edge_index == 0) { // left
				for (x = offset; x >= -offset; x--) {
					if (depth_gray.at<unsigned char>(y, x + xx) <= 128) {
						++x;
						found = true;
						break;
					}
				}
			} 
			else { // right
				for (x = -offset; x <= offset; x++) {
					if (depth_gray.at<unsigned char>(y, x + xx) <= 128) {
						--x;
						found = true;
						break;
					}
				}
			}

			if (!found) {
				//std::cout << "[Error] edge not found!\n";
				error("edge point not found(y=%d)\n", y);
				continue;
			}
			//cout << x + xx << endl;
			edge_point2ds.push_back(vector2f(x + xx, y));
		}

		if (edge_point2ds.size() <= 2) {
			error("no enough edge points\n");
			return -1;
		}

		// debug edge points
		//Scalar red(255, 0, 0);
		float edge_x = edge_index == 0 ? -1 : 100000;
		Vec3b red(0, 0, 255);
		for (auto& p : edge_point2ds) {
			//cout << "(" << p.x << ", " << p.y << ")\n";
			//cv::circle(depth_rgb, Point(p.x, p.y), .5f, red);
			edge_index == 0 ? edge_x = max(edge_x, p.x) : edge_x = min(edge_x, p.x);
			depth_rgb.at<Vec3b>(p.y, p.x) = red;
		}
		imshow("edge points", depth_rgb);
		//waitKey();

		vector2f o, d;
		vector<vector2f> edge_point2ds_swap_xy;
		edge_point2ds_swap_xy.reserve(edge_point2ds.size());
		for (auto& p : edge_point2ds) {
			edge_point2ds_swap_xy.push_back(vector2f(p.y, p.x));
		}
		linear_regression(edge_point2ds_swap_xy, o, d);
		//cout << "o = [" << o.x << ", " << o.y << "] , d = [" << d.x << ", " << d.y << "]\n";

		//float A = d.y, B = -d.x, C = o.y * d.x - o.x * d.y;
		float A = -d.x, B = d.y, C = o.y * d.x - o.x * d.y;
		if (A < 0) {
			A = -A; B = -B; C = - C;
		}
		std::cout << "fitted line: A = " << A << ",  B = " << B << ", C = " << C << endl;

		if (A <= 0.9f) {
			error("[fitted line] A <= 0.9\n");
			for (auto& p : edge_point2ds) {
				std::cout << "(" << p.x << ", " << p.y << ")\n";
			}
		}

		//{
		//	float y0 = top.y;
		//	float y1 = bottom.y;
		//	if (i == 0) {
		//		*out_x_left  = max((B*y0+C)/(-A), (B*y1+C)/(-A));
		//	} else {
		//		*out_x_right = min((B*y0+C)/(-A), (B*y1+C)/(-A));
		//	}
		//}
		edge_index == 0 ? *out_x_left = edge_x : *out_x_right = edge_x;

		float t = 1.0f / sqrtf(A*A + B*B);
		dist_to_line.reserve(dist_to_line.size() + edge_point2ds.size());
		for (auto& p : edge_point2ds) {
			float d = fabsf(A * p.x + B * p.y + C) * t;
			dist_to_line.push_back(d);
			//cout << d << endl;
		}
	}

	std::cout << "out_x_left = " << *out_x_left << ", out_x_right = " << *out_x_right << endl;
	
	float lateral_noise = calc_STD(dist_to_line);
	//std::cout << "lateral noise: " << lateral_noise << endl;
	warning("lateral noise: %lf\n", lateral_noise);
	return lateral_noise;
}

void calc_points( const vector<UINT16>& depth_pixels, cv::Rect roi , vector<vector3f>& point_cloud, const int width, const int height, const float fovx, const float fovy )
{
	point_cloud.resize(roi.width * roi.height);
	const float DegreesToRadians = 3.14159265359f / 180.0f;
	const float xScale = tanf(fovx * DegreesToRadians * 0.5f) * 2.0f / width;
	const float yScale = tanf(fovy * DegreesToRadians * 0.5f) * 2.0f / height;
	int	half_width = width / 2;
	int	half_height = height / 2;
	for (int j = 0; j < roi.height; j++){
		for (int i = 0; i < roi.width; i++){
			//unsigned short pixel_depth = depth_pixels[idx];
			//UINT16 pixel_depth = depth_image.at<UINT16>(roi.y + j, roi.x + i);
			int x = i + roi.x;
			int y = j + roi.y;

			int index = y * width + x;
			auto pixel_depth = depth_pixels[index];
			float	depth = -pixel_depth * 0.001f;	//	unit in meters
			//cout << depth << endl;

			int idx = j*roi.width + i;
			point_cloud[idx].x = -(x + 0.5f - half_width) * xScale * depth;
			point_cloud[idx].y = (y + 0.5f - half_height) * yScale * depth;
			point_cloud[idx].z = depth;
			//cout << depth << endl;
		}
	}

	return;
}

vector3f pca( const std::vector<vector3f> &points )
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

void linear_regression( const vector<vector2f>& points, vector2f& o, vector2f& d )
{
	const float n = (float)points.size();
	assert(n >= 2);
	float ave_x = 0.f, ave_y = 0.f;
	float cross_sum = 0.f, sqr_sum = 0.f;
	for (auto &p :points) {
		ave_x += p.x/n;
		ave_y += p.y/n;
		cross_sum += p.x*p.y;
		sqr_sum += p.x*p.x;
	}
	d.x = sqr_sum - n*ave_x*ave_x;
	d.y = d.x==0.f?1.f:(cross_sum -n*ave_x*ave_y);
	d.normalize();

	o.x = ave_x;
	o.y = ave_y;
}

double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}