#define _CRT_SECURE_NO_WARNINGS
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

using codex::math::vector3f;
using codex::math::vector2f;

using namespace std;
using namespace cv;
typedef unsigned short UINT16;
typedef UINT16 DepthType;

#define WIDTH 640
#define HEIGHT 480
#define WIDTH_COLOR 1440
#define HEIGHT_COLOR 1080

#define NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV                 (58.5f)
#define NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV                   (45.6f)
#define V1_FOV_X NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV
#define V1_FOV_Y NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV

template<typename T>
Mat vector_to_img_uc(const vector<T>& vec, int w, int h, float scale = 1) {
	Mat img(h, w, CV_8U);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img.at<unsigned char>(j, i) = unsigned char(vec[idx] * scale);
		}
	return img;
}

double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

std::vector<std::vector<cv::Point>> findSquaresInImage(cv::Mat _image)
{
	std::vector<std::vector<cv::Point> > squares;
	cv::Mat pyr, timg, gray0(_image.size(), CV_8U), gray;
	int thresh = 50, N = 11;
	cv::pyrDown(_image, pyr, cv::Size(_image.cols / 2, _image.rows / 2));
	cv::pyrUp(pyr, timg, _image.size());
	std::vector<std::vector<cv::Point> > contours;
	for (int c = 0; c < 3; c++) {
		int ch[] = { c, 0 };
		mixChannels(&timg, 1, &gray0, 1, ch, 1);
		//auto& gray0 = timg;
		for (int l = 0; l < N; l++) {
			if (l == 0) {
				cv::Canny(gray0, gray, 0, thresh, 5);
				cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));
			}
			else {
				gray = gray0 >= (l + 1) * 255 / N;
			}
			cv::findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
			std::vector<cv::Point> approx;
			for (size_t i = 0; i < contours.size(); i++)
			{
				cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
				if (approx.size() == 4 && fabs(contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx))) {
					double maxCosine = 0;

					for (int j = 2; j < 5; j++)
					{
						double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					if (maxCosine < 0.3) {
						squares.push_back(approx);
					}
				}
			}
		}
	}
	return squares;
}

cv::Mat debugSquares(const std::vector<std::vector<cv::Point> >& squares, cv::Mat& image)
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

void find_rect(Mat image)
{
	cv::imshow("input2", image);
	//Mat depth_mat_denoise;
	//cv::pyrMeanShiftFiltering(image, depth_mat_denoise, 25, 10);
	//imshow("depth denoise", depth_mat_denoise);
	Mat mask;
	Mat temp;
	//cv::bilateralFilter(image, temp, 5, 5 * 2, 5 / 2);
	
	//cv::imshow("after", temp);

	threshold(image, mask, 40, 255, CV_THRESH_BINARY);
	cv::imshow("mask", mask);

	// find contours
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	

	// draw contours and find biggest contour
	int biggest_contour_index = -1;
	float biggest_contour_area = 0;
	Mat drawing = Mat::zeros(mask.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); ++i) {
		Scalar color(0, 100, 0);
		//Scalar color(100);
		drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point());
		float ct_area = (float)contourArea(contours[i]);
		if (ct_area > biggest_contour_area) {
			biggest_contour_area = ct_area;
			biggest_contour_index = i;
		}
	}

	if (biggest_contour_index < 0) {
		cout << "no contour found" << endl;
		//return -1;
	}

	RotatedRect bounding_box = cv::minAreaRect(contours[biggest_contour_index]);
	Point2f corners[4];
	bounding_box.points(corners);
	Scalar white(255, 255, 255);
	cv::line(drawing, corners[0], corners[1], white);
	cv::line(drawing, corners[1], corners[2], white);
	cv::line(drawing, corners[2], corners[3], white);
	cv::line(drawing, corners[3], corners[4], white);

	// display
	
	cv::imshow("drawing2", drawing);
}

void boarder(Mat image) {
	for (int j = 0; j < image.rows; ++j) {
		image.at<unsigned char>(j, 0) = 0;
		image.at<unsigned char>(j, 1) = 0;
		image.at<unsigned char>(j, image.cols-1) = 0;
		image.at<unsigned char>(j, image.cols - 2) = 0;
	}		
	for (int i = 0; i < image.cols; ++i) {
		image.at<unsigned char>(0, i) = 0;
		image.at<unsigned char>(1, i) = 0;
		image.at<unsigned char>(image.rows-1, i) = 0;
		image.at<unsigned char>(image.rows - 2, i) = 0;
	}
}

vector3f pca(const std::vector<vector3f> &points) {
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

/**linear fit for a set of 2D points, 
   points on the resulting line can be expressed as p = o + d*t, 
* 
* */
void linear_regression(const vector<vector2f>& points, vector2f& o, vector2f& d) {
	const int n = points.size();
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

static void calc_points(const vector<UINT16>& depth_pixels, cv::Rect roi , vector<vector3f>& point_cloud, const int width, const int height,
	const float fovx, const float fovy)
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

void find_squares(Mat& image, vector<vector<Point> >& squares)
{
	// blur will enhance edge detection
	Mat blurred(image);
	medianBlur(image, blurred, 9);

	Mat gray0(blurred.size(), CV_8U), gray;
	vector<vector<Point> > contours;

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
}

inline float lerp(float a, float b, float t) {
	return (1-t) * a + t *b;
}

template<typename T>
static float calc_STD(const vector<T>& v) {
	float sum = std::accumulate(v.begin(), v.end(), 0.0f);
	float mean = float(sum) / v.size();
	float ret_std = 0.f;
	for (auto i : v) {
		ret_std += (i - mean) * (i - mean);
	}
	ret_std = sqrtf(ret_std / (v.size()-1));
	return ret_std;
}

// corners: left_top, left_bottom, right_top, right_bottom
float get_lateral_noise(const Mat& depth_gray, const Point2f corners[4])
{
	//cv::line(depth_gray, p1, p2, cv::Scalar(0, 0, 255), 1, 8); // blue
	//imshow("draw line", depth_gray);

	assert(corners[0].y < corners[1].y);
	assert(corners[2].y < corners[3].y);
	assert(corners[0].x < corners[2].x);
	assert(corners[1].x < corners[3].x);

	Mat depth_threshold;
	threshold(depth_gray, depth_threshold, 10, 255, THRESH_BINARY);
	imshow("depth_threshold", depth_threshold);

	vector<vector2f> edge_point2ds;
	vector<float> dist_to_line;
	for (int i = 0; i < 2; ++i)
	{
		auto& top = corners[i*2];
		auto& bottom = corners[i*2+1];

		edge_point2ds.clear();
		edge_point2ds.reserve(bottom.y - top.y + 1);

		for (int y = top.y; y <= bottom.y; y++) {
			int xx = (int)lerp(top.x, bottom.x, float(y - top.y + 1) / (bottom.y - top.y + 1));
			int x = 10;
			bool found = false;
			if (i == 0) { // left
				for (x = 10; x >= -10; x--) {
					if (depth_gray.at<unsigned char>(y, x + xx) <= 20) {
						--x;
						found = true;
						break;
					}
				}
			} 
			else { // right
				for (x = -10; x <= 10; x++) {
					if (depth_gray.at<unsigned char>(y, x + xx) <= 20) {
						++x;
						found = true;
						break;
					}
				}
			}

			if (!found) {
				std::cout << "[Error] edge not found!\n";
				continue;
			}
			//cout << x + xx << endl;
			edge_point2ds.push_back(vector2f(x + xx, y));
		}

		vector2f o, d;
		linear_regression(edge_point2ds, o, d);
		//cout << "o = [" << o.x << ", " << o.y << "] , d = [" << d.x << ", " << d.y << "]\n";

		float A = d.y, B = -d.x, C = o.y * d.x - o.x * d.y;
		std::cout << "fitted line: A = " << A << ",  B = " << B << ", C = " << C << endl;

		float t = 1.0f / sqrtf(A*A + B*B);

		dist_to_line.reserve(dist_to_line.size() + edge_point2ds.size());
		for (auto& p : edge_point2ds) {
			float d = fabsf(A * p.x + B * p.y + C) * t;
			//cout << d << endl;
			dist_to_line.push_back(d);
			//cout << d << endl;
		}

	}

	float lateral_noise = calc_STD(dist_to_line);
	std::cout << "lateral noise: " << lateral_noise << endl;
	return lateral_noise;
}

void process(string depth_bin_path, string IR_path, int width, int height)
{
	vector<UINT16> raw_depth(width * height, 0);
	cout << depth_bin_path << endl;
	ifstream is(depth_bin_path, ios::binary);
	if (!is.good()) {
		cout << "Error: file " << depth_bin_path << " not found.\n";
	}
	is.read((char*)&raw_depth[0], width * height * sizeof(UINT16));
	is.close();

	Mat depth_gray = vector_to_img_uc(raw_depth, width, height, 0.1);
	imshow("raw depth", depth_gray);

	Mat depth_rgb;
	cvtColor(depth_gray, depth_rgb, CV_GRAY2BGR);

	Mat ir_mat = imread(IR_path);
	//Mat ir_mat_gray;
	//cv::cvtColor(ir_mat, ir_mat_gray, CV_BGR2GRAY);
	//Mat temp;
	//cv::pyrMeanShiftFiltering(ir_mat, temp, 5, 100);
	//cv::imshow("after meanshift", temp);
	//cv::cvtColor(temp, ir_mat_gray, CV_BGR2GRAY);

	
	Mat drawing = Mat::zeros(ir_mat.size(), CV_8UC3);
	imshow("IR", ir_mat);
	//debugSquares(findSquaresInImage(temp), drawing);
	//cv::imshow("drawing", drawing);


	//boarder(ir_mat_gray);
	//threshold(ir_mat_gray, ir_mat_gray, 20, 255, 0);
	//find_rect(ir_mat_gray);

	vector<vector<Point> > points;
	find_squares(ir_mat, points);
	Mat depth_rgb_rect;
	depth_rgb.copyTo(depth_rgb_rect);
	debugSquares(points, depth_rgb_rect);
	cv::imshow("draw rect", depth_rgb_rect);

	cv::RotatedRect minRect = minAreaRect(cv::Mat(points[0]));
	cv::Point2f rect_points[4];
	minRect.points(rect_points);
	
	Point2f corners[4];
	// 0 1 2 3
	// left_top left_bottom right_top right_bottom
	auto mean_p2 = std::accumulate(rect_points, rect_points+3, Point2f(0, 0)) / 4.0f;
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

	int top_y    = max(corners[0].y, corners[2].y);
	int bottom_y = min(corners[1].y, corners[3].y);
	const int h = bottom_y - top_y;
	assert(h > 0);
	int y_offset_top = int(h * 0.1f);
	int y_offset_bottom = int(h * 0.3f);

	corners[2].y = corners[0].y = top_y + y_offset_top;		// top
	corners[3].y = corners[1].y = bottom_y - y_offset_bottom;			// bottom

	float lateral_noise = get_lateral_noise(depth_gray, corners);
	assert(lateral_noise >= 0);
	int x_offset = (3.0f * lateral_noise);
	cout << x_offset << endl;

	// get axial noise
	int left_x  = max(corners[0].x, corners[1].x) + x_offset;
	int right_x = min(corners[2].x, corners[3].x) - x_offset;
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

	cv::Rect roi(left_x, corners[0].y, right_x - left_x, h);
	//imshow("roi", depth_rgb(roi));
	vector<vector3f> proj_points;
	calc_points(raw_depth, roi, proj_points, width, height, V1_FOV_X, V1_FOV_Y);
	auto normal = pca(proj_points);
	//cout << "normal of fitted plane: [" << normal.x << ' ' << normal.y << ' ' << normal.z << ']' << endl;

	vector3f mean_p3 = std::accumulate(proj_points.begin(), proj_points.end(), vector3f(0, 0, 0)) / proj_points.size();
	cout << "mean point: [" << mean_p3.x << ' ' << mean_p3.y << ' ' << mean_p3.z << ']' << endl;

	float A = normal.x, B = normal.y, C = normal.z;
	float D = -(A * mean_p3.x + B * mean_p3.y + C * mean_p3.z);
	cout << "fitted plane: A = " << A << ", B = " << B << ", C = " << C << ", D = " << D << endl;

	vector<float> depth_diff(roi.width * roi.height);
	float t = 1.0f / sqrtf(A*A+B*B+C*C);
	for (int j = 0; j < roi.height; ++j) {
		for (int i = 0; i < roi.width; ++i) {
			int x = i + roi.x;
			int y = j + roi.y;
			int idx1 = y * width + x;
			int idx2 = j * roi.width + i;	// index in roi region

			auto& p = proj_points[idx2];
			float dist_to_plane = 0.f;
			depth_diff[idx2] = fabsf(A*p.x + B*p.y + C*p.z + D) * t;
			//cout << depth_diff[idx2] << endl;
		}
	}

	float axial_noise = calc_STD(depth_diff);
	cout << "axial noise: " << axial_noise;
}

int main()
{
	string number_ID = "D:\\yyk\\image\\NoiseModel_1113\\1447405285-0";
	process(number_ID + "-a.bin", number_ID + "-a-IR.bmp", 640, 480);
	//process("D:\\yyk\\image\\NoiseModel_1113\\1447405324-1-b.bin",
	//	"D:\\yyk\\image\\NoiseModel_1113\\1447405324-1-b-IR.bmp", 512, 424);
	waitKey();
	return 0;
}