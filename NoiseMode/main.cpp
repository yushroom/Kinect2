#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include <utility>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <algorithm>

using namespace std;
using namespace cv;
typedef unsigned short UINT16;

#define WIDTH 640
#define HEIGHT 480
#define WIDTH_COLOR 1440
#define HEIGHT_COLOR 1080

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

cv::Mat debugSquares(std::vector<std::vector<cv::Point> > squares, cv::Mat image)
{
	for (int i = 0; i < squares.size(); i++) {
		// draw contour
		//cv::drawContours(image, squares, i, cv::Scalar(255, 0, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

		// draw bounding rect
		cv::Rect rect = boundingRect(cv::Mat(squares[i]));
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
		float ct_area = contourArea(contours[i]);
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
	debugSquares(points, depth_rgb);
	cv::imshow("drawing", depth_rgb);
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