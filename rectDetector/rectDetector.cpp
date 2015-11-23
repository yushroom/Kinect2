#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <io.h>
#include <algorithm>

using namespace cv;
using namespace std;

typedef unsigned short UINT16;

static int scale = 2;
static const int max_scale = 5;
static int thresh = 5;
static int max_thresh = 10;

static cv::Point corners[4];
static int corner_index = 0;
static cv::Point last_corners[4];

template<typename T>
static Mat vector_to_img_uc(const vector<T>& vec, int w, int h, float scale = 1) {
	Mat img(h, w, CV_8U);
	auto max_ele = *std::max_element(vec.begin(), vec.end());
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img.at<unsigned char>(j, i) = unsigned char(float(vec[idx]) / max_ele * 255.f);
		}
	}
	return img;
}

static void load_raw_depth_from_bin_file(string depth_bin_path, vector<UINT16>& out_raw_depth, int width, int height)
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

inline float dist(float x, float y, const Point& p) {
	return sqrtf((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y));
}

void on_mouse(int event, int x, int y, int flags, void* dr)
{
	//auto& p = corners[corner_index];
	if (event == cv::EVENT_LBUTTONDOWN) {
		//cout << "down" << endl;
		int re_select_index = -1;
		for (int i = 0; i < corner_index; ++i) {
			if (dist(x, y, corners[i]) <= 4) {
				re_select_index = i;
				break;
			}
		}

		if (re_select_index >= 0) {
			//cout << "move corner " << re_select_index;
			auto& p = corners[re_select_index];
			p.x = x;
			p.y = y;
		}
		else {
			if (corner_index < 4) {
				//cout << "add corner " << corner_index << endl;
				auto& p = corners[corner_index];
				p.x = x;
				p.y = y;
				corner_index++;
			}

		}
	}

	if (flags == cv::EVENT_FLAG_LBUTTON) {
		//cout << "flag" << endl;
		int re_select_index = -1;
		for (int i = 0; i < corner_index; ++i) {
			if (dist(x, y, corners[i]) <= 4) {
				re_select_index = i;
				break;
			}
		}

		if (re_select_index >= 0) {
			//cout << "move corner " << re_select_index;
			auto& p = corners[re_select_index];
			p.x = x;
			p.y = y;
		}
	}
}

void thresh_callback(int, void*)
{
	
}

#define PATH_FILE "D:\\yyk\\capture\\person\\path.txt"

bool process(const string& image_path)
{
	cout << "detect " << image_path << endl;
	cv::namedWindow("IR", 1);
	cv::namedWindow("Depth", 1);
	cv::setMouseCallback("IR", on_mouse, 0);
	cv::setMouseCallback("Depth", on_mouse, 0);
	cv::createTrackbar("scale", "Depth", &scale, max_scale);
	cv::createTrackbar("thresh", "Depth", &thresh, max_thresh);

	cv::Mat img = cv::imread(image_path, 1);

	if (img.rows == 0) {
		cout << "image " << image_path << " not exists.\n";
		waitKey();
		exit(1);
	}

	string out_file_name = image_path.substr(0, image_path.size() - 3) + "txt";
	{
		if (_access_s(out_file_name.c_str(), 0) == 0) {
			ifstream fin(out_file_name);
			vector<int> c;
			int t;
			while (fin >> t)
				c.push_back(t);
			if (c.size() != 8) {
				cout << "corners in file not match!\n";
			}
			else {
				for (int i = 0; i < 4; ++i) {
					corners[i].x = c[i * 2];
					corners[i].y = c[i * 2 + 1]; 
				}
				corner_index = 4;
				cout << "    load corners from file.\n";
			}
		}
	}

	string depth_bin_path = image_path.substr(0, image_path.size() - 3) + "bin";

	vector<UINT16> raw_depth;
	load_raw_depth_from_bin_file(depth_bin_path, raw_depth, img.cols, img.rows);

	Mat depth_gray, depth_rgb;
	depth_gray = vector_to_img_uc(raw_depth, img.cols, img.rows, 0.08f);
	cvtColor(depth_gray, depth_rgb, CV_GRAY2BGR);

	//imshow("Depth", depth_rgb);

	int last_corner = 0;
	while (true) {

		Mat canvas_IR, canvas_depth;
		img.copyTo(canvas_IR);
		depth_rgb.copyTo(canvas_depth);
		cv::threshold(canvas_depth, canvas_depth, 255.0f * float(thresh) / max_thresh, 255, THRESH_TOZERO_INV);
		canvas_depth *= float(scale);

		for (int i = 0; i < corner_index; ++i) {
			cv::circle(canvas_IR, corners[i], 6, Scalar(0, 0, 255), 2);
			cv::circle(canvas_depth, corners[i], 6, Scalar(0, 0, 255), 2);
		}

		for (int i = 1; i < corner_index; ++i) {
			cv::line(canvas_IR, corners[i - 1], corners[i], cv::Scalar(0, 255, 255), 1);
			cv::line(canvas_depth, corners[i - 1], corners[i], cv::Scalar(0, 255, 255), 1);
		}
		if (corner_index == 4) {
			cv::line(canvas_IR, corners[0], corners[3], cv::Scalar(0, 255, 255), 1);
			cv::line(canvas_depth, corners[0], corners[3], cv::Scalar(0, 255, 255), 1);
		}

		cv::imshow("IR", canvas_IR);
		cv::imshow("Depth", canvas_depth);
		//cv::rectangle(img, point_c, point, CV_RGB(0, 0, 255), 2);

		char c = cv::waitKey(33.3f);
		if (c == 27) // ESC
			exit(1);
		else if (c == 'c' || c == 'C') {
			corner_index = 0;
		}
		else if (c == '-') {
			corner_index--;
			if (corner_index < 0)
				corner_index = 0;
		}
		else if (c == '+') {
			corner_index++;
		}
		else if (c == 'p' || c == 'P') {
			cout << "next corner: " << corner_index << endl;
		}
		else if (c == 13) { // Enter
			// save corners and -> next image
			//string out_file_name = image_path.substr(0, image_path.size() - 3) + "-IR.txt";
			if (corner_index >= 4) {
				ofstream fout(out_file_name);
				cout << "write corners to: " << out_file_name << endl;
				for (int i = 0; i < 4; ++i) {
					fout << corners[i].x << ' ' << corners[i].y << ' ';
				}
				fout.close();
				for (int i = 0; i < 4; i++)
					last_corners[i] = corners[i];
			}

			//corner_index = 0;
			return true;
		}
		else if (c == 'A') {
			for (int i = 0; i < 4; i++)
				corners[i] = last_corners[i];
		}
		else if (c == 'O') {	// undo
			return false;
		}
	}
}

int main()
{
	string path = "./path_for_rectDetector.txt";
	ifstream fin(path);
	if (!fin) {
		cout << "file " << path << " not exists.\n";
		return 1;
	}
	string str;
	vector<string> image_path_list;
	while (std::getline(fin, str)) {
		//cout << str << endl;
		image_path_list.push_back(str);
	}

	for (int img_idx = 0; img_idx < image_path_list.size(); ++img_idx) {
		if (!process(image_path_list[img_idx])) {
			img_idx -= 2;
			if (img_idx < 0) img_idx = 0;
		}
	}
	return 0;
}