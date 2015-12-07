#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include <utility>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <algorithm>

using namespace cv;
using namespace std;

#define WIDTH 640
#define HEIGHT 480
#define WIDTH_COLOR 1440
#define HEIGHT_COLOR 1080

const float DEPTH_INVALID = 1e30f;
#define VALID_DEPTH_TEST(a) (a < DEPTH_INVALID)

#define EXTRINSICS_FILE_PATH "D:\\yyk\\image\\extrinsics.yml"
#define INTRINSICS_FILE_PATH "D:\\yyk\\image\\intrinsics.yml"
#define PATH_FILE "D:\\yyk\\image\\IRandRGB\\test_data\\path.txt"

#define V1_FX (586.881)
#define V1_FY (585.482)
#define V1_CX (321.222)
#define V1_CY (232.1)

//#define V2_FX (364.440)
//#define V2_FY (363.688)
//#define V2_CX (259.818)
//#define V2_CY (207.523)

#define V2_RESIZED_FX (413.864)
#define V2_RESIZED_FY (412.790)
#define V2_RESIZED_CX (325.169)
#define V2_RESIZED_CY (234.573)

#define COLOR1_FX (1.2190619346697913e+003)
#define COLOR1_FY (1.2142525420277843e+003)
#define COLOR1_CX (7.1961076663682809e+002)
#define COLOR1_CY (5.8672029500349447e+002)

#define COLOR2_FX (1.0430106891024377e+003)
#define COLOR2_FY (1.0424784070539822e+003)
#define COLOR2_CX (7.2206218448958009e+002)
#define COLOR2_CY (5.2676160551478370e+002)

vector<pair<float, float>>	undistortLookupTable_depth[2];
vector<pair<float, float>>	undistortLookupTable_color[2];
Mat R_IR1_IR2;
Mat R_color1_color2;
Mat R_IR2_color2;
Mat T_IR1_IR2;
Mat T_color1_color2;
Mat T_IR2_color2;
Mat RT_IR1_to_IR2;
Mat RT_color1_to_color2;
Mat RT_IR2_to_color2;

typedef unsigned short UINT16;

Mat ResizeAndToCVImage_depth(const vector<UINT16>& depth_pixels){
	Mat mat = Mat::zeros(Size(512, 424), CV_16UC1);
	for (int j = 0; j < 424; j++) {
		for (int i = 0; i < 512; ++i) {
			mat.at<UINT16>(j, i) = depth_pixels[j * 512 + i];
		}
	}
	//printf("%d %d\n", mat.cols, mat.rows);
	cv::resize(mat, mat, Size(580, 480), 0, 0, INTER_NEAREST);
	//printf("%d %d\n", mat.cols, mat.rows);
	Mat new_mat = Mat::zeros(Size(640, 480), mat.type());
	mat.copyTo(new_mat(Rect(30, 0, 580, 480)));
	return new_mat;
}

Mat ToCVImage_depth(const vector<UINT16>& src)
{
	Mat temp(Size(WIDTH, HEIGHT), CV_16UC1);
	for (int j = 0; j < HEIGHT; j++) {
		for (int i = 0; i < WIDTH; ++i) {
			temp.at<UINT16>(j, i) = src[j * WIDTH + i];
		}
	}
	return temp;
}

template<typename T>
vector<T> mat_to_vector(const Mat& img) {
	int w = img.cols;
	int h = img.rows;
	//cout << w << ' ' << h <<endl;
	vector<T> vec;
	vec.resize(w*h);
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			vec[idx] = img.at<T>(j, i);
		}
	}
	return vec;
}

template<typename T>
Mat vector_to_img_uc(const vector<T>& vec, int w = WIDTH, int h = HEIGHT, float scale = 1) {
	Mat img(h, w, CV_8U);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img.at<unsigned char>(j, i) = unsigned char(vec[idx] * scale);
		}
	return img;
}

template <typename T>
inline float bilinear(float x, float y, const vector<T>& src, int width, int height, bool ignore_zero = false) {

	int ix0 = x, iy0 = y, ix1 = ix0 + 1, iy1 = iy0 + 1;
	float qx1 = x - ix0, qy1 = y - iy0, qx0 = 1.f - qx1, qy0 = 1.f - qy1;

	float weight = 0.f, value = 0.f;
	float w[] = { qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1 };
	//pair<int, int> xy[] = { { ix0, iy0 }, { ix1, iy0 }, { ix0, iy1 }, { ix1, iy1 } };
	int xx[] = { ix0, ix1, ix0, ix1 };
	int yy[] = { iy0, iy0, iy1, iy1 };

	for (int j = 0; j < 4; j++) {
		if (xx[j] < 0 || xx[j] >= width || yy[j] < 0 || yy[j] >= height)
			continue;
		int idx = xx[j] + yy[j] * width;
		if (ignore_zero && src[idx] == 0) continue;
		value += src[idx] * w[j];
		weight += w[j];
	}

	return (weight == 0.f ? DEPTH_INVALID : value / weight);
	//return static_cast<T>(weight == 0.f ? DEPTH_INVALID : value / weight);
}


Mat average_color_pixels(const Mat& color_image, int w = WIDTH, int h = HEIGHT)
{
	//vector<float> ret(w * h, 0);
	Mat ret = Mat::zeros(h, w, color_image.type());
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			//int idx = j * WIDTH + i;
			int count = 0;
			float sum = 0;
			for (int k = -1; k <= 1; ++k) {
				if (j + k < 0 || j + k >= h)
					continue;
				for (int l = -1; l <= 1; ++l) {
					if (l + i < 0 || l + i >= w)
						continue;
					//int idx = (j + k) * w + (i + l);
					unsigned char val = color_image.at<unsigned char>(j+k , i+l);
					if (val != 0) {
						count++;
						sum += val;
					}
				}
			}
			//int idx = j * w + i;
			//assert(count != 0);
			if (count != 0)
				ret.at<unsigned char>(j, i) = unsigned char(sum / count);
		}
	}
	return ret;
}

Mat calc_points(const vector<float>& depth, float fx, float fy, float cx, float cy, bool cropped = true, cv::Rect roi = Rect(0, 0, WIDTH, HEIGHT)) {
	float inv_fx = 1.f / fx;
	float inv_fy = 1.f / fy;
	//float x, y, z;
	int w = roi.width;
	int h = roi.height;
	Mat points = Mat::zeros(4, w * h, CV_32F);
	for (int j = roi.y; j < roi.y + roi.height; ++j)
		for (int i = roi.x; i < roi.x + roi.width; ++i) {
			int idx = (j - roi.y) * w + (i - roi.x);
			float d = depth[idx];
			if (!VALID_DEPTH_TEST(d))
				continue;
			//T d = cropped ? depth_image.at<T>(j-roi.y, i-roi.x) : depth_image.at<f>(j, i);
			//float x_scale = inv_fx * d;
			//float y_scale = inv_fy * d;
			points.at<float>(0, idx) = (i + 0.5f - cx) * inv_fx * d; // x
			points.at<float>(1, idx) = (j + 0.5f - cy) * inv_fy * d;
			points.at<float>(2, idx) = d;
			points.at<float>(3, idx) = 1;
		}

	return points;
}

enum ProjectionType {
	v1_to_v2 = 0,
	v2_to_v1 = 1
};

void process(const string& color1_path, const string& color2_path, const string& depth1_path, const string& depth2_path, const string& prefix, const string& bin_path_v1tov2, const string& bin_path_v2tov1, bool save_temp_file)
{
	vector<unsigned char> color_und[2];
	vector<float> depth_und[2];
	{
		// read depth file
		vector<UINT16>	depth_pixels[2];
		depth_pixels[0].resize(WIDTH*HEIGHT);
		depth_pixels[1].resize(512 * 424);
		{
			FILE *depth_in;
			cout << depth1_path << endl;
			fopen_s(&depth_in, depth1_path.c_str(), "rb");
			fread(&depth_pixels[0][0], sizeof(UINT16), depth_pixels[0].size(), depth_in);
			fclose(depth_in);
			cout << depth2_path << endl;
			fopen_s(&depth_in, depth2_path.c_str(), "rb");
			fread(&depth_pixels[1][0], sizeof(UINT16), depth_pixels[1].size(), depth_in);
			fclose(depth_in);
		}

		Mat img_color[2];
		Mat img_depth[2];
		vector<unsigned char> color[2];

		//img_depth[0] = ToCVImage_depth(depth_pixels[0]);
		img_depth[1] = ResizeAndToCVImage_depth(depth_pixels[1]);
		depth_pixels[1] = mat_to_vector<UINT16>(img_depth[1]);

		std::cout << color1_path << '\n' << color2_path << '\n';
		// size 1440 * 1080
		img_color[0] = imread(color1_path, 0);
		img_color[1] = imread(color2_path, 0);

		for (int i = 0; i < 2; ++i) {
			color[i] = mat_to_vector<unsigned char>(img_color[i]);
			color_und[i].resize(WIDTH_COLOR * HEIGHT_COLOR);
			depth_und[i].resize(WIDTH * HEIGHT);
		}

		for (int j = 0; j < 2; ++j) {
			for (int i = 0; i < WIDTH * HEIGHT; i++) {
				auto entry = undistortLookupTable_depth[j][i];
				depth_und[j][i] = bilinear<UINT16>(entry.first, entry.second, depth_pixels[j], WIDTH, HEIGHT, true);
			}
			for (int i = 0; i < WIDTH_COLOR * HEIGHT_COLOR; i++) {
				auto entry = undistortLookupTable_color[j][i];
				float ir = bilinear<unsigned char>(entry.first, entry.second, color[j], WIDTH_COLOR, HEIGHT_COLOR);
				color_und[j][i] = (VALID_DEPTH_TEST(ir) ? ir : 0);
			}
		}
	}

	{
		Mat out_depth1 = vector_to_img_uc(depth_und[0], WIDTH, HEIGHT, 0.1f);
		Mat out_depth2 = vector_to_img_uc(depth_und[1], WIDTH, HEIGHT, 0.1f);
		Mat out_color1 = vector_to_img_uc(color_und[0], WIDTH_COLOR, HEIGHT_COLOR);
		Mat out_color2 = vector_to_img_uc(color_und[1], WIDTH_COLOR, HEIGHT_COLOR);
		imshow("depth 1", out_depth1);
		imshow("depth 2", out_depth2);
		imshow("color 1", out_color1);
		imshow("color 2", out_color2);
		imwrite("d:\\project_color\\depth1.bmp", out_depth1);
		imwrite("d:\\project_color\\depth2.bmp", out_depth2);
		imwrite("d:\\project_color\\color1.bmp", out_color1);
		imwrite("D:\\project_color\\color2.bmp", out_color2);
	}

	// 1->2
	{
		Mat points_1 = calc_points(depth_und[0], V1_FX, V1_FY, V1_CX, V1_CY);

		//Mat RT_IR1_to_color1 = RT_IR1_to_IR2.inv() * RT_IR2_to_color2.inv() * RT_color1_to_color2;
		Mat RT_IR1_to_color1 = RT_color1_to_color2.inv() * RT_IR2_to_color2 * RT_IR1_to_IR2;
		Mat points_1_in_color1 = RT_IR1_to_color1 * points_1;
		Mat points_1_in_color1_to_color2 = RT_color1_to_color2 * points_1_in_color1;
		Mat img_new_color1 = Mat::zeros(HEIGHT_COLOR, WIDTH_COLOR, CV_8U);

		for (int j = 0; j < HEIGHT_COLOR; ++j)
		{
			int jj = int(float(j * HEIGHT) / float(HEIGHT_COLOR) + 0.5f);
			for (int i = 0; i < WIDTH_COLOR; ++i)
			{
				int ii = int(float(i * WIDTH) / float(WIDTH_COLOR) + 0.5f);
				int idx = jj * WIDTH + ii;
				float w = points_1_in_color1_to_color2.at<float>(3, idx);
				if (w <= 1e-2f && w >= -1e-2f) continue;
				float z = points_1_in_color1_to_color2.at<float>(2, idx) / w;
				if (z <= 1e-2f && z >= -1e-2f) continue;
				int x = int(COLOR2_FX * points_1_in_color1_to_color2.at<float>(0, idx) / w / z + COLOR2_CX - 0.5f);
				int y = int(COLOR2_FY * points_1_in_color1_to_color2.at<float>(1, idx) / w / z + COLOR2_CY - 0.5f);
				if (x >= 0 && x < WIDTH_COLOR && y >= 0 && y < HEIGHT_COLOR)
				{
					img_new_color1.at<unsigned char>(y, x) = color_und[0][j * WIDTH_COLOR + i];
				}
			}
		}
		img_new_color1 = average_color_pixels(img_new_color1, WIDTH_COLOR, HEIGHT_COLOR);
		imshow("new color1 to 2", img_new_color1);
		imwrite("D:\\project_color\\color_1to2.bmp", img_new_color1);
	}

	// 2->1
	{
		Mat points_2 = calc_points(depth_und[1], V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
		Mat points_2_in_color2 = RT_IR2_to_color2 * points_2;
		cout << RT_color1_to_color2.inv() << endl << endl;
		Mat points_2_in_color2_to_color1 = RT_color1_to_color2.inv() * points_2_in_color2;
		Mat img_new_color2 = Mat::zeros(HEIGHT_COLOR, WIDTH_COLOR, CV_8U);

		for (int j = 0; j < HEIGHT_COLOR; ++j)
		{
			int jj = int(float(j * HEIGHT) / float(HEIGHT_COLOR) + 0.5f);
			for (int i = 0; i < WIDTH_COLOR; ++i)
			{
				int ii = int(float(i * WIDTH) / float(WIDTH_COLOR) + 0.5f);
				int idx = jj * WIDTH + ii;
				float w = points_2_in_color2_to_color1.at<float>(3, idx);
				if (w <= 1e-2f && w >= -1e-2f) continue;
				float z = points_2_in_color2_to_color1.at<float>(2, idx) / w;
				if (z <= 1e-2f && z >= -1e-2f) continue;
				int x = int(COLOR1_FX * points_2_in_color2_to_color1.at<float>(0, idx) / w / z + COLOR1_CX - 0.5f);
				int y = int(COLOR1_FY * points_2_in_color2_to_color1.at<float>(1, idx) / w / z + COLOR1_CY - 0.5f);
				if (x >= 0 && x < WIDTH_COLOR && y >= 0 && y < HEIGHT_COLOR)
				{
					img_new_color2.at<unsigned char>(y, x) = color_und[1][j * WIDTH_COLOR + i];
				}
			}
		}
		img_new_color2 = average_color_pixels(img_new_color2, WIDTH_COLOR, HEIGHT_COLOR);
		imshow("new color2 to 1", img_new_color2);
		imwrite("D:\\project_color\\color_2to1.bmp", img_new_color2);
	}
	waitKey();
}
   
void load_extrinsic_parm(Mat& R, Mat& T, Mat& RT, const string extrinsic_filename)
{
	cout << extrinsic_filename << endl;
	FileStorage fs;
	fs.open(extrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filename);
	}
	fs["R"] >> R;
	fs["T"] >> T;
	RT = Mat::eye(4, 4, CV_32F);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j) {
			RT.at<float>(i, j) = R.at<double>(i, j);
		}
#if 0
	RT.at<float>(3, 0) = T.at<double>(0) * 10;
	RT.at<float>(3, 1) = T.at<double>(1) * 10;
	RT.at<float>(3, 2) = T.at<double>(2) * 10;
	RT.at<float>(3, 3) = 1.f;
#else
	RT.at<float>(0, 3) = T.at<double>(0) * 10;
	RT.at<float>(1, 3) = T.at<double>(1) * 10;
	RT.at<float>(2, 3) = T.at<double>(2) * 10;
	RT.at<float>(3, 3) = 1.f;
#endif
	//RT = RT.t();
	cout << RT << endl << endl;
}

inline void ReadCalibratedUndistortionTable(vector<pair<float, float>> &undistortLT, int width, int height, const char *filepath)
{
	int npixel = width * height;
	FILE *fp;
	fopen_s(&fp, filepath, "r");
	undistortLT.resize(width*height);
	vector<pair<float, float>> src, dest;
	for (int i = 0; i < npixel; i++) {
		int x0, y0;
		float x1, y1;
		fscanf_s(fp, "%d %d %f %f", &x0, &y0, &x1, &y1);
		src.push_back(std::make_pair(x0, y0));
		dest.push_back(std::make_pair(x1, y1));
		undistortLT[x0 + y0*width] = std::make_pair(x1, y1);
	}
	fclose(fp);
	return;
}

int main(int argc, char* argv[])
{
	load_extrinsic_parm(R_color1_color2, T_color1_color2, RT_color1_to_color2, "d:\\yyk\\extrinsic_color1_and_color2.yml");
	load_extrinsic_parm(R_IR2_color2, T_IR2_color2, RT_IR2_to_color2, "d:\\yyk\\extrinsic_IR2_and_color2.yml");
	load_extrinsic_parm(R_IR1_IR2, T_IR1_IR2, RT_IR1_to_IR2, "d:\\yyk\\extrinsics_IR1_and_IR2.yml");
	
	Mat IR1_pos = Mat::zeros(4, 1, CV_32F);
	IR1_pos.at<float>(3) = 1;
	Mat IR2_pos = RT_IR1_to_IR2 * IR1_pos;
	Mat color2_pos = RT_IR2_to_color2 * IR2_pos;
	Mat color1_pos = RT_color1_to_color2.inv() * color2_pos;

	cout << IR1_pos << endl << IR2_pos << endl << color1_pos << color2_pos << endl;

	ReadCalibratedUndistortionTable(undistortLookupTable_depth[0], WIDTH, HEIGHT, "D:\\yyk\\table_depth1.txt");
	ReadCalibratedUndistortionTable(undistortLookupTable_depth[1], WIDTH, HEIGHT, "D:\\yyk\\table_depth2.txt");
	ReadCalibratedUndistortionTable(undistortLookupTable_color[0], WIDTH_COLOR, HEIGHT_COLOR, "D:\\yyk\\table_color1.txt");
	ReadCalibratedUndistortionTable(undistortLookupTable_color[1], WIDTH_COLOR, HEIGHT_COLOR, "D:\\yyk\\table_color2.txt");

	//string path_file(PATH_FILE);
	ifstream fin(PATH_FILE);
	string str;
	vector<string> image_path_list;
	while (fin >> str) {
		image_path_list.push_back(str);
	}
	while (image_path_list.size() % 4 != 0) {
		image_path_list.pop_back();
	}
	cout << "image path list: " << image_path_list.size() << endl;

	bool save_temp_file = true;

	//int count = 100;

	for (int i = 0; i < image_path_list.size() / 4; ++i) {
		const string& IR1_path = image_path_list[i * 4];
		const string& depth1_path = image_path_list[i * 4 + 1];
		const string& IR2_path = image_path_list[i * 4 + 2];
		const string& depth2_path = image_path_list[i * 4 + 3];
		string prefix = IR1_path.substr(0, IR1_path.find("-a.bmp"));
		string bin_path_v1tov2 = prefix + "-bb.bin";
		string bin_path_v2tov1 = prefix + "-aa.bin";
		process(IR1_path, IR2_path, depth1_path, depth2_path, prefix, bin_path_v1tov2, bin_path_v2tov1, save_temp_file);

		//if (--count <= 0) break;
	}
	return 0;
}