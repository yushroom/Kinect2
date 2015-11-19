#define _CRT_SECURE_NO_WARNINGS
#include <vector>
#include <utility>
#include <atlimage.h>
#include <codex_math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <algorithm>
#include <advmath.h>
#include "fit_gaussian.h"

using namespace cv;
using namespace std;

typedef unsigned short UINT16;

//#define WRITE_BB_BIN_ONLY

#ifdef _DEBUG
#define WRITE_FILE
#endif

//#define WRITE_FILE

#ifdef WRITE_FILE
static const bool write_file = true;
#else
static const bool write_file = false;
#endif

#define WIDTH 640
#define HEIGHT 480

//#define ROOT_DIR			"D:\\yyk\\image\\"
//#define NUMBER_ID			"1446086914-1"
//#define IMAGE_DIR			ROOT_DIR NUMBER_ID "\\"
//#define PREFIX_A			IMAGE_DIR "a"
//#define PREFIX_B			IMAGE_DIR "b"
//#define INFRARED_V1			PREFIX_A ".bmp"
//#define INFRARED_V2			PREFIX_B ".bmp"
//#define POINTCLOUND_V1		PREFIX_A "-point.ply"
//#define POINTCLOUND_V2		PREFIX_B "-point.ply"
//#define POINTCLOUND_ALL_1	IMAGE_DIR "point-all-1.ply"
//#define POINTCLOUND_ALL_2	IMAGE_DIR "point-all-2.ply"
//#define DEPTH_V1_PATH		PREFIX_A ".bin"
//#define DEPTH_V2_PATH		PREFIX_B ".bin"
//
//#define UNDISTORT_IR1_PATH	PREFIX_A "-IR-und.bmp"
//#define UNDISTORT_IR2_PATH	PREFIX_B "-IR-und.bmp"
//#define UNDISTORT_DEPTH1_PATH	PREFIX_A "-d-und.bmp"
//#define UNDISTORT_DEPTH2_PATH	PREFIX_B "-d-und.bmp"
//#define V1_TO_V2_IMAGE		IMAGE_DIR "v1-to-v2.bmp"
//#define V2_TO_V1_IMAGE		IMAGE_DIR "v2-to-v1.bmp"
//
//#define POINT_CLOUD_1		PREFIX_A "-points.ply"
//#define POINT_CLOUD_2		PREFIX_B "-points.ply"
//#define POINT_CLOUD_DIFF	PREFIX_A "-points-diff.ply"

#define EXTRINSICS_FILE_PATH "D:\\yyk\\image\\extrinsics.yml"
#define INTRINSICS_FILE_PATH "D:\\yyk\\image\\intrinsics.yml"
#define TABLE1 "D:\\yyk\\image\\table1_zfx.txt"
#define TABLE2 "D:\\yyk\\image\\table2_zfx.txt"

#define V1_FX (587.088)
#define V1_FY (585.688)
#define V1_CX (321.198)
#define V1_CY (232.202)

//#define V2_FX (364.440)
//#define V2_FY (363.688)
//#define V2_CX (259.818)
//#define V2_CY (207.523)

#define V2_RESIZED_FX (415.035)
#define V2_RESIZED_FY (413.996)
#define V2_RESIZED_CX (325.237)
#define V2_RESIZED_CY (234.963)

const int	depth_width = 640;
const int	depth_height = 480;

const float DEPTH_INVALID = 1e30f;
#define VALID_DEPTH_TEST(a) (a < DEPTH_INVALID)

string bin_prefix;

void solve_for_bias(const int w, const int h, const std::vector<float> &I,
					const float alpha, const float lambda, const int niter,
					std::vector<float> &result)
{
	const int NUM_NEIGHBOR = 4;
	static int dir[NUM_NEIGHBOR * 2] = { -1, 0, 1, 0, 0, -1, 0, 1 };

	advmath::la_matrix_csr<float> A;
	std::vector<float> vb;

	std::vector<int> reg2idx, idx2reg;
	reg2idx.resize(w*h, -1);
	
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			int idx = x+y*w;
			if (VALID_DEPTH_TEST(I[idx]))
			{
				reg2idx[idx] = idx2reg.size();
				idx2reg.push_back(idx);
			}
		}
	A.column = idx2reg.size();
	//A.column = w*h;

	for (int i = 0; i < idx2reg.size(); i++)
	//for (int i = 0; i < w*h; i++)
	{
		int idx = //i,
				idx2reg[i],
			x = idx % w, y = idx / w;
		int idx_first, idx_last;

		//data term
		if (!VALID_DEPTH_TEST(I[idx])) continue;

		idx_first = (int)A.val.size();
		idx_last = (int)A.val.size();

		A.val.push_back(alpha);
		A.cols.push_back(i);

		A.ptrb.push_back(idx_first);
		A.ptre.push_back(idx_last + 1);
		vb.push_back(alpha*I[idx]);
		A.row++;

		//smoothness terms
		for (int j = 0; j < NUM_NEIGHBOR; j++)
		{
			int xx = x + dir[j * 2 + 0],
				yy = y + dir[j * 2 + 1];
			if (xx >= 0 && xx < w && yy >= 0 && yy < h && idx < xx + yy*w 
				&& reg2idx[xx + yy*w] >= 0 && 
				VALID_DEPTH_TEST(I[xx + yy*w])
				//&& VALID_DEPTH_TEST(I[xx + yy*w])
				)
			{
				idx_first = (int)A.val.size();
				idx_last = (int)A.val.size();

				A.val.push_back(1.0f);
				A.cols.push_back(i);
				A.val.push_back(-1.0f);
				A.cols.push_back(reg2idx[xx + yy*w]);
				//A.cols.push_back(xx + yy*w);

				A.ptrb.push_back(idx_first);
				A.ptre.push_back(idx_last + 2);
				vb.push_back(0);
				A.row++;
			}
		}
	}

	advmath::la_vector<float> b(vb);
	//sparse least square
	advmath::la_vector<float> x;
	advmath::ssplsqr(x, A, b, lambda, niter, true);

	result.assign(w*h, DEPTH_INVALID);
	for (int i = 0; i < idx2reg.size(); i++)
		result[idx2reg[i]] = x.v[i];

	//for (int i = 0; i < w*h; i++)
	//	result[i] = x.v[i];
}

void adjust_for_bias(std::vector<float> &depth, const std::vector<float> &bias)
{
	for (int i = 0; i < depth.size(); i++)
		depth[i] = (VALID_DEPTH_TEST(bias[i]) ? depth[i] - bias[i] : 0);
}

void prepare_for_solving_bias(std::vector<float> &depth, 
							  const std::vector<float> &depth1,
							  const std::vector<float> &depth2)
{
	depth.resize(depth1.size());
	for (int i = 0; i < depth.size(); i++)
		depth[i] = (VALID_DEPTH_TEST(depth1[i]) && VALID_DEPTH_TEST(depth2[i])) ? depth2[i]-depth1[i] : DEPTH_INVALID;
}

Mat ResizeAndToCVImage(const vector<UINT16>& depth_pixels){
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

Mat ToCVImage(const vector<UINT16>& src)
{
	Mat temp(Size(WIDTH, HEIGHT), CV_16UC1);
	for (int j = 0; j < HEIGHT; j++) {
		for (int i = 0; i < WIDTH; ++i) {
			temp.at<UINT16>(j, i) = src[j * WIDTH + i];
		}
	}
	return temp;
}

#define ZERO(x) ((fabsf(x) < 1e-3f))

//inline void writePly(const Mat& points, const Mat& IR, const char *filename, int w = WIDTH, int h = HEIGHT, 
//					 float r = 1, float g = 1, float b = 1, float tx = 0, float ty = 0, float tz = 0)
//{
//	FILE *fp;
//	fopen_s(&fp, filename, "w");
//	fprintf(fp, "ply\n");
//	fprintf(fp, "format ascii 1.0\n");
//	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
//	fprintf(fp, "element vertex %d\n", w * h);
//	fprintf(fp, "property float x\n");
//	fprintf(fp, "property float y\n");
//	fprintf(fp, "property float z\n");
//	fprintf(fp, "property uchar red\n");
//	fprintf(fp, "property uchar green\n");
//	fprintf(fp, "property uchar blue\n");
//	fprintf(fp, "element face 0\n");
//	fprintf(fp, "property list uchar int vertex_index\n");
//	fprintf(fp, "end_header\n");
//
//	for (int j = 0; j < h; ++j)
//		for (int i = 0; i < w; ++i) {
//			int idx = j * w + i;
//			int c = IR.at<unsigned char>(j, i);
//			float x = points.at<float>(0, idx) + tx;
//			float y = points.at<float>(1, idx) + ty;
//			float z = points.at<float>(2, idx) + tz;
//			fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, unsigned char(c*r), unsigned char(c*g), unsigned char(c*b));
//		}
//	fclose(fp);
//}

inline void writePly(const Mat& points, const vector<unsigned char>& IR, const string& filename, int w = WIDTH, int h = HEIGHT, 
					 float r = 1, float g = 1, float b = 1, float tx = 0, float ty = 0, float tz = 0)
{
	FILE *fp;
	fopen_s(&fp, filename.c_str(), "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", w * h);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");

	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			//int c = IR.at<unsigned char>(j, i);
			int c = IR[idx];
			float x = points.at<float>(0, idx) + tx;
			float y = points.at<float>(1, idx) + ty;
			y = -y;
			float z = points.at<float>(2, idx) + tz;
			z = -z;
			fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, unsigned char(c*r), unsigned char(c*g), unsigned char(c*b));
		}
		fclose(fp);
}

inline void writePly2(const Mat& points1, const Mat& points2, const Mat& IR1, const Mat& IR2, const char *filename,
					  float tx1 = 0, float ty1 = 0, float tz1 = 0, float tx2 = 0, float ty2 = 0, float tz2 = 0)
{
	int offset = 0;
	FILE *fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", (WIDTH - offset * 2) * (HEIGHT - offset * 2) * 2);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");


	for (int j = offset; j < HEIGHT - offset; ++j)
		for (int i = offset; i < WIDTH - offset; ++i) {
			int idx = j * WIDTH + i;
			int c = IR1.at<unsigned char>(j, i);
			float x = points1.at<float>(0, idx) + tx1;
			float y = points1.at<float>(1, idx) + ty1;
			float z = points1.at<float>(2, idx) + tz1;
			//if (x * y * z == 0)
			//	continue;
			fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, c, c, 0);	// v1 Yellow
		}
		for (int j = offset; j < HEIGHT - offset; ++j)
			for (int i = offset; i < WIDTH - offset; ++i) {
				int idx = j * WIDTH + i;
				int c = IR2.at<unsigned char>(j, i);
				float x = points2.at<float>(0, idx) + tx2;
				float y = points2.at<float>(1, idx) + ty2;
				float z = points2.at<float>(2, idx) + tz2;
				//if (x * y * z == 0)
				//	continue;
				fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, 0, 0, c);	// v2 blue
			}
			//for (int i = 0; i < WIDTH * HEIGHT; i++) {
			//	fprintf_s(fp, "%f %f %f %d %d %d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, infrared_pixels[i], infrared_pixels[i], infrared_pixels[i]);
			//}
			fclose(fp);
}

template <typename T>
Mat calc_points(Mat depth_image, float fx, float fy, float cx, float cy, bool cropped = true, cv::Rect roi = Rect(0, 0, WIDTH, HEIGHT)) {
	float inv_fx = 1.f / fx;
	float inv_fy = 1.f / fy;
	//float x, y, z;
	int w = roi.width;
	int h = roi.height;
	Mat points(3, w * h, CV_32F);
	for (int j = roi.y; j < roi.y + roi.height; ++j)
		for (int i = roi.x; i < roi.x + roi.width; ++i) {
			int idx = (j-roi.y) * w + (i-roi.x);
			T d = cropped ? depth_image.at<T>(j-roi.y, i-roi.x) : depth_image.at<T>(j, i);
			//float x_scale = inv_fx * d;
			//float y_scale = inv_fy * d;
			points.at<float>(0, idx) = (i + 0.5f - cx) * inv_fx * d; // x
			points.at<float>(1, idx) = (j + 0.5f - cy) * inv_fy * d;
			points.at<float>(2, idx) = d;
		}

		return points;
}

Mat calc_points(vector<float> depth, float fx, float fy, float cx, float cy, bool cropped = true, cv::Rect roi = Rect(0, 0, WIDTH, HEIGHT)) {
	float inv_fx = 1.f / fx;
	float inv_fy = 1.f / fy;
	//float x, y, z;
	int w = roi.width;
	int h = roi.height;
	Mat points = Mat::zeros(3, w * h, CV_32F);
	for (int j = roi.y; j < roi.y + roi.height; ++j)
		for (int i = roi.x; i < roi.x + roi.width; ++i) {
			int idx = (j-roi.y) * w + (i-roi.x);
			float d = depth[idx];
			if (!VALID_DEPTH_TEST(d))
				continue;
			//T d = cropped ? depth_image.at<T>(j-roi.y, i-roi.x) : depth_image.at<f>(j, i);
			//float x_scale = inv_fx * d;
			//float y_scale = inv_fy * d;
			points.at<float>(0, idx) = (i + 0.5f - cx) * inv_fx * d; // x
			points.at<float>(1, idx) = (j + 0.5f - cy) * inv_fy * d;
			points.at<float>(2, idx) = d;
		}

		return points;
}

Mat calc_points2(Mat depth_image, float fx, float fy, float cx, float cy) {
	float inv_fx = 1.f / fx;
	float inv_fy = 1.f / fy;
	//float x, y, z;
	int w = depth_image.cols;
	int h = depth_image.rows;
	Mat points(3, w * h, CV_32F);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			UINT16 d = depth_image.at<float>(j, i);
			//float x_scale = inv_fx * d;
			//float y_scale = inv_fy * d;
			points.at<float>(0, idx) = (i + 0.5f - cx) * inv_fx * d; // x
			points.at<float>(1, idx) = (j + 0.5f - cy) * inv_fy * d;
			points.at<float>(2, idx) = d;
		}

		return points;
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
			vec[idx] = img.at<T>(j,i);
		}
	}
	return vec;
}

template<typename T1, typename T2>
vector<T1> mat_to_vector(const Mat& img) {
	int w = img.cols;
	int h = img.rows;
	vector<T1> vec;
	vec.resize(w*h);
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			vec[idx] = img.at<T2>(j,i);
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

Mat vector_to_img_f(const vector<float>& vec, int w, int h, float scale = 1) {
	Mat img(h, w, CV_32F);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img.at<float>(j, i) = float(vec[idx] * scale);
		}
		return img;
}

Mat imgf_to_imguc(const Mat& img_f, float scale = 1) {
	int w = img_f.cols;
	int h = img_f.rows;
	Mat img_uc (h, w, CV_8U);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img_uc.at<unsigned char>(j, i) = unsigned char(img_f.at<float>(j, i) * scale);
		}
		return img_uc;
}

void write_normalized_image(const Mat &img, const char *filename)
{
	int w = img.cols, h = img.rows;
	double vmin, vmax;
	cv::minMaxLoc(img, &vmin, &vmax);

	Mat output = Mat::zeros(h, w, CV_8U);
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			float v = (img.at<float>(y, x) - vmin) / (vmax - vmin);
			output.at<BYTE>(y, x) = BYTE(v * 255);
		}

		imwrite(filename, output);
}

template <typename T>
void write_normalized_vector(const vector<T> &img, const string& filename, int w = WIDTH, int h = HEIGHT)
{
	//auto vmax = *std::max_element(img.begin(), img.end());
	//auto vmin = *std::min_element(img.begin(), img.end());
	float vmax = -1, vmin = DEPTH_INVALID;
	for (int i = 1; i < w * h; ++i) {
		if (img[i] != 0 && img[i] < vmin) vmin = img[i];
		if (VALID_DEPTH_TEST(img[i]) && img[i] > vmax) vmax = img[i];
	}

	cout << "[write_normalized_vector] max = " << vmax << "  min = " << vmin << endl;
	waitKey();

	Mat output = Mat::zeros(h, w, CV_8U);
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			int idx = x + y * w;
			float v = float(img[idx] - vmin) / float(vmax - vmin);
			output.at<BYTE>(y, x) = BYTE(v * 255);
		}

	imwrite(filename, output);
}

inline void ReadCalibratedUndistortionTable(vector<pair<float, float>> &undistortLT, int width, int height, const char *filepath)
{
	int npixel = width * height;
	FILE *fp;
	fopen_s(&fp, filepath, "r");
	undistortLT.resize(width*height);
	vector<pair<float, float>> src, dest;
	for (int i = 0; i<npixel; i++) {
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

template <typename T>
inline float bilinear(float x, float y, const vector<T>& src, int width, int height, bool ignore_zero = false) {
	int ix0 = x, iy0 = y, ix1 = ix0 + 1, iy1 = iy0 + 1;
	float qx1 = x - ix0, qy1 = y - iy0, qx0 = 1.f - qx1, qy0 = 1.f - qy1;

	float weight = 0.f, value = 0.f;
	float w[] = { qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1 };
	//pair<int, int> xy[] = { { ix0, iy0 }, { ix1, iy0 }, { ix0, iy1 }, { ix1, iy1 } };
	int xx[] = {ix0, ix1, ix0, ix1};
	int yy[] = {iy0, iy0, iy1, iy1};

	for (int j = 0; j<4; j++) {
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


float fr(UINT16 val) {
	int range = 1;
	if (val > range) 
		return 0;
	else
		return (range-val) / range; 
}
static vector<UINT16> resize_image(const vector<UINT16>& vec, int width = 512, int height = 424, int target_w = 580, int target_h = 480, bool ignore_zero = true)
{
	vector<UINT16> ret(WIDTH * HEIGHT, 0);

	// Bilateral filter
	float fx, fy, qx0, qx1, qy0, qy1;
	int ix0, ix1, iy0, iy1;

	for (int y = 0; y < target_h; ++y) {
		fy = float(y) / target_h * height;
		iy0 = int(fy); iy1 = iy0 + 1;
		qy1 = fy - iy0; qy0 = 1.f - qy1;

		for (int x = 0; x < target_w; ++x) {
			fx = float(x) / target_w * width;
			ix0 = int(fx); ix1 = ix0 + 1;
			qx1 = fx - ix0; qx0 = 1.f - qx1;

			UINT16 I0 = vec[ix0 + iy0 * width];

			float weight = 0.f, value = 0.f;
			float w[] = { qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1 };
			//pair<int, int> xy[] = { { ix0, iy0 }, { ix1, iy0 }, { ix0, iy1 }, { ix1, iy1 } };
			int xx[] = {ix0, ix1, ix0, ix1};
			int yy[] = {iy0, iy0, iy1, iy1};

			for (int j = 0; j<4; j++) {
				if (xx[j] < 0 || xx[j] >= width || yy[j] < 0 || yy[j] >= height)
					continue;
				int idx = xx[j] + yy[j] * width;
				UINT16 I = vec[idx];
				if (ignore_zero && I == 0) continue;
				w[j] *= fr(fabsf(I - I0));
				value += I * w[j];
				weight += w[j];
			}

			ret[x + 30 + y * WIDTH] = (weight == 0.f) ? 0 : value / weight;
		}
	}

	return ret;
}


vector<float> average_depth_pixels(const vector<float>& depth, int w = WIDTH, int h = HEIGHT)
{
	vector<float> ret(WIDTH * HEIGHT, 0);
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
					int idx = (j + k) * w + (i+l);
					float val = depth[idx];
					if (val != 0 && VALID_DEPTH_TEST(val)) {
						count ++;
						sum += val;
					}
				}
			}
			int idx = j * w + i;
			//assert(count != 0);
			if (count != 0)
				ret[idx] = sum / count;
		}
	}
	return ret;
}

void SaveToBin(const vector<float>& vec, const string& file_path){
	int w = WIDTH;
	int h = HEIGHT;
	assert(vec.size() == WIDTH * HEIGHT);
	vector<UINT16> bin(w * h, 0);
	for (int i = 0; i < vec.size(); ++i)
		bin[i] = (UINT16)vec[i];
	ofstream os(file_path, ios::binary);
	os.write((char*)&bin[0], w * h * sizeof(UINT16));
	os.close();
}

// 640 * 480 Mat<float> -> 512 * 424 vector<UINT16>
void ResizeAndSaveToBin(const vector<float>& vec, const string& file_path){
	assert(vec.size() == WIDTH * HEIGHT);
	// 640*480 -> 580*480
	int w = 580, h = 480;
	int offset = (640 - w) / 2;
	Mat mat = Mat::zeros(h, w, CV_32F);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; ++i) {
			int idx = (i + offset + j * 640);
			mat.at<float>(j, i) = vec[idx];
		}
	}
	//imshow("test", vector_to_img_uc(vec));
	//imshow("test1", mat);
	//waitKey();
	
	w = 512;
	h = 424;

	vector<UINT16> bin(w * h, 0);

	//printf("%d %d\n", mat.cols, mat.rows);
	cv::resize(mat, mat, Size(w, h), 0, 0, INTER_NEAREST);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; ++i) {
			int idx = i + j * w;
			float d = mat.at<float>(j, i);
			bin[idx] = (UINT16)d;
		}
	}

	ofstream os(file_path, ios::binary);
	os.write((char*)&bin[0], w * h * sizeof(UINT16));
	os.close();
}

enum ProjectionType {
	v1_to_v2 = 0,
	v2_to_v1 = 1
};

//1 -> 2, [points] of v1, [IR_und] of v1, [depth_und] of v2
//2 -> 1, [points] of v2, [IR_und] of v2, [depth_und] of v1
void project_to_another_camera(const Mat& R, const Mat& T, const Mat& points_1, const vector<unsigned char>& IR_und, const vector<float>& depth_und, const string& prefix, const string& bin_path, const ProjectionType proj_type, bool save_temp_file)
{
	float fx, fy, cx, cy;
	if (proj_type == v1_to_v2) {
		fx = V2_RESIZED_FX;
		fy = V2_RESIZED_FY;
		cx = V2_RESIZED_CX;
		cy = V2_RESIZED_CY;
	} else {
		fx = V1_FX;
		fy = V1_FY;
		cx = V1_CX;
		cy = V1_CY;
	}

	Mat trans = Mat::zeros(3, 3, CV_32F);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j) {
			if (proj_type == v1_to_v2)
				trans.at<float>(i, j) = R.at<double>(i, j);
			else
				trans.at<float>(i, j) = R.at<double>(j, i);
		}
	//for (int i = 0; i < 3; ++i)
	//	trans.at<float>(i, 3) = T.at<double>(i);

	//points_1 = trans * points_1;
	Mat points_1_1 = trans * points_1;
	double tx = T.at<double>(0) * 10;
	double ty = T.at<double>(1) * 10;
	double tz = T.at<double>(2) * 10;
	if (proj_type == v2_to_v1) {
		tx = -tx; ty = -ty; tz = -tz;
	}
	//tx = ty = tz = 0;
	//cout << "tx = " << tx << "  ty = " << ty << "  tz = " << tz << endl;

	//int count = 0;
	//cout << img_IR[1].type() << endl;
	Mat img_new_IR = Mat::zeros(HEIGHT, WIDTH, CV_8U);
	Mat img_new_depth = Mat::zeros(HEIGHT, WIDTH, CV_32F);
	vector<float> depth1_to_2(WIDTH * HEIGHT, DEPTH_INVALID);

	//Mat img = img_IR_1_undistort.clone();
	for (int j = 0; j < HEIGHT; ++j) {
		for (int i = 0; i < WIDTH; ++i) {
			int idx = j * WIDTH + i;
			int x, y;
			float z = points_1.at<float>(2, idx);
			if (z <= 1e-2f && z >= -1e-2f) {
				continue;
			}
			z = points_1_1.at<float>(2, idx) + tz;
			x = int(fx * (points_1_1.at<float>(0, idx) + tx) / z + cx - 0.5f);
			y = int(fy * (points_1_1.at<float>(1, idx) + ty) / z + cy - 0.5f);
			//cout << x << ' ' << y << ' ' << z << '\n';
			if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
				img_new_IR.at<unsigned char>(y, x) = IR_und[j*WIDTH + i];
				img_new_depth.at<float>(y, x) = z;
				depth1_to_2[y*WIDTH + x] = z;
			}
		}
	}

	if (save_temp_file) {
		if (proj_type == v1_to_v2)
		{
			imwrite(prefix + "-v1tov2.bmp", img_new_IR);
			write_normalized_vector(depth1_to_2, prefix + "-v1tov2-depth.bmp");
		} else {
			imwrite(prefix + "-v2tov1.bmp", img_new_IR);
			write_normalized_vector(depth1_to_2, prefix + "-v2tov1-depth.bmp");
			depth1_to_2 = average_depth_pixels(depth1_to_2);
			write_normalized_vector(depth1_to_2, prefix + "-v2tov1-depth-average.bmp");
		}
	}

	vector<float> depth, bias;
	float smooth = 0.350f;
	prepare_for_solving_bias(depth, depth1_to_2, depth_und);

	vector<float> depth_f;
	if (proj_type == v1_to_v2) {
		depth_f = fit_depth(depth_und, depth1_to_2, WIDTH, HEIGHT, 5, 1, 1, 1, 1, 1, 0);
		write_normalized_vector(depth_f, "d:\\fit_depth-1to2.bmp");
	} else {
		depth_f = fit_depth(depth1_to_2, depth_und, WIDTH, HEIGHT, 5, 1, 1, 1, 1, 1, 0);
		write_normalized_vector(depth_f, "d:\\fit_depth_2to1.bmp");
	}
	

	solve_for_bias(WIDTH, HEIGHT, depth, smooth, 0, 500, bias);

	//Mat new_points_1 = calc_points(depth1_to_2, fx, fy, cx, cy);
	//Mat new_points_2 = calc_points(depth_und, fx, fy, cx, cy);

	if (save_temp_file)
	{
		auto bias_ = bias;
		for (int j = 0; j < bias.size(); ++j)
			if (!VALID_DEPTH_TEST(bias_[j])) bias_[j] = 0;
		Mat img = vector_to_img_f(bias_, WIDTH, HEIGHT);
		string path = prefix + "-bias.bmp";
		write_normalized_image(img, path.c_str());
	}


	vector<float> depth1_without_bias = depth_und;
	adjust_for_bias(depth1_without_bias, bias);

	//Mat points_2_without_bias = calc_points(depth1_without_bias, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);

	if (proj_type == v1_to_v2) {
		//cout << endl << endl << bin_prefix << endl << endl;
		//waitKey();
		ResizeAndSaveToBin(depth1_to_2, bin_prefix + "-1to2.bin");
		ResizeAndSaveToBin(depth1_without_bias, bin_path);
	} else {
		//cout << endl << endl << bin_prefix << endl << endl;
		//waitKey();
		SaveToBin(depth1_to_2, bin_prefix + "-2to1.bin");
		SaveToBin(depth1_without_bias, bin_path);
	}
	

#if 0
	writePly(new_points_1, IR_und[0], prefix + "-a.ply", WIDTH, HEIGHT, 1, 1, 0);
	writePly(new_points_2, IR_und[1], prefix + "-b.ply", WIDTH, HEIGHT, 0, 0, 1);
	writePly(points_2_without_bias, IR_und[1], prefix + "-diff.ply", WIDTH, HEIGHT, 0, 1, 0);

	vector<float> bias2;
	solve_for_bias(WIDTH, HEIGHT, depth1_to_2, smooth, 0, 500, bias2);
	Mat points_comp = calc_points(bias2, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
	writePly(points_comp, IR_und[1], prefix + "-test_bias.ply", WIDTH, HEIGHT, 1, 0, 0);

	{
		auto bias_ = bias2;
		for (int j = 0; j < bias.size(); ++j)
			if (!VALID_DEPTH_TEST(bias_[j])) bias_[j] = 0;
		Mat img = vector_to_img_f(bias_, WIDTH, HEIGHT);
		string path = prefix + "-test-bias2.bmp";
		write_normalized_image(img, path.c_str());
	}
#endif
}


vector<pair<float, float>>	undistortLookupTable[2];
Mat R, T, E, F;

void process(const string& ir1_path, const string& ir2_path, const string& depth1_path, const string& depth2_path, const string& prefix, const string& bin_path_v1tov2, const string& bin_path_v2tov1, bool save_temp_file)
{
	//Mat R, T, E, F;
	//{
	//	const char * extrinsic_filename = EXTRINSICS_FILE_PATH;
	//	FileStorage fs;
	//	fs.open(extrinsic_filename, CV_STORAGE_READ);
	//	if (!fs.isOpened())
	//	{
	//		printf("Failed to open file %s\n", extrinsic_filename);
	//	}
	//	fs["R"] >> R;
	//	fs["T"] >> T;
	//}

	//Mat img_IR_und[2];
	//Mat img_depth_und[2];
	vector<unsigned char> IR_und[2];
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

		Mat img_IR[2];
		Mat img_depth[2];
		vector<unsigned char> IR[2];

		// load IRV1 and resize IRV2;
		img_depth[0] = ToCVImage(depth_pixels[0]);
		{
			// resize v2;
			//img_depth[1] = ResizeAndToCVImage(depth_pixels[1]);
			//depth_pixels[1] = mat_to_vector<UINT16>(img_depth[1]);
			//auto d1 = mat_to_vector<UINT16>(img_depth[1]);
			//write_normalized_vector(d1, "D:\\1.bmp");

			//depth_pixels[1] = resize_image(depth_pixels[1]);
			//auto d2 = resize_image(depth_pixels[1]);
			//write_normalized_vector(d2, "D:\\2.bmp");
			depth_pixels[1] = resize_image(depth_pixels[1]);
		}

		std::cout << ir1_path << '\n' << ir2_path << '\n';
		img_IR[0] = imread(ir1_path, 0);
		{
			Mat mat = imread(ir2_path, 0);
			cv::resize(mat, mat, Size(580, 480));
			img_IR[1] = Mat::zeros(Size(640, 480), mat.type());
			mat.copyTo(img_IR[1](Rect(30, 0, 580, 480)));
		}

		for (int i = 0; i < 2; ++i) {
			IR[i] = mat_to_vector<unsigned char>(img_IR[i]);
			IR_und[i].resize(WIDTH * HEIGHT);
			depth_und[i].resize(WIDTH * HEIGHT);
		}

		//vector<pair<float, float>>	undistortLookupTable[2];
		//ReadCalibratedUndistortionTable(undistortLookupTable[0], WIDTH, HEIGHT, TABLE1);
		//ReadCalibratedUndistortionTable(undistortLookupTable[1], WIDTH, HEIGHT, TABLE2);

		Mat table_x(HEIGHT, WIDTH, CV_8U);
		Mat table_y(HEIGHT, WIDTH, CV_8U);
		for (int j = 0; j < HEIGHT; ++j) {
			for (int i = 0; i < WIDTH; ++i) {
				int idx = j * WIDTH + i;
				auto entry = undistortLookupTable[1][idx];
				float x = entry.first;
				float y = entry.second;
				table_x.at<unsigned char>(j, i) = (x / 640.f) * 255;
				table_y.at<unsigned char>(j, i) = (y / 480.f) * 255;
			}
		}

#ifndef WRITE_BB_BIN_ONLY
		//write_normalized_vector(depth_pixels[0], prefix + "-a-depth-n.bmp");
		//write_normalized_vector(depth_pixels[1], prefix + "-b-depth-n.bmp");
#endif // !WRITE_BB_BIN_ONLY

		for (int j = 0; j < 2; ++j) {
			for (int i = 0; i < WIDTH * HEIGHT; i++) {
				auto entry = undistortLookupTable[j][i];
				float ir = bilinear<unsigned char>(entry.first, entry.second, IR[j], WIDTH, HEIGHT);
				IR_und[j][i] = (VALID_DEPTH_TEST(ir) ? ir : 0);
				depth_und[j][i] = bilinear<UINT16>(entry.first, entry.second, depth_pixels[j], WIDTH, HEIGHT, true);
			}
		}
	}

	SaveToBin(depth_und[0], prefix + "-a-depth-und.bin");
	SaveToBin(depth_und[1], prefix + "-b-depth-und.bin");

	if (save_temp_file) {
		imwrite(prefix + "-a-IR-und.bmp", vector_to_img_uc(IR_und[0], WIDTH, HEIGHT));
		imwrite(prefix + "-b-IR-und.bmp", vector_to_img_uc(IR_und[1], WIDTH, HEIGHT));
		write_normalized_vector(depth_und[0], prefix + "-a-depth-und.bmp");
		write_normalized_vector(depth_und[1], prefix + "-b-depth-und.bmp");
	}
	write_normalized_vector(depth_und[0], prefix + "-a-depth-und.bmp");
	write_normalized_vector(depth_und[1], prefix + "-b-depth-und.bmp");

	Mat points_1 = calc_points(depth_und[0], V1_FX, V1_FY, V1_CX, V1_CY);
	Mat points_2 = calc_points(depth_und[1], V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);


	// 1 -> 2
	project_to_another_camera(R, T, points_1, IR_und[0], depth_und[1], prefix, bin_path_v1tov2, v1_to_v2, save_temp_file);
	project_to_another_camera(R, T, points_2, IR_und[1], depth_und[0], prefix, bin_path_v2tov1, v2_to_v1, save_temp_file);
}

#define TEST_ROOT_DIR "D:\\yyk\\capture\\test\\"
#define TEST_NUMBER_ID "1446100378-10"
#define TEST_SAVE_TEMP_FILE_DIR "D:\\yyk\\capture\\test\\temp\\"
//#define TEST_OUPUT_BIN_PATH_v2tov1 "D:\\yyk\\capture\\test\\" TEST_NUMBER_ID "-aa.bin"
//#define TEST_OUPUT_BIN_PATH_v1tov2 "D:\\yyk\\capture\\test\\" TEST_NUMBER_ID "-bb.bin"
#define PATH_FILE "D:\\yyk\\capture\\desk\\path.txt"
 
int main(int argc, char* argv[])
{
	{
		const char * extrinsic_filename = EXTRINSICS_FILE_PATH;
		FileStorage fs;
		fs.open(extrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
		}
		fs["R"] >> R;
		fs["T"] >> T;
	}

	ReadCalibratedUndistortionTable(undistortLookupTable[0], WIDTH, HEIGHT, TABLE1);
	ReadCalibratedUndistortionTable(undistortLookupTable[1], WIDTH, HEIGHT, TABLE2);

	string path_file[] = {"D:\\yyk\\capture\\corner\\path.txt", "D:\\yyk\\capture\\desk\\path.txt", "D:\\yyk\\capture\\person\\path.txt"};

	for (int path_index = 0; path_index < 3; path_index++)
	{
		//string path_file(PATH_FILE);
		ifstream fin(path_file[path_index]);
		string str;
		vector<string> image_path_list;
		while (fin >> str) {
			image_path_list.push_back(str);
		}
		while (image_path_list.size() % 4 != 0) {
			image_path_list.pop_back();
		}
		cout << "image path list: " << image_path_list.size() << endl;

		bool save_temp_file = false;

		//int count = 100;

		for (int i = 0; i < image_path_list.size() / 4; ++i) {
			const string& IR1_path		= image_path_list[i * 4];
			const string& depth1_path	= image_path_list[i * 4 + 1];
			const string& IR2_path		= image_path_list[i * 4 + 2];
			const string& depth2_path	= image_path_list[i * 4 + 3];
			string prefix = IR1_path.substr(0, IR1_path.find("-a.bmp"));
			string bin_path_v1tov2 = prefix + "-bb-1119.bin";
			string bin_path_v2tov1 = prefix + "-aa-1119.bin";
			bin_prefix = prefix;
			process(IR1_path, IR2_path, depth1_path, depth2_path, prefix, bin_path_v1tov2, bin_path_v2tov1, save_temp_file);	

			//if (--count <= 0) break;
		}

	}


	return 0;
}

int old_main(int argc, char* argv[])
{
	string root_dir = TEST_ROOT_DIR;
	string number_id = TEST_NUMBER_ID;
	string save_temp_file_dir = TEST_SAVE_TEMP_FILE_DIR;
	//string bin_path_v2tov1 = TEST_OUPUT_BIN_PATH_v2tov1;
	//string bin_path_v1tov2 = TEST_OUPUT_BIN_PATH_v1tov2;
	bool save_temp_file = false;

	if (argc >= 3) {
		root_dir = argv[1];
		number_id = string(argv[2]);
		//bin_path_1 = string(argv[3]);
	}
	if (argc >= 4) {
		save_temp_file = true;
		save_temp_file_dir = string(argv[3]);
	}

	std::cout << number_id << ' ' << save_temp_file_dir << endl;
	string prefix = root_dir + number_id;
	string IR1_path = prefix + "-a.bmp";
	string IR2_path = prefix + "-b.bmp";
	string depth1_path = prefix + "-a.bin";
	string depth2_path = prefix + "-b.bin";
	string bin_path_v2tov1 = prefix + "-aa.bin";
	string bin_path_v1tov2 = prefix + "-bb.bin";
	prefix = save_temp_file_dir+number_id;
	bin_prefix = root_dir + number_id;

	process(IR1_path, IR2_path, depth1_path, depth2_path, prefix, bin_path_v1tov2, bin_path_v2tov1, save_temp_file);
	return 0;
}