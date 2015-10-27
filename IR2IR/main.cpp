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

using namespace cv;
using namespace std;

typedef unsigned short UINT16;

#ifdef _DEBUG
#define WRITE_FILE
#endif

#define WRITE_FILE

#ifdef WRITE_FILE
static const bool write_file = true;
#else
static const bool write_file = false;
#endif

#define WIDTH 640
#define HEIGHT 480

#define ROOT_DIR			"D:\\yyk\\image\\"
#define NUMBER_ID			"01-15-55-0"
#define IMAGE_DIR			ROOT_DIR NUMBER_ID "\\"
#define PREFIX_A			IMAGE_DIR "a-" NUMBER_ID
#define PREFIX_B			IMAGE_DIR "b-" NUMBER_ID
#define INFRARED_V1			PREFIX_A ".bmp"
#define INFRARED_V2			PREFIX_B ".bmp"
#define POINTCLOUND_V1		PREFIX_A "-point.ply"
#define POINTCLOUND_V2		PREFIX_B "-point.ply"
#define POINTCLOUND_ALL_1	IMAGE_DIR "point-all-1.ply"
#define POINTCLOUND_ALL_2	IMAGE_DIR "point-all-2.ply"
#define DEPTH_V1_PATH		PREFIX_A ".bin"
#define DEPTH_V2_PATH		PREFIX_B ".bin"

#define UNDISTORT_IR1_PATH	PREFIX_A "-IR-und.bmp"
#define UNDISTORT_IR2_PATH	PREFIX_B "-IR-und.bmp"
#define UNDISTORT_DEPTH1_PATH	PREFIX_A "-d-und.bmp"
#define UNDISTORT_DEPTH2_PATH	PREFIX_B "-d-und.bmp"
#define V1_TO_V2_IMAGE		IMAGE_DIR "v1-to-v2.bmp"
#define V2_TO_V1_IMAGE		IMAGE_DIR "v2-to-v1.bmp"

#define POINT_CLOUD_1		PREFIX_A "-points.ply"
#define POINT_CLOUD_2		PREFIX_B "-points.ply"
#define POINT_CLOUD_DIFF	PREFIX_A "-points-diff.ply"

//const char * extrinsic_filename = "D:\\yyk\\extrinsics.yml";
//const char * intrinsic_filename = "D:\\yyk\\intrinsics.yml";

#define EXTRINSICS_FILE_PATH ROOT_DIR "extrinsics.yml"
#define INTRINSICS_FILE_PATH ROOT_DIR "intrinsics.yml"
#define TABLE1	ROOT_DIR "table1.txt"
#define TABLE2	ROOT_DIR "table2.txt"

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

const int	depth_width = 640;
const int	depth_height = 480;

const float DEPTH_INVALID = 1e30f;
#define VALID_DEPTH_TEST(a) (a < DEPTH_INVALID)

void solve_for_bias(const int w, const int h, const std::vector<float> &I,
					const float alpha, const float lambda, const int niter,
					std::vector<float> &result)
{
	const int NUM_NEIGHBOR = 4;
	static int dir[NUM_NEIGHBOR * 2] = { -1, 0, 1, 0, 0, -1, 0, 1 };

	advmath::la_matrix_csr<float> A;
	std::vector<float> vb;
	A.column = w*h;

	std::vector<int> reg2idx, idx2reg;
	reg2idx.resize(w*h, -1);

	int count = 0;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			int idx = x + y*w;
			if (VALID_DEPTH_TEST(I[idx]))
			{
				reg2idx[idx] = count;
				idx2reg.push_back(idx);
				count++;
			}
		}

		for (int i = 0; i < idx2reg.size(); i++)
		{
			int idx = idx2reg[i],
				x = idx % w, y = idx / w;
			int idx_first, idx_last;

			//data term
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
				if (xx >= 0 && xx < w && yy >= 0 && yy < h && idx < xx + yy*w && reg2idx[xx + yy*w] >= 0 && 
					VALID_DEPTH_TEST(I[reg2idx[xx + yy*w]]))
				{
					idx_first = (int)A.val.size();
					idx_last = (int)A.val.size();

					A.val.push_back(1.0f);
					A.cols.push_back(i);
					A.val.push_back(-1.0f);
					A.cols.push_back(reg2idx[xx + yy*w]);

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
}

void adjust_for_bias(std::vector<float> &depth, const std::vector<float> &bias)
{
	for (int i = 0; i < depth.size(); i++)
		depth[i] = (VALID_DEPTH_TEST(depth[i]) ? depth[i]-bias[i] : 0);
}

void prepare_for_solving_bias(std::vector<float> &depth, 
							  const std::vector<float> &depth1,
							  const std::vector<float> &depth2)
{
	depth.resize(depth1.size());
	for (int i = 0; i < depth.size(); i++)
		depth[i] = (VALID_DEPTH_TEST(depth1[i]) && VALID_DEPTH_TEST(depth2[i])) ? depth2[i]-depth1[i] : DEPTH_INVALID;
}

Mat ResizeAndToCVImage(const vector<UINT16> depth_pixels){
	Mat mat = Mat::zeros(Size(512, 424), CV_16UC1);
	for (int j = 0; j < 424; j++) {
		for (int i = 0; i < 512; ++i) {
			mat.at<UINT16>(j, i) = depth_pixels[j * 512 + i];
		}
	}
	//printf("%d %d\n", mat.cols, mat.rows);
	cv::resize(mat, mat, Size(580, 480));
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

// points: Mat(3, WIDTH * HEIGHT)
// IR: Mat(HEIGHT, WIDTH)

#define ZERO(x) ((fabsf(x) < 1e-3f))

inline void writePly(const Mat& points, const Mat& IR, const char *filename, int w = WIDTH, int h = HEIGHT, 
					 float r = 1, float g = 1, float b = 1, float tx = 0, float ty = 0, float tz = 0)
{
	FILE *fp;
	fopen_s(&fp, filename, "w");
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
			int c = IR.at<unsigned char>(j, i);
			float x = points.at<float>(0, idx) + tx;
			float y = points.at<float>(1, idx) + ty;
			float z = points.at<float>(2, idx) + tz;
			fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, unsigned char(c*r), unsigned char(c*g), unsigned char(c*b));
		}
	fclose(fp);
}

inline void writePly_roi(const Mat& points, const Mat& IR, const char *filename, bool cropped = false, cv::Rect rect = Rect(0, 0, WIDTH, HEIGHT), float r = 1, float g = 1, float b = 1)
{
	FILE *fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", rect.width * rect.height);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");

	if (!cropped) {
		for (int j = 0; j < rect.height; ++j) {
			for (int i = 0; i < rect.width; ++i) {
				int idx = j * rect.width + i;
				int c = IR.at<unsigned char>(j+rect.height, i+rect.width);
				float x = points.at<float>(0, idx);
				float y = points.at<float>(1, idx);
				float z = points.at<float>(2, idx);
				fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, int(r*c), int(g*c), int(b*c));
			}
		}
	}


	fclose(fp);
}


struct Vertex {
	float x, y, z;
	unsigned char r, g, b;
};

//inline void writePly_binary(const Mat& points, const Mat& IR, const char *filename)
//{
//	ofstream fout(filename, ios::binary);
//	fout << "ply\n";
//	fout << "format binary_little_endian 1.0\n";
//	fout << "element vertex " << WIDTH * HEIGHT << '\n';
//	fout << "property float x\n"
//		<< "property float y\n"
//		<< "property float z\n"
//		<< "property uchar red\n"
//		<< "property uchar green\n"
//		<< "property uchar blue\n"
//		<< "element face 0\n"
//		<< "property list uchar int vertex_index\n"
//		<< "end_header\n";
//
//	vector<Vertex> vertices;
//	vertices.reserve(WIDTH * HEIGHT);
//	for (int j = 0; j < HEIGHT; ++j)
//		for (int i = 0; i < WIDTH; ++i) {
//			int idx = j * WIDTH + i;
//			int c = IR.at<unsigned char>(j, i);
//			float x = points.at<float>(0, idx);
//			float y = points.at<float>(1, idx);
//			float z = points.at<float>(2, idx);
//			//fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, c, c, c);
//			//fout << x << y << z << c << c << c;
//			if (z !=)
//			vertices.push_back({ x, y, z, c, c, c });
//		}
//	fout.write((char*)&vertices[0], vertices.size() * sizeof(Vertex));
//	fout.close();
//}

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
	Mat points(3, w * h, CV_32F);
	for (int j = roi.y; j < roi.y + roi.height; ++j)
		for (int i = roi.x; i < roi.x + roi.width; ++i) {
			int idx = (j-roi.y) * w + (i-roi.x);
			float d = depth[idx];
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

void flood_fill(Mat& img, int offset = 2)
{
	Mat temp = img.clone();
	for (int j = 0; j < HEIGHT; ++j) {
		int i = 0;
		while (i < WIDTH && img.at<UINT16>(j, i) == 0)
			++i;
		//cout << i << endl;
		for (int k = 0; k < offset && i + k < WIDTH; ++k)
			temp.at<UINT16>(j, i + k) = 0;

		i = WIDTH - 1;
		while (i >= 0 && img.at<UINT16>(j, i) == 0)
			--i;
		for (int k = 0; k < offset && i - k >= 0; ++k)
			temp.at<UINT16>(j, i - k) = 0;
	}
	for (int i = 0; i < WIDTH; ++i) {
		int j = 0;
		while (j < HEIGHT && img.at<UINT16>(j, i) == 0)
			++j;
		for (int k = 0; k < offset && j + k < HEIGHT; ++k)
			temp.at<UINT16>(j + k, i) = 0;

		j = HEIGHT - 1;
		while (j >= 0 && img.at<UINT16>(j, i) == 0)
			--j;
		for (int k = 0; k < offset && j - k >= 0; ++k)
			temp.at<UINT16>(j - k, i) = 0;
	}


	temp.copyTo(img);
}

void get_valid_rect(const Mat& image, int* out_x, int* out_y, int* out_width, int* out_height)
{
	*out_x = 110;
	*out_y = 60;
	*out_width = 500 - 110;
	*out_height = 360 - 60;
}

void image_show(const char* title, const Mat& image) {
	if (write_file)
		imshow(title, image);
}

void image_write(const char* path, const Mat& image) {
	if (write_file)
		imwrite(path, image);
}

template<typename T>
vector<T> mat_to_vector(const Mat& img) {
	int w = img.cols;
	int h = img.rows;
	vector<float> vec;
	vec.resize(w*h);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			vec[idx] = img.at<T>(j,i);
		}
	return vec;
}

template<typename T1, typename T2>
vector<T1> mat_to_vector(const Mat& img) {
	int w = img.cols;
	int h = img.rows;
	vector<float> vec;
	vec.resize(w*h);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			vec[idx] = img.at<T2>(j,i);
		}
		return vec;
}

Mat vector_to_img_uc(const vector<float>& vec, int w, int h, float scale) {
	Mat img(h, w, CV_8U);
	for (int j = 0; j < h; ++j)
		for (int i = 0; i < w; ++i) {
			int idx = j * w + i;
			img.at<unsigned char>(j, i) = unsigned char(vec[idx] * scale);
		}
	return img;
}

Mat vector_to_img_f(const vector<float>& vec, int w, int h, float scale) {
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

	image_write(filename, output);
}

void solve_for_bias(const int w, const int h, const std::vector<float> &I,
	const float alpha, const float lambda, const int niter,
	std::vector<float> &result)
{
	const int NUM_NEIGHBOR = 4;
	static int dir[NUM_NEIGHBOR * 2] = { -1, 0, 1, 0, 0, -1, 0, 1 };

	advmath::la_matrix_csr<float> A;
	std::vector<float> vb;
	A.column = w*h;

	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			int idx_first, idx_last, idx = x + y*w;

			//data term
			idx_first = (int)A.val.size();
			idx_last = (int)A.val.size();

			A.val.push_back(alpha);
			A.cols.push_back(idx);

			A.ptrb.push_back(idx_first);
			A.ptre.push_back(idx_last + 1);
			vb.push_back(alpha*I[idx]);
			A.row++;

			//smoothness terms
			for (int i = 0; i < NUM_NEIGHBOR; i++)
			{
				int xx = x + dir[i * 2 + 0],
					yy = y + dir[i * 2 + 1];
				if (xx >= 0 && xx < w && yy >= 0 && yy < h && idx < xx + yy*w)
				{
					idx_first = (int)A.val.size();
					idx_last = (int)A.val.size();

					A.val.push_back(1.0f);
					A.cols.push_back(idx);
					A.val.push_back(-1.0f);
					A.cols.push_back(xx + yy*w);

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

	result.resize(x.length);
	std::memcpy(&result[0], x.v, sizeof(float)*x.length);
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

inline float bilinear(float x, float y, UINT16* src, int width, int height, bool ignore_zero = false) {
	int ix0 = x, iy0 = y, ix1 = ix0 + 1, iy1 = iy0 + 1;
	float qx1 = x - ix0, qy1 = y - iy0, qx0 = 1.f - qx1, qy0 = 1.f - qy1;

	float weight = 0.f, value = 0.f;
	float w[] = { qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1 };
	pair<int, int> xy[] = { { ix0, iy0 }, { ix1, iy0 }, { ix0, iy1 }, { ix1, iy1 } };

	for (int j = 0; j<4; j++) {
		if (xy[j].first < 0 || xy[j].first >= width
			|| xy[j].second < 0 || xy[j].second >= height)
			continue;
		int idx = xy[j].first + xy[j].second * width;
		if (ignore_zero && src[idx] == 0) continue;
		value += src[idx] * w[j];
		weight += w[j];
	}

	return weight == 0.f ? 0.f : value / weight;
}

void undistort2(InputArray _src, OutputArray _dst, InputArray _cameraMatrix,
    InputArray _distCoeffs, InputArray _newCameraMatrix = noArray())
{
    Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();
    Mat distCoeffs = _distCoeffs.getMat(), newCameraMatrix = _newCameraMatrix.getMat();

    _dst.create(src.size(), src.type());
    Mat dst = _dst.getMat();

    CV_Assert(dst.data != src.data);

    int stripe_size0 = std::min(std::max(1, (1 << 12) / std::max(src.cols, 1)), src.rows);
    Mat map1(stripe_size0, src.cols, CV_16SC2), map2(stripe_size0, src.cols, CV_16UC1);

    Mat_<double> A, Ar, I = Mat_<double>::eye(3, 3);

    cameraMatrix.convertTo(A, CV_64F);
    if (!distCoeffs.empty())
        distCoeffs = Mat_<double>(distCoeffs);
    else
    {
        distCoeffs.create(5, 1, CV_64F);
        distCoeffs = 0.;
    }

    if (!newCameraMatrix.empty())
        newCameraMatrix.convertTo(Ar, CV_64F);
    else
        A.copyTo(Ar);

    double v0 = Ar(1, 2);
#define OUTPUT_LOOKUP_TABLE
#ifdef OUTPUT_LOOKUP_TABLE
    FILE *fp = fopen("E:\\table.txt", "w");
#endif
    for (int y = 0; y < src.rows; y += stripe_size0)
    {
        int stripe_size = std::min(stripe_size0, src.rows - y);
        Ar(1, 2) = v0 - y;
        Mat map1_part = map1.rowRange(0, stripe_size),
            map2_part = map2.rowRange(0, stripe_size),
            dst_part = dst.rowRange(y, y + stripe_size);

        initUndistortRectifyMap(A, distCoeffs, I, Ar, Size(src.cols, stripe_size),
            map1_part.type(), map1_part, map2_part);
#ifdef OUTPUT_LOOKUP_TABLE
        Mat mapx, mapy;
        mapx.create(stripe_size, src.cols, CV_32FC1);
        mapy.create(stripe_size, src.cols, CV_32FC1);
        cv::convertMaps(map1_part, map2_part, mapx, mapy, CV_32FC1);

        for (int i = 0; i<stripe_size; i++)
        for (int j=0; j<512; j++)
            fprintf(fp, "%g %g %g %g\n", (float)j, (float)(y + i), mapx.at<float>(i, j), mapy.at<float>(i, j));
#endif
        remap(src, dst_part, map1_part, map2_part, INTER_LINEAR, BORDER_CONSTANT);
    }
#ifdef OUTPUT_LOOKUP_TABLE
    fclose(fp);
    exit(0);
#endif
}

int main()
{
	Mat cameraMatrix[2], distCoeffs[2];
	//cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	//cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;

	{
		const char * extrinsic_filename = EXTRINSICS_FILE_PATH;
		const char * intrinsic_filename = INTRINSICS_FILE_PATH;
		FileStorage fs;
		fs.open(intrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename);
		}
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];

		fs.open(extrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
		}

		fs["R"] >> R;
		fs["T"] >> T;
	}

	// read depth file
	vector<UINT16>	depth_pixels[2];
	depth_pixels[0].resize(WIDTH*HEIGHT);
	depth_pixels[1].resize(512 * 424);
	{
		FILE *depth_in;
		printf("%s\n", DEPTH_V1_PATH);
		fopen_s(&depth_in, DEPTH_V1_PATH, "rb");
		fread(&depth_pixels[0][0], sizeof(UINT16), depth_pixels[0].size(), depth_in);
		fclose(depth_in);
		printf("%s\n", DEPTH_V2_PATH);
		fopen_s(&depth_in, DEPTH_V2_PATH, "rb");
		fread(&depth_pixels[1][0], sizeof(UINT16), depth_pixels[1].size(), depth_in);
		fclose(depth_in);
	}

	{
		Mat img_depth_1 = ToCVImage(depth_pixels[1]);
		depth_pixels[1] = mat_to_vector<UINT16>(img_depth_1);
	}
	Mat img_IR[2];
	std::cout << INFRARED_V1 << '\n' << INFRARED_V2 << '\n';
	img_IR[0] = imread(INFRARED_V1, 0);
	{
		Mat mat = imread(INFRARED_V2, 0);
		cv::resize(mat, mat, Size(580, 480));
		img_IR[1].create(Size(640, 480), mat.type());
		mat.copyTo(img_IR[1](Rect(30, 0, 580, 480)));
	}

	vector<UINT16> IR[2];
	vector<UINT16> IR_und[2];
	//vector<UINT16> depth[2];
	vector<float> depth_und[2];

	for (int i = 0; i < 2; ++i) {
		IR[i] = mat_to_vector<UINT16>(img_IR[i]);
		IR_und[i].resize(WIDTH * HEIGHT);
		depth_und[i].resize(WIDTH * HEIGHT);
	}

	vector<pair<float, float>>	undistortLookupTable[2];
	ReadCalibratedUndistortionTable(undistortLookupTable[0], WIDTH, HEIGHT, TABLE1);
	ReadCalibratedUndistortionTable(undistortLookupTable[1], WIDTH, HEIGHT, TABLE2);
	for (int j = 0; j < 2; ++j) {
		for (int i = 0; i < WIDTH * HEIGHT; i++) {
			auto entry = undistortLookupTable[0][i];
			IR[j][i] = bilinear(entry.first, entry.second, &IR[j][0], WIDTH, HEIGHT);
			depth_und[j][i] = bilinear(entry.first, entry.second, &depth_pixels[j][0], WIDTH, HEIGHT, true);
		}
	}

	//Mat map11, map12, map21, map22;
	//Size img_size(WIDTH, HEIGHT);
	// undistort only;
	//cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], Mat::eye(3, 3, CV_64F), cameraMatrix[0], img_size, CV_16SC2, map11, map12);
	//cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], Mat::eye(3, 3, CV_64F), cameraMatrix[1], img_size, CV_16SC2, map21, map22);


	//// undistort
	////Mat img_IR_1_undistort = img_IR[0].clone();
	//Mat img_IR_1_undistort(HEIGHT, WIDTH, img_IR[0].type());
	////cv::undistort(img_IR[0], img_IR_1_undistort, cameraMatrix[0], distCoeffs[0]);
	//Mat img_IR_2_undistort(HEIGHT, WIDTH, img_IR[1].type());
	////cv::undistort(img_IR[1], img_IR_2_undistort, cameraMatrix[1], distCoeffs[1]);
	//Mat img_depth_1_undistort(HEIGHT, WIDTH, img_depth_1.type());
	////cv::undistort(img_depth_1, img_depth_1_undistort, cameraMatrix[0], distCoeffs[0]);
	//Mat img_depth_2_undistort(HEIGHT, WIDTH, img_depth_2.type());
	////cv::undistort(img_depth_2, img_depth_2_undistort, cameraMatrix[1], distCoeffs[1]);

	//remap(img_IR[0], img_IR_1_undistort, map11, map12, INTER_LINEAR);
	//remap(img_depth_1, img_depth_1_undistort, map11, map12, INTER_LINEAR);
	//remap(img_IR[1], img_IR_2_undistort, map21, map22, INTER_LINEAR);
	//remap(img_depth_2, img_depth_2_undistort, map21, map22, INTER_LINEAR);
	//flood_fill(img_depth_1_undistort, 4);
	//flood_fill(img_depth_2_undistort, 8);

	//Mat points_1 = calc_points<UINT16>(img_depth_1_undistort, V1_FX, V1_FY, V1_CX, V1_CY);
	//Mat points_2 = calc_points<UINT16>(img_depth_2_undistort, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
	//if (write_file) {
	//	writePly(points_1, img_IR_1_undistort, POINTCLOUND_V1);
	//	writePly(points_2, img_IR_2_undistort, POINTCLOUND_V2);
	//}

	Mat points_1 = calc_points(depth_und[0], V1_FX, V1_FY, V1_CX, V1_CY);
	Mat points_2 = calc_points(depth_und[1], V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);

	// 1 -> 2
	{
		Mat trans = Mat::zeros(3, 3, CV_32F);
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				trans.at<float>(i, j) = R.at<double>(i, j);
		//for (int i = 0; i < 3; ++i)
		//	trans.at<float>(i, 3) = T.at<double>(i);

		//points_1 = trans * points_1;
		Mat points_1_1 = trans * points_1;
		double tx = T.at<double>(0) * 10;
		double ty = T.at<double>(1) * 10;
		double tz = T.at<double>(2) * 10;
		//tx = ty = tz = 0;
		//cout << "tx = " << tx << "  ty = " << ty << "  tz = " << tz << endl;

		writePly2(points_1_1, points_2, img_IR_1_undistort, img_IR_2_undistort, POINTCLOUND_ALL_2, tx, ty, tz, 0, 0, 0);
		writePly(points_1_1, img_IR_1_undistort, POINT_CLOUD_1, WIDTH, HEIGHT, 1, 1, 0, tx, ty, tz);
		writePly(points_2, img_IR_2_undistort, POINT_CLOUD_2, WIDTH, HEIGHT, 0, 0, 1);

		//int count = 0;
		//cout << img_IR[1].type() << endl;
		Mat img_new_IR = Mat::zeros(HEIGHT, WIDTH, img_IR[1].type());
		Mat img_new_depth = Mat::zeros(HEIGHT, WIDTH, CV_32F);
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
				x = int(V2_RESIZED_FX * (points_1_1.at<float>(0, idx) + tx) / z + V2_RESIZED_CX - 0.5f);
				y = int(V2_RESIZED_FY * (points_1_1.at<float>(1, idx) + ty) / z + V2_RESIZED_CY - 0.5f);
				//cout << x << ' ' << y << ' ' << z << '\n';
				if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
					img_new_IR.at<unsigned char>(y, x) = img_IR_1_undistort.at<unsigned char>(j, i);
					img_new_depth.at<float>(y, x) = z;
				}
			}
		}
		Mat img_depth_2_undistort_f = Mat::zeros(HEIGHT, WIDTH, CV_32F);
		for (int j = 0; j < HEIGHT; ++j)
			for (int i = 0; i < WIDTH; ++i) {
				UINT16 val = img_depth_2_undistort.at<UINT16>(j, i);
				if (val <  15000)
					img_depth_2_undistort_f.at<float>(j, i) = (float)img_depth_2_undistort.at<UINT16>(j, i);
			}

		
		//double min, max;
		//minMaxLoc(img_depth_2_undistort_f, &min, &max);

		Mat img_new_depth_cropped;
		int width_crop = 0, height_crop = 0;
		Rect roi;
		{
			int x = 0, y = 0;
			get_valid_rect(img_new_depth, &x, &y, &width_crop, &height_crop);
			roi = Rect(x, y, width_crop, height_crop);
			img_new_depth(roi).copyTo(img_new_depth_cropped);
		}
		
		write_normalized_image(img_new_depth_cropped, "D:\\depth_1_und.bmp");
		write_normalized_image(img_depth_2_undistort_f(roi), "D:\\depth_2_und.bmp");
		Mat img_diff = img_new_depth_cropped - img_depth_2_undistort_f(roi);
		double min, max;
		cv::minMaxLoc(img_diff, &min, &max);
		std::cout << "min and max: " << min << ' ' << max << endl;
		Mat img_diff_g;
		//cv::GaussianBlur(img_diff, img_diff_g, Size(0, 0), 4, 0);
		//img_diff = img_diff_g;
		
		Mat img_diff_pos = Mat::zeros(height_crop, width_crop, CV_8U);
		Mat img_diff_neg = Mat::zeros(height_crop, width_crop, CV_8U);
		Mat img_diff_neg_f = Mat::zeros(height_crop, width_crop, CV_32F);

		//img_diff *= 1.f / max;
		double inv_max = 255.f / std::max(abs(max), abs(min))  * 4.f;
		//int count = 0;
		for (int j = 0; j < height_crop; ++j)
			for (int i = 0; i < width_crop; ++i) {
				auto val = img_diff.at<float>(j, i);
				if (val <= -0.1) {
					img_diff_neg.at<unsigned char>(j, i) = unsigned char(-val * inv_max);
					img_diff_neg_f.at<float>(j, i) = -val;
				}
				else {
					img_diff_pos.at<unsigned char>(j, i) = unsigned char(val * inv_max);
					//count++;
				}
			}
		//cout << "count " << count << endl;
		//cout << "count = " << count << endl;
		//imshow("RGB", img);
		imshow("cropped neg", img_diff_neg);
		image_write(V1_TO_V2_IMAGE, img_new_IR);
		image_write("D:\\depth_pos.bmp", img_diff_pos);
		image_write("D:\\depth_neg.bmp", img_diff_neg);

		
		vector<float> I = mat_to_vector(img_diff_neg_f);
		std::cout << I.size() << endl;
		vector<float> result;
		solve_for_bias(width_crop, height_crop, I, .35, 0, 500, result);
		
		Mat img_uc = vector_to_img_uc(result, width_crop, height_crop, inv_max);
		imshow("result", img_uc);

		Mat img_diff_new = img_depth_2_undistort_f(roi) - vector_to_img_f(result, width_crop, height_crop, 1);
		imshow("diff result", imgf_to_imguc(img_diff_new));

		auto points = calc_points<float>(img_diff_new, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY, true, roi);
		writePly(points, img_IR_2_undistort(roi), POINT_CLOUD_DIFF, width_crop, height_crop, 0, 1, 0);
	}

	//system("pause");
	char c = (char)waitKey();
	//if (c == 27 || c == 'q' || c == 'Q')
	//	return 0;
	return 0;
}