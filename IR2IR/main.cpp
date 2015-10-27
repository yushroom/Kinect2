#include <vector>
#include <utility>
#include <atlimage.h>
#include <codex_math.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/contrib/contrib.hpp"

#include <fstream>
#include <algorithm>
#include "Timer.hpp"
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

#define ROOT_DIR			"D:\\project\\"
#define NUMBER_ID			"02-43-09-0"
#define IMAGE_DIR			ROOT_DIR NUMBER_ID "\\"
#define PREFIX_A			IMAGE_DIR "a-" NUMBER_ID
#define PREFIX_B			IMAGE_DIR "b-" NUMBER_ID
//#define DEPTH_V1			IMAGE_DIR "a" NUMBER_ID "-depth.bmp"
//#define DEPTH_V2			IMAGE_DIR "b" NUMBER_ID "-depth-resized.bmp"
#define INFRARED_V1			PREFIX_A ".bmp"
#define INFRARED_V2			PREFIX_B "-resized.bmp"
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

//const char * extrinsic_filename = "D:\\yyk\\extrinsics.yml";
//const char * intrinsic_filename = "D:\\yyk\\intrinsics.yml";

#define EXTRINSICS_FILE_PATH ROOT_DIR "extrinsics.yml"
#define INTRINSICS_FILE_PATH ROOT_DIR "intrinsics.yml"

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

inline void writePly(const Mat& points, const Mat& IR, const char *filename)
{
	FILE *fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", WIDTH * HEIGHT);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");

	for (int j = 0; j < HEIGHT; ++j)
		for (int i = 0; i < WIDTH; ++i) {
			int idx = j * WIDTH + i;
			int c = IR.at<unsigned char>(j, i);
			float x = points.at<float>(0, idx);
			float y = points.at<float>(1, idx);
			float z = points.at<float>(2, idx);
			fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, c, c, c);
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


Mat calc_points(Mat depth_image, float fx, float fy, float cx, float cy) {
	float inv_fx = 1.f / fx;
	float inv_fy = 1.f / fy;
	//float x, y, z;
	Mat points(3, WIDTH * HEIGHT, CV_32F);
	for (int j = 0; j < HEIGHT; ++j)
		for (int i = 0; i < WIDTH; ++i) {
			int idx = j * WIDTH + i;
			UINT16 d = depth_image.at<UINT16>(j, i);
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

void write_normalized_image(const Mat &img, const char *filename)
{
	int w = img.cols, h = img.rows;
	double vmin, vmax;
	minMaxLoc(img, &vmin, &vmax);

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
		for (int x = 0; x < w; y++)
		{
			int idx_first, idx_last, idx = x + y*w;

			//data term
			idx_first = (int)A.val.size();
			idx_last = (int)A.val.size();

			A.val.push_back(alpha);
			A.cols.push_back(idx);

			A.ptrb.push_back(idx_first);
			A.ptre.push_back(idx_last + 1);
			vb.push_back(I[idx]);
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
					A.val.push_back(1.0f);
					A.cols.push_back(xx + yy*w);

					A.ptrb.push_back(idx_first);
					A.ptre.push_back(idx_last + 1);
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
	memcpy(&result[0], x.v, sizeof(float)*x.length);
}

int main()
{
	Timer timer_all("All");
	timer_all.begin();

	Timer timer_prepare("All");
	timer_prepare.begin();

	Timer timer_mat("Mat");
	timer_mat.begin();
	Mat cameraMatrix[2], distCoeffs[2];
	//cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	//cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;
	//Mat R1, R2, P1, P2, Q;
	timer_mat.end();
	timer_mat.print();

	Timer timer_read_file("Read file");
	timer_read_file.begin();

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

		//Mat R, T;
		fs["R"] >> R;
		fs["T"] >> T;
		//fs["R1"] >> R1;
		//fs["R2"] >> R2;
		//fs["P1"] >> P1;
		//fs["P2"] >> P2;
		//fs["Q"] >> Q;
		//printf("T = (%lf, %lf, %lf)\n", T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0));
	}


	vector<UINT16>	depth_v1_pixels;
	vector<UINT16>	depth_v2_pixels;
	depth_v1_pixels.resize(WIDTH*HEIGHT);
	depth_v2_pixels.resize(512 * 424);
	// read depth file
	{
		FILE *depth_in;
		printf("%s\n", DEPTH_V1_PATH);
		fopen_s(&depth_in, DEPTH_V1_PATH, "rb");
		fread(&depth_v1_pixels[0], sizeof(UINT16), depth_v1_pixels.size(), depth_in);
		fclose(depth_in);
		printf("%s\n", DEPTH_V2_PATH);
		fopen_s(&depth_in, DEPTH_V2_PATH, "rb");
		fread(&depth_v2_pixels[0], sizeof(UINT16), depth_v2_pixels.size(), depth_in);
		fclose(depth_in);
	}

	UINT max_ele = *std::max_element(depth_v2_pixels.begin(), depth_v2_pixels.end());

	Mat img_depth_1 = ToCVImage(depth_v1_pixels);
	Mat img_depth_2 = ResizeAndToCVImage(depth_v2_pixels);
	double min, max;
	minMaxLoc(img_depth_2, &min, &max);
	//imshow("depth image 1", img_depth_1);
	//imshow("depth image 2", img_depth_2);

	Mat img_IR[2];
	cout << INFRARED_V1 << '\n' << INFRARED_V2 << '\n';
	img_IR[0] = imread(INFRARED_V1, 0);
	img_IR[1] = imread(INFRARED_V2, 0);

	timer_read_file.end();
	timer_read_file.print();

	Timer timer_init_undistort_map("init undistort map");
	timer_init_undistort_map.begin();
	Mat map11, map12, map21, map22;
	Size img_size(WIDTH, HEIGHT);
	// undistort only;
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], Mat::eye(3, 3, CV_64F), cameraMatrix[0], img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], Mat::eye(3, 3, CV_64F), cameraMatrix[1], img_size, CV_16SC2, map21, map22);
	timer_init_undistort_map.end();
	timer_init_undistort_map.print();

	Timer timer_test("Test");
	timer_test.begin();

	//Timer timer_undistort("undistort");
	//timer_undistort.begin();

	// test undistort
	//Mat img_IR_1_undistort = img_IR[0].clone();
	Mat img_IR_1_undistort(HEIGHT, WIDTH, img_IR[0].type());
	undistort(img_IR[0], img_IR_1_undistort, cameraMatrix[0], distCoeffs[0]);
	//imshow("IR1 [undistort]", img_IR_1_undistort);
	//image_write(UNDISTORT_IR1_PATH, img_IR_1_undistort);
	//Mat img_IR_2_undistort = img_IR[1].clone();
	Mat img_IR_2_undistort(HEIGHT, WIDTH, img_IR[1].type());
	undistort(img_IR[1], img_IR_2_undistort, cameraMatrix[1], distCoeffs[1]);
	//imshow("IR2 [undistort]", img_IR_2_undistort);
	//image_write(UNDISTORT_IR2_PATH, img_IR_2_undistort);

	//Mat img_depth_1_undistort = img_depth_1.clone();
	Mat img_depth_1_undistort(HEIGHT, WIDTH, img_depth_1.type());
	undistort(img_depth_1, img_depth_1_undistort, cameraMatrix[0], distCoeffs[0]);
	//img_depth_1_undistort *= 1.f / 255;
	//imshow("depth un",img_depth_1_undistort);
	//Mat img_depth_2_undistort = img_depth_2.clone();
	Mat img_depth_2_undistort(HEIGHT, WIDTH, img_depth_2.type());
	undistort(img_depth_2, img_depth_2_undistort, cameraMatrix[1], distCoeffs[1]);


	//remap(img_IR[0], img_IR_1_undistort, map11, map12, INTER_LINEAR);
	//remap(img_depth_1, img_depth_1_undistort, map11, map12, INTER_LINEAR);
	//remap(img_IR[1], img_IR_2_undistort, map21, map22, INTER_LINEAR);
	//remap(img_depth_2, img_depth_2_undistort, map21, map22, INTER_LINEAR);
	//Timer timer_flood_fill("Flood fill");
	//timer_flood_fill.begin();
	flood_fill(img_depth_1_undistort, 5);
	flood_fill(img_depth_2_undistort, 10);
	//timer_flood_fill.end();
	//timer_flood_fill.print();
	//timer_undistort.end();
	//timer_undistort.print();


#if 0
	//Mat temp = img_depth_1_undistort.clone();
	Mat temp(HEIGHT, WIDTH, img_depth_1_undistort.type());
	for (int j = 0; j < HEIGHT; ++j)
		for (int i = 0; i < WIDTH; ++i)
			temp.at<UINT16>(j, i) = int(img_depth_1_undistort.at<UINT16>(j, i)) % 256;
	//cvtColor(temp, temp, CV_GRAY2RGB);
	image_write(UNDISTORT_DEPTH1_PATH, temp);
	for (int j = 0; j < HEIGHT; ++j)
		for (int i = 0; i < WIDTH; ++i)
			temp.at<UINT16>(j, i) = int(img_depth_2_undistort.at<UINT16>(j, i)) % 256;
	image_write(UNDISTORT_DEPTH2_PATH, temp);
#endif

	//Timer timer_calc_points("Calc points");
	//timer_calc_points.begin();
	Mat points_1 = calc_points(img_depth_1_undistort, V1_FX, V1_FY, V1_CX, V1_CY);
	Mat points_2 = calc_points(img_depth_2_undistort, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
	if (write_file) {
		writePly(points_1, img_IR_1_undistort, POINTCLOUND_V1);
		writePly(points_2, img_IR_2_undistort, POINTCLOUND_V2);
	}
	//timer_calc_points.end();
	//timer_calc_points.print();

#if 1
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
		minMaxLoc(img_diff, &min, &max);
		cout << "min and max: " << min << ' ' << max << endl;
		Mat img_diff_g;
		GaussianBlur(img_diff, img_diff_g, Size(0, 0), 4, 0);
		img_diff = img_diff_g;
		
		Mat img_diff_pos = Mat::zeros(height_crop, width_crop, CV_8U);
		Mat img_diff_neg = Mat::zeros(height_crop, width_crop, CV_8U);

		//img_diff *= 1.f / max;
		double inv_max = 255.f / std::max(abs(max), abs(min))  * 4.f;
		//int count = 0;
		for (int j = 0; j < height_crop; ++j)
			for (int i = 0; i < width_crop; ++i) {
				auto val = img_diff.at<float>(j, i);
				if (val <= -0.1) {
					img_diff_neg.at<unsigned char>(j, i) = unsigned char(-val * inv_max);
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
	}
#endif
#if 0
	// 2 -> 1
	//Timer timer_2to1("2->1");
	//timer_2to1.begin();
	{
		Mat trans = Mat::zeros(3, 3, CV_32F);
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				trans.at<float>(i, j) = R.at<double>(j, i);
		//for (int i = 0; i < 3; ++i)
		//	trans.at<float>(i, 3) = -10*T.at<double>(i);

		//points_1 = trans * points_1;
		Mat points_2_2 = trans * points_2;
		double tx = T.at<double>(0) * -10;
		double ty = T.at<double>(1) * -10;
		double tz = T.at<double>(2) * -10;
		//tx = ty = tz = 0;
		//cout << "tx = " << tx << "  ty = " << ty << "  tz = " << tz << endl;

		Timer timer_write_ply("write ply");
		timer_write_ply.begin();
		writePly2(points_1, points_2_2, img_IR_1_undistort, img_IR_2_undistort, POINTCLOUND_ALL_1);
		timer_write_ply.end();
		timer_write_ply.print();

		Mat img = Mat::zeros(HEIGHT, WIDTH, img_IR[1].type());
		for (int j = 0; j < HEIGHT; ++j) {
			for (int i = 0; i < WIDTH; ++i) {
				int idx = j * WIDTH + i;
				int x, y;
				float z = points_2.at<float>(2, idx);
				if (z <= 1e-2f && z >= -1e-2f) {
					continue;
				}
				z = points_2_2.at<float>(2, idx) + tz;
				x = int(V1_FX * (points_2_2.at<float>(0, idx) + tx) / z + V1_CX - 0.5f);
				y = int(V1_FY * (points_2_2.at<float>(1, idx) + ty) / z + V1_CY - 0.5f);
				//z = points_2_2.at<float>(2, idx);
				//x = int(V1_FX * (points_2_2.at<float>(0, idx)) / z + V1_CX - 0.5f);
				//y = int(V1_FY * (points_2_2.at<float>(1, idx)) / z + V1_CY - 0.5f);
				//cout << x << ' ' << y << ' ' << z << '\n';
				if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
					img.at<unsigned char>(y, x) = img_IR_2_undistort.at<unsigned char>(j, i);
				}
			}
		}
		//imshow("RGB2", img);
		image_write(V2_TO_V1_IMAGE, img);
	}
	//timer_2to1.end();
	//timer_2to1.print();
	timer_test.end();
	timer_test.print();
#endif

	timer_all.end();
	timer_all.print();
	//system("pause");
	char c = (char)waitKey();
	//if (c == 27 || c == 'q' || c == 'Q')
	//	return 0;
	return 0;
}