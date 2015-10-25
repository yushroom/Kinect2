#include <vector>
#include <utility>
#include <atlimage.h>
#include <codex_math.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"
using namespace cv;
using namespace std;

typedef unsigned short UINT16;

#define WIDTH 640
#define HEIGHT 480

#define ROOT_DIR			"D:\\project\\"
#define NUMBER_ID			"04-31-42-0"
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
	Mat mat(Size(512, 424), CV_16UC1);
	for (int j = 0; j < 424; j++) {
		for (int i = 0; i < 512; ++i) {
			mat.at<UINT16>(j, i) = depth_pixels[j * 512 + i];
		}
	}
	//printf("%d %d\n", mat.cols, mat.rows);
	cv::resize(mat, mat, Size(580, 480));
	//printf("%d %d\n", mat.cols, mat.rows);
	Mat new_mat(Size(640, 480), mat.type());
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
	//int count = 0;
	//for (int i = 0; i < WIDTH * HEIGHT; ++i) {
	//	float x = points.at<float>(0, i);
	//	float y = points.at<float>(1, i);
	//	float z = points.at<float>(2, i);
	//	if (!ZERO(x*y*z))
	//		count++;
	//}
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
			//if (ZERO(x*y*z))
			//	continue;
			fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, c, c, c);
		}
	//for (int i = 0; i < WIDTH * HEIGHT; i++) {
	//	fprintf_s(fp, "%f %f %f %d %d %d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, infrared_pixels[i], infrared_pixels[i], infrared_pixels[i]);
	//}
	fclose(fp);
}

inline void writePly2(const Mat& points1, const Mat& points2, const Mat& IR1, const Mat& IR2, const char *filename)
{
	int offset = 0;
	FILE *fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", (WIDTH - offset*2) * (HEIGHT - offset*2) * 2);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");


	for (int j = offset; j < HEIGHT-offset; ++j)
		for (int i = offset; i < WIDTH-offset; ++i) {
			int idx = j * WIDTH + i;
			int c = IR1.at<unsigned char>(j, i);
			float x = points1.at<float>(0, idx);
			float y = points1.at<float>(1, idx);
			float z = points1.at<float>(2, idx);
			//if (x * y * z == 0)
			//	continue;
			fprintf_s(fp, "%f %f %f %d %d %d\n", x, y, z, c, c, 0);	// v1 Yellow
		}
	for (int j = offset; j < HEIGHT-offset; ++j)
		for (int i = offset; i < WIDTH-offset; ++i) {
			int idx = j * WIDTH + i;
			int c = IR2.at<unsigned char>(j, i);
			float x = points2.at<float>(0, idx);
			float y = points2.at<float>(1, idx);
			float z = points2.at<float>(2, idx);
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
	Mat points(3, WIDTH * HEIGHT, CV_32F);
	for (int j = 0; j < HEIGHT; ++j)
		for (int i = 0; i < WIDTH; ++i) {
			int idx = j * WIDTH + i;
			UINT16 d = depth_image.at<UINT16>(j, i);
			//if (d == 0)
			//	cout << "0";
			float x = (i - cx) / fx * d;
			float y = (j - cy) / fy * d;
			float z = d;
			points.at<float>(0, idx) = (i + 0.5f - cx) / fx * d; // x
			points.at<float>(1, idx) = (j + 0.5f - cy) / fy * d;
			points.at<float>(2, idx) = d;
			//points.at<float>(3, idx) = 1.f;
			//cout << x << ' ' << y << ' ' << z << '\n';
		}

	return points;
}

int main()
{
	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;
	Mat R1, R2, P1, P2, Q;

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
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		printf("T = (%lf, %lf, %lf)\n", T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0));
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

	Mat img_depth_1 = ToCVImage(depth_v1_pixels);
	Mat img_depth_2 = ResizeAndToCVImage(depth_v2_pixels);
	//imshow("depth image 1", img_depth_1);
	//imshow("depth image 2", img_depth_2);

	Mat img_IR[2];
	cout << INFRARED_V1 << '\n' << INFRARED_V2 << '\n';
	img_IR[0] = imread(INFRARED_V1, 0);
	img_IR[1] = imread(INFRARED_V2, 0);

	// test undistort
	//Mat img_IR_1_undistort = img_IR[0].clone();
	Mat img_IR_1_undistort(HEIGHT, WIDTH, img_IR[0].type());
	undistort(img_IR[0], img_IR_1_undistort, cameraMatrix[0], distCoeffs[0]);
	//imshow("IR1 [undistort]", img_IR_1_undistort);
	imwrite(UNDISTORT_IR1_PATH, img_IR_1_undistort);
	//Mat img_IR_2_undistort = img_IR[1].clone();
	Mat img_IR_2_undistort(HEIGHT, WIDTH, img_IR[1].type());
	undistort(img_IR[1], img_IR_2_undistort, cameraMatrix[1], distCoeffs[1]);
	//imshow("IR2 [undistort]", img_IR_2_undistort);
	imwrite(UNDISTORT_IR2_PATH, img_IR_2_undistort);

	//Mat img_depth_1_undistort = img_depth_1.clone();
	Mat img_depth_1_undistort(HEIGHT, WIDTH, img_depth_1.type());
	undistort(img_depth_1, img_depth_1_undistort, cameraMatrix[0], distCoeffs[0]);
	//img_depth_1_undistort *= 1.f / 255;
	//imshow("depth un",img_depth_1_undistort);
	//Mat img_depth_2_undistort = img_depth_2.clone();
	Mat img_depth_2_undistort(HEIGHT, WIDTH, img_depth_2.type());
	undistort(img_depth_2, img_depth_2_undistort, cameraMatrix[1], distCoeffs[1]);

	//Mat temp = img_depth_1_undistort.clone();
	Mat temp(HEIGHT, WIDTH, img_depth_1_undistort.type());
	for (int j = 0; j < HEIGHT; ++j)
		for (int i = 0; i < WIDTH; ++i)
			temp.at<UINT16>(j, i) = int(img_depth_1_undistort.at<UINT16>(j, i)) % 256;
	//cvtColor(temp, temp, CV_GRAY2RGB);
	imwrite(UNDISTORT_DEPTH1_PATH, temp);
	for (int j = 0; j < HEIGHT; ++j)
		for (int i = 0; i < WIDTH; ++i)
			temp.at<UINT16>(j, i) = int(img_depth_2_undistort.at<UINT16>(j, i)) % 256;
	imwrite(UNDISTORT_DEPTH2_PATH, temp);

	imwrite("D:\\d.bmp", img_depth_2);
	imwrite("D:\\d_u.bmp", img_depth_2_undistort);
	writePly(calc_points(img_depth_2, V1_FX, V1_FY, V1_CX, V1_CY), img_IR[1], "D:\\point.ply");

	Mat points_1 = calc_points(img_depth_1_undistort, V1_FX, V1_FY, V1_CX, V1_CY);
	Mat points_2 = calc_points(img_depth_2_undistort, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);

	writePly(points_1, img_IR_1_undistort, POINTCLOUND_V1);
	writePly(points_2, img_IR_2_undistort, POINTCLOUND_V2);

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

		writePly2(points_1_1, points_2, img_IR_1_undistort, img_IR_2_undistort, POINTCLOUND_ALL_2);

		//int count = 0;
		//cout << img_IR[1].type() << endl;
		Mat img = Mat::zeros(HEIGHT, WIDTH, img_IR[1].type());
		//Mat img = img_IR_1_undistort.clone();
		for (int j = 0; j < HEIGHT; ++j) {
			for (int i = 0; i < WIDTH; ++i) {
				int idx = j * WIDTH + i;
				int x, y;
				float z = points_1_1.at<float>(2, idx) + tz;
				x = int(V2_RESIZED_FX * (points_1_1.at<float>(0, idx) + tx) / z + V2_RESIZED_CX - 0.5f);
				y = int(V2_RESIZED_FY * (points_1_1.at<float>(1, idx) + ty) / z + V2_RESIZED_CY - 0.5f);
				//cout << x << ' ' << y << ' ' << z << '\n';
				if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
					img.at<unsigned char>(y, x) = img_IR_1_undistort.at<unsigned char>(j, i);
					//cout << idx << ' ' << x << ' ' << y << ' ' << z << '\n';
					//count++;
				}
			}
		}
		//cout << "count = " << count << endl;
		//imshow("RGB", img);
		imwrite(V1_TO_V2_IMAGE, img);
	}

	{
		Mat trans = Mat::zeros(3, 3, CV_32F);
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				trans.at<float>(i, j) = R.at<double>(j, i);
		//for (int i = 0; i < 3; ++i)
		//	trans.at<float>(i, 3) = T.at<double>(i);

		//points_1 = trans * points_1;
		Mat points_2_2 = trans * points_2;
		double tx = T.at<double>(0) * -10;
		double ty = T.at<double>(1) * -10;
		double tz = T.at<double>(2) * -10;
		//tx = ty = tz = 0;
		//cout << "tx = " << tx << "  ty = " << ty << "  tz = " << tz << endl;

		writePly2(points_1, points_2_2, img_IR_1_undistort, img_IR_2_undistort, POINTCLOUND_ALL_1);

		int count = 0;
		//cout << img_IR[1].type() << endl;
		Mat img = Mat::zeros(HEIGHT, WIDTH, img_IR[1].type());
		//Mat img = img_IR_1_undistort.clone();
		for (int j = 0; j < HEIGHT; ++j) {
			for (int i = 0; i < WIDTH; ++i) {
				int idx = j * WIDTH + i;
				int x, y;
				float z = points_2.at<float>(2, idx);
				if (z <= 1e-2f && z >= -1e-2f) {
					count++;
					continue;
				}
				z = points_2_2.at<float>(2, idx) + tz;
				x = int(V1_FX * (points_2_2.at<float>(0, idx) + tx) / z + V1_CX - 0.5f);
				y = int(V1_FY * (points_2_2.at<float>(1, idx) + ty) / z + V1_CY - 0.5f);
				//cout << x << ' ' << y << ' ' << z << '\n';
				if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
					img.at<unsigned char>(y, x) = img_IR_2_undistort.at<unsigned char>(j, i);
					//cout << idx << ' ' << x << ' ' << y << ' ' << z << '\n';
					//count++;
				}
			}
		}
		cout << "count = " << count << endl;
		//imshow("RGB2", img);
		imwrite(V2_TO_V1_IMAGE, img);
	}


	//Mat img = img_IR_1_undistort.

	//char c = (char)waitKey();
	//if (c == 27 || c == 'q' || c == 'Q')
		return 0;
}