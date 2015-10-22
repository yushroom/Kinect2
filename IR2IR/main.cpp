#include <vector>
#include <utility>
#include <atlimage.h>
#include <codex_math.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"

using namespace cv;

typedef codex::math::vector::vector3<float>		vector3f;
typedef codex::math::vector::vector3<int>		vector3i;

using std::vector;
using std::pair;

typedef unsigned short      UINT16, *PUINT16;
typedef unsigned int		UINT;

const char*					lT_path = "E:\\table.txt";
#define KINECT
#define PREFIX				"D:\\yyk\\image\\"
#define NUMBER_ID			"-02-41-37-0"
#define DEPTH_V1			PREFIX "a" NUMBER_ID "-depth.bmp"
#define DEPTH_V2			PREFIX "b" NUMBER_ID "-depth-resized.bmp"
#define INFRARED_V1			PREFIX "a" NUMBER_ID ".bmp"
#define INFRARED_V2			PREFIX "b" NUMBER_ID "-resized.bmp"
#define POINTCLOUND_V1		PREFIX "a" NUMBER_ID "_v1_point.ply"
#define POINTCLOUND_V2		PREFIX "b" NUMBER_ID "_v2_point.ply"
#define POINTCLOUNDRGB		PREFIX NUMBER_ID "_point_rgb.ply"
#define DEPTH_V1_PATH		"D:\\yyk\\image\\k1_depth.bin"
#define DEPTH_V2_PATH		"D:\\yyk\\image\\k2_depth.bin"

const char * extrinsic_filename = "D:\\yyk\\extrinsics.yml";
const char * intrinsic_filename = "D:\\yyk\\intrinsics.yml";

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

typedef UINT16 DEPTH_TYPE;

const int nv_per_line = 1000;
const float axis_length = 300;

const int	depth_width = 640;
const int	depth_height = 480;

const int	depth_width_v2 = 640;
const int	depth_height_v2 = 480;

vector<pair<int, int>> ts = {
	{ 342, 146 },
	{ 391, 137 },
	{ 222, 151 },
	{ 257, 143 },
	{ 347, 221 },
	{ 398, 221 },
	{ 226, 227 },
	{ 261, 228 } };


vector<pair<float, float>>	undistortLookupTable;

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

inline UINT bilinear(float x, float y, UINT16* src, int width, int height, bool ignore_zero = false) {
	int ix0 = x, iy0 = y, ix1 = ix0 + 1, iy1 = iy0 + 1;
	float qx1 = x - ix0, qy1 = y - iy0, qx0 = 1.f - qx1, qy0 = 1.f - qy1;

	float weight = 0.f, value = 0.f;
	float w[] = { qx0*qy0, qx1*qy0, qx0*qy1, qx1*qy1 };
	pair<int, int> xy[] = { { ix0, iy0 }, { ix1, iy0 }, { ix0, iy1 }, { ix1, iy1 } };

	for (int j = 0; j < 4; j++) {
		if (xy[j].first < 0 || xy[j].first >= width
			|| xy[j].second < 0 || xy[j].second >= height)
			continue;
		int idx = xy[j].first + xy[j].second * width;
		if (ignore_zero && src[idx] == 0) continue;
		value += src[idx] * w[j];
		weight += w[j];
	}

	return static_cast<UINT>(weight == 0.f ? 0.f : value / weight);
}

inline void writePly(const vector<vector3f>& point_cloud, const vector<UINT16>& infrared_pixels, const char *filename)
{
	FILE *fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", point_cloud.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");

	for (int i = 0; i < point_cloud.size(); i++) {
		fprintf_s(fp, "%f %f %f %d %d %d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, infrared_pixels[i], infrared_pixels[i], infrared_pixels[i]);
	}
	fclose(fp);
}

inline void drawALine(const vector3f& s, const vector3f& t, const int n, const vector3i& rgb, FILE *fp) {
	vector3f d = (t - s) / n;
	for (int i = 0; i < n; i++)	{
		vector3f p = s + d*i;
		fprintf_s(fp, "%g %g %g %d %d %d\n", p.x, p.y, p.z, rgb.x, rgb.y, rgb.z);
	}
}
///222 151
inline void writePly(
	const vector<vector3f>& point_cloud,
	const vector<UINT16>& infrared_pixels,
	const vector3f& o1, const vector3f &x1, const vector3f &y1, const vector3f &z1,
	const vector3f& o2, const vector3f &x2, const vector3f &y2, const vector3f &z2,
	const char *filename,
	const vector<pair<vector3f, vector3f>> lines = {})
{

	FILE *fp;
	fopen_s(&fp, filename, "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment file created by Microsoft Kinect Fusion\n");
	fprintf(fp, "element vertex %d\n", point_cloud.size() + nv_per_line*(6 + lines.size()));
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "element face 0\n");
	fprintf(fp, "property list uchar int vertex_index\n");
	fprintf(fp, "end_header\n");


	for (int i = 0; i < point_cloud.size(); i++) {
		fprintf_s(fp, "%f %f %f %d %d %d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, infrared_pixels[i], infrared_pixels[i], infrared_pixels[i]);
	}
	vector3f o[] = { o1, o2 };
	vector3f t[] = { x1, y1, z1, x2, y2, z2 };
	vector3i rgb[] = { vector3i(255, 0, 0), vector3i(0, 255, 0), vector3i(0, 0, 255) };

	for (int i = 0; i < 6; i++) {
		vector3f from = o[i / 3], dir = t[i];
		vector3f to = from + dir.normalized_vector()*axis_length;
		vector3i color = rgb[i % 3];
		drawALine(from, to, nv_per_line, color, fp);
	}
	for (int i = 0; i < lines.size(); i++)
		drawALine(lines[i].first, lines[i].second, nv_per_line, vector3i(0, 255, 255), fp);
	fclose(fp);
}

inline vector3f unproject(const int x, const int y, const float fx, const float fy, const float cx, const float cy) {
	float zz = 1.f;
	float xx = (x - cx) / fx * zz;
	float yy = (y - cy) / fy * zz;
	return{ xx, yy, zz };
}

inline void calc_points(const vector<DEPTH_TYPE>& depth_pixels, const vector<UINT16>& infrared_pixels, vector<vector3f>& point_cloud, int width, int height,
	const float fx, const float fy, const float cx, const float cy)
{
	point_cloud.resize(width*height);
	for (int i = 0; i < depth_pixels.size(); i++) {
		int w = i % width;
		int h = i / width;
		h = height - 1 - h;
		point_cloud[i] = unproject(w, h, fx, fy, cx, cy)*depth_pixels[i];
	}
}

inline void project(const vector3f& point, int &x, int &y, const float &fx, const float &fy, const float &cx, const float &cy) {
	x = fx*point.x / point.z + cx;
	y = fy*point.y / point.z + cy;
}

vector<DEPTH_TYPE>	depth_v1_pixels;
vector<UINT16>  infrared_v1_pixels;
vector<DEPTH_TYPE>	depth_v2_pixels;
vector<UINT16>  infrared_v2_pixels;

vector<DEPTH_TYPE>	depth_v1_pixels_und;
vector<UINT16>	infrared_v1_pixels_und;
vector<DEPTH_TYPE>	depth_v2_pixels_und;
vector<UINT16>	infrared_v2_pixels_und;

vector<vector3f> normals_und;
vector<vector3f> points_v1_und;
vector<vector3f> points_v2_und;

//void calc_normals(const vector<DEPTH_TYPE> &depth_pixels, vector<vector3f>& normals, int width, int height) {
//	vector<vector3f> points(depth_pixels.size());
//	normals.resize(depth_pixels.size());
//	for (int i = 0; i < height; i++)
//		for (int j = 0; j < width; j++) {
//			int w = j, h = i;
//			points[i*width + j] = unproject(w, h, IR_FX, IR_FY, IR_CX, IR_CY);
//			//printf("%g %g %g\n", points[i*width + j].x, points[i*width + j].y, points[i*width + j].z);
//		}
//	for (int i = 0; i < height; i++)
//		for (int j = 0; j < width; j++) {
//			if (i == 0 || i == height - 1 || j == 0 || j == width - 1) continue;
//			pair<int, int> coor[] = { { i - 1, j }, { i + 1, j }, { i, j - 1 }, { i, j + 1 } };
//			vector3f *p[4];
//			for (int k = 0; k < 4; k++)
//				p[k] = &points[coor[k].first*width + coor[k].second];
//			vector3f v1 = *p[1] - *p[0], v2 = *p[3] - *p[2];
//			normals[i*width + j] = v1;
//
//			normals[i*width + j].normalize();
//			//normals[i*width + j] = points[i*width + j];
//		}
//}


void resize_v2(const vector<UINT16> depth_pixels, vector<UINT16>* out_depth_pixels){
	Mat mat(Size(512, 424), CV_16UC1);
	//Mat mat(depth_pixels);
	for (int j = 0; j < 424; j++) {
		for (int i = 0; i < 512; ++i) {
			mat.at<UINT16>(j, i) = depth_pixels[j * 512 + i];
		}
	}
	printf("%d %d\n", mat.cols, mat.rows);
	//mat.resize(580, 480);
	cv::resize(mat, mat, Size(580, 480));
	printf("%d %d\n", mat.cols, mat.rows);
	Mat new_mat(Size(640, 480), mat.type());
	mat.copyTo(new_mat(Rect(30, 0, 580, 480)));
	//for (int )
	//new_mat.copyTo(*out_depth_pixels);
	for (int j = 0; j < 480; j++) {
		for (int i = 0; i < 640; ++i) {
			(*out_depth_pixels)[j * 640 + i] = new_mat.at<UINT16>(j, i);
		}
	}
}


void undistort(vector<UINT16>& src, vector<UINT16>& dest, const Mat& m, const Mat& d)
{
	Mat temp(Size(depth_width, depth_height), CV_16UC1);
	Mat temp2(Size(depth_width, depth_height), CV_16UC1);
	for (int j = 0; j < depth_height; j++) {
		for (int i = 0; i < depth_width; ++i) {
			temp.at<UINT16>(j, i) = src[j * depth_width + i];
		}
	}
	//printf("%lf %lf, %lf, %lf\n", m1.at<double>(0, 0), m2.at<double>(0, 0), d1.at<double>(0, 0), d2.at<double>(0, 0));
	cv::undistort(temp, temp2, m, d);
	//cv::undistort(infrared_v2_pixels, infrared_v2_pixels_und, m2, d2);
	for (int j = 0; j < depth_height; j++) {
		for (int i = 0; i < depth_width; ++i) {
			dest[j * depth_width + i] = temp2.at<UINT16>(j, i);
		}
	}
}

int main()
{
	//ReadCalibratedUndistortionTable(undistortLookupTable, depth_width, depth_height, lT_path);
	depth_v1_pixels.resize(depth_width*depth_height);
	infrared_v1_pixels.resize(depth_width*depth_height);
	depth_v1_pixels_und.resize(depth_width*depth_height);
	infrared_v1_pixels_und.resize(depth_width*depth_height);

	depth_v2_pixels.resize(512*424);
	infrared_v2_pixels.resize(depth_width_v2*depth_height_v2);
	depth_v2_pixels_und.resize(depth_width_v2*depth_height_v2);
	infrared_v2_pixels_und.resize(depth_width_v2*depth_height_v2);
	FILE *depth_in;
	printf("%s\n", DEPTH_V1_PATH);
	fopen_s(&depth_in, DEPTH_V1_PATH, "rb");
	fread(&depth_v1_pixels[0], sizeof(DEPTH_TYPE), depth_v1_pixels.size(), depth_in);
	fclose(depth_in);

	printf("%s\n", DEPTH_V2_PATH);
	fopen_s(&depth_in, DEPTH_V2_PATH, "rb");
	fread(&depth_v2_pixels[0], sizeof(DEPTH_TYPE), depth_v2_pixels.size(), depth_in);
	fclose(depth_in);
	CImage infrared_v1, depth_v1, infrared_v2, depth_v2;
	CImage depth;
	infrared_v1.Load(INFRARED_V1);
	infrared_v2.Load(INFRARED_V2);
	depth_v1.Load(DEPTH_V1);
	depth_v2.Load(DEPTH_V2);

	for (int i = 0; i < depth_height; i++)
		for (int j = 0; j < depth_width; j++) {
			COLORREF c = infrared_v1.GetPixel(j, i);
			infrared_v1_pixels[i*depth_width + j] = GetRValue(c);
			c = infrared_v2.GetPixel(j, i);
			infrared_v2_pixels[i*depth_width + j] = GetRValue(c);
		}
	for (int i = 0; i < depth_height_v2; i++)
		for (int j = 0; j < depth_width_v2; j++) {
			COLORREF c = infrared_v2.GetPixel(j, i);
			infrared_v2_pixels[i*depth_width_v2 + j] = GetRValue(c);
		}

	resize_v2(depth_v2_pixels, &depth_v2_pixels_und);

	{
		FileStorage fs;
		fs.open(intrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename);
			return -1;
		}
		Mat m1, d1, m2, d2;
		fs["M1"] >> m1;
		fs["D1"] >> d1;
		fs["M2"] >> m2;
		fs["D2"] >> d2;

		undistort(infrared_v1_pixels, infrared_v1_pixels_und, m1, d1);
		undistort(depth_v1_pixels, depth_v1_pixels_und, m1, d1);
		undistort(infrared_v2_pixels, infrared_v2_pixels_und, m2, d2);
		undistort(depth_v2_pixels_und, depth_v2_pixels_und, m2, d2);
	}


	{
		CImage infrared;
		infrared.Create(depth_width, depth_height, 32);
		for (int i = 0; i < depth_height; i++)
			for (int j = 0; j < depth_width; j++)
				infrared.SetPixelRGB(j, i, infrared_v1_pixels_und[i*depth_width + j],
				infrared_v1_pixels_und[i*depth_width + j],
				infrared_v1_pixels_und[i*depth_width + j]);
		infrared.Save("D:\\yyk\\image\\k1_IR_undisort.png");

		for (int i = 0; i < depth_height; i++)
			for (int j = 0; j < depth_width; j++)
				infrared.SetPixelRGB(j, i, infrared_v2_pixels_und[i*depth_width + j],
				infrared_v2_pixels_und[i*depth_width + j],
				infrared_v2_pixels_und[i*depth_width + j]);
		infrared.Save("D:\\yyk\\image\\k2_IR_undisort.png");
	}

	// save v1
	for (int i = 0; i < depth_height; i++)
		for (int j = 0; j < depth_width; j++)
			depth_v1.SetPixelRGB(j, i, depth_v1_pixels[i*depth_width + j], depth_v1_pixels[i*depth_width + j],
				depth_v1_pixels[i*depth_width + j]);
	depth_v1.Save("D:\\yyk\\image\\k1_depth.png");

	// save v2
	for (int i = 0; i < depth_height_v2; i++)
		for (int j = 0; j < depth_width_v2; j++)
			depth_v2.SetPixelRGB(j, i, depth_v2_pixels_und[i*depth_width_v2 + j], depth_v2_pixels_und[i*depth_width_v2 + j],
			depth_v2_pixels_und[i*depth_width_v2 + j]);
	depth_v2.Save("D:\\yyk\\image\\k2_depth.png");

	calc_points(depth_v1_pixels_und, infrared_v1_pixels_und, points_v1_und, depth_width, depth_height,
		V1_FX, V1_FY, V1_CX, V1_CY);
	writePly(points_v1_und, infrared_v1_pixels_und, POINTCLOUND_V1);

	//calc_points(depth_v2_pixels_und, infrared_v2_pixels_und, points_v2_und, depth_width_v2, depth_height_v2,
	//	V2_FX, V2_FY, V2_CX, V2_CY);
	calc_points(depth_v2_pixels_und, infrared_v2_pixels_und, points_v2_und, depth_width_v2, depth_height_v2,
		V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
	writePly(points_v2_und, infrared_v2_pixels_und, POINTCLOUND_V2);
 

	FileStorage fs;
	fs.open(extrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filename);
		return -1;
	}

	Mat R, T;
	fs["R"] >> R;
	fs["T"] >> T;

	codex::math::matrix4x4f trans;
	codex::math::matrix::identity(trans);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			trans.n[i][j] = R.at<double>(i, j);
	for (int i = 0; i < 3; i++)
		trans.n[i][3] = T.at<double>(i);
	printf("%g\n", T.at<double>(0) / 10);

	vector3f o1, x1(25, 0, 0), y1(0, 25, 0), z1(0, 0, 25);
	vector3f
		o2 = trans * o1,
		x2 = trans * x1 - o2,
		y2 = trans * y1 - o2,
		z2 = trans * z1 - o2;
	printf("%g %g %g %g %g %g\n", o1.x, o1.y, o1.z, o2.x, o2.y, o2.z);

	//vector<pair<vector3f, vector3f>> ps;
	//for (int i = 0; i < ts.size(); i++) {
	//	float fx = i % 2 == 0 ? V1_FX : V2_FX;
	//	float fy = i % 2 == 0 ? V1_FY : V2_FY;
	//	float cx = i % 2 == 0 ? V1_CX : V2_CX;
	//	float cy = i % 2 == 0 ? V1_CY : V2_CY;
	//	vector3f t = unproject(ts[i].first, depth_height - 1 - ts[i].second, fx, fy, cx, cy) * 1000;
	//	if (i % 2 == 0) t = trans * t;
	//	ps.push_back(std::make_pair(i % 2 == 0 ? o1 : o2, t));
	//}
	//writePly(points_v2_und, infrared_v2_pixels_und,
	//	o1, x1, y1, z1,
	//	o2, x2, y2, z2,
	//	POINTCLOUNDRGB,
	//	ps);

	CImage rgb;
	rgb.Create(depth_width_v2, depth_height_v2, 32);
	for (int i = 0; i < points_v1_und.size(); i++) {
		int x, y;
		points_v1_und[i] = trans * points_v1_und[i];
		project(points_v1_und[i], x, y, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
		//printf("%d %d\n", x, y);
		if (x < 0 || x >= depth_width_v2 || y < 0 || y >= depth_height_v2) continue;
		rgb.SetPixelRGB(x, depth_height_v2 - 1 - y, infrared_v1_pixels_und[i], infrared_v1_pixels_und[i], infrared_v1_pixels_und[i]);
	}
	rgb.Save("D:\\yyk\\image\\rgb.png");

	{
		CImage rgb;
		rgb.Create(depth_width_v2, depth_height_v2, 32);
		auto inv_trans = trans.inversed_matrix();
		for (int i = 0; i < points_v1_und.size(); i++) {
			int x, y;
			points_v1_und[i] = inv_trans * points_v1_und[i];
			project(points_v1_und[i], x, y, V2_RESIZED_FX, V2_RESIZED_FY, V2_RESIZED_CX, V2_RESIZED_CY);
			//printf("%d %d\n", x, y);
			if (x < 0 || x >= depth_width_v2 || y < 0 || y >= depth_height_v2) continue;
			rgb.SetPixelRGB(x, depth_height_v2 - 1 - y, infrared_v1_pixels_und[i], infrared_v1_pixels_und[i], infrared_v1_pixels_und[i]);
		}
		rgb.Save("D:\\yyk\\image\\rgb2.png");
	}
	return 0;
}