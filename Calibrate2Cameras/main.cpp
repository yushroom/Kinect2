/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
Oct. 3, 2008
Right to use this code in any way you want without warranty, support or any guarantee of it working.

BOOK: It would be nice if you cited it:
Learning OpenCV: Computer Vision with the OpenCV Library
by Gary Bradski and Adrian Kaehler
Published by O'Reilly Media, October 3, 2008

AVAILABLE AT:
http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
Or: http://oreilly.com/catalog/9780596516130/
ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

OPENCV WEBSITES:
Homepage:      http://opencv.org
Online docs:   http://docs.opencv.org
Q&A forum:     http://answers.opencv.org
Issue tracker: http://code.opencv.org
GitHub:        https://github.com/Itseez/opencv/
************************************************** */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

static int print_help()
{
	cout <<
		" Given a list of chessboard images, the number of corners (nx, ny)\n"
		" on the chessboards, and a flag: useCalibrated for \n"
		"   calibrated (0) or\n"
		"   uncalibrated \n"
		"     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
		"         matrix separately) stereo. \n"
		" Calibrate the cameras and display the\n"
		" rectified results along with the computed disparity images.   \n" << endl;
	cout << "Usage:\n ./stereo_calib -w board_width -h board_height [-nr /*dot not view results*/] <image list XML/YML file>\n" << endl;
	return 0;
}


static void InitIntrinsicMatrix(Mat &cameraMatrix, Mat& distCoeffs, double camera[], double distCoe[])
{
	cameraMatrix = Mat::eye(3, 3, CV_64F), distCoeffs = Mat::zeros(4, 1, CV_64F);
	double cameraMat[][3] = {
			{ camera[0], 0, camera[2] },
			{ 0, camera[1], camera[3] },
			{ 0, 0, 1 } };
	//		distCoe[] = {
	//			0.12829104246656706,
	//			-0.28001339944717657,
	//			0.00072822381144142909,
	//			-0.0015974931045705622
	//		};
	for (int i = 0; i<3; i++)
		for (int j = 0; j<3; j++)
			cameraMatrix.at<double>(i, j) = cameraMat[i][j];
	for (int i = 0; i<4; i++)
		distCoeffs.at<double>(i) = distCoe[i];
}

double cameraIR[] = { 387.56565394355408, 387.43950296593346, 259.15112583019447, 206.03282313805542 };
double cameraRGB_cropped[] = { 433.58680632154045, 434.03600324270764, 260.750267855103, 206.79593763084404 };
double distCoeffIR[] = { 0.1401352285492751, -0.27910635794735372, 0.0025445951650809925, 0.00251065759443636 };
double distCoeffRGB_cropped[] = { 0.079720648168906463, -0.015686540595863777, 0.0038426409245932592, 0.00056035216571513 };

#include <math.h>
#define M_PI       3.14159265358979323846
const double fovyLeft = 45.f / 180.f * M_PI;
// double cameraLeft[] = { 512.f/424 / 2.f / atanf(fovyLeft / 2.f), 424 / 2.f / atanf(fovyLeft / 2.f), 512 / 2.f, 424 / 2.f };
double cameraLeft[] = { 511.137, 511.192, 512 / 2.f, 424 / 2.f };
double distCoeffLeft[] = { 0, 0, 0, 0 };
static void
StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated = true, bool showRectified = true)
{
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	bool displayCorners = true;//true;
	const int maxScale = 2;
	const float squareSize = 1.f;  // Set this to your actual square size
	// ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			Mat img = imread(filename, 0);
			if (img.empty())
				break;
			if (imageSize == Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				Mat timg = img;
				resize(timg, img, imageSize);
				//cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				//break;
			}
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			if (displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
				30, 0.01));
		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
		}
	}
	cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		cout << "Error: too little pairs to run the calibration\n";
		system("Pause");
		return;
	}

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;

	//double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
	//	cameraMatrix[0], distCoeffs[0],
	//	cameraMatrix[1], distCoeffs[1],
	//	imageSize, R, T, E, F,
	//	TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
	//	CV_CALIB_FIX_ASPECT_RATIO +
	//	CV_CALIB_ZERO_TANGENT_DIST +
	//	CV_CALIB_SAME_FOCAL_LENGTH +
	//	CV_CALIB_RATIONAL_MODEL +
	//	CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
	{
		InitIntrinsicMatrix(cameraMatrix[0], distCoeffs[0], cameraLeft, distCoeffLeft);
		InitIntrinsicMatrix(cameraMatrix[1], distCoeffs[1], cameraLeft, distCoeffLeft);
		//InitIntrinsicMatrix(cameraMatrix[1], distCoeffs[1], cameraRGB_cropped, distCoeffRGB_cropped);
	}
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_INTRINSIC,
		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)
		);
	cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " << err / npoints << endl;

	// save intrinsic parameters
	FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open("extrinsics.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < nimages; i++)
	{
		Rect rect;
		for (k = 1; k >= 0; k--)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(255, 0, 255), 3, 8);
				if (rect.width == 0) rect = vroi;
				Mat dump = canvasPart(rect);
				string filename = goodImageList[i * 2 + k];
				filename.insert(filename.find_first_of('.'), "_valid");
				//cout << filename;
				//system("Pause");
				imwrite(filename, dump);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
}


static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}

int main(int argc, char** argv)
{
	Size boardSize;
	string imagelistfn;
	bool showRectified = true;

	for (int i = 1; i < argc; i++)
	{
		if (string(argv[i]) == "-w")
		{
			if (sscanf(argv[++i], "%d", &boardSize.width) != 1 || boardSize.width <= 0)
			{
				cout << "invalid board width" << endl;
				return print_help();
			}
		}
		else if (string(argv[i]) == "-h")
		{
			if (sscanf(argv[++i], "%d", &boardSize.height) != 1 || boardSize.height <= 0)
			{
				cout << "invalid board height" << endl;
				return print_help();
			}
		}
		else if (string(argv[i]) == "-nr")
			showRectified = false;
		else if (string(argv[i]) == "--help")
			return print_help();
		else if (argv[i][0] == '-')
		{
			cout << "invalid option " << argv[i] << endl;
			return 0;
		}
		else
			imagelistfn = argv[i];
	}

	if (imagelistfn == "")
	{
		imagelistfn = "stereo_calib.xml";
		boardSize = Size(9, 6);
	}
	boardSize = Size(9, 6);
	imagelistfn = "E:\\rect_input\\stereo_calib.xml";
#ifdef SAMPLE
	imagelistfn = "D:\\Program Files\\opencv\\sources\\samples\\cpp\\stereo_calib.xml";
#endif
	vector<string> imagelist;
	bool ok = readStringList(imagelistfn, imagelist);
#ifdef SAMPLE
	string prefix = string("D:\\Program Files\\opencv\\sources\\samples\\cpp\\");
	for (int i=0; i<imagelist.size(); i++) {
		imagelist[i] = prefix + imagelist[i];
	}
#endif
//#define RESIZE
#ifdef RESIZE
	{
		Mat target = imread(imagelist[0]);
		Size imageSize = target.size();
		for (auto& name : imagelist) {
			Mat src = imread(name);
			if (src.size() != imageSize) {
				Mat rt;
				int w = src.size().width*imageSize.height/src.size().height, 
					h = src.size().height*imageSize.height/src.size().height;
				Size s_size = Size(w, h);
				resize(src, rt, s_size);
				Rect rect(w/2-imageSize.width/2, 0,imageSize.width, h);	   
				Mat rt2 = rt(rect);
				imwrite(name, rt2);
			}
		}
		return 0;
	}
#endif
	if (!ok || imagelist.empty())
	{
		cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
		return print_help();
	}

	StereoCalib(imagelist, boardSize, true, showRectified);
	return 0;
}