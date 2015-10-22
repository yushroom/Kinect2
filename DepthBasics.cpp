//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <stdio.h>
#include <strsafe.h>
#include <sstream>
#include <fstream>
#include "resource.h"
#include "DepthBasics.h"
#include "RenderFish.hpp"
#include "Math.hpp"
#include "NuiApi.h"

using namespace std;

#define IR_FX (387.56565394355408)
#define IR_FY (387.43950296593346)
#define IR_CX (259.15112583019447)
#define IR_CY (206.03282313805542)

#define PREFIX				"D:\\yyk\\ply"
//#define POINTCLOUND1			PREFIX "a_.ply"
//#define POINTCLOUND2			PREFIX "b_point2.ply"
#define IMAGE_PATH_PREFIX_A		"D:\\yyk\\image"
#define IMAGE_PATH_PREFIX_W		L"D:\\yyk\\image"

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(
	_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR lpCmdLine,
	_In_ int nShowCmd
	)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);
	
	log_system_init();

	CDepthBasics application;
	application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CDepthBasics::CDepthBasics() :
	m_hWnd(NULL),
	m_nStartTime(0),
	m_nLastCounter(0),
	m_nFramesSinceUpdate(0),
	m_fFreq(0),
	m_nNextStatusTime(0LL),
	m_bSaveScreenshot(false),
	m_pD2DFactory(NULL),
	m_pDrawDepth1(NULL),
	m_pDrawDepth2(NULL)
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}
}


/// <summary>
/// Destructor
/// </summary>
CDepthBasics::~CDepthBasics()
{
	// clean up Direct2D renderer
	if (m_pDrawDepth2)
	{
		delete m_pDrawDepth2;
		m_pDrawDepth2 = NULL;
	}

	if (m_pDrawDepth1)
	{
		delete m_pDrawDepth1;
		m_pDrawDepth1 = NULL;
	}

	if (m_pDrawInfrared1)
	{
		delete m_pDrawInfrared1;
		m_pDrawInfrared1 = NULL;
	}

	if (m_pDrawInfrared2)
	{
		delete m_pDrawInfrared2;
		m_pDrawInfrared2 = NULL;
	}


	// clean up Direct2D
	SafeRelease(m_pD2DFactory);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CDepthBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
	MSG       msg = { 0 };
	WNDCLASS  wc;

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"DepthBasicsAppDlgWndClass";

	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDD_APP),
		NULL,
		(DLGPROC)CDepthBasics::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, nCmdShow);

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		Update();

		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_KEYDOWN)
			{
				info("key down\n");
			}
			// If a dialog message will be taken care of by the dialog proc
			if (hWndApp && IsDialogMessageW(hWndApp, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}


	}

	return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CDepthBasics::Update()
{
	m_kinectV2.Update();
	m_kinectV1.Update();

	if (m_bSaveScreenshot) {

		GeneratePointCloud();
		m_bSaveScreenshot = false;
	}

	//m_nFramesSinceUpdate++;
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	CDepthBasics* pThis = NULL;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<CDepthBasics*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<CDepthBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message)
	{
	case WM_INITDIALOG:
	{
		// Bind application window handle
		m_hWnd = hWnd;

		// Init Direct2D
		D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

		// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
		// We'll use this to draw the data we receive from the Kinect to the screen
		m_pDrawDepth1 = new ImageRenderer();
		int w = KinectSensorV1::cDepthWidth, h = KinectSensorV1::cDepthHeight;
		HRESULT hr = m_pDrawDepth1->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, w, h, w * sizeof(RGBQUAD));
		if (FAILED(hr))
		{
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		}

		m_pDrawInfrared1 = new ImageRenderer();
		hr = m_pDrawInfrared1->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW3), m_pD2DFactory, w, h, w * sizeof(RGBQUAD));
		if (FAILED(hr))
		{
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		}

		w = KinectSensorV2::cDepthWidth;
		h = KinectSensorV2::cDepthHeight;
		m_pDrawDepth2 = new ImageRenderer();
		hr = m_pDrawDepth2->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW2), m_pD2DFactory, w, h, w * sizeof(RGBQUAD));
		if (FAILED(hr))
		{
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		}

		m_pDrawInfrared2 = new ImageRenderer();
		hr = m_pDrawInfrared2->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW4), m_pD2DFactory, w, h, w * sizeof(RGBQUAD));
		if (FAILED(hr))
		{
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		}

		// Get and initialize the default Kinect sensor
		InitializeDefaultSensor();
	}
	break;

	// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		break;

	case WM_DESTROY:
		// Quit the main message pump
		PostQuitMessage(0);
		break;

	case WM_KEYDOWN:
	{
		switch (wParam)
		{
		case VK_ESCAPE:
			SendMessage(hWnd, WM_CLOSE, 0, 0);
			break;
		case VK_RETURN:
			info("print\n");
			break;
		default:
			break;
		}
	}
	break;

		// Handle button press
	case WM_COMMAND:
		// If it was for the screenshot control and a button clicked event, save a screenshot next frame 
		if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			info("button clicked\n");
			SetStatusMessage(L"button clicked", 10000, true);
			m_bSaveScreenshot = true;
		}
		break;
	}

	return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = m_kinectV2.InitializeDefaultSensor();
	m_kinectV2.SetImageRender(m_pDrawDepth2, m_pDrawInfrared2);
	hr = m_kinectV1.CreateFirstConnected();
	m_kinectV1.SetImageRender(m_pDrawDepth1, m_pDrawInfrared1);

	return hr;
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CDepthBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
	INT64 now = GetTickCount64();

	if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
	{
		SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
		m_nNextStatusTime = now + nShowTimeMsec;

		return true;
	}

	return false;
}

/// <summary>
/// Get the name of the file where screenshot will be stored.
/// </summary>
/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT CDepthBasics::GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, LPWSTR lpszFilePath_depth, UINT nFilePathSize, int kinect_version)
{
	WCHAR* pszKnownPath = IMAGE_PATH_PREFIX_W;
	//HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);
	HRESULT hr = S_OK;
	if (SUCCEEDED(hr))
	{
		// Get the time
		WCHAR szTimeString[MAX_PATH];
		GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

		if (kinect_version == 1) {
			StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\a-%s-%d.bmp", pszKnownPath, szTimeString, m_nScreenShotCount);
			StringCchPrintfW(lpszFilePath_depth, nFilePathSize, L"%s\\a-%s-%d-depth.bmp", pszKnownPath, szTimeString, m_nScreenShotCount);
		}
		else{
			StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\b-%s-%d.bmp", pszKnownPath, szTimeString, m_nScreenShotCount);
			StringCchPrintfW(lpszFilePath_depth, nFilePathSize, L"%s\\b-%s-%d-depth.bmp", pszKnownPath, szTimeString, m_nScreenShotCount);
		}
	}

	//if (pszKnownPath)
	//{
	//	CoTaskMemFree(pszKnownPath);
	//}

	return hr;
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = { 0 };

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes = 1;                         // Default
	bmpInfoHeader.biSizeImage = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = { 0 };

	bfh.bfType = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

	// Create the file on disk to write to
	HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	// Return if error opening file
	if (NULL == hFile)
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the RGB Data
	if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}

typedef Vec3 vector3f;
typedef UINT16 DepthType;
typedef BYTE InfraredType;

inline vector3f unproject(const int x, const int y, const float fx, const float fy, const float cx, const float cy) {
	float zz = 1.f;
	float xx = (x - cx) / fx * zz;
	float yy = (y - cy) / fy * zz;
	return{ xx, yy, zz };
}

inline void calc_points(const vector<DepthType>& depth_pixels, const vector<BYTE>& infrared_pixels, vector<vector3f>& point_cloud, int width, int height,
	const float fovx, const float fovy)
//inline void calc_points(const UINT16* depth_pixels, const UINT16* infrared_pixels, vector<vector3f>& point_cloud, int width, int height,
//	const float fx, const float fy, const float cx, const float cy)
{
	//Assert(depth_pixels.size() == width * height);
	//point_cloud.resize(width*height);
	//for (int i = 0; i < depth_pixels.size(); i++) {
	//	int w = i % width;
	//	int h = i / width;
	//	h = height - 1 - h;
	//	point_cloud[i] = /*unproject(w, h, fx, fy, cx, cy) **/ depth_pixels[i];
	//}

	point_cloud.resize(width*height);
	const float DegreesToRadians = 3.14159265359f / 180.0f;
	const float xScale = tanf(fovx * DegreesToRadians * 0.5f) * 2.0f / width;
	const float yScale = tanf(fovy * DegreesToRadians * 0.5f) * 2.0f / height;
	int	half_width = width / 2;
	int	half_height = height / 2;
	for (int j = 0; j < height; j++){
		for (int i = 0; i < width; i++){
			int idx = j*width + i;
			unsigned short pixel_depth = depth_pixels[idx];
			float	depth = -pixel_depth * 0.001;	//	unit in meters

			//auto pos = NuiTransformDepthImageToSkeleton(i, j, pixel_depth, NUI_IMAGE_RESOLUTION_640x480);
			//point_cloud[idx].x = pos.x / pos.w;
			//point_cloud[idx].y = pos.y / pos.w;
			//point_cloud[idx].z = pos.z / pos.w;

			//point_cloud[idx].x= -(i + 0.5 - half_width) * xyScale * depth;
			//point_cloud[idx].y = (j + 0.5 - half_height) * xyScale * depth;
			//point_cloud[idx].z = depth;		//	in OpenGL coordinate

			point_cloud[idx].x = -(i + 0.5f - half_width) * xScale * depth;
			point_cloud[idx].y = (j  + 0.5f - half_height) * yScale * depth;
			point_cloud[idx].z = depth;
		}
	}

	return;
}

inline void writePly(const vector<vector3f>& point_cloud, const vector<BYTE>& infrared_pixels, const char *filename)
//inline void writePly(const vector<vector3f>& point_cloud, const RGBQUAD* infrared_pixels, const char *filename)
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
		//fprintf_s(fp, "%f %f %f %d %d %d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, 255, 255, 255);
	}
	fclose(fp);
}


void CDepthBasics::GeneratePointCloud()
{
#if 0
	vector<vector3f> points_und1;
	calc_points(m_kinectV1.rawDepthData, m_kinectV1.rawInfraredData, points_und1, KinectSensorV1::cDepthWidth, KinectSensorV1::cDepthHeight,
		KinectSensorV1::fovx, KinectSensorV1::fovy);

	char path[MAX_PATH];

	sprintf(path, "%s\\a_%04d.ply", PREFIX, m_nScreenShotCount);

	writePly(points_und1, m_kinectV1.rawInfraredData, path);
	info("save file to %s\n", path);

	vector<vector3f> points_und;
	calc_points(m_kinectV2.rawDepthData, m_kinectV2.rawInfraredData, points_und, KinectSensorV2::cDepthWidth, KinectSensorV2::cDepthHeight,
		KinectSensorV2::fovx, KinectSensorV2::fovy);

	sprintf(path, "%s\\b_%04d.ply", PREFIX, m_nScreenShotCount);

	writePly(points_und, m_kinectV2.rawInfraredData, path);
	info("save file to %s\n", path);
#endif
	SaveInfraredImage();
}

// uint16
void sava_raw_depth(vector<UINT16> raw_depth, int size, const char* filename) {
	ofstream os(filename, ios::binary);
	os.write((char*)&raw_depth[0], size * sizeof(UINT16));
	os.close();
}

void CDepthBasics::SaveInfraredImage()
{
	WCHAR szScreenshotPath[MAX_PATH];
	WCHAR szScreenshotPath_depth[MAX_PATH];
#if 1
	//wsprintf(szScreenshotPath, L"%s\\a_%04d.bmp", IMAGE_PATH_PREFIX_W, m_nScreenShotCount);
	GetScreenshotFileName(szScreenshotPath, szScreenshotPath_depth, _countof(szScreenshotPath), 1);
	SaveBitmapToFile(reinterpret_cast<BYTE*>(m_kinectV1.m_pTempColorBuffer), KinectSensorV1::cDepthWidth, KinectSensorV1::cDepthHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);
	SaveBitmapToFile(m_kinectV1.m_depthRGBX, KinectSensorV1::cDepthWidth, KinectSensorV1::cDepthHeight, sizeof(RGBQUAD) * 8, szScreenshotPath_depth);
	info("image a saved.\n");
	sava_raw_depth(m_kinectV1.rawDepthData, KinectSensorV1::cDepthWidth * KinectSensorV1::cDepthHeight, "d:\\yyk\\image\\k1_depth.bin");
	//info("save file to %s\\a_%04d.bmp\n", IMAGE_PATH_PREFIX_A, m_nScreenShotCount);

#endif
#if 1
	//wsprintf(szScreenshotPath, L"%s\\b_%04d.bmp", IMAGE_PATH_PREFIX_W, m_nScreenShotCount);
	GetScreenshotFileName(szScreenshotPath, szScreenshotPath_depth, _countof(szScreenshotPath), 2);
	SaveBitmapToFile(reinterpret_cast<BYTE*>(m_kinectV2.m_pInfraredRGBX), KinectSensorV2::cDepthWidth, KinectSensorV2::cDepthHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);
	SaveBitmapToFile(reinterpret_cast<BYTE*>(m_kinectV2.m_pDepthRGBX), KinectSensorV2::cDepthWidth, KinectSensorV2::cDepthHeight, sizeof(RGBQUAD) * 8, szScreenshotPath_depth);
	//info("save file to %s\\b_%04d.bmp\n", IMAGE_PATH_PREFIX_A, m_nScreenShotCount);
	info("image b saved.\n");
	sava_raw_depth(m_kinectV2.rawDepthData, KinectSensorV2::cDepthWidth * KinectSensorV2::cDepthHeight, "d:\\yyk\\image\\k2_depth.bin");
#endif
	m_nScreenShotCount++;
}
