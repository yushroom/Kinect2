//------------------------------------------------------------------------------
// <copyright file="DepthBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "ImageRenderer.h"
#include "KinectV2.h"
#include "KinectV1.h"

class CDepthBasics
{
	//// for V2
	//static const int        cDepthWidthV2 = 512;
	//static const int        cDepthHeightV2 = 424;
	//// for V1
	//static const int        cDepthWidthV1 = 640;
	//static const int        cDepthHeightV1 = 480;
	//static const int        cBytesPerPixel = 4;

public:
	CDepthBasics();
	~CDepthBasics();

	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Creates the main window and begins processing
	/// </summary>
	/// <param name="hInstance"></param>
	/// <param name="nCmdShow"></param>
	int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
	HWND                    m_hWnd;
	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	INT64                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;
	bool                    m_bSaveScreenshot;

	// Current Kinect
	//IKinectSensor*          m_pKinectSensor;

	// Depth reader
	//IDepthFrameReader*      m_pDepthFrameReader;
	KinectSensorV1			m_kinectV1;
	KinectSensorV2			m_kinectV2;

	// Direct2D
	ImageRenderer*			m_pDrawDepth1 = nullptr;
	ImageRenderer*          m_pDrawDepth2 = nullptr;
	ImageRenderer*			m_pDrawInfrared1 = nullptr;
	ImageRenderer*          m_pDrawInfrared2 = nullptr;
	ID2D1Factory*           m_pD2DFactory = nullptr;
	//RGBQUAD*                m_pDepthRGBX;

	/// <summary>
	/// Main processing function
	/// </summary>
	void                    Update();

	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                 InitializeDefaultSensor();


	/// <summary>
	/// Set the status bar message
	/// </summary>
	/// <param name="szMessage">message to display</param>
	/// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
	/// <param name="bForce">force status update</param>
	bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

	/// <summary>
	/// Get the name of the file where screenshot will be stored.
	/// </summary>
	/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
	/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
	/// <returns>
	/// S_OK on success, otherwise failure code.
	/// </returns>
	HRESULT                 GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize);

	/// <summary>
	/// Save passed in image data to disk as a bitmap
	/// </summary>
	/// <param name="pBitmapBits">image data to save</param>
	/// <param name="lWidth">width (in pixels) of input image data</param>
	/// <param name="lHeight">height (in pixels) of input image data</param>
	/// <param name="wBitsPerPixel">bits per pixel of image data</param>
	/// <param name="lpszFilePath">full file path to output bitmap to</param>
	/// <returns>indicates success or failure</returns>
	HRESULT                 SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
};

