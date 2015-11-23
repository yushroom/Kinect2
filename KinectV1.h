#pragma once
#include "stdafx.h"
#include "ImageRenderer.h"
#include "RenderFish.hpp"

struct INuiSensor;

class KinectSensorV1
{
public:
	static const int        cDepthWidth = 640;
	static const int        cDepthHeight = 480;

	
	static const bool		cRevieveRGB = false;
	static const int		cRGBWidth = 1280;
	static const int		cRGBHeight = 960;


	static const int        cBytesPerPixel = 4;
	static const float		fovx;
	static const float		fovy;

	vector<UINT16>			rawDepthData;
	vector<BYTE>			rawInfraredData;

	RGBQUAD*                m_pTempColorBuffer = nullptr;
	BYTE*                   m_depthRGBX = nullptr;
	BYTE*					m_colorRGBX = nullptr;

	bool flip = true;

private:
	INuiSensor*             m_pNuiSensor			= NULL;
	HANDLE                  m_pDepthStreamHandle	= INVALID_HANDLE_VALUE;
	HANDLE                  m_hNextDepthFrameEvent	= INVALID_HANDLE_VALUE;
	HANDLE                  m_pColorStreamHandle	= INVALID_HANDLE_VALUE;
	HANDLE                  m_hNextColorFrameEvent  = INVALID_HANDLE_VALUE;
	HANDLE                  m_pRGBStreamHandle		= INVALID_HANDLE_VALUE;
	HANDLE                  m_hNextRGBFrameEvent	= INVALID_HANDLE_VALUE;

	ImageRenderer*			m_pDrawDepth			= nullptr;
	ImageRenderer*			m_pDrawColor			= nullptr;
	ImageRenderer*			m_pDrawRGB				= nullptr;

public:
	KinectSensorV1() {
		m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];
		m_colorRGBX = new BYTE[cRGBWidth*cRGBHeight*cBytesPerPixel];
		m_pTempColorBuffer = new RGBQUAD[cDepthWidth * cDepthHeight];
		rawDepthData.resize(cDepthWidth * cDepthHeight);
	}
	~KinectSensorV1();

	void SetImageRender(ImageRenderer* pDrawDepth, ImageRenderer* pDrawInfrared, ImageRenderer* pDrawRGB) {
		m_pDrawDepth = pDrawDepth;
		m_pDrawColor = pDrawInfrared;
		m_pDrawRGB = pDrawRGB;
	}

	void Update();

	void ProcessDepth();
	void ProcessColor();
	void ProcessRGB();

	HRESULT CreateFirstConnected();

	long long			depth_timestamp;
};

