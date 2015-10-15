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
	static const int        cBytesPerPixel = 4;
	static const float		fovx;
	static const float		fovy;

	vector<UINT16>			rawDepthData;
	vector<BYTE>			rawInfraredData;

	RGBQUAD*                m_pTempColorBuffer = nullptr;

private:
	INuiSensor*             m_pNuiSensor			= NULL;
	HANDLE                  m_pDepthStreamHandle	= INVALID_HANDLE_VALUE;
	HANDLE                  m_hNextDepthFrameEvent	= INVALID_HANDLE_VALUE;
	HANDLE                  m_pColorStreamHandle	= INVALID_HANDLE_VALUE;
	HANDLE                  m_hNextColorFrameEvent  = INVALID_HANDLE_VALUE;

	ImageRenderer*			m_pDrawDepth			= nullptr;
	ImageRenderer*			m_pDrawColor			= nullptr;

	BYTE*                   m_depthRGBX	= nullptr;
	

public:
	KinectSensorV1() {
		m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];
		m_pTempColorBuffer = new RGBQUAD[cDepthWidth * cDepthHeight];
	}
	~KinectSensorV1();

	void SetImageRender(ImageRenderer* pDrawDepth, ImageRenderer* pDrawInfrared) {
		m_pDrawDepth = pDrawDepth;
		m_pDrawColor = pDrawInfrared;
	}

	void Update();

	void ProcessDepth();
	void ProcessColor();

	HRESULT CreateFirstConnected();
};

