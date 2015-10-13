#pragma once
#include "stdafx.h"
#include "ImageRenderer.h"

struct IKinectSensor;
struct IDepthFrameReader;
struct IDepthFrame;
struct IFrameDescription;

struct DepthFrameDataV2
{
	INT64 nTime = 0;
	UINT16* pBuffer = nullptr;
	int nWidth = 0;
	int nHeight = 0;
	USHORT nDepthMinReliableDistance = 0;
	USHORT nDepthMaxDistance = 0;
};

class KinectSensorV2
{
private:
	// Current Kinect
	IKinectSensor*		m_pKinectSensor  = nullptr;
	// Depth reader
	IDepthFrameReader*  m_pDepthFrameReader = nullptr;

	IDepthFrame* pDepthFrame = NULL;
	IFrameDescription* pFrameDescription = NULL;

	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	RGBQUAD*                m_pDepthRGBX	= nullptr;
	ImageRenderer*			m_pImageRender = nullptr;

public:
	KinectSensorV2();
	~KinectSensorV2();
	HRESULT InitializeDefaultSensor();
	void Update();
	HRESULT GetFrame(DepthFrameDataV2* pFrameData);
	void ClearFrame();

	void SetImageRender(ImageRenderer* pRender) {
		m_pImageRender = pRender;
	}

	/// <summary>
	/// Handle new depth data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// <param name="nMinDepth">minimum reliable depth</param>
	/// <param name="nMaxDepth">maximum reliable depth</param>
	/// </summary>
	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);
	void ProcessDepth(DepthFrameDataV2& frameData) {
		ProcessDepth(frameData.nTime, frameData.pBuffer, frameData.nWidth, frameData.nHeight, frameData.nDepthMinReliableDistance, frameData.nDepthMaxDistance);
	}
};