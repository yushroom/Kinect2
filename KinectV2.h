#pragma once
#include "stdafx.h"
#include "ImageRenderer.h"
#include "RenderFish.hpp"

// InfraredSourceValueMaximum is the highest value that can be returned in the InfraredFrame.
// It is cast to a float for readability in the visualization code.
#define InfraredSourceValueMaximum static_cast<float>(USHRT_MAX)

// The InfraredOutputValueMinimum value is used to set the lower limit, post processing, of the
// infrared data that we will render.
// Increasing or decreasing this value sets a brightness "wall" either closer or further away.
#define InfraredOutputValueMinimum 0.01f 

// The InfraredOutputValueMaximum value is the upper limit, post processing, of the
// infrared data that we will render.
#define InfraredOutputValueMaximum 1.0f

// The InfraredSceneValueAverage value specifies the average infrared value of the scene.
// This value was selected by analyzing the average pixel intensity for a given scene.
// Depending on the visualization requirements for a given application, this value can be
// hard coded, as was done here, or calculated by averaging the intensity for each pixel prior
// to rendering.
#define InfraredSceneValueAverage 0.08f

/// The InfraredSceneStandardDeviations value specifies the number of standard deviations
/// to apply to InfraredSceneValueAverage. This value was selected by analyzing data
/// from a given scene.
/// Depending on the visualization requirements for a given application, this value can be
/// hard coded, as was done here, or calculated at runtime.
#define InfraredSceneStandardDeviations 3.0f

struct IKinectSensor;
struct IDepthFrameReader;
struct IDepthFrame;
struct IFrameDescription;
struct IInfraredFrameReader;

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
public:
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const float		fovx;
	static const float		fovy;

	vector<UINT16>			rawDepthData;
	vector<BYTE>			rawInfraredData;
	RGBQUAD*                m_pInfraredRGBX = nullptr;
	RGBQUAD*                m_pDepthRGBX = nullptr;

private:
	// Current Kinect
	IKinectSensor*			m_pKinectSensor  = nullptr;
	// Depth reader
	IDepthFrameReader*		m_pDepthFrameReader = nullptr;
	// Infrared reader
	IInfraredFrameReader*	m_pInfraredFrameReader = nullptr;

	IDepthFrame*			pDepthFrame			= NULL;
	IFrameDescription*		pFrameDescription	= NULL;

	ImageRenderer*			m_pDrawDepth	= nullptr;
	ImageRenderer*          m_pDrawInfrared = nullptr;

public:
	KinectSensorV2();
	~KinectSensorV2();
	HRESULT InitializeDefaultSensor();
	void Update();

	void SetImageRender(ImageRenderer* pDrawDepth, ImageRenderer* pDrawInfrared) {
		m_pDrawDepth = pDrawDepth;
		m_pDrawInfrared = pDrawInfrared;
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
	//void ProcessDepth(DepthFrameDataV2& frameData) {
	//	ProcessDepth(frameData.nTime, frameData.pBuffer, frameData.nWidth, frameData.nHeight, frameData.nDepthMinReliableDistance, frameData.nDepthMaxDistance);
	//}

	/// <summary>
	/// Handle new infrared data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// </summary>
	void ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight);

	const RGBQUAD* GetDepthBuffer() const {
		return m_pDepthRGBX;
	}

	const RGBQUAD* GetInfraredBuffer() const {
		return m_pInfraredRGBX;
	}

};