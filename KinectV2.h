#include "stdafx.h"

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

public:
	~KinectSensorV2();
	HRESULT InitializeDefaultSensor();
	HRESULT GetFrame(DepthFrameDataV2* pFrameData);
	void ClearFrame();
};