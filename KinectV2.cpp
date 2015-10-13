#include "KinectV2.h"
#include "C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc\Kinect.h"

KinectSensorV2::~KinectSensorV2()
{
	SafeRelease(m_pDepthFrameReader);
	if (m_pKinectSensor != nullptr)
		m_pKinectSensor->Close();
	SafeRelease(m_pKinectSensor);
}

HRESULT KinectSensorV2::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		SafeRelease(pDepthFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		//SetStatusMessage(L"No ready Kinect found!", 10000, true);
		return E_FAIL;
	}

	return hr;
}

HRESULT KinectSensorV2::GetFrame(DepthFrameDataV2* pFrameData)
{
	if (!m_pDepthFrameReader)
	{
		return E_FAIL;
	}

	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (!SUCCEEDED(hr))
	{
		SafeRelease(pDepthFrame);
		return E_FAIL;
	}

	//INT64 nTime = 0;
	IFrameDescription* pFrameDescription = NULL;
	//int nWidth = 0;
	//int nHeight = 0;
	//USHORT nDepthMinReliableDistance = 0;
	//USHORT nDepthMaxDistance = 0;
	UINT nBufferSize = 0;
	//UINT16 *pBuffer = NULL;

	hr = pDepthFrame->get_RelativeTime(&pFrameData->nTime);

	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
	}

	if (SUCCEEDED(hr))
	{
		hr = pFrameDescription->get_Width(&pFrameData->nWidth);
	}

	if (SUCCEEDED(hr))
	{
		hr = pFrameDescription->get_Height(&pFrameData->nHeight);
	}

	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->get_DepthMinReliableDistance(&pFrameData->nDepthMinReliableDistance);
	}

	if (SUCCEEDED(hr))
	{
		// In order to see the full range of depth (including the less reliable far field depth)
		// we are setting nDepthMaxDistance to the extreme potential depth threshold
		pFrameData->nDepthMaxDistance = USHRT_MAX;

		// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
		//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
	}

	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &(pFrameData->pBuffer));
	}

	SafeRelease(pFrameDescription);
	return hr;
}
