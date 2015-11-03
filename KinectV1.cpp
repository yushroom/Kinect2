#include "KinectV1.h"
#include "C:\Program Files\Microsoft SDKs\Kinect\v1.8\inc\NuiApi.h"

const float KinectSensorV1::fovx = NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV;

const float KinectSensorV1::fovy = NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV;

KinectSensorV1::~KinectSensorV1()
{
	if (m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
	}

	if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hNextDepthFrameEvent);
	}

	// done with depth pixel data
	delete[] m_depthRGBX;

	if (m_hNextColorFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hNextColorFrameEvent);
	}
	delete[] m_pTempColorBuffer;
	m_pTempColorBuffer = NULL;

	SafeRelease(m_pNuiSensor);
}

void KinectSensorV1::Update()
{
	if (NULL == m_pNuiSensor)
	{
		return;
	}

	if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0))
	{
		ProcessDepth();
	}
	 
	if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0))
	{
		ProcessColor();
	}

	if (cRevieveRGB && WAIT_OBJECT_0 == WaitForSingleObject(m_hNextRGBFrameEvent, 0))
	{
		ProcessRGB();
	}
}

void KinectSensorV1::ProcessDepth()
{
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the depth frame
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		return;
	}

	BOOL nearMode;
	INuiFrameTexture* pTexture;

	// Get the depth image pixel texture
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
	depth_timestamp = imageFrame.liTimeStamp.QuadPart;
	if (FAILED(hr))
	{
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	pTexture->LockRect(0, &LockedRect, NULL, 0);



	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		// Get the min and max reliable depth for the current frame
		int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
		int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

		BYTE * rgbrun = m_depthRGBX;
		if (flip) {
			rgbrun = m_depthRGBX + (cDepthWidth * cDepthHeight * cBytesPerPixel) - 1;
		}
		const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

		// end pixel is start + width*height - 1
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		//rawDepthData.clear();
		//rawDepthData.reserve(cDepthWidth * cDepthHeight);
		int raw_depth_data_idx = 0;
		if (flip) {
			raw_depth_data_idx = cDepthWidth * cDepthHeight-1;
		}
		rawDepthData.resize(cDepthWidth * cDepthHeight);

		while (pBufferRun < pBufferEnd)
		{
			// discard the portion of the depth that contains only the player index
			USHORT depth = pBufferRun->depth;
			if (flip) {
				rawDepthData[raw_depth_data_idx--] = (depth >= minDepth && depth <= maxDepth) ? depth : 0;
			}
			else {
				rawDepthData[raw_depth_data_idx++] = (depth >= minDepth && depth <= maxDepth) ? depth : 0;
			}

			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			BYTE intensity = static_cast<BYTE>(depth >= minDepth && depth <= maxDepth ? depth % 256 : 0);

			if (flip) {
				--rgbrun;					// a
				*(rgbrun--) = intensity;	// r
				*(rgbrun--) = intensity;	// g
				*(rgbrun--) = intensity;	// b
			}
			else {
				// Write out blue byte
				*(rgbrun++) = intensity;
				// Write out green byte
				*(rgbrun++) = intensity;
				// Write out red byte
				*(rgbrun++) = intensity;
				// We're outputting BGR, the last byte in the 32 bits is unused so skip it
				// If we were outputting BGRA, we would write alpha here.
				++rgbrun;
			}

			// Increment our index into the Kinect's depth buffer
			++pBufferRun;
		}

		// Draw the data with Direct2D
		m_pDrawDepth->Draw(m_depthRGBX, cDepthWidth * cDepthHeight * cBytesPerPixel);
	}

	// We're done with the texture so unlock it
	pTexture->UnlockRect(0);

	pTexture->Release();

ReleaseFrame:
	// Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
}

void KinectSensorV1::ProcessColor()
{
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the color frame
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		return;
	}

	INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		if (m_pTempColorBuffer == NULL)
		{
			m_pTempColorBuffer = new RGBQUAD[cDepthWidth * cDepthHeight];
		}

		rawInfraredData.clear();


		for (int j = 0; j < cDepthHeight; ++j) {
			for (int i = 0; i < cDepthWidth; ++i) {
				int idx = j * cDepthWidth + i;
				USHORT depth = reinterpret_cast<USHORT*>(LockedRect.pBits)[idx];
				BYTE intensity = depth >> 8;

				rawInfraredData.push_back(intensity);

				if (flip) {
					idx = (cDepthHeight - 1 - j) * cDepthWidth + (cDepthWidth - 1 - i);
				}

				RGBQUAD *pQuad = &m_pTempColorBuffer[idx];
				pQuad->rgbBlue = intensity;
				pQuad->rgbGreen = intensity;
				pQuad->rgbRed = intensity;
				pQuad->rgbReserved = 255;
			}
		}
		//for (int i = 0; i < cDepthWidth * cDepthHeight; ++i)
		//{
		//	USHORT depth = reinterpret_cast<USHORT*>(LockedRect.pBits)[i];
		//	BYTE intensity = depth >> 8;

		//	rawInfraredData.push_back(intensity);

		//	int idx = i;
		//	if (flip) {
		//		idx =
		//	}

		//	RGBQUAD *pQuad = &m_pTempColorBuffer[i];
		//	pQuad->rgbBlue = intensity;
		//	pQuad->rgbGreen = intensity;
		//	pQuad->rgbRed = intensity;
		//	pQuad->rgbReserved = 255;
		//}

		// Draw the data with Direct2D
		m_pDrawColor->Draw(reinterpret_cast<BYTE*>(m_pTempColorBuffer), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
	}

	// We're done with the texture so unlock it
	pTexture->UnlockRect(0);

	// Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);
}

void KinectSensorV1::ProcessRGB()
{
	
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the color frame
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pRGBStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		return;
	}

	INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		//memcpy(m_colorRGBX, LockedRect.pBits, sizeof(BYTE)*cRGBWidth*cRGBHeight*cBytesPerPixel);
		for (int i = 0; i < cRGBHeight; ++i)
			for (int j = 0; j < cRGBWidth; ++j) {
				int idx_src = i*cRGBWidth + j;
				int idx_dst = (cRGBHeight - 1 - i)*cRGBWidth + cRGBWidth-1-j;
				for (int b = 0; b < cBytesPerPixel; b++)
					m_colorRGBX[idx_dst*cBytesPerPixel + b] = LockedRect.pBits[idx_src*cBytesPerPixel + b];
			}
		m_pDrawRGB->Draw(m_colorRGBX, cRGBWidth*cRGBHeight*sizeof(BYTE)*cBytesPerPixel);
	}

	// We're done with the texture so unlock it
	pTexture->UnlockRect(0);

	// Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pRGBStreamHandle, &imageFrame);
}

HRESULT KinectSensorV1::CreateFirstConnected()
{
	INuiSensor * pNuiSensor;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr))
	{
		return hr;
	}

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (NULL != m_pNuiSensor)
	{
		// Initialize the Kinect and specify that we'll be using depth
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when depth data is available
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a depth image stream to receive depth frames
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				m_hNextDepthFrameEvent,
				&m_pDepthStreamHandle);
		}

		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when color data is available
			m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a color image stream to receive color frames
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR_INFRARED,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				m_hNextColorFrameEvent,
				&m_pColorStreamHandle);
		}
		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when rgb data is available
			m_hNextRGBFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a color image stream to receive color frames
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR,
				NUI_IMAGE_RESOLUTION_1280x960,
				0,
				2,
				m_hNextRGBFrameEvent,
				&m_pRGBStreamHandle);
		}
	}

	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		//SetStatusMessage(L"No ready Kinect found!");
		return E_FAIL;
	}

	return hr;
}
