#include "HV_CAM_DAHUA.h"
#include <queue>

threadsafe_queue<CFrame> image_safe_queue;
bool is_callback_ok;
////构造函数
//HV_CAM_DAHUA::HV_CAM_DAHUA()
//{
//
//}

//析构函数
HV_CAM_DAHUA::~HV_CAM_DAHUA()
{

}

HV_CAM_DAHUA::HV_CAM_DAHUA()
{
	openParam.width = 720;
	openParam.height = 540;
	openParam.paddingX = 0;
	openParam.paddingY = 0;
	openParam.dataSize = 388800;
	openParam.pixelForamt = gvspPixelBayRG8;
	nBGRBufferSize = 720 * 540 * 3;
}
//HV_CAM_DAHUA::~HV_CAM_DAHUA()
//{
//}

void HV_CAM_DAHUA::onGetFrame(const CFrame& pFrame)
{
	// 判断帧的有效性
	// judge the validity of frame
	bool isValid = pFrame.valid();
	if (!isValid)
	{
		printf("frame is invalid!\n");
		return;
	}
	else
	{
		pImage = pFrame.getImage();
		uint8_t* pBGRbuffer = new uint8_t[nBGRBufferSize];
		IMGCNV_ConvertToBGR24((unsigned char*)pImage, &openParam, pBGRbuffer, &nBGRBufferSize);
		_imgIDs.push(pFrame.getBlockId());
		_imgBuffer.push(pBGRbuffer);
		delete pBGRbuffer;
	}

	return;
}

void HV_CAM_DAHUA::onCallbackfun(const CFrame& pFrame)
{
	//cout << "okkkkkkkkkkk!!!!" << endl;
	// 标准输出换行
	// standard output line feed
	//printf("\r\n");
	//chrono::steady_clock::time_point begin_time = chrono::steady_clock::now();
	// 判断帧的有效性
	// judge the validity of frame
	bool isValid = pFrame.valid();
	if (!isValid)
	{
		printf("frame is invalid!\n");
		return;
	}
	else
	{
		uint64_t blockId = pFrame.getBlockId();
		printf("blockId = %d.\n", blockId);
		isGrabbingFlag = true;
	}

	get_count++;
	cout << "!!!!!!!!!Invoke Call back count: " << get_count << endl;
	//is_callback_ok = true;
	CFrame temp = pFrame;
	image_safe_queue.push(temp.clone());
	

	//openParam.width = pFrame.getImageWidth();
	//openParam.height = pFrame.getImageHeight();
	//openParam.paddingX = pFrame.getImagePadddingX();
	//openParam.paddingY = pFrame.getImagePadddingY();
	//openParam.dataSize = pFrame.getImageSize();
	//openParam.pixelForamt = pFrame.getImagePixelFormat();
	////cout << openParam.width << "," <<openParam.height<< endl;

	//pImage = pFrame.getImage();
	//nBGRBufferSize = pFrame.getImageWidth() * pFrame.getImageHeight() * 3;
	//uint8_t* pBGRbuffer = new uint8_t[nBGRBufferSize];
	//status = IMGCNV_ConvertToBGR24((unsigned char*)pImage, &openParam, pBGRbuffer, &nBGRBufferSize);
	//image = Mat(pFrame.getImageHeight(), pFrame.getImageWidth(), CV_8UC3, (uint8_t*)pBGRbuffer);

	////namedWindow("1", WINDOW_AUTOSIZE);
	////imshow("1", image);
	////waitKey(100);

	//img_list.push_back(image.clone());
	//delete pBGRbuffer;
	//chrono::steady_clock::time_point end_time = chrono::steady_clock::now();
	//cout << "get image to queue cost time: " << chrono::duration_cast<chrono::microseconds>(end_time - begin_time).count() << endl;

	return;
}


bool HV_CAM_DAHUA::scanCameraDevice()
{
	bool isDiscoverySuccess = systemObj.discovery(scanCameraPtrList);
	if (!isDiscoverySuccess)// 读取失败
	{
		cout << "Discovery device fail." << endl;
		return isDiscoverySuccess;
	}

	if (0 == scanCameraPtrList.size())
	{
		cout << "No camera device find." << endl;
		return false;
	}

	cout << scanCameraPtrList.size() << " camera detected!" << endl;
	return isDiscoverySuccess;
}

bool HV_CAM_DAHUA::linkCamera(string DeviceSerialNumber)
{
	//根据序列号选取相机
	for (int i = 0; i < scanCameraPtrList.size(); i++)
	{
		cameraSptr = scanCameraPtrList[i];
		cout << "CameraID: " << (string)cameraSptr->getSerialNumber() << endl;
		if ((string)cameraSptr->getSerialNumber() == DeviceSerialNumber)
		{
			cout << "Camera " << DeviceSerialNumber << " selected!" << endl;
			return true;
		}
		else
		{
			continue;
		}
	}
	cout << "Camera selection failed!" << endl;
	return false;
}

bool HV_CAM_DAHUA::openCamera()
{
	//连接相机
	if (!cameraSptr->connect())
	{
		cout << "Connect camera fail." << endl;
		return false;
	}
	else
	{
		cout << "Connect camera succeed!" << endl;
		return true;
	}
}

bool HV_CAM_DAHUA::closeCamera()
{
	// 断开相机设备
	// disconnect camera device
	if (!cameraSptr->disConnect())
	{
		cout << "DisConnect camera fail." << endl;
		return false;
	}
	else
	{
		cout << "DisConnect camera succeed." << endl;
	}
	return true;
}

bool HV_CAM_DAHUA::setCameraAcquisitionFrameRate(float _AcquisitionFrameRate)
{
	CDoubleNode nodeDouble(cameraSptr, "AcquisitionFrameRate");
	int nRet = nodeDouble.setValue(_AcquisitionFrameRate);
	if (!nRet)
	{
		cout << "Set AcquisitionFrameRate failed." << endl;
		return false;
	}
	else
	{
		cout << "Set AcquisitionFrameRate succeed." << endl;
		return true;
	}
}

bool HV_CAM_DAHUA::setCameraROI(int _OffsetX, int _OffsetY, int _Width, int _Height)
{
	CIntNode nodeWidth(cameraSptr, "Width");
	CIntNode nodeHeight(cameraSptr, "Height");
	CIntNode nodeOffsetX(cameraSptr, "OffsetX");
	CIntNode nodeOffsetY(cameraSptr, "OffsetY");
	int nRet = nodeWidth.setValue(_Width);
	if (!nRet)
	{
		cout << "Set Width failed." << endl;
		return false;
	}
	else
	{
		cout << "Set Width succeed." << endl;
	}

	nRet = nodeHeight.setValue(_Height);
	if (!nRet)
	{
		cout << "Set Height failed." << endl;
		return false;
	}
	else
	{
		cout << "Set Height succeed." << endl;
	}

	nRet = nodeOffsetX.setValue(_OffsetX);
	if (!nRet)
	{
		cout << "Set OffsetX failed." << endl;
		return false;
	}
	else
	{
		cout << "Set OffsetX succeed." << endl;
	}

	nRet = nodeOffsetY.setValue(_OffsetY);
	if (!nRet)
	{
		cout << "Set OffsetY failed." << endl;
		return false;
	}
	else
	{
		cout << "Set OffsetY succeed." << endl;
	}
	return true;
}

bool HV_CAM_DAHUA::setCameraExposureMode(ExposureMode _ExposureAuto)
{
	CEnumNode nodeEnum(cameraSptr, "ExposureAuto");
	if (_ExposureAuto == EXPOSURE_AUTO_MODE_OFF)
	{
		int nRet = nodeEnum.setValueBySymbol("Off");
		if (!nRet)
		{
			cout << "Set exposure mode failed." << endl;
			return false;
		}
		else
		{
			cout << "Set exposure mode succeed." << endl;
			return true;
		}
	}
	else if (_ExposureAuto == EXPOSURE_AUTO_MODE_ONCE)
	{
		int nRet = nodeEnum.setValueBySymbol("Once");
		if (!nRet)
		{
			cout << "Set exposure mode failed." << endl;
			return false;
		}
		else
		{
			cout << "Set exposure mode succeed." << endl;
			return true;
		}
	}
	else if (_ExposureAuto == EXPOSURE_AUTO_MODE_CONTINUOUS)
	{
		int nRet = nodeEnum.setValueBySymbol("Continuous");
		if (!nRet)
		{
			cout << "Set exposure mode failed." << endl;
			return false;
		}
		else
		{
			cout << "Set exposure mode succeed." << endl;
			return true;
		}
	}
	cout << "Set exposure mode failed." << endl;
	return false;
}

bool HV_CAM_DAHUA::setCameraExposureTime(float _ExposureTime)
{
	CDoubleNode nodeExposureTime(cameraSptr, "ExposureTime");
	int nRet = nodeExposureTime.setValue(_ExposureTime);
	if (!nRet)
	{
		cout << "Set exposuretime failed." << endl;
		return false;
	}
	else
	{
		cout << "Set exposuretime succeed." << endl;
		return true;
	}
}

bool HV_CAM_DAHUA::setCameraBalanceWihteAuto(BalanceWhiteMode _BalanceWhiteAuto)
{
	CEnumNode nodeEnum(cameraSptr, "BalanceWhiteAuto");
	if (_BalanceWhiteAuto == BALANCEWHITE_AUTO_OFF)
	{
		int nRet = nodeEnum.setValueBySymbol("Off");
		if (!nRet)
		{
			cout << "Set BalanceWhiteMode failed." << endl;
			return false;
		}
		else
		{
			cout << "Set BalanceWhiteMode succeed." << endl;
			return true;
		}
	}
	else if (_BalanceWhiteAuto == BALANCEWHITE_AUTO_ONCE)
	{
		int nRet = nodeEnum.setValueBySymbol("Once");
		if (!nRet)
		{
			cout << "Set BalanceWhiteMode failed." << endl;
			return false;
		}
		else
		{
			cout << "Set BalanceWhiteMode succeed." << endl;
			return true;
		}
	}
	else if (_BalanceWhiteAuto == BALANCEWHITE_AUTO_CONTINUOUS)
	{
		int nRet = nodeEnum.setValueBySymbol("Continuous");
		if (!nRet)
		{
			cout << "Set BalanceWhiteMode failed." << endl;
			return false;
		}
		else
		{
			cout << "Set BalanceWhiteMode succeed." << endl;
			return true;
		}
	}
	cout << "Set BalanceWhiteMode failed." << endl;
	return false;
}

bool HV_CAM_DAHUA::setCameraReverseXY(bool _ReverseX, bool _ReverseY)
{
	CBoolNode nodeBoolX(cameraSptr, "ReverseX");
	int nRet = nodeBoolX.setValue(_ReverseX);
	if (!nRet)
	{
		cout << "Set ReverseX failed." << endl;
		return false;
	}
	else
	{
		cout << "Set ReverseX succeed." << endl;
	}
	CBoolNode nodeBoolY(cameraSptr, "ReverseY");
	nRet = nodeBoolY.setValue(_ReverseY);
	if (!nRet)
	{
		cout << "Set ReverseY failed." << endl;
		return false;
	}
	else
	{
		cout << "Set ReverseY succeed." << endl;
		return true;
	}
}

bool HV_CAM_DAHUA::setCameraExposureGain(float _GainRaw)
{
	CDoubleNode nodeDouble(cameraSptr, "GainRaw");
	int nRet = nodeDouble.setValue(_GainRaw);
	if (!nRet)
	{
		cout << "Set GainRaw failed." << endl;
		return false;
	}
	else
	{
		cout << "Set GainRaw succeed." << endl;
		return true;
	}
}

bool HV_CAM_DAHUA::setCameraBrightness(int _Brightness)
{
	CIntNode nodeBrightness(cameraSptr, "Brightness");
	int nRet = nodeBrightness.setValue(_Brightness);
	if (!nRet)
	{
		cout << "Set Brightness failed." << endl;
		return false;
	}
	else
	{
		cout << "Set Brightness succeed." << endl;
		return true;
	}
}


bool HV_CAM_DAHUA::setCameraImageType(PixelFormat _PixelFormat)
{
	sptrFormatControl = systemObj.createImageFormatControl(cameraSptr);
	if (_PixelFormat == BayerRG8)
	{
		CEnumNode nodeCameraImageType(cameraSptr, "PixelFormat");
		int nRet = nodeCameraImageType.setValueBySymbol("BayerRG8");
		if (!nRet)
		{
			cout << "Set CameraImageType failed." << endl;
			return false;
		}
		else
		{
			cout << "Set CameraImageType succeed." << endl;
			format = _PixelFormat;
			return true;
		}
	}
	else if (_PixelFormat == BayerGB8)
	{
		CEnumNode nodeCameraImageType(cameraSptr, "PixelFormat");
		int nRet = nodeCameraImageType.setValueBySymbol("BayerGB8");
		if (!nRet)
		{
			cout << "Set CameraImageType failed." << endl;
			return false;
		}
		else
		{
			cout << "Set CameraImageType succeed." << endl;
			format = _PixelFormat;
			return true;
		}
	}
	else if (_PixelFormat == BayerRG10Packed)
	{
		CEnumNode nodeCameraImageType(cameraSptr, "PixelFormat");
		int nRet = nodeCameraImageType.setValueBySymbol("BayerRG10Packed");
		if (!nRet)
		{
			cout << "Set CameraImageType failed." << endl;
			return false;
		}
		else
		{
			cout << "Set CameraImageType succeed." << endl;
			format = _PixelFormat;
			return true;
		}
	}
	else if (_PixelFormat == BayerGB10Packed)
	{
		CEnumNode nodeCameraImageType(cameraSptr, "PixelFormat");
		int nRet = nodeCameraImageType.setValueBySymbol("BayerGB10Packed");
		if (!nRet)
		{
			cout << "Set CameraImageType failed." << endl;
			return false;
		}
		else
		{
			cout << "Set CameraImageType succeed." << endl;
			format = _PixelFormat;
			return true;
		}
	}
	else if (_PixelFormat == BayerGB10)
	{
		CEnumNode nodeCameraImageType(cameraSptr, "PixelFormat");
		int nRet = nodeCameraImageType.setValueBySymbol("BayerGB10");
		if (!nRet)
		{
			cout << "Set CameraImageType failed." << endl;
			return false;
		}
		else
		{
			cout << "Set CameraImageType succeed." << endl;
			format = _PixelFormat;
			return true;
		}
	}
	else if (_PixelFormat == BayerRG10)
	{
		CEnumNode nodeCameraImageType(cameraSptr, "PixelFormat");
		int nRet = nodeCameraImageType.setValueBySymbol("BayerRG10");
		if (!nRet)
		{
			cout << "Set CameraImageType failed." << endl;
			return false;
		}
		else
		{
			cout << "Set CameraImageType succeed." << endl;
			format = _PixelFormat;
			return true;
		}
	}
}

bool HV_CAM_DAHUA::cameraStartGrabbing()
{
	bool isStartGrabbingSuccess = streamPtr->startGrabbing();
	if (!isStartGrabbingSuccess)
	{
		cout << "StartGrabbing fail." << endl;
		return false;
	}
	else
	{
		cout << "StartGrabbing..." << endl;
		return true;
	}
}

bool HV_CAM_DAHUA::cameraStopGrabbing()
{
	bool isStopGrabbingSuccess = streamPtr->stopGrabbing();
	if (!isStopGrabbingSuccess)
	{
		cout << "StopGrabbing fail." << endl;
		return false;
	}
	else
	{
		cout << "StopGrabbing succeed." << endl;
		return true;
	}
}

bool HV_CAM_DAHUA::getMatImage(Mat& _Img, uint32_t timeoutMS)
{
	CFrame frame;
	if (!streamPtr)
	{
		cout << "streamPtr is NULL." << endl;
		return false;
	}
	bool isSuccess = streamPtr->getFrame(frame, timeoutMS);
	if (!isSuccess)
	{
		cout << "GetFrame fail." << endl;
		return false;
	}
	//判断帧的有效性
	//Judge the validity of frame
	bool isValid = frame.valid();
	if (!isValid)
	{
		cout << "GetFrame is not valid." << endl;
		return false;
	}
	openParam.width = frame.getImageWidth();
	openParam.height = frame.getImageHeight();
	openParam.paddingX = frame.getImagePadddingX();
	openParam.paddingY = frame.getImagePadddingY();
	openParam.dataSize = frame.getImageSize();
	openParam.pixelForamt = frame.getImagePixelFormat();

	pImage = frame.getImage();
	int nBGRBufferSize = frame.getImageWidth() * frame.getImageHeight() * 3;
	uint8_t* pBGRbuffer = new uint8_t[nBGRBufferSize];
	status = IMGCNV_ConvertToBGR24((unsigned char*)pImage, &openParam, pBGRbuffer, &nBGRBufferSize);
	memcpy(_Img.data, pBGRbuffer, sizeof(uchar) * nBGRBufferSize);
	delete pBGRbuffer;
	return true;
}

bool HV_CAM_DAHUA::getMonoImage(Mat& _Img, uint32_t timeoutMS)
{
	CFrame frame;
	if (!streamPtr)
	{
		return false;
	}
	bool isSuccess = streamPtr->getFrame(frame, timeoutMS);
	if (!isSuccess)
	{
		return false;
	}

	//判断帧的有效性
	//Judge the validity of frame
	bool isValid = frame.valid();
	if (!isValid)
	{
		return false;
	}
	_Img = Mat(frame.getImageHeight(), frame.getImageWidth(), CV_8U, (uint8_t*)frame.getImage());
	/*_Img = image.clone();*/
	return true;
}

bool HV_CAM_DAHUA::setCameraTriggerMode(HV_CAM_DAHUA::TriggerMode _TriggerMode)
{
	sptrAcquisitionControl = systemObj.createAcquisitionControl(cameraSptr);

	if (NULL == sptrAcquisitionControl.get())
	{
		printf("create AcquisitionControl object fail.\n");
		// 实际应用中应及时释放相关资源，如diconnect相机等，不宜直接return
		// in practical application, relevant resources should be released in time, such as diconnect camera, etc. it is not suitable to return directly
		return false;
	}
	if (_TriggerMode == TRIGGER_MODE_OFF)
	{
		CEnumNode nodeTriggerMode(cameraSptr, "TriggerMode");
		int nRet = nodeTriggerMode.setValueBySymbol("Off");
		if (!nRet)
		{
			cout << "Set TriggerMode failed." << endl;
			return false;
		}
		else
		{
			cout << "Set TriggerMode OFF." << endl;
			return true;
		}
	}
	else
	{
		if (_TriggerMode == TRIGGER_MODE_SOFTWARE)
		{
			CEnumNode nodeTriggerMode = sptrAcquisitionControl->triggerMode();
			int nRet = nodeTriggerMode.setValueBySymbol("On");
			if (!nRet)
			{
				cout << "Set TriggerMode failed." << endl;
				return false;
			}
			else
			{
				cout << "Set TriggerMode ON." << endl;
			}
			CEnumNode nodeTriggersource = sptrAcquisitionControl->triggerSource();
			nRet = nodeTriggersource.setValueBySymbol("Software");
			if (!nRet)
			{
				cout << "Set TriggerSource failed." << endl;
				return false;
			}
			else
			{
				cout << "Set TriggerSource Software." << endl;
				return true;
			}

		}
		else if (_TriggerMode == TRIGGER_MODE_LINE1)
		{

			CEnumNode nodeTriggersource = sptrAcquisitionControl->triggerSource();
			bool nRet = nodeTriggersource.setValueBySymbol("Line1");
			if (!nRet)
			{
				cout << "Set TriggerSource failed." << endl;
				return false;
			}
			else
			{
				cout << "Set TriggerSource Line1." << endl;
				//return true;
			}
			CEnumNode nodeTriggerSeletor = sptrAcquisitionControl->triggerSelector();
			nRet = nodeTriggerSeletor.setValueBySymbol("FrameStart");
			if (nRet != true)
			{
				cout << "set trigger Seletor fail." << endl;
				return false;
			}
			else
			{
				cout << "set trigger Seletor succeed." << endl;
			}
			CEnumNode nodeTriggerMode = sptrAcquisitionControl->triggerMode();
			nRet = nodeTriggerMode.setValueBySymbol("On");
			if (!nRet)
			{
				cout << "Set TriggerMode failed." << endl;
				return false;
			}
			else
			{
				cout << "Set TriggerMode succeed." << endl;
			}
			return true;
		}
		else if (_TriggerMode == TRIGGER_MODE_LINE2)
		{
			CEnumNode nodeTriggersource = sptrAcquisitionControl->triggerSource();
			bool nRet = nodeTriggersource.setValueBySymbol("Line2");
			if (!nRet)
			{
				cout << "Set TriggerSource failed." << endl;
				return false;
			}
			else
			{
				cout << "Set TriggerSource Line1." << endl;
				//return true;
			}
			CEnumNode nodeTriggerSeletor = sptrAcquisitionControl->triggerSelector();
			nRet = nodeTriggerSeletor.setValueBySymbol("FrameStart");
			if (nRet != true)
			{
				cout << "set trigger Seletor fail." << endl;
				return false;
			}
			else
			{
				cout << "set trigger Seletor succeed." << endl;
			}
			CEnumNode nodeTriggerMode = sptrAcquisitionControl->triggerMode();
			nRet = nodeTriggerMode.setValueBySymbol("On");
			if (!nRet)
			{
				cout << "Set TriggerMode failed." << endl;
				return false;
			}
			else
			{
				cout << "Set TriggerMode succeed." << endl;
			}
			return true;
		}
		//设置触发器 default FrameStart
		CEnumNode nodeTriggerSeletor = sptrAcquisitionControl->triggerSelector();
		bool nRet = nodeTriggerSeletor.setValueBySymbol("FrameStart");
		if (nRet != true)
		{
			cout << "set trigger Seletor fail." << endl;
			return false;
		}
		else
		{
			cout << "set trigger Seletor succeed." << endl;
		}
	}
	cout << "Set TriggerMode failed." << endl;
	return false;
}

bool HV_CAM_DAHUA::createStream()
{
	streamPtr = systemObj.createStreamSource(cameraSptr);
	if (NULL == streamPtr.get())
	{
		printf("create stream obj  fail.\n");
		// 实际应用中应及时释放相关资源，如diconnect相机等，不宜直接return
		// in practical application, relevant resources should be released in time, such as diconnect camera, etc. it is not suitable to return directly
		return false;
	}
	else
	{
		printf("create stream obj  succeed.\n");
		// 实际应用中应及时释放相关资源，如diconnect相机等，不宜直接return
		// in practical application, relevant resources should be released in time, such as diconnect camera, etc. it is not suitable to return directly
		return true;
	}
}

bool HV_CAM_DAHUA::registerCallback()
{
	bool bRet = streamPtr->attachGrabbing(IStreamSource::Proc(&HV_CAM_DAHUA::onCallbackfun, this));
	if (!bRet)
	{
		printf("attach Grabbing fail.\n");
		// 实际应用中应及时释放相关资源，如diconnect相机等，不宜直接return
		// in practical application, relevant resources should be released in time, such as diconnect camera, etc. it is not suitable to return directly
		return false;
	}
	else
	{
		printf("attach Grabbing succeed.\n");
		// 实际应用中应及时释放相关资源，如diconnect相机等，不宜直接return
		// in practical application, relevant resources should be released in time, such as diconnect camera, etc. it is not suitable to return directly
		return true;
	}
}


bool HV_CAM_DAHUA::unregisterCallback()
{
	bool bRet = streamPtr->detachGrabbing(IStreamSource::Proc(&HV_CAM_DAHUA::onCallbackfun, this));
	if (!bRet)
	{
		printf("detach Grabbing fail.\n");
		// 实际应用中应及时释放相关资源，如diconnect相机等，不宜直接return
		// in practical application, relevant resources should be released in time, such as diconnect camera, etc. it is not suitable to return directly
		return false;
	}
	else
	{
		printf("detach Grabbing succeed.\n");
		// 实际应用中应及时释放相关资源，如diconnect相机等，不宜直接return
		// in practical application, relevant resources should be released in time, such as diconnect camera, etc. it is not suitable to return directly
		return true;
	}
}