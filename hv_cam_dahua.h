#pragma once

#ifndef __HV_CAM_DAHUA_H__
#define __HV_CAM_DAHUA_H__

//c++ stand lib
#include <iostream>
#include <string>
#include <queue>

//Dahua lib
#include <GenICam/System.h>
#include <GenICam/Camera.h>
#include <GenICam/ImageFormatControl.h>//修改视窗
#include <GenICam/ParameterNode.h>//修改曝光时间等
#include <GenICam/StreamSource.h>//开始/停止采集图像
#include <GenICam/ImageConvert.h>//图像格式转换
#include <GenICam/Frame.h>
#include <GenICam/AcquisitionControl.h>
#include <GenICam/StreamSource.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/dnn.hpp>

#include "thread_safe_queue.h"

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace cv;
using namespace std;

extern threadsafe_queue<CFrame> image_safe_queue;
extern bool is_callback_ok;
class HV_CAM_DAHUA
{
public:
	/**
	*曝光模式
	**/
	enum ExposureMode
	{
		EXPOSURE_AUTO_MODE_OFF,//自动曝光关闭
		EXPOSURE_AUTO_MODE_ONCE,//自动曝光 一次
		EXPOSURE_AUTO_MODE_CONTINUOUS// 自动曝光 连续
	};
	/**
	*白平衡模式
	**/
	enum BalanceWhiteMode
	{
		BALANCEWHITE_AUTO_OFF,//自动白平衡 关闭
		BALANCEWHITE_AUTO_ONCE,//自动白平衡 一次
		BALANCEWHITE_AUTO_CONTINUOUS//自动白平衡 连续
	};

	/**
	*触发模式
	**/
	enum TriggerMode
	{
		TRIGGER_MODE_OFF,//触发模式关闭
		TRIGGER_MODE_SOFTWARE,//软件触发模式
		TRIGGER_MODE_LINE1,//LINE1 触发模式
		TRIGGER_MODE_LINE2//LINE2 触发模式
	};
	/**
	*PixelFormat
	**/
	enum PixelFormat
	{
		BayerRG8,
		BayerGB8,
		BayerRG10Packed,
		BayerGB10Packed,
		BayerRG10,
		BayerGB10,
		Mono8,
		Mono10,
		Mono10Packed
	};
	// parameters


	//相机系统
	CSystem& systemObj = CSystem::getInstance();
	//设备信息列表，用来存储设备列表
	TVector<ICameraPtr> scanCameraPtrList;
	//相机指针
	ICameraPtr cameraSptr;
	//图像属性控制
	IImageFormatControlPtr sptrFormatControl;
	//流对象
	IStreamSourcePtr streamPtr;
	//OpenParam对象
	IMGCNV_SOpenParam openParam;
	IMGCNV_EErr status;

	IAcquisitionControlPtr sptrAcquisitionControl;

	IEventSubscribePtr eventPtr;//事件对象  eng://event object

	int nBGRBufferSize;

	PixelFormat format;

	const void* pImage;
	queue<uchar*> _imgBuffer;
	queue<int> _imgIDs;
	Mat image;


	HV_CAM_DAHUA();
	~HV_CAM_DAHUA();
	bool scanCameraDevice();
	bool linkCamera(string DeviceSerialNumber);
	bool openCamera();
	bool closeCamera();
	bool flag_read = false;
	bool isGrabbingFlag = false;
	//设置帧率
	bool setCameraAcquisitionFrameRate(float _AcquisitionFrameRate);
	//设置ROI
	bool setCameraROI(int _OffsetX, int _OffsetY, int _Width, int _Height);

	//设置曝光模式
	bool setCameraExposureMode(ExposureMode _ExposureAuto);

	//设置相机的曝光时间
	bool setCameraExposureTime(float _ExposureTime);

	//设置自动白平衡
	bool setCameraBalanceWihteAuto(BalanceWhiteMode _BalanceWhiteAuto);

	//设置x,y轴翻转
	bool setCameraReverseXY(bool _ReverseX, bool _ReverseY);

	//设置手动增益
	bool setCameraExposureGain(float _GainRaw);

	//设置亮度
	bool setCameraBrightness(int _Brightness);

	//设置图片类型
	bool setCameraImageType(PixelFormat _PixelFormat);

	//开始抓图
	bool cameraStartGrabbing();

	//停止抓图
	bool cameraStopGrabbing();
	//获取图片 加上延时
	bool getMatImage(Mat& _Img, uint32_t timeoutMS = INFINITE);

	bool getMonoImage(Mat& _Img, uint32_t timeoutMS = INFINITE);

	//设置相机的触发模式
	bool setCameraTriggerMode(HV_CAM_DAHUA::TriggerMode _TriggerMode);
	bool createStream();
	bool registerCallback();
	bool unregisterCallback();


	void onCallbackfun(const CFrame& pFrame);
	void onGetFrame(const CFrame& pFrame);
	int get_count = 0;
	vector<Mat> img_list;

};

#endif