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
#include <GenICam/ImageFormatControl.h>//�޸��Ӵ�
#include <GenICam/ParameterNode.h>//�޸��ع�ʱ���
#include <GenICam/StreamSource.h>//��ʼ/ֹͣ�ɼ�ͼ��
#include <GenICam/ImageConvert.h>//ͼ���ʽת��
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
	*�ع�ģʽ
	**/
	enum ExposureMode
	{
		EXPOSURE_AUTO_MODE_OFF,//�Զ��ع�ر�
		EXPOSURE_AUTO_MODE_ONCE,//�Զ��ع� һ��
		EXPOSURE_AUTO_MODE_CONTINUOUS// �Զ��ع� ����
	};
	/**
	*��ƽ��ģʽ
	**/
	enum BalanceWhiteMode
	{
		BALANCEWHITE_AUTO_OFF,//�Զ���ƽ�� �ر�
		BALANCEWHITE_AUTO_ONCE,//�Զ���ƽ�� һ��
		BALANCEWHITE_AUTO_CONTINUOUS//�Զ���ƽ�� ����
	};

	/**
	*����ģʽ
	**/
	enum TriggerMode
	{
		TRIGGER_MODE_OFF,//����ģʽ�ر�
		TRIGGER_MODE_SOFTWARE,//�������ģʽ
		TRIGGER_MODE_LINE1,//LINE1 ����ģʽ
		TRIGGER_MODE_LINE2//LINE2 ����ģʽ
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


	//���ϵͳ
	CSystem& systemObj = CSystem::getInstance();
	//�豸��Ϣ�б������洢�豸�б�
	TVector<ICameraPtr> scanCameraPtrList;
	//���ָ��
	ICameraPtr cameraSptr;
	//ͼ�����Կ���
	IImageFormatControlPtr sptrFormatControl;
	//������
	IStreamSourcePtr streamPtr;
	//OpenParam����
	IMGCNV_SOpenParam openParam;
	IMGCNV_EErr status;

	IAcquisitionControlPtr sptrAcquisitionControl;

	IEventSubscribePtr eventPtr;//�¼�����  eng://event object

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
	//����֡��
	bool setCameraAcquisitionFrameRate(float _AcquisitionFrameRate);
	//����ROI
	bool setCameraROI(int _OffsetX, int _OffsetY, int _Width, int _Height);

	//�����ع�ģʽ
	bool setCameraExposureMode(ExposureMode _ExposureAuto);

	//����������ع�ʱ��
	bool setCameraExposureTime(float _ExposureTime);

	//�����Զ���ƽ��
	bool setCameraBalanceWihteAuto(BalanceWhiteMode _BalanceWhiteAuto);

	//����x,y�ᷭת
	bool setCameraReverseXY(bool _ReverseX, bool _ReverseY);

	//�����ֶ�����
	bool setCameraExposureGain(float _GainRaw);

	//��������
	bool setCameraBrightness(int _Brightness);

	//����ͼƬ����
	bool setCameraImageType(PixelFormat _PixelFormat);

	//��ʼץͼ
	bool cameraStartGrabbing();

	//ֹͣץͼ
	bool cameraStopGrabbing();
	//��ȡͼƬ ������ʱ
	bool getMatImage(Mat& _Img, uint32_t timeoutMS = INFINITE);

	bool getMonoImage(Mat& _Img, uint32_t timeoutMS = INFINITE);

	//��������Ĵ���ģʽ
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