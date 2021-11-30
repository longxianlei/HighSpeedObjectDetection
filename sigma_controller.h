#pragma once
#ifndef __SIGMA_CONTROLLER_H__
#define __SIGMA_CONTROLLER_H__



#include <string.h>
#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>
#include <malloc.h>
#include <mmsystem.h>
#include <string>

//#define COM_PORT			3
//#define DA_STEP_PCT			1		// %
//#define DA_STEP_TIME		0.5		// ms
//#define DA_STEP_DLY			0.1		// us
//#define DA_EXP				25000	// us    should be same with HC_EXP
//#define DA_CYCLE			30000	// us
//#define DA_PULSE_W			100		// us

using namespace std;

class CSigmaController
{
public:
	CSigmaController(void);
	~CSigmaController(void);
	void StringtoHex(BYTE* GB, int glen, BYTE* SB, int* slen);
	std::string StringToHex(LPCSTR lpSrc, char chTag);


	//////当前AD转换///
	float m_X1CurrentPos;  // 水平光路X
	float m_Y1CurrentPos;  // 水平光路Y
	HANDLE hDeviceHandle;
	ULONG ulSmplBufferSize = 1;	// 输出缓存大小
								//DASMPLREQ SmplConfig;       // 输出条件设置结构
	WORD wSmplData[1][2];		// 输出数据存储
								// 打开设备
								//bool openDevice();
								//bool closeDevice();
								// 振镜旋转
								//bool on_rotate(float value1, float value2);
								//bool on_rotate1(float value1, float value2);
								// 返回原点
	void backToCenter(int axis);
	//////////////////////////////////////////////
	bool on_rotate1_usb(float value1, float value2);
	bool on_rotate1_usb_time(float value1, float value2);

	HANDLE hCom;//串口句柄
	BOOL Moveflag;//电机运行状态
	int itimer;

	//打开关闭串口
	BOOL OpenController(int CommPortNum);
	BOOL CloseController(void);

	//初始化电机系统，电机运动到初始位置
	BOOL InitialSystem(int port);
	BOOL MoveToLogicOri(int axis);
	BOOL STEPMove(int axis, float xp, float yp);

	//发送指令
	BOOL SendCommand(string str);
	BOOL SendCommand(float xp, float yp);
	// steppro   %    steppro=1 :: 1%
	// steptime  ms
	// stepdelay us
	BOOL  InitialSystem_STEP(float steppro, float steptime, float stepdelay);//步长初始化
	BOOL  InitialSystem_EXP(float exptime);//曝光初始化  us
	BOOL  InitialSystem_CYT(float cycletime);//周期初始化  us
	BOOL  InitialSystem_PW(float Pwidth);//脉宽初始化    us
	BOOL  InitialSystem_CLR(bool cleanFIFO);//脉宽初始化    us
	BOOL  SendCommandPosi(float xp, float yp);
	BOOL  SendCommandPosi_time(float xp, float yp);


	BOOL RecieveStatus(string& str, int* length);

	//设置速度
	//BOOL SetVelocity(int axis,int velocity);
	//BOOL SetVelocity(int slow,int fast,int rate);
	//BOOL SetContinuousVelocity(int axis,int velocity);
	//BOOL SetContinuousVelocityAll(int velocity1,int velocity2);

	//BOOL SetLogicOri(int axis);
	//BOOL SetLogicOriAll();


	//BOOL MoveToLogicOriALL();

	//运动到物理原点
	BOOL MoveToMechanicalOri(int axis);

	//BOOL ContinuousMove(int axis,char orientation);

	//BOOL RelativeMove(int axis ,char orientation ,int step);
	//BOOL RelativeMoveAll(char orientation1 ,int step1,char orientation2 ,int step2);

	//绝对运动
	//BOOL AbsoluteMove(int axis,char orientation ,int pos);


	//BOOL GetPose(int axis ,int& pose);
	//BOOL GetAllPose( int& pose1,int& pose2);

	//获得电机当前状态
	//BOOL GetMoveStatus(::CString& str);

	//急停
	//BOOL EStop(int axis);
	//BOOL EStop(void);
	//BOOL EStopAll();

	//BOOL WaitForStop(void);

	bool m_stateflag;//控制器打开标志


};

#endif // !__SIGMA_CONTROLLER_H__