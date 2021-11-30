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


	//////��ǰADת��///
	float m_X1CurrentPos;  // ˮƽ��·X
	float m_Y1CurrentPos;  // ˮƽ��·Y
	HANDLE hDeviceHandle;
	ULONG ulSmplBufferSize = 1;	// ��������С
								//DASMPLREQ SmplConfig;       // ����������ýṹ
	WORD wSmplData[1][2];		// ������ݴ洢
								// ���豸
								//bool openDevice();
								//bool closeDevice();
								// ����ת
								//bool on_rotate(float value1, float value2);
								//bool on_rotate1(float value1, float value2);
								// ����ԭ��
	void backToCenter(int axis);
	//////////////////////////////////////////////
	bool on_rotate1_usb(float value1, float value2);
	bool on_rotate1_usb_time(float value1, float value2);

	HANDLE hCom;//���ھ��
	BOOL Moveflag;//�������״̬
	int itimer;

	//�򿪹رմ���
	BOOL OpenController(int CommPortNum);
	BOOL CloseController(void);

	//��ʼ�����ϵͳ������˶�����ʼλ��
	BOOL InitialSystem(int port);
	BOOL MoveToLogicOri(int axis);
	BOOL STEPMove(int axis, float xp, float yp);

	//����ָ��
	BOOL SendCommand(string str);
	BOOL SendCommand(float xp, float yp);
	// steppro   %    steppro=1 :: 1%
	// steptime  ms
	// stepdelay us
	BOOL  InitialSystem_STEP(float steppro, float steptime, float stepdelay);//������ʼ��
	BOOL  InitialSystem_EXP(float exptime);//�ع��ʼ��  us
	BOOL  InitialSystem_CYT(float cycletime);//���ڳ�ʼ��  us
	BOOL  InitialSystem_PW(float Pwidth);//�����ʼ��    us
	BOOL  InitialSystem_CLR(bool cleanFIFO);//�����ʼ��    us
	BOOL  SendCommandPosi(float xp, float yp);
	BOOL  SendCommandPosi_time(float xp, float yp);


	BOOL RecieveStatus(string& str, int* length);

	//�����ٶ�
	//BOOL SetVelocity(int axis,int velocity);
	//BOOL SetVelocity(int slow,int fast,int rate);
	//BOOL SetContinuousVelocity(int axis,int velocity);
	//BOOL SetContinuousVelocityAll(int velocity1,int velocity2);

	//BOOL SetLogicOri(int axis);
	//BOOL SetLogicOriAll();


	//BOOL MoveToLogicOriALL();

	//�˶�������ԭ��
	BOOL MoveToMechanicalOri(int axis);

	//BOOL ContinuousMove(int axis,char orientation);

	//BOOL RelativeMove(int axis ,char orientation ,int step);
	//BOOL RelativeMoveAll(char orientation1 ,int step1,char orientation2 ,int step2);

	//�����˶�
	//BOOL AbsoluteMove(int axis,char orientation ,int pos);


	//BOOL GetPose(int axis ,int& pose);
	//BOOL GetAllPose( int& pose1,int& pose2);

	//��õ����ǰ״̬
	//BOOL GetMoveStatus(::CString& str);

	//��ͣ
	//BOOL EStop(int axis);
	//BOOL EStop(void);
	//BOOL EStopAll();

	//BOOL WaitForStop(void);

	bool m_stateflag;//�������򿪱�־


};

#endif // !__SIGMA_CONTROLLER_H__