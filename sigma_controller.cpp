#include "sigma_controller.h"
#define XY_MAXPOSITION			500000//XY��ÿ���ƶ�����������
#define XY_DIVIDE				20//XY��ϸ�֣�ϸ�ֺ�һ�������ƶ�һ��΢��


CSigmaController::CSigmaController(void)
{
	m_stateflag = false;
	Moveflag = FALSE;
	m_X1CurrentPos = 0;
	m_Y1CurrentPos = 0;
}

CSigmaController::~CSigmaController(void)
{

}

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
// ����ת


BOOL  CSigmaController::InitialSystem_STEP(float steppro, float steptime, float stepdelay)
{

	/////steppro   �������� 1%   steppro=1
	DWORD dwBytesWrite = 20;
	unsigned char Buf[10] = { (char)0xFF,(char)0xAA,(char)0x88,(char)0x55,(char)0x11,(char)0x03,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };

	int steppro_value = (int)(steppro / 0.1);
	Buf[6] = steppro_value / 256;
	Buf[7] = steppro_value % 256;

	/////steptime   ����ʱ��ms  0.1ms   steptime=0.1
	int steptime_value = (int)(steptime / 0.02);
	Buf[8] = steptime_value % 256;

	/////stepdelay   ������ʱus  0.1us   stepdelay=0.1
	int stepdelay_value = (int)(stepdelay / 0.01);
	Buf[9] = stepdelay_value % 256;


	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	//ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��

	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ��!" << endl;
		return FALSE;
	}
	//��ʱ����
	//PurgeComm(hCom, PURGE_TXABORT |	PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;

}


BOOL  CSigmaController::InitialSystem_EXP(float exptime)
{

	/////exptime   �ع�ʱ��  us ��λ  5000us
	DWORD dwBytesWrite = 20;
	unsigned char Buf[10] = { (char)0xFF,(char)0xAA,(char)0x88,(char)0x55,(char)0x11,(char)0x02,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };

	int exptime_value = (int)(exptime * 1000 / 10);//500000
	int highvalue = exptime_value / (65536);
	int lowvalue = exptime_value % (65536);
	Buf[6] = BYTE(highvalue >> 8);   // 1(10����) -> 0x01(16����)
	Buf[7] = highvalue & 0xFF;
	Buf[8] = BYTE(lowvalue >> 8); // 1(10����) -> 0x01(16����)
	Buf[9] = lowvalue & 0xFF;

	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	//ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��

	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ��!" << endl;
		return FALSE;
	}
	//��ʱ����
	//PurgeComm(hCom, PURGE_TXABORT |	PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;

}


BOOL  CSigmaController::InitialSystem_CYT(float cycletime)
{

	/////cycletime   ����ʱ��  us ��λ  10 000us
	DWORD dwBytesWrite = 20;
	unsigned char Buf[10] = { (char)0xFF,(char)0xAA,(char)0x88,(char)0x55,(char)0x11,(char)0x01,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };

	int cycletime_value = (int)(cycletime * 1000 / 10);//1000000
	int highvalue = cycletime_value / (65536);
	int lowvalue = cycletime_value % (65536);
	Buf[6] = BYTE(highvalue >> 8);   // 1(10����) -> 0x01(16����)
	Buf[7] = highvalue & 0xFF;
	Buf[8] = BYTE(lowvalue >> 8); // 1(10����) -> 0x01(16����)
	Buf[9] = lowvalue & 0xFF;

	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	//ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��

	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ��!" << endl;
		return FALSE;
	}
	//��ʱ����
	//PurgeComm(hCom, PURGE_TXABORT |	PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;

}


BOOL  CSigmaController::InitialSystem_PW(float Pwidth)
{

	/////Pwidth   ����ʱ��  us ��λ  100us
	DWORD dwBytesWrite = 20;
	unsigned char Buf[10] = { (char)0xFF,(char)0xAA,(char)0x88,(char)0x55,(char)0x11,(char)0x04,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };

	int Pwidth_value = (int)(Pwidth * 1000 / 10);//1000000

	Buf[8] = BYTE(Pwidth_value >> 8); // 1(10����) -> 0x01(16����)
	Buf[9] = Pwidth_value & 0xFF;

	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	//ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��

	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ��!" << endl;
		return FALSE;
	}
	//��ʱ����
	//PurgeComm(hCom, PURGE_TXABORT |	PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;

}

BOOL  CSigmaController::InitialSystem_CLR(bool cleanFIFO)
{
	DWORD dwBytesWrite = 20;
	unsigned char Buf[10] = { (char)0xFF,(char)0xAA,(char)0x88,(char)0x55,(char)0x11,(char)0x05,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };

	Buf[9] = cleanFIFO ? 0x01 : 0x00;

	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	//ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��

	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ��!" << endl;
		return FALSE;
	}
	//��ʱ����
	//PurgeComm(hCom, PURGE_TXABORT |	PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;

}

BOOL  CSigmaController::SendCommandPosi_time(float xp, float yp)
{
	DWORD dwBytesWrite = 20;
	unsigned char Buf[10] = { (char)0xFF,(char)0xAA,(char)0x88,(char)0x55,(char)0x11,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };
	int valueX = (int)(xp * (pow(2, 14) - 1) * 0.1 + 8192);
	Buf[6] = valueX / 256;
	Buf[7] = valueX % 256;
	int valueY = (int)(yp * (pow(2, 14) - 1) * 0.1 + 8192);
	Buf[8] = valueY / 256;
	Buf[9] = valueY % 256;


	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��
	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ��!" << endl;
		return FALSE;
	}
	//��ʱ����
	PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;
}


BOOL  CSigmaController::SendCommandPosi(float xp, float yp)
{
	DWORD dwBytesWrite = 20;
	unsigned char Buf[10] = { (char)0xFF,(char)0xAA,(char)0x88,(char)0x55,(char)0x11,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };
	int valueX = (int)(xp * (pow(2, 14) - 1) * 0.1 + 8192);
	Buf[6] = valueX / 256;
	Buf[7] = valueX % 256;
	int valueY = (int)(yp * (pow(2, 14) - 1) * 0.1 + 8192);
	Buf[8] = valueY / 256;
	Buf[9] = valueY % 256;

	//cout << "Send X value: " << valueX <<", High: "<<int(valueX / 256)<<", Low: "<<int(valueX % 256)<< endl;
	//cout << "Send Y value: " << valueY <<", High: "<<int(valueY / 256)<<", Low: "<<int(valueY % 256)<< endl;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	//ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��
	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ��!" << endl;
		return FALSE;
	}
	//��ʱ����
	//PurgeComm(hCom, PURGE_TXABORT |	PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;
}
// USB DA



bool CSigmaController::on_rotate1_usb_time(float value1, float value2)
{

	int	nRet;
	nRet = SendCommandPosi_time(value1, value2);
	if (nRet != TRUE) {
		cout << "DaOutputDA errr" << nRet << endl;
		//DaClose(hDeviceHandle);
		return FALSE;
	}
	m_X1CurrentPos = value1;
	m_Y1CurrentPos = value2;
	return TRUE;


}
bool CSigmaController::on_rotate1_usb(float value1, float value2)
{
	int	nRet;
	nRet = SendCommandPosi(value1, value2);
	if (nRet != TRUE) {
		cout << "DaOutputDA errr" << nRet << endl;
		//DaClose(hDeviceHandle);
		return FALSE;
	}
	m_X1CurrentPos = value1;
	m_Y1CurrentPos = value2;
	return TRUE;
}


// ����ԭ��
void CSigmaController::backToCenter(int axis)
{
	if (axis == 1)
	{
		on_rotate1_usb(0, m_Y1CurrentPos);
	}
	if (axis == 2)
	{
		on_rotate1_usb(m_X1CurrentPos, 0);
	}

}


void CSigmaController::StringtoHex(BYTE* GB, int glen, BYTE* SB, int* slen)

{

	int i;    //����������ַ���
	int a = 0;

	char temp;   //�����ַ��������ж��Ƿ�Ϊ�ո�����������

	char temp1, temp2;   //����һ���ֽڵ������ַ�  ����EB����temp1='E',temp2 = 'B'

	*slen = 0;  //�����16�����ַ�������

	for (i = 0; i < glen; i++)

	{

		temp = GB[i];

		if (temp == ' ')

			continue;



		if (a == 0)

			temp1 = GB[i];

		if (a == 1)

			temp2 = GB[i];

		a++;



		if (a == 2)

		{

			a = 0;

			temp1 = temp1 - '0';

			if (temp1 > 10)

				temp1 = temp1 - 7;

			temp2 = temp2 - '0';

			if (temp2 > 10)

				temp2 = temp2 - 7;



			SB[*slen] = temp1 * 16 + temp2;

			(*slen)++;

		}

	}



}


//�򿪴��ڣ�ͬ��ͨ��
BOOL CSigmaController::OpenController(int CommPortNum)
{
	if (!m_stateflag)
	{
		string portnum;
		portnum = "COM" + to_string(CommPortNum);

		hCom = CreateFileA(portnum.c_str(), GENERIC_READ | GENERIC_WRITE,
			0, NULL, OPEN_EXISTING, 0, NULL);

		if (hCom == (HANDLE)-1)
		{
			cout << "��sigma COMʧ�ܣ�" << endl;
			return FALSE;
		}
		SetupComm(hCom, 100, 100);//�����������������Ϊ100

		COMMTIMEOUTS TimeOuts;
		//�趨����ʱ,�ڶ�һ�����뻺���������ݺ���������������أ������Ƿ������Ҫ����ַ���
		TimeOuts.ReadIntervalTimeout = MAXDWORD;
		TimeOuts.ReadTotalTimeoutMultiplier = 0;
		TimeOuts.ReadTotalTimeoutConstant = 0;

		//�趨д��ʱ
		TimeOuts.WriteTotalTimeoutMultiplier = 100;
		TimeOuts.ReadTotalTimeoutConstant = 500;

		SetCommTimeouts(hCom, &TimeOuts);//���ó�ʱ

		DCB dcb;
		GetCommState(hCom, &dcb);
		dcb.BaudRate = 921600;//������Ϊ921600
		dcb.ByteSize = 8;//ÿ���ֽ���8λ
		dcb.Parity = NOPARITY;//����żУ��λ
		dcb.StopBits = ONESTOPBIT;//һ��ֹͣλ
		SetCommState(hCom, &dcb);

		PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);
		m_stateflag = true;
	}
	return TRUE;
}


//�رմ���
BOOL CSigmaController::CloseController(void)
{
	if (m_stateflag)
	{
		m_stateflag = false;
		return (::CloseHandle(hCom));
	}
	else
		return TRUE;

}

BOOL CSigmaController::InitialSystem(int port)
{

	bool success = MoveToMechanicalOri(port);//����
	if (success)
	{
		m_X1CurrentPos = -5;
		m_Y1CurrentPos = -5;
	}
	return TRUE;
}

string DecIntToHexStr(long long num)
{
	string str;
	long long Temp = num / 16;
	int left = num % 16;
	if (Temp > 0)
		str += DecIntToHexStr(Temp);
	if (left < 10)
		str += (left + '0');
	else
		str += ('A' + left - 10);
	return str;
}

string DecStrToHexStr(string str)
{
	long long Dec = 0;
	for (int i = 0; i < str.size(); ++i)
		Dec = Dec * 10 + str[i] - '0';
	return DecIntToHexStr(Dec);
}






BOOL  CSigmaController::SendCommand(float xp, float yp)
{
	string str = "AA5500000000";
	int length = str.length();//�ַ�������
							  //char * Buf=(char *)str.GetBuffer(length);//ָ��
							  /////////170 85 
	unsigned char Buf[6] = { (char)0xAA,(char)85,(char)0x3F,(char)0x00,(char)0x3F,(char)0x00 };
	int valueX = (int)(xp * (pow(2, 14) - 1) * 0.1 + 8192);
	string cst = DecIntToHexStr(valueX);
	Buf[2] = valueX / 256;
	Buf[3] = valueX % 256;

	int valueY = (int)(yp * (pow(2, 14) - 1) * 0.1 + 8192);
	//string cst = DecIntToHexStr(valueX);
	Buf[4] = valueY / 256;
	Buf[5] = valueY % 256;

	DWORD dwBytesWrite = length;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��
	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ�ܣ�" << endl;
		return FALSE;
	}
	PurgeComm(hCom, PURGE_TXABORT |
		PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;
}
//�򴮿ڷ���ָ��
BOOL CSigmaController::SendCommand(string str)
{
	int length = str.length();//�ַ�������
							  //char * Buf=(char *)str.GetBuffer(length);//ָ��

	unsigned char Buf[6] = { (char)0xAA,(char)0x55,(char)0x00,(char)0x00,(char)0x00,(char)0x00 };

	DWORD dwBytesWrite = length;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(hCom, &dwErrorFlags, &ComStat);//ɾ�������Ա��ڶ�дָ��
	bWriteStat = WriteFile(hCom, Buf, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		cout << "д����ʧ�ܣ�" << endl;
		return FALSE;
	}
	PurgeComm(hCom, PURGE_TXABORT |
		PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	return TRUE;
}

BOOL CSigmaController::RecieveStatus(string& str, int* length)
{
	char lpInBuffer[512];
	DWORD dwBytesRead = 512;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	OVERLAPPED m_osRead;
	memset(&m_osRead, 0, sizeof(OVERLAPPED));
	m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	ClearCommError(hCom, &dwErrorFlags, &ComStat);
	dwBytesRead = min(dwBytesRead, (DWORD)ComStat.cbInQue);
	if (!dwBytesRead)
		return FALSE;
	BOOL bReadStatus;
	bReadStatus = ReadFile(hCom, lpInBuffer,
		dwBytesRead, &dwBytesRead, &m_osRead);

	if (!bReadStatus) //���ReadFile��������FALSE
	{
		if (GetLastError() == ERROR_IO_PENDING)
			//GetLastError()��������ERROR_IO_PENDING,�����������ڽ��ж����� 
		{
			WaitForSingleObject(m_osRead.hEvent, 2000);
			//ʹ��WaitForSingleObject�����ȴ���ֱ����������ɻ���ʱ�Ѵﵽ2����
			//�����ڶ�����������Ϻ�m_osRead��hEvent�¼����Ϊ���ź�
			PurgeComm(hCom, PURGE_TXABORT |
				PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
			return dwBytesRead;
		}
		return 0;
	}
	PurgeComm(hCom, PURGE_TXABORT |
		PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	str = lpInBuffer;
	*length = dwBytesRead;
	return TRUE;
}


std::string CSigmaController::StringToHex(LPCSTR lpSrc, char chTag)
{
	//���ַ���ת��Ϊ16���Ƶ��ַ���; chTagΪ�ֽ�������Ϊ0x20��ʾ�ո�
	std::string strDest;
	unsigned char* pSrc = (unsigned char*)lpSrc;
	char  buf[2];
	long dwSize = strlen(lpSrc);
	for (long dwIndex = 0; dwIndex < dwSize; ++dwIndex)
	{
		unsigned char c0 = *pSrc >> 4;
		if (c0 >= 0x0 && c0 <= 0x9)
		{
			buf[0] = c0 - 0 + '0';
		}
		else
		{
			buf[0] = c0 - 10 + 'A';
		}
		unsigned char c1 = *pSrc++ & 0x0F;
		if (c1 >= 0x0 && c1 <= 0x9)
		{
			buf[1] = c1 - 0 + '0';
		}
		else
		{
			buf[1] = c1 - 10 + 'A';
		}
		strDest += buf[0];
		strDest += buf[1];
		if (0 != chTag)
			strDest += chTag;
	}
	return strDest;
}

unsigned int persist_ssl_hashKeyConvert(char* pUserInput, unsigned char* pKeyArray)
{
	if (NULL == pUserInput || NULL == pKeyArray)
	{
		return 0;
	}

	unsigned int uiKeySize = strlen(pUserInput) / 2;
	int i = 0;
	char cTempor = 0;

	while (i < uiKeySize)
	{
		if (*pUserInput >= '0' && *pUserInput <= '9')
		{
			cTempor = *pUserInput - 48;
		}
		else
		{
			cTempor = 0xa + (*pUserInput - 'a');
		}

		pKeyArray[i] = cTempor;
		pUserInput++;

		if (*pUserInput >= '0' && *pUserInput <= '9')
		{
			cTempor = *pUserInput - 48;
		}
		else
		{
			cTempor = 0xa + (*pUserInput - 'a');
		}

		pKeyArray[i] = (pKeyArray[i] << 4) | cTempor;
		pUserInput++;
		i++;
	}

	return uiKeySize;
}


BOOL CSigmaController::MoveToMechanicalOri(int axis)
{

	if (SendCommand(-5, -5) == TRUE)
	{
		return TRUE;
	}
	else
		return FALSE;
}


BOOL CSigmaController::STEPMove(int axis, float xp, float yp)
{

	if (SendCommand(xp, yp) == TRUE)
	{
		m_X1CurrentPos = xp;
		m_Y1CurrentPos = yp;
		return TRUE;
	}
	else
		return FALSE;

}

BOOL CSigmaController::MoveToLogicOri(int axis)
{
	if (axis == 1)
	{
		if (SendCommand(-5, m_Y1CurrentPos) == TRUE)
		{
			m_X1CurrentPos = -5;
			return TRUE;
		}
		else
			return FALSE;

	}
	else
	{
		if (SendCommand(m_X1CurrentPos, -5) == TRUE)
		{
			m_Y1CurrentPos = -5;
			return TRUE;
		}
		else
			return FALSE;
	}

}