/*
* �ļ�����[TJST3DScannerApi.h]
* ���ߣ��������ھۿƼ�-JXB��
* ������������3άɨ���豸���������,ֻ֧��64λ���򿪷���
* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 04 - 03
* �޸����ݣ����½���

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 05 - 31
* �޸����ݣ�
	1.����ͶӰ���ƽӿڣ�����LED�Ŀ��أ����������ȣ����ڣ�����ͶӰ����ɫ����
	2.֧�ֵ��Ʊ���.ply�ȸ�ʽ֧��
	3.֧�ְ��������������ʼ���豸���Զ����������豸
* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 07 - 09
* �޸����ݣ�
	1.�޸�������Ŀ�������ɷ�ʽ��1���������� 2������ϵת�� 3����Χ���˲�
	2.����һ����Ŀ�������ɷ�ʽ��1���������� 2����Χ���˲� 3���ӵ�ȥ��
* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 07 - 17
* �޸����ݣ�
	1.���ӵ���3��궨���ݵĺ����ӿ�
	2.����3���Ƕ�ȡ�ñ任����ĺ����ӿ�
	3.���ӵ�������ϵת�������ӿ�  TJST3DChangeCoordinate
* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 07 - 30
* �޸����ݣ�
	1.������������ӿڣ�TJST3DSetRasterType(). �߷ֱ��������Ҫ�л����ƣ�ɨ��Ч������á�ͶӰ����Ҫ����������.
	2.����ȡ���������ƽӿ� TJST3DGetMinTriggerImageNum
* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 10 - 09
* �޸����ݣ�
	1.���Ӵӱ�־��ȡ��ת������Ľӿ�
	2.���������������ƽӿ�
	3.���Ӽ��������ӿ�

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 10 - 12
* �޸����ݣ�
	1.���������豸ip��ַ���ýӿ� TJST3DSetNetdevIP
	2.���ӱ궨���ʶ������ӿ�
* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 10 - 15
* �޸����ݣ�
	1.���ӵ��Ʒָ�ӿ� TJST3DPointcloudDivision
	2.����TJST3DRangeNew �� TJST3DRangeDelete �ӿ������ C#���

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 11 - 06
* �޸����ݣ�
	1.�޸�ģ���ʼ���ӿ� TJST3DModuleInit
	2.���ӻ�е�۱궨���� TJST3DCalibrationRobot

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2019 - 11 - 12
* �޸����ݣ�
	1.���ӻ�е�۱궨���� TJST3DCalibrationRobot2

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2020 - 01 - 17
* �޸����ݣ�
	1.���ӵ���ʹ��Mask�ӿ� TJST3DUseMaskImage
	2.���ӵ�����ɫ�ӿ�	   TJST3DAddColorImage
	3.����ȡ����������һ��ɨ��ͼ(һ��ΪͶӰ��ɫ)��������MaskImage �� ColorImage ���ӿں���Ϊ TJST3DGetLeftCameraLastTriggerImagebuff
	4.����ȡ����������һ��ɨ��ͼ��TJST3DGetRightCameraLastTriggerImagebuff

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2020 - 04 - 23
* �޸����ݣ�
	1. ���ӵ��Ƽ��㴦����ѡ��ӿ� TJST3DSetProcessorUnit

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2020 - 07 - 28
* �޸����ݣ�
	1.�����µ�ģ���ʼ���ӿ�--�������Ӷ����豸ʱ��������һ���豸�ӿ� TJST3DModuleInitOneDev

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2020 - 08 - 05
* �޸����ݣ�
	1.���ӵ�Ŀģʽ���ܺ�����
		1)TJST3DSetModuleType
		2)TJST3DLoadSingleStdData
		3)TJST3DSetWorkMode
	2.����ɨ��ģʽʹ��G���������  TJST3DSetTriggerGray

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2020 - 10 - 12
* �޸����ݣ�
	1.����ȡ�ô�����ӿ� TJST3DGetLastErrorCode

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2020 - 10 - 29
* �޸����ݣ�
	1.����������ݱ���ӿ� TJST3DGetDepthdata

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2021 - 04 - 21
	* �޸����ݣ�
	1.����˫��Ŀ�궨�ϲ��ӿ�

* �޸��ˣ���JXB��
* �޸�ʱ�䣺2021 - 09 - 09
	* �޸����ݣ�
	1.����ɢ��ģʽ�������ɽӿ� TJST3DCreatePointcloudRasterSpeckle
	2.����ɢ��ģʽ�豸���ƽӿ� TJST3DContinueScan

*/
#pragma once
#include "RangeData.h"
#ifdef _LINUX_OS_
#define TJ3DSDK_API __attribute__((visibility("default")))
#else //_LINUX_OS_
#ifdef TJST3DSCANNERAPI_EXPORTS
#define TJ3DSDK_API extern "C" __declspec(dllexport)
#else
#define TJ3DSDK_API extern "C" __declspec(dllimport)
#endif
#endif//_LINUX_OS_

#ifndef IN
#define IN
#endif
#ifndef OUT
#define OUT
#endif

typedef enum
{
	TJST3DERR_SUCESS = 0,
	TJST3DERR_CONNECT_PRJ,		//1.ͶӰ������ʧ��
	TJST3DERR_CONNECT_CAM,		//2.�������ʧ��
	TJST3DERR_TRIGGER,			//3.ͶӰ���޷�����
	TJST3DERR_IMAGE_NOTENOUGH,	//4.����ͼƬ���ݲ���
	TJST3DERR_OUTBOX,			//5.������Χ��
	TJST3DERR_REMOVE_ERRPAR,	//6.�쳣��ȥ�ӵ����
	TJST3DERR_FUNC_PAR,			//7.����ĺ�������
	TJST3DERR_DEV_OPEN,			//8.�豸û�д�
	TJST3DERR_FRINGEIMAGE,		//9.����ͼ��������
	TJST3DERR_NUM
}TJST3DError_t;

//�ص�����ԭ��
typedef void(__stdcall * TJST3DRasterSpeckleCallback)(sn3DCore::sn3DRangeData* pPointcloud, unsigned char* pColorImagedata,bool bResult, TJST3DError_t errCode);


/*ȡ��������ýӿ��쳣����ʱ�Ĵ���(����ԭ�����)
����:
	�������
*/
TJ3DSDK_API TJST3DError_t TJST3DGetLastErrorCode();

/*���õ������ɴ�����,��TJST3DModuleInit ���� TJST3DCalibCameraGroup ǰ����
����˵����
	nMode��0 ʹ��CPU��1 ʹ��GPU�������Զ�ѡ��
*/
TJ3DSDK_API void TJST3DSetProcessorUnit(int nMode);

/*����ģ������
����˵����
	nModuleType��0 ˫Ŀģʽ��1 ��Ŀ˫�����2 ��Ŀ���������3 ��Ŀ���������4 �����ģʽ
	(����������ְ��������ͶӰ�������۷���ͬͶӰ����)
*/

TJ3DSDK_API void TJST3DSetModuleType(int nModuleType=0);

/*��Ŀģʽ���ù���״̬
����˵����
	nWorkMode��0 ɨ��ģʽ��1�궨ģʽ
*/

TJ3DSDK_API bool TJST3DSetWorkMode(int nWorkMode = 0);

/*��Ŀģʽ�����׼�����ļ�
����˵����
	strFileName:�ļ�ȫ·����
	nVideoWidth:�������ֱ���
	nVideoHeight:�������ֱ���
	nFringeNum:��������ʱ��Ҫ����������
����:
	�ɹ�true
	ʧ��false
*/
TJ3DSDK_API bool TJST3DLoadSingleStdDataA(const char* strPathName, int nVideoWidth = 1280, int nVideoHeight = 1024, int nFringeNum = 24);
TJ3DSDK_API bool TJST3DLoadSingleStdDataW(const wchar_t* strPathName, int nVideoWidth = 1280, int nVideoHeight = 1024, int nFringeNum = 24);

/*ģ���ʼ��
����˵����
	nMode: 0 USB�豸��1 �����豸
	nDevNum: �豸����(��),����0.(1��Ϊ1̨ͶӰ����2̨���)
����:
	�ɹ�true
	ʧ��false
*/
TJ3DSDK_API bool TJST3DModuleInit(IN int nMode, IN int nDevNum);

/*ģ���ʼ��--�������Ӷ����豸ʱ��������һ���豸
����˵����
	nMode: 0 USB�豸��1 �����豸
	nDevIndex: �豸��ţ���0��ʼ
����:
	�ɹ�true
	ʧ��false
*/
TJ3DSDK_API bool TJST3DModuleInitOneDev(IN int nMode, IN int nDevIndex);

/*�ͷ���Դ,�ر��豸*/
TJ3DSDK_API void TJST3DModuleUninit();

/*�ж�ģ���ʼ���Ƿ�ɹ�
����:
	�ɹ�true
	ʧ��false
*/
TJ3DSDK_API bool TJST3DModuleIsInitedOK();


TJ3DSDK_API bool TJST3DSetVideoImageColor(bool bRGBColor);

/*��ȡͼ��ֱ��ʼ�����λ������û�г�ʼ�����ʼ��ʧ�ܵ�����·���true����ֵΪģ��Ĭ�ϵ���ֵ
����˵����
	pWidth ͼ����
	pHeight ͼ��߶�
	pBitCount ����λ�����Ҷ�ͼ�� = 8��BRGͼ��=24
����:
	�ɹ�true
	ʧ��false
*/
TJ3DSDK_API bool TJST3DGetImageFormat(OUT int* pWidth, OUT int* pHeight, OUT int* pBitCount);

/*��ȡ��Ƶ�ֱ��ʼ�����λ������TJST3DGetImageFormat ��ͬ���ǣ�û�г�ʼ�����ʼ��ʧ�ܵ�����·��� false
����˵����
	pWidth ͼ����
	pHeight ͼ��߶�
	pBitCount ����λ�����Ҷ�ͼ�� = 8��BRGͼ��=24
����:
	�ɹ�true
	ʧ��false
*/
TJ3DSDK_API bool TJST3DVideoFormat(OUT int* pWidth, OUT int* pHeight, OUT int* pBitCount);

/*�������Ԥ������
�����������C++��ΪHWND ���ͣ�
	hwndLeft ���������ָ��
	hwndRight���������ָ�롣
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetDisplayHwnd(IN void* hwndLeft, IN void* hwndRight);

TJ3DSDK_API bool TJST3DSetDisplayHwndThird(IN void* hwndThird);

/*�������Ԥ�����ڴ�С(�������֧�֣�*/
TJ3DSDK_API bool TJST3DResizeDisplayHwnd();

/*��ʼ��Ƶ,����ɼ�ģʽ
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DVideoStart();

/*��ͣ��Ƶ*/
TJ3DSDK_API void TJST3DVideoPause();

/*�ж��Ƿ��ڲɼ�ģʽ
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DVideoIsGrabbing();

/*�л��������,�л��������Ҫ���±궨
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DExchangeLeftRightCam();

/*���õ�ǰ���������
�������:
	fGainLeft ���������ֵ0-15
	fGainRight ���������ֵ0-15
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetGainFloat(IN float fGainLeft, IN float fGainRight);

/*����ָ�����������
�������:
	nGroup �����ID�����ֻ��һ���������Ϊ0
	fGainLeft ���������ֵ0-15
	fGainRight ���������ֵ0-15
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetGainFloatEx(IN int nGroup,IN float fGainLeft, IN float fGainRight);

TJ3DSDK_API bool TJST3DSetAutoGain(IN int nGroup,IN bool bAuto);

//�����ع�ʱ�䣺���д򿪵��������ͬһ������ (ֻ�����������ʹ�ã�һ����Ч)
TJ3DSDK_API bool TJST3DSetExposureTime(IN float fExposureTime); 
TJ3DSDK_API bool TJST3DSetTriggerDelaye(IN float fTrDelay);

/*ȡ��֧�ֵ�ɨ��FPS
���������
	pFps ����֧�ֵ�ֵ
�������:
	nMaxNum pFpsָ��(����)������һ����Ϊ5�͹���
���أ�
	�ɹ� ֧�ֵ�����;
	ʧ�� 0;
*/
TJ3DSDK_API int TJST3DGetSupportScanFps(OUT int* pFps, IN int nMaxNum);

/*����ɨ��֡��:�̶��ع�ʱ��
�������:
	nFps ����֧�ֵ�FPSֵ��130W���:120,60,30 ; 600W����� 30,15
	nTriggerNum ����������
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetScanFps(IN int nFps,IN int nTriggerNum);

TJ3DSDK_API bool TJST3DSetScanerPar(IN float fExposureTime, IN float fTriggerDelay, IN int nRepeat, IN int nTriggerNum, IN int nStart, IN int nStartImg);


/*����ɨ��ģʽ(��Ĭ��)��ɨ����Զ������ģʽ
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DEnterScanMode();
/*������Ƶģʽ
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DEnterContinuousMode();

/*��ȡ������С������
���أ�
	�ɹ� ����������,������;
	ʧ�� 0;
*/
TJ3DSDK_API int TJST3DGetMinTriggerImageNum();

/*���ô���������
�������:
	nTriggers ����������nTriggers>=12�����д򿪵��������ͬһ������
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetTriggerImageNum(int nTriggers);

/*����ʹ��G������Ҷȴ���
���������
	byteGray�� 255 ʹ��T ������� ʹ��G xxx���xxx�������һ�Ű����Ҷ�ֵ
*/
TJ3DSDK_API void TJST3DSetTriggerGray(unsigned char byteGray);

/*�������еĴ���ͼ�����ݵ�ָ��Ŀ¼
�������:
	strPath ָ����Ŀ¼·��
	nCameraGroup �����ID�����ֻ��һ���������Ϊ0
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSaveTriggerImagesA(IN const char* strPath, IN int nCameraGroup=0);
TJ3DSDK_API bool TJST3DSaveTriggerImagesW(IN const wchar_t* strPath,IN int nCameraGroup=0);
TJ3DSDK_API bool TJST3DSaveTriggerImagesExA(IN const char* strPath, IN int nCameraGroup);

/*����������и�ץȡ1��ͼ:TJST3DCatchImage
ע�⣺
	ֻ������Ƶģʽ��ץͼ
���������
	pLeftImgbuf �����ͼ���ڴ�ָ�룬�û�������ͷ��ڴ�
	pRightImgbuf �����ͼ���ڴ�ָ�룬�û�������ͷ��ڴ�
���������
	nMaxBufsize �û�������ڴ���ֽڴ�С.����ǲ�ɫ����������ڴ��С=��Ƶ��x��Ƶ��x3
���أ�
	0 ʧ�ܡ�
	����ͼ�������ֽ���
*/
TJ3DSDK_API unsigned int TJST3DCatchImage(OUT unsigned char* pLeftImgbuf, OUT unsigned char* pRightImgbuf, IN unsigned int nMaxBufsize);
TJ3DSDK_API unsigned int TJST3DCatchImageEx(OUT unsigned char* pLeftImgbuf, OUT unsigned char* pRightImgbuf, IN unsigned int nMaxBufsize,IN bool bRGB);
TJ3DSDK_API unsigned int TJST3DCatchImage3Ex(OUT unsigned char* pLeftImgbuf, OUT unsigned char* pRightImgbuf, OUT unsigned char* pThirdImgbuf, IN unsigned int nMaxBufsize, IN bool bRGB);

TJ3DSDK_API bool TJST3DCatchLeftCameraImage(OUT unsigned char* pLeftImgbuf, IN unsigned int nMaxBufsize);
TJ3DSDK_API bool TJST3DCatchRightCameraImage(OUT unsigned char* pRightImgbuf, IN unsigned int nMaxBufsize);
/*����ͼ�����ݵ�bmpͼƬ
���������
	strFileFullname �ļ�ȫ·��(����չ��.bmp)
	pImageBuf ͼ��ָ��
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSaveBmpImageA(IN const char* strFileFullname, IN unsigned char* pImageBuf, IN int bitCount = 8);
TJ3DSDK_API bool TJST3DSaveBmpImageW(IN const wchar_t* strFileFullname, IN unsigned char* pImageBuf, IN int bitCount = 8);
TJ3DSDK_API bool TJST3DSaveBmpImageRotate90A(IN const char* strFileFullname, IN unsigned char* pImageBuf, IN int bitCount = 8);

/*ȡ�ñ궨��Բ���ά����:TJST3DGetCalibImage2DPoints
���������
	pImageCalibLeft ��ͼ���ڴ�ָ��
	pImageCalibRight ��ͼ���ڴ�ָ��
���������
	fXLeft ��ͼ��Բ��X�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
	fYLeft ��ͼ��Բ��Y�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
	fXRight ��ͼ��Բ��X�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
	fYRight ��ͼ��Բ��Y�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DGetCalibImage2DPoints(unsigned char* pImageCalibLeft, unsigned char* pImageCalibRight,
	double* fXLeft, double* fYLeft, double* fXRight, double* fYRight);

/*ȡ�ñ궨��Բ���ά����(��Ŀģʽ):TJST3DGetCalibImage2DPointsSingle
���������
	mode 0 �������1 �����
	pImageCalib ͼ���ڴ�ָ��
	pImagePhaseArr ����ɼ��ĺ�������ͼ	��СΪ	24*nImageW*nImageH
���������
	fXLeft ��ͼ��Բ��X�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
	fYLeft ��ͼ��Բ��Y�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
	fXRight ��ͼ��Բ��X�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
	fYRight ��ͼ��Բ��Y�������飬�Ҳ����ĵ���-1���棬�û�������ͷ��ڴ� 17*14
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DGetCalibImage2DPointsSingle(int mode, unsigned char* pImageCalib, unsigned char* pImagePhaseArr,
	double* fXLeft, double* fYLeft, double* fXRight, double* fYRight);

/*�궨��������:TJST3DSetCalibParameter
���������
	fFocusLeft �������ͷ���࣬��λmm
	fPixelSizeLeft ��������سߴ磬��λmm
	fFocusLeft �������ͷ���࣬��λmm
	fPixelSizeLeft ��������سߴ磬��λmm
	fSizeX �궨��������е����ܺͣ���λmm
	fSizeY �궨���������е����ܺͣ���λmm
*/
TJ3DSDK_API void TJST3DSetCalibParameter(double fFocusLeft, double fPixelSizeLeft, double fFocusRight, double fPixelSizeRight, double fSizeX, double fSizeY);
/*�����������(��Ŀ�㷨ʹ��):TJST3DSetLightProjectioParameter
���������
	nImageW���������ֱ���
	nImageH���������ֱ���
	fFocus��������࣬��λmm
	fPixelSize��������سߴ磬��λmm
*/
TJ3DSDK_API void TJST3DSetLightProjectioParameter(int nGroupId,int nImageW, int nImageH, double fFocus, double fPixelSize);

/*�����궨:TJST3DCalibCameraGroup
���������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	nImageNum ����궨��ͼ������
	nImageW ͼ����
	nImageH ͼ��߶�
	pLeftImageArr ��������б궨ͼ���ڴ����ݣ�������nImageNum����ƥ��
	pRightImageArr ��������б궨ͼ���ڴ����ݣ�������nImageNum����ƥ��
	ccfLeft ������궨�����ļ���
	ccfRight ������궨�����ļ���
	table bable�ļ���
	box box�ļ���
���������
	fCalibErr �û������ڴ� ����궨���(Ϊ0ʱҲ�Ǳ궨�ɹ�)
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DCalibCameraGroupA(int nCameraGroupID, int nImageNum, int nImageW, int nImageH,unsigned char* pLeftImageArr, unsigned char* pRightImageArr, double* fCalibErr,
	const char* ccfLeft, const char* ccfRight, const char* table, const char* box);

TJ3DSDK_API bool TJST3DCalibCameraGroupW(int nCameraGroupID, int nImageNum, int nImageW, int nImageH,unsigned char* pLeftImageArr, unsigned char* pRightImageArr, double* fCalibErr,
	const wchar_t* ccfLeft, const wchar_t* ccfRight, const wchar_t* table, const wchar_t* box);

/*����궨(��Ŀģʽ):TJST3DCalibCameraGroupSingle
���������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	nMode 0 �����;1 �����
	nImageNum ����궨��ͼ������
	nImageW ͼ����
	nImageH ͼ��߶�
	pImageArr ����ɼ�����ͼ�����ڴ�
	pImagePhaseArr ����ɼ��ĺ�������ͼ�����ڴ�ָ��
	ccfLeft ������궨�����ļ���
	ccfRight ������궨�����ļ���
	table bable�ļ���
	box box�ļ���
���������
	fCalibErr �û������ڴ� ����궨���(Ϊ0ʱҲ�Ǳ궨�ɹ�)
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DCalibCameraGroupSingleA(int nCameraGroupID,int nMode, int nImageNum, int nImageW, int nImageH, unsigned char* pImageArr, unsigned char* pImagePhaseArr, double* fCalibErr,
	const char* ccfLeft, const char* ccfRight, const char* table, const char* box);

TJ3DSDK_API bool TJST3DCalibCameraGroupSingleW(int nCameraGroupID,int nMode, int nImageNum, int nImageW, int nImageH, unsigned char* pImageArr, unsigned char* pImagePhaseArr, double* fCalibErr,
	const wchar_t* ccfLeft, const wchar_t* ccfRight, const wchar_t* table, const wchar_t* box);

TJ3DSDK_API bool TJST3DCalibCameraGroupDoubleSingleA(int nImageNum, int nImageW, int nImageH,
	unsigned char* pImageArrLeft, unsigned char* pImagePhaseArrLeft,
	unsigned char* pImageArrRight, unsigned char* pImagePhaseArrRight,
	const char* ccfLeftSingle0, const char* ccfRightSingle0, const char* tableSingle0, const char* boxSingle0,
	const char* ccfLeftSingle1, const char* ccfRightSingle1, const char* tableSingle1, const char* boxSingle1,
	const char* ccfLeftdouble, const char* ccfRightdouble, const char* tabledouble, const char* boxdouble);

TJ3DSDK_API bool TJST3DCalibCameraGroupDoubleSingleW(int nImageNum, int nImageW, int nImageH,
	unsigned char* pImageArrLeft, unsigned char* pImagePhaseArrLeft,
	unsigned char* pImageArrRight, unsigned char* pImagePhaseArrRight,
	const wchar_t* ccfLeftSingle0, const wchar_t* ccfRightSingle0, const wchar_t* tableSingle0, const wchar_t* boxSingle0,
	const wchar_t* ccfLeftSingle1, const wchar_t* ccfRightSingle1, const wchar_t* tableSingle1, const wchar_t* boxSingle1,
	const wchar_t* ccfLeftdouble, const wchar_t* ccfRightdouble, const wchar_t* tabledouble, const wchar_t* boxdouble);


/*���������궨����:TJST3DAddCalibCameraGroup
���������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	nImageW ͼ����
	nImageH ͼ��߶�
	ccfLeft ������궨�����ļ���
	ccfRight ������궨�����ļ���
	table bable�ļ���
	box box�ļ���
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DAddCalibCameraGroupA(int nCameraGroupID, int nImageW, int nImageH, const char* ccfLeft, const char* ccfRight, const char* table, const char* box);
TJ3DSDK_API bool TJST3DAddCalibCameraGroupW(int nCameraGroupID, int nImageW, int nImageH, const wchar_t* ccfLeft, const wchar_t* ccfRight, const wchar_t* table, const wchar_t* box);

/*�޸İ�Χ����Ϣ
���������
	nCameraGroupID �����ʶ���
	minx,miny,minz ��ʾһ����
	maxx,maxy,maxz ��ʾ��һ����
	box   ����·��
ע�⣺
	��������ǣ���������P(x,y,z)����minx<x<maxx,miny<y<maxy,minz<z<maxzʱ��Żᱣ��
	��Ҫ����AddCalibCameraGroup�Ժ�����޸Ķ�Ӧ������box��Ϣ
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DSetBoxA(int nCameraGroupID, float minx, float miny, float minz, float maxx, float maxy, float maxz, const char* box);
TJ3DSDK_API bool TJST3DSetBoxW(int nCameraGroupID, float minx, float miny, float minz, float maxx, float maxy, float maxz, const wchar_t* box);

TJ3DSDK_API bool TJST3DReadBoxW(const wchar_t* boxFile, double x[], double y[], double z[]);
TJ3DSDK_API bool TJST3DReadBoxA(const char* boxFile, double x[], double y[], double z[]);

TJ3DSDK_API unsigned int TJST3DGetImageBuffpoints(unsigned int nStart,int nImages, unsigned char** ppLeft, unsigned char** ppRight, bool brgb);
TJ3DSDK_API unsigned int TJST3DGetImageBuffpointsEx(int nGroup,unsigned int nStart, int nImages, unsigned char** ppLeft, unsigned char** ppRight, bool brgb);
TJ3DSDK_API unsigned int TJST3DGetImageBuffpoints3C(unsigned int nStart, int nImages, unsigned char** ppLeft, unsigned char** ppRight, unsigned char** ppThird, bool brgb);
/*����ͶӰ��������
���������
	nRasterType �������ͣ�0 һ������(Ĭ��ֵ)�� 1 ����߷ֱ��������2 ������
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DSetRasterType(int nRasterType);

/*�����Ƿ�����������ͼ(��ɫ) 
���������
	bColor true ������ɫ�����һ������ͼ�������ɵ������Զ���ɫ��false �����ӣ�Ĭ�ϣ�
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DSetColorPoints(bool bColor);

/*�����������ͼ(��ɫ)
���������
	pPointcloud ���ɵĵ���ָ��
	pColorImagedata BGR(888)���ɫͼ������ָ��,�ֱ��ʱ��������ɵ���ʱɨ������ķֱ���һ��
����ֵ��
	��
*/
TJ3DSDK_API void TJST3DAddColorImage(sn3DCore::sn3DRangeData* pPointcloud, unsigned char* pColorImagedata);

/*����mask����
*/
TJ3DSDK_API void TJST3DSetMinMaxMaskth(int nMinMaskth, int nMaxMaskth);

/*ʹ��MaskImageɾ����������
���������
	pPointcloud ���ɵĵ���ָ��
	pMaskImagedata 8λ�Ҷ�ͼ������ָ��,�ֱ��ʱ��������ɵ���ʱɨ������ķֱ���һ��
	nMinMaskth С�ڴ˻Ҷ�ֵ����Ӧ�ĵ�������ɾ��
	nMaxMaskth ���ڴ˻Ҷ�ֵ����Ӧ�ĵ�������ɾ��
����ֵ��
	��
˵����
	��� nMinMaskth=0��nMaxMaskth=255���Ǿ���������Ĳ���
	��� nMinMaskth=1��nMaxMaskth=1���Ǿ�ֻ�����Ҷ�ֵΪ1����Ӧ�ĵ�������
*/
TJ3DSDK_API void TJST3DUseMaskImage(sn3DCore::sn3DRangeData* pPointcloud, unsigned char* pMaskImagedata, unsigned char nMinMaskth, unsigned char nMaxMaskth);

/*ȡ����������һ��ɨ��ͼ(һ��ΪͶӰ��ɫ)��������MaskImage �� ColorImage
���������
	bColor Ϊtrue,����BRG(888)���ݣ�Ϊfalse,����8λ�Ҷ�����
����ֵ��
	�ɹ���ͼ������ָ�룬����Ҫ�û��ͷš�����һ��ɨ��֮ǰ������ʹ��
	ʧ�ܣ�NULL
˵����
	������ǵ����Ϊ�ڰ����������ʹ��bColor=true���Ƿ��ص�RGB��ֵ��һ����
*/
TJ3DSDK_API unsigned char* TJST3DGetLeftCameraLastTriggerImagebuff(bool bColor = false);

/*ȡ����������һ��ɨ��ͼ(һ��ΪͶӰ��ɫ)
���������
	bColor Ϊtrue,����BRG(888)���ݣ�Ϊfalse,����8λ�Ҷ�����
����ֵ��
	�ɹ���ͼ������ָ�룬����Ҫ�û��ͷš�����һ��ɨ��֮ǰ������ʹ��
	ʧ�ܣ�NULL
*/
TJ3DSDK_API unsigned char* TJST3DGetRightCameraLastTriggerImagebuff(bool bColor = false);


/*����sn3DRangeData��ָ��
����ֵ��
	�ǿ�ָ��  �ɹ�
	NULL	  ʧ��
*/
TJ3DSDK_API sn3DCore::sn3DRangeData* TJST3DRangeNew();

/*�ͷ�sn3DRangeData��ָ��
���������
	pRangedata ȷ��Ϊ sn3DCore::sn3DRangeData* ������Чָ��(�ǿգ���ǰû���ͷŹ�)
*/
TJ3DSDK_API void TJST3DRangeDelete(sn3DCore::sn3DRangeData* pRangedata);

/*���ط�������
���������
	pRangedata ȷ��Ϊ sn3DCore::sn3DRangeData* ������Чָ��
*/
TJ3DSDK_API unsigned int TJST3DGetRangeNors(sn3DCore::sn3DRangeData* pRangedata);

/*���ص�������
���������
	pRangedata ȷ��Ϊ sn3DCore::sn3DRangeData* ������Чָ��
*/
TJ3DSDK_API unsigned int TJST3DGetRangePoints(sn3DCore::sn3DRangeData* pRangedata);

/*�ѵ������ݽṹָ���е���ά��ת��Ϊ��������*/
/*
���������
	pRangeData: �������ݽṹָ��
	pPointArrayX������ָ�룬�����ά��Xֵ�����û�������ͷ��ڴ�
	pPointArrayY������ָ�룬�����ά��Yֵ�����û�������ͷ��ڴ�
	pPointArrayZ������ָ�룬�����ά��Zֵ�����û�������ͷ��ڴ�
	nArraySize�����ݳ��ȡ��������Ҫ����ά������
���أ�
	ʵ��ת������ά����
*/
TJ3DSDK_API unsigned int TJST3DConvertRangePointToFloatArray(sn3DCore::sn3DRangeData* pRangeData, float* pPointArrayX, float* pPointArrayY, float* pPointArrayZ, unsigned int nArraySize);

/*�ѵ������ݽṹָ���е���ά����ת��Ϊ��������*/
/*
���������
	pRangeData: �������ݽṹָ��
	pNorArrayX������ָ�룬�����ά����Xֵ�����û�������ͷ��ڴ�
	pNorArrayY������ָ�룬�����ά����Yֵ�����û�������ͷ��ڴ�
	pNorArrayZ������ָ�룬�����ά����Zֵ�����û�������ͷ��ڴ�
	nArraySize�����ݳ��ȡ��������Ҫ����ά������
���أ�
	ʵ��ת������ά����
*/
TJ3DSDK_API unsigned int TJST3DConvertRangeNorToFloatArray(sn3DCore::sn3DRangeData* pRangeData, float* pNorArrayX, float* pNorArrayY, float* pNorArrayZ, unsigned int nArraySize);

/*������ά�㷨�򸡵�ֵ
���������
	pRangedata ȷ��Ϊ sn3DCore::sn3DRangeData* ������Чָ��
	nIndex �����кţ�ȷ����ֵ������Χ
	xyz  0������x��1����y��2 ����z�� ������Ч
���أ�
	��ȷ���� ָ����ŵ�ɨ��ֵ
	���󷵻� �̶�ֵ -0.1234
*/
TJ3DSDK_API float TJST3DGeNorDataFromRangedata(sn3DCore::sn3DRangeData* pRangedata, unsigned int nIndex, unsigned int xyz);


/*������ά�㸡��ֵ
���������
	pRangedata ȷ��Ϊ sn3DCore::sn3DRangeData* ������Чָ��
	nIndex �����кţ�ȷ����ֵ������Χ
	xyz  0������x��1����y��2 ����z�� ������Ч
���أ�
	��ȷ���� ָ����ŵ�ɨ��ֵ
	���󷵻� �̶�ֵ -0.1234
*/
TJ3DSDK_API float TJST3DGetXYZDataFromRangedata(sn3DCore::sn3DRangeData* pRangedata, unsigned int nIndex, unsigned int xyz);

/**/
TJ3DSDK_API bool TJST3DGetTriggerBuffInfo(unsigned char** pLeft, unsigned char** pRight, int* pTrnum, int nOffset=0, bool bColor=false, int nGray=-1);
/*��������ģʽ*/
TJ3DSDK_API bool TJST3DStartContinueTrigger(unsigned int nMaxImageNum, unsigned char* pLeftImageBuff, unsigned char* pRightImageBuff, unsigned int nMaxMemSize);
TJ3DSDK_API unsigned int TJST3DStopContinueTrigger();
/*����ͼ���������ɵ��ƣ�TJST3DCreatePointcloudFromImageMemery
���������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	nImageNum ������������ͼ������
	nImageW ͼ����
	nImageH ͼ��߶�
	pLeftImageArr �������������ͼ���ڴ����ݣ�������nImageNum����ƥ��
	pRightImageArr �������������ͼ���ڴ����ݣ�������nImageNum����ƥ��
	pPointcloud ���ɵĵ����࣬���û�������ͷ��ڴ�
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageMemery(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageMemeryNoProcess(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud);

TJ3DSDK_API bool TJST3DCreatePointcloudFromImageMemeryColor(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageGrayCode(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud,bool bColor);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageGrayCodeP(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, unsigned char* pMaskImage, unsigned char* pColorImage, sn3DCore::sn3DRangeData* pPointcloud, bool bColor,int nMode);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImage3CAll(unsigned char *color_image, unsigned char *imageL, unsigned char *imageT, unsigned char *imageR, sn3DCore::sn3DRangeData *p, sn3DCore::sn3DRangeData *p0, sn3DCore::sn3DRangeData *p1, sn3DCore::sn3DRangeData *p2, unsigned char *Mask,unsigned char *Color);
/*ʵʱɨ�����ɵ��ƣ�TJST3DCreatePointcloudFromScan
���������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	pPointcloud ���ɵĵ����࣬���û�������ͷ��ڴ�
	bCreate �Ƿ񴴽����ơ�true �������ƣ�false ����Ӳ��ɨ�裬��������������
	hEvent ���Ϊ�ǿ�ָ�룬ɨ�����(���ǵ��ƴ������,���ɼ����㹻����������)ʱ�����SetEvent(hEvent)
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DCreatePointcloudFromScan(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud,bool bCreate=true,void* hEvent=0);

/*ɢ����������ɨ��
������
	bEnable�� true,��ʼɨ�裻false ֹͣɨ��
	callBackfunc���������ɺ�Ļص�����
*/
TJ3DSDK_API bool TJST3DContinueScan(bool bEnable, TJST3DRasterSpeckleCallback callBackfunc = NULL);

/*ɢ�ߵ���ɨ��
������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	pPointcloud ���ɵĵ����࣬���û�������ͷ��ڴ�
	bCreate �Ƿ񴴽����ơ�true �������ƣ�false ����Ӳ��ɨ�裬��������������
*/
TJ3DSDK_API bool TJST3DCreatePointcloudRasterSpeckle(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud,bool bCreate=true);

/*���ɨ��(�ɼ���ͬ����ɨ��ͼ)���ɵ���:TJST3DCreatePointcloudFromMultScan
���������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	pPointcloud ���ɵĵ����࣬���û�������ͷ��ڴ�
	nScanNums ɨ���������Χ��2--4
	nScanFps ɨ��FPSָ��(����),��ͨ�� TJST3DGetSupportScanFps ������ȡ��֧�ֵ�ɨ��FPS����
	fGain    �������ָ��(����)��ֵ�ķ�ΧΪ0.1-15.0
	nLedLight LED����ָ��(����)��ֵ�ķ�ΧΪ10-255
	nMinMaskth minMaskthָ��(����)��ֵ�ķ�ΧΪ0-255
	nMaxMaskth maxMaskthָ��(����)��ֵ�ķ�ΧΪ0-255
	bCreate �Ƿ񴴽����ơ�true �������ƣ�false ����Ӳ��ɨ�裬��������������
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DCreatePointcloudFromMultScan(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud, int nScanNums, int* nScanFps, float* fGain, int* nLedLight, int* nMinMaskth, int*nMaxMaskth, bool bCreate=true);
TJ3DSDK_API bool TJST3DCreatePointcloudFromMultScanNoProcess(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud, int nScanNums, int* nScanFps, float* fGain, int* nLedLight, int* nMinMaskth, int*nMaxMaskth, bool bCreate = true);

/*ICP�Զ�ƴ��
���������
	nNums ��Ҫ�ϲ��ĵ�������
	pPointclouds ��Ҫ�ϲ��ĵ���ָ��
	fSearchRadFac �����뾶
	iSampleSize ����
	iniFile ���������ļ�
���������
	diff Ϊȫ��ƽ�����
	rot 4x4ת������ 
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DMatchPointcloudA(int nNums, sn3DCore::sn3DRangeData* pPointclouds[], float fSearchRadFac, int iSampleSize, float *rot, float &diff, const char* iniFile);
TJ3DSDK_API bool TJST3DMatchPointcloudW(int nNums, sn3DCore::sn3DRangeData* pPointclouds[], float fSearchRadFac, int iSampleSize, float *rot, float &diff, const wchar_t* iniFile);
/*�ϲ�����(��ά��ͷ���)
���������
	nNums ��Ҫ�ϲ��ĵ�������
	pPointclouds ��Ҫ�ϲ��ĵ���ָ��
���������
	pPointcloud_out �ϲ���ĵ���ָ��(���û�������ͷ��ڴ�)
����ֵ:
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DMergePointcloud(int nNums, sn3DCore::sn3DRangeData* pPointclouds[], sn3DCore::sn3DRangeData* pPointcloud_out);

/*���Ʒָ�
���������
	pInput ��Ҫ�ָ�ĵ���
	px py pz 3�������꣬����ȷ���ָ�ƽ��
���������
	pOut1 �ָ��ĵ���1 ����ָ��(���û�������ͷ��ڴ�)
	pOut2 �ָ��ĵ���2 ����ָ��(���û�������ͷ��ڴ�)
����ֵ:
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DPointcloudDivision(const sn3DCore::sn3DRangeData* pInput, double px[3], double py[3], double pz[3], sn3DCore::sn3DRangeData* pOut1, sn3DCore::sn3DRangeData* pOut2);

/*�Ե��ƽ��пռ����
���������
	pointcloud_in ԭʼ����
	fSampleRadius �����뾶
�������:
	pointcloud_out ���������
����ֵ:
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool  TJST3DPointcloudSpaceSampling(sn3DCore::sn3DRangeData* pointcloud_in, float fSampleRadius,sn3DCore::sn3DRangeData *pointcloud_out);


/*��������(MESH)
���������
	pointcloud_in ԭʼ����
	strParfile �����ļ�ȫ·��(���ļ���)
���������
	mesh_out meshָ��
����ֵ:
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool  TJST3DPointcloudToMeshA(sn3DCore::sn3DRangeData* pointcloud_in,const char* strParfile, sn3DCore::sn3DRangeData *mesh_out);
TJ3DSDK_API bool  TJST3DPointcloudToMeshW(sn3DCore::sn3DRangeData* pointcloud_in, const wchar_t* strParfile, sn3DCore::sn3DRangeData *mesh_out);

/*����ȥ���ӵ㷽����TJST3DSetRemoveOutliersMethod
���������
	nMethod 0 ��ʾ�����㣬1 ��ʽ1���� ��2 ��ʽ2����(���̼߳���)
*/
TJ3DSDK_API void TJST3DSetRemoveOutliersMethod(int nMethod);

/*����ȥ���ӵ������TJST3DSetRemoveOutliersPar
���������
	fGrowRad���뾶
	nMinNumPts:�뾶�����ٵ���
*/
TJ3DSDK_API void TJST3DSetRemoveOutliersPar(double fGrowRad, int nMinNumPts);

/*��������ļ���TJST3DSavePointcloudTofile
���������
	pPointcloud ���ɵĵ�����ָ��
	strFileFullname �����ļ�ȫ·��,֧��.txt .ply
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DSavePointcloudTofileA(sn3DCore::sn3DRangeData* pPointcloud, const char* strFileFullname);
TJ3DSDK_API bool TJST3DSavePointcloudTofileW(sn3DCore::sn3DRangeData* pPointcloud, const wchar_t* strFileFullname);

/*�򿪵����ļ���TJST3DLoadPointcloudFromfile
���������
	pPointcloud ���ɵĵ�����ָ��,���û�������ͷ��ڴ�
	strFileFullname �����ļ�ȫ·��,֧��.txt .ply
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DLoadPointcloudFromfileA(sn3DCore::sn3DRangeData* pPointcloud, const char* strFileFullname);
TJ3DSDK_API bool TJST3DLoadPointcloudFromfileW(sn3DCore::sn3DRangeData* pPointcloud, const wchar_t* strFileFullname);

/*��ʾ���ƴ��ڣ�TJST3DCreate3DViewer
���������
	strName ��ʾ���ڵ�����(��ʱ��Ч)
*/
TJ3DSDK_API void TJST3DCreate3DViewerA(const char* strName=NULL);
TJ3DSDK_API void TJST3DCreate3DViewerW(const wchar_t* strName=NULL);

/*�رյ��ƴ��ڣ�TJST3DCloseViewer*/
TJ3DSDK_API void TJST3DCloseViewer();

/*��ʾ���ɵĵ��ƣ�TJST3DShowPoint
�������:
	pPointcloud ���ɵĵ�����ָ��
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DShowPoint(sn3DCore::sn3DRangeData * pPointcloud);

/*�����ӵ�
�������:
	posΪ����ͷ������
	viewΪ����ͷ�۲ⷽ��
	upΪ����ͷ�����Ϸ�����
˵����
	Ĭ������Ϊ�۲�z��������   y��������Ϊ�Ϸ�
*/
TJ3DSDK_API void TJST3DSetViewPoint(double posX, double posY, double posZ, double viewX = 0.0, double viewY = 0.0, double viewZ = 1.0, double upX = 0.0, double upY = 1.0, double upZ = 0.0);

/*������ɫ����ķ�Χ��TJST3DSetSizeColorShow
�������:
	zmin Z�����Сֵ
	zmax Z����ϴ�ֵ
˵����
	��������z�᷽�򽥱�
	��zmin=zmaxʱ���Զ��õ��ƵĹ�ģ���䡣
	������Χ�ĵ�����ʾΪ��ɫ
*/
TJ3DSDK_API void TJST3DSetSizeColorShow(float zmin, float zmax);

/*�л���ǰ����飺TJST3DSelectCameraGroup
�������:
	nCamGroup ��������,��0��ʼ
����ֵ��
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool TJST3DSelectCameraGroup(int nCamGroup);

/*��ȡ��ǰ�����
����ֵ��
	�����ID�����ֻ��һ�������ʼ�շ���0
*/
TJ3DSDK_API int TJST3DGetCameraGroup();


/*����ͶӰģʽ
�������:
	nPrjMode 0 ���� 1 ���� 2 ʮ�� 3 ���̸�
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetProject(IN int nPrjMode);


/*����ͶӰģʽ,ָ�������
�������:
	nCamGroup	�����ID
	nPrjMode 0 ���� 1 ���� 2 ʮ�� 3 ���̸�
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetProjectEx(IN int nCamGroup ,IN int nPrjMode);

/*����ͶӰ����,�ϵ�󲻻���Ч
�������:
	nLight ͶӰ����ֵ��10-255������ʱ���ڸ���������ʱע��ɢ��
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetLedLight(IN unsigned char nLight);

/*����ͶӰ�������ã��ϵ�����Ч
�������:
	nLight ͶӰ����ֵ��10-255������ʱ���ڸ���������ʱע��ɢ��
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSaveLedLight(IN unsigned char nLight);

/*��LED*/
TJ3DSDK_API void TJST3DLedOn();
/*�ر�LED*/
TJ3DSDK_API void TJST3DLedOff();

/*����ͶӰ����,�ϵ�󲻻���Ч
�������:
	nColor ͶӰ��ɫ,0 1 2 3 ����� �� �� ��
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetLedColor(IN unsigned char nColor);

/*�������һ��ͶӰ�Ҷ�(������)
�������:
	nGray 0-255�Ҷ�ֵ
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DSetPrjGray(IN unsigned char nGray);

/*�궨ת��
�������:
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	nImageNum ������������ͼ������
	nImageW ͼ����
	nImageH ͼ��߶�
	ImageL ��������б궨ͼ���ڴ����ݣ�������nImageNum����ƥ��
	Imager ��������б궨ͼ���ڴ����ݣ�������nImageNum����ƥ��
	saveName ��������·��
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DCalibrateAxisA(IN int nCamGroup, IN int nImageNum, IN int nImageW, IN int nImageH, IN unsigned char* ImageL, IN unsigned char* ImageR, IN OUT const char* saveName);
TJ3DSDK_API bool TJST3DCalibrateAxisW(IN int nCamGroup, IN int nImageNum,IN int nImageW, IN int nImageH ,IN unsigned char* ImageL, IN unsigned char* ImageR,IN OUT const wchar_t* saveName);

TJ3DSDK_API bool TJST3DCalibrateAxisExA(IN int nCamGroup, IN int nImageNum, IN int nImageW, IN int nImageH, IN unsigned char* ImageL, IN unsigned char* ImageR, IN float *Angle ,IN OUT const char* saveName);
TJ3DSDK_API bool TJST3DCalibrateAxisExW(IN int nCamGroup, IN int nImageNum, IN int nImageW, IN int nImageH, IN unsigned char* ImageL, IN unsigned char* ImageR, IN float *Angle ,IN OUT const wchar_t* saveName);

/*�����������
�������:
	pXAxisFilename ˮƽת������ļ�
	pYAxisFilename ��ֱת������ļ�
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DLoadAxisCalibratePointsA(IN const char* pXAxisFilename, IN const char* pYAxisFilename);
TJ3DSDK_API bool TJST3DLoadAxisCalibratePointsW(IN const wchar_t* pXAxisFilename, IN const wchar_t* pYAxisFilename);

TJ3DSDK_API bool TJST3DLoadAxis3CalibratePointsA(IN const char* pXAxisFilename, IN const char* pYAxisFilename, IN const char* pZAxisFilename);
TJ3DSDK_API bool TJST3DLoadAxis3CalibratePointsW(IN const wchar_t* pXAxisFilename, IN const wchar_t* pYAxisFilename, IN const wchar_t* pZAxisFilename);

/*������ת�ĽǶȵõ�4x4����任����
�������:      
	xAngle ˮƽת����ת�Ƕ�(-360--+360)
	yAngle ��ֱת����ת�Ƕ�(-360--+360)
���������
	pRTMat 4x4����任���� ���û�������ͷ��ڴ�
���أ�
	�ɹ� true;
	ʧ�� false;
C/C++���þ�����
	{
		float RTMat [16];
		TJST3DAnglexToRotateTrans(60,80,RTMat);
	}	
*/
TJ3DSDK_API bool TJST3DAnglesToRotateTrans(IN float xAngle, IN float yAngle,OUT float* pRTMat);
TJ3DSDK_API bool TJST3DAnglesAxis3ToRotateTrans(IN float xAngle, IN float yAngle, IN float zAngle, OUT float* pRTMat);


//������Ҫ�򿪵��������IPv4��ַ
TJ3DSDK_API bool TJST3DSetNetdevIP(int nGroupIndex, const char* prjIP, const char* leftCamIP = NULL, const char* rightCamIP = NULL);
//���ñ궨��Բ��ʶ�����
TJ3DSDK_API void TJST3DSetCalibImage2DPoints(double sigma, double tlow, double thigh, double pointQuality, double circularity, int minDiameter, int maxDiameter);
//�����־��
TJ3DSDK_API void TJST3DClearMarkPoints();
/*�ӱ�־��ͼ�����ݵõ���תƽ�ƾ���*/
TJ3DSDK_API bool TJST3DMarkPointsAlign(int nCameraGroupID, int nImageW, int nImageH, unsigned char* pImageMarkLeft, unsigned char* pImageMarkRight, double* fRotateTrans);
/*��ɨ���־��ͼ������(�ڴ�)�õ���תƽ�ƾ���*/
TJ3DSDK_API bool TJST3DMarkPointsAlignEx(int nCameraGroupID,double* fRotateTrans);

TJ3DSDK_API bool TJST3DSaveTextureFileW(const wchar_t* filePathname);

/*��ʼ�� ����ʶ��ģ�� */
TJ3DSDK_API bool TJST3DWorkpieceModuleInitA(const char* strCadfile, int nDetectpoint, const char* strDetectpointFile[]);
TJ3DSDK_API bool TJST3DWorkpieceModuleInitW(const wchar_t* strCadfile, int nDetectpoint, const wchar_t* strDetectpointFile[]);
/*���ɴ�ƴ�����ļ�*/
TJ3DSDK_API unsigned int TJST3DCreateCoarseSstitchingA(const sn3DRangeData* pointcloud, unsigned int lastResult,const char* strCoarseFileUp, const char* strCoarseFileDown,const char* strDebugOut1=NULL, const char* strDebugOut2 = NULL);
TJ3DSDK_API unsigned int TJST3DCreateCoarseSstitchingW(const sn3DRangeData* pointcloud, unsigned int lastResult,const wchar_t* strCoarseFileUp, const wchar_t* strCoarseFileDown, const wchar_t* strDebugOut1 =NULL, const wchar_t* strDebugOut2 = NULL);
/*�����ƴ�����ļ�*/
TJ3DSDK_API bool TJST3DLoadCoarseSstitchingA(const char* strCoarseFileUp, const char* strCoarseFileDown);
TJ3DSDK_API bool TJST3LoadCoarseSstitchingW(const wchar_t* strCoarseFileUp, const wchar_t* strCoarseFileDown);
/*����ʶ�����*/
TJ3DSDK_API unsigned int TJST3DWorkpieceModuleCalResult(const sn3DRangeData* pointcloud);

/*ɨ��ʱ���������Ϣ
���������
	nUseDepth: 0 �����㣬1 ֻ����Z��2����XYZ
*/
TJ3DSDK_API void TJST3DSetCalDepthdata(int nUseDepth);

/*���������Ϣ
���������
	strFullPath���ļ���
	bTxt��		true �����ı���ʽ��false ��������Ƹ�ʽ
���أ�
	�ɹ�:true
	ʧ��:false
*/
TJ3DSDK_API bool TJST3DSaveDepthdataA(const char* strFullPath, bool bTxt=false);
TJ3DSDK_API bool TJST3DSaveDepthdataW(const wchar_t* strFullPath,bool bTxt=false);

/*ȡ���������ָ�루�����ȸ����ͣ�
���أ�
	�ɹ�:����ָ���ַ����ʹ�����ģʽ1ʱ���������Zֵ(����Ϊͼ�񳤳˿�); ��ʹ�����ģʽ2ʱ�����ݰ�XYZXYZ......XYZ����(����Ϊͼ�񳤳˿��3)
	ʧ��:NULL(0)
*/
TJ3DSDK_API const float* TJST3DGetDepthdata();
/*������ģ��*/

/*ת����������ϵ
���������
	nCameraGroupID �����ʶ��ţ���Ҫ���ֶ��������nCameraGroupID=0��Ϊ�������
	pPointcloud ����ָ��
	nMode ���Ͳ�����˵����Mode = 1 ԭ��任������ĵ�һ���궨��λ��ƽ��;  Mode = 2  ԭ��(0,0,0)�任�����������;
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DChangeCoordinate(IN int nCamGroup, IN OUT sn3DCore::sn3DRangeData* pPointcloud, IN int nMode);
/*��е�ֱ궨*/
TJ3DSDK_API bool TJST3DCalibrationRobot(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char * pLeftImageArr, unsigned char * pRightImageArr, float RobotPoint[][6], float * MatRT);

TJ3DSDK_API bool TJST3DCalibrationRobot2(int nCameraGroupID, int nPointNum, float PointL[][2], float PointR[][2], float RobotPoint[][3], float *MatRT);

TJ3DSDK_API bool TJST3DCalibrationRobot3A(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char * pLeftImageArr, unsigned char * pRightImageArr,float RobotPoint[][6],const char* pstrCalibFile, int mode);
TJ3DSDK_API bool TJST3DCalibrationRobot3W(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char * pLeftImageArr, unsigned char * pRightImageArr, float RobotPoint[][6], const wchar_t* pstrCalibFile, int mode);

/*����ת���궨������ϵ������������ϵ
���������
	pointcloudin �궨������ϵ����
	SaveFileName �����˱궨�ļ�ȫ·��
	mode �����˹�����ʽ��0 �������⣻1 �������� 
	RobotPoint �����˵�ǰ���ꡣ��mode=1 ʱ��Ч
���������
	pointcloudout ����������ϵ����
���أ�
	�ɹ� true;
	ʧ�� false;
*/
TJ3DSDK_API bool TJST3DChangePointCloudToRobotA(sn3DRangeData *pointcloudin, sn3DRangeData *pointcloudout, const char* SaveFileName, int mode = 0, float RobotPoint[6] = NULL);
TJ3DSDK_API bool TJST3DChangePointCloudToRobotW(sn3DRangeData *pointcloudin, sn3DRangeData *pointcloudout, const wchar_t* SaveFileName, int mode = 0, float RobotPoint[6] = NULL);

/*����������ת*/
TJ3DSDK_API void TJST3DGetTransformMat(double fXAngle, double fYAngle, double fZAngle, float* pRT);
TJ3DSDK_API void TJST3DRangedataTransformMat(sn3DCore::sn3DRangeData* pPointcloud, float* pRT);
//���ڲ��ԣ������߲�Ҫ���ã�����Ը�
TJ3DSDK_API void TJST3DDebugSetImageWH(int nW, int nH);
TJ3DSDK_API unsigned int TJST3DDebugGetTriggerNumsLeft();
TJ3DSDK_API unsigned int TJST3DDebugGetTriggerNumsRight();
TJ3DSDK_API void TJSTTriggerAllDevice(int nDevNum,int nTrNums=13);
TJ3DSDK_API int BINGLANGInitA(const char* ccfLeft1, const char* ccfRight1, const char* table1, const char* box1, const char* ccfLeft2, const char* ccfRight2, const char* table2, const char* box2);
TJ3DSDK_API int BINGLANGInitW(const wchar_t* ccfLeft1, const wchar_t* ccfRight1, const wchar_t* table1, const wchar_t* box1, const wchar_t* ccfLeft2, const wchar_t* ccfRight2, const wchar_t* table2, const wchar_t* box2);

TJ3DSDK_API int BINGLANGInit3CA(const char* ccfLeft1, const char* ccfRight1, const char* table1, const char* box1,
	const char* ccfLeft2, const char* ccfRight2, const char* table2, const char* box2,
	const char* ccfLeft3, const char* ccfRight3, const char* table3, const char* box3);
TJ3DSDK_API int BINGLANGInit3CW(const wchar_t* ccfLeft1, const wchar_t* ccfRight1, const wchar_t* table1, const wchar_t* box1, 
	const wchar_t* ccfLeft2, const wchar_t* ccfRight2, const wchar_t* table2, const wchar_t* box2,
	const wchar_t* ccfLeft3, const wchar_t* ccfRight3, const wchar_t* table3, const wchar_t* box3);

TJ3DSDK_API bool BINGLANGCalibCamera3CA(unsigned char* pImgL, unsigned char* pImgM, unsigned char* pImgR, int nImgNum,
	const char* ccfLeft1, const char* ccfRight1, const char* table1, const char* box1,
	const char* ccfLeft2, const char* ccfRight2, const char* table2, const char* box2,
	const char* ccfLeft3, const char* ccfRight3, const char* table3, const char* box3);
TJ3DSDK_API bool BINGLANGCalibCamera3CW(unsigned char* pImgL, unsigned char* pImgM, unsigned char* pImgR,int nImgNum,
	const wchar_t* ccfLeft1, const wchar_t* ccfRight1, const wchar_t* table1, const wchar_t* box1,
	const wchar_t* ccfLeft2, const wchar_t* ccfRight2, const wchar_t* table2, const wchar_t* box2,
	const wchar_t* ccfLeft3, const wchar_t* ccfRight3, const wchar_t* table3, const wchar_t* box3);

TJ3DSDK_API bool BINGLANGCalculationA(int nCamGroup,sn3DCore::sn3DRangeData* pPointcloud,const char* savefilename1, const char* savefilename2,const char* savePointcloud,const char* saveBoardPointcloud=NULL);
TJ3DSDK_API bool BINGLANGCalculationW(int nCamGroup,sn3DCore::sn3DRangeData* pPointcloud,const wchar_t* savefilename1, const wchar_t* savefilename2,const wchar_t* savePointcloud, const wchar_t* saveBoardPointcloud = NULL);
TJ3DSDK_API bool BINGLANGCalculationDebugA(int nCamGroup,sn3DCore::sn3DRangeData* pPointcloud, const char* savefilename1, const char* savefilename2);
TJ3DSDK_API bool BINGLANGCalculationDebugW(int nCamGroup,sn3DCore::sn3DRangeData* pPointcloud, const wchar_t* savefilename1, const wchar_t* savefilename2);
TJ3DSDK_API bool BINGLANGGetTriggerBufPoint(unsigned char** pLeft, unsigned char** pRight, bool bColor, int nGray=-1);
TJ3DSDK_API bool BINGLANGGetTriggerBufPointEx(unsigned char** pLeft, unsigned char** pRight, bool bColor, int nGray, int nNum);
TJ3DSDK_API bool BINGLANGGetTriggerBufPoint3Ex(unsigned char** pLeft, unsigned char** pRight,unsigned char** pThird, bool bColor, int nGray=-1);
TJ3DSDK_API bool BINGLANGGetRawTriggerBufPointEx(unsigned char** pLeft, unsigned char** pRight,unsigned int* pSingleImageSize,int nGray, int nNum);
TJ3DSDK_API bool BINGLANGRecvTriggerCompeleted(bool bColor);
TJ3DSDK_API bool BINGLANGLoadSegInfoA(const char* seginfo0, const char* seginfo1);
TJ3DSDK_API bool BINGLANGLoadSegInfoW(const wchar_t* seginfo0, const wchar_t* seginfo1);
TJ3DSDK_API bool BINGLANGCalibSegA(int nCamGroup,unsigned char *imageL, unsigned char *imageR,double *X, double *Y, const char* savefile);
TJ3DSDK_API bool BINGLANGCalibSegW(int nCamGroup,unsigned char *imageL, unsigned char *imageR, double *X, double *Y, const wchar_t* savefile);

TJ3DSDK_API bool BINGLANGCalibSegNewA(unsigned char *imageL, unsigned char *imageR, double *X, double *Y, const char* seqfile,const char* channefile,int gongwei);
TJ3DSDK_API bool BINGLANGCalibSegNewW(unsigned char *imageL, unsigned char *imageR, double *X, double *Y, const wchar_t* seqfile, const wchar_t* channefile,int gongwei);

TJ3DSDK_API void BINGLANGMergePointCloud(sn3DCore::sn3DRangeData *p1, sn3DCore::sn3DRangeData *p2, sn3DCore::sn3DRangeData *output);
TJ3DSDK_API void BINGLANGSetMinMaxMaskth(unsigned char nMinMaskth, unsigned char nMaxMaskth);
TJ3DSDK_API bool BINGLANGMultDevScan(sn3DCore::sn3DRangeData **pPointcloudArr,const char* strDebugSaveDir=NULL);
TJ3DSDK_API bool BINGLANGDebugDevScan(sn3DCore::sn3DRangeData *pPointcloud,const char* strDebugSaveDir = NULL);
TJ3DSDK_API void BINGLANGMultDevScanPrintRuntime();
TJ3DSDK_API bool BINGLANGAIInit(const char *calibfile0, const char *calibfile1, const char *svmfile, const char *pbfile);
TJ3DSDK_API bool BINGLANGCreatePointCloudImageA(int nCamGroup, const char* strSavefile = NULL);
TJ3DSDK_API bool BINGLANGCreatePointCloudImagew(int nCamGroup, const wchar_t* strSavefile = NULL);
TJ3DSDK_API void BINGLANGSetDepthZRange(double zming0, double zmaxg0, double zming1, double zmaxg1);
TJ3DSDK_API void BINGLANGSetGridplateColorMaskThreshold(int nColor, int nThreshold);

TJ3DSDK_API bool GOUQICalculationA(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, const char* savefilename1, const char* saveSegPointcloud, const char* saveBoardPointcloud = NULL);
TJ3DSDK_API bool GOUQICalculationW(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, const wchar_t* savefilename1, const wchar_t* saveSegPointcloud, const wchar_t* saveBoardPointcloud = NULL);
TJ3DSDK_API bool GOUQICalculationDebugA(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, const char* savefilename1);
TJ3DSDK_API bool GOUQICalculationDebugW(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, const wchar_t* savefilename1);

TJ3DSDK_API bool GOUQICalculationExA(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, sn3DCore::sn3DRangeData* pPointcloud_Lu, const char* savefilename1, const char* saveSegPointcloud, const char* saveSegPointcloud_Lu, const char* saveBoardPointcloud = NULL);
TJ3DSDK_API bool GOUQICalculationExW(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, sn3DCore::sn3DRangeData* pPointcloud_Lu, const wchar_t* savefilename1, const wchar_t* saveSegPointcloud, const wchar_t* saveSegPointcloud_Lu, const wchar_t* saveBoardPointcloud = NULL);
TJ3DSDK_API bool GOUQICalculationExNew(sn3DCore::sn3DRangeData* pPointcloud, int nMode);

TJ3DSDK_API bool GOUQICalculationDebugExA(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, sn3DCore::sn3DRangeData* pPointcloud_Lu, const char* savefilename1);
TJ3DSDK_API bool GOUQICalculationDebugExW(int nCamGroup, sn3DCore::sn3DRangeData* pPointcloud, sn3DCore::sn3DRangeData* pPointcloud_Lu, const wchar_t* savefilename1);
TJ3DSDK_API bool GOUQICalculationDebugExNew(sn3DCore::sn3DRangeData* pPointcloud, int nMode);
TJ3DSDK_API bool GOUQISetBox(double zmin, double zmax, const char* seqfile);
TJ3DSDK_API bool GOUQICreateChannel(sn3DCore::sn3DRangeData* pPointcloud, const char* channelfile);
TJ3DSDK_API bool GOUQICreateMaskGQ(unsigned char* pLeftImage, unsigned char* pRightImage, unsigned char* pMaskImage, unsigned char* pColorImage);
TJ3DSDK_API bool GOUQICreateMask_GQ_P(unsigned char *imageML, unsigned char *imageMR, unsigned char *imagePL, unsigned char *imagePR, unsigned char *Mask, unsigned char *Color);
TJ3DSDK_API bool GOUQISetMode(int nMode);

//FlexMatchģ��
/*���(3)�궨 FLEXMATCHCalibCamera
���������
	nImageNum �궨ͼ����
	nImageW	  �궨ͼ����ֱ���
	nImageH	  �궨ͼ����ֱ���
	pImgL     ������궨ͼ�ڴ�����,8λ�Ҷȡ��ڴ�ռ��СΪ nImageW*nImageH*nImageNum
	pImgM     ������궨ͼ�ڴ�����,8λ�Ҷȡ��ڴ�ռ��СΪ nImageW*nImageH*nImageNum
	pImgR     ������궨ͼ�ڴ�����,8λ�Ҷȡ��ڴ�ռ��СΪ nImageW*nImageH*nImageNum

	ccfLeft,ccfRight,table,box �����һ��궨����ļ���������
���أ�
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool FLEXMATCHCalibCameraA(int nImageNum, int nImageW, int nImageH, unsigned char* pImgL, unsigned char* pImgM, unsigned char* pImgR,
	const char* ccfLeft1, const char* ccfRight1, const char* table1, const char* box1,
	const char* ccfLeft2, const char* ccfRight2, const char* table2, const char* box2,
	const char* ccfLeft3, const char* ccfRight3, const char* table3, const char* box3);
TJ3DSDK_API bool FLEXMATCHCalibCameraW(int nImageNum, int nImageW, int nImageH, unsigned char* pImgL, unsigned char* pImgM, unsigned char* pImgR, 
	const wchar_t* ccfLeft1, const wchar_t* ccfRight1, const wchar_t* table1, const wchar_t* box1,
	const wchar_t* ccfLeft2, const wchar_t* ccfRight2, const wchar_t* table2, const wchar_t* box2,
	const wchar_t* ccfLeft3, const wchar_t* ccfRight3, const wchar_t* table3, const wchar_t* box3);

TJ3DSDK_API bool FLEXMATCHCreateRangeData(unsigned char* pLeftImageArr, unsigned char* pRightImageArr, unsigned char* pThirdImageArr,
	sn3DCore::sn3DRangeData* pointcloud);

TJ3DSDK_API bool FLEXMATCHCreatModelA(sn3DCore::sn3DRangeData* PointcloudSor, unsigned char* ImageSor, const char* filename);
TJ3DSDK_API bool FLEXMATCHCreatModelW(sn3DCore::sn3DRangeData* PointcloudSor, unsigned char* ImageSor, const wchar_t* filename);

TJ3DSDK_API bool FLEXMATCHReadModelA(const char* filename);
TJ3DSDK_API bool FLEXMATCHReadModelW(const wchar_t* filename);

/*FLEXMATCHSetMatchParam
���������
	PointcloudSor ������ָ��
	ImageSor ��άͼ������ָ��
�������
	Tr ��SDK�����ڴ棬�û��ͷ�
���أ�
	���� �ɹ�
	0 ʧ��
*/
TJ3DSDK_API int FLEXMATCHMatchUseImage(sn3DCore::sn3DRangeData *PointcloudSor, unsigned char *ImageSor, float** Tr);

/*FLEXMATCHSetMatchParam
���������
	npar ��������
	param ����ָ��
���أ�
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool FLEXMATCHSetMatchParam(int npar,double* param);

/*ɨ������ɵ���:FLEXMATCHCreateRangeDataFromScan
���������
	fGain ɨ��ʱ��������棬С��0ʱ�����������
���������
	PointcloudSor ������ָ�룬�û�������ͷ��ڴ�
���أ�
	true �ɹ�
	false ʧ��
*/
TJ3DSDK_API bool FLEXMATCHCreateRangeDataFromScan(float fGain, sn3DCore::sn3DRangeData* PointcloudSor);


/*������и�ץȡ1��ͼ:FLEXMATCHCatchImage
ע�⣺
	ֻ������Ƶģʽ��ץͼ
���������
	fGain ץͼʱ���������
	nMaxBufsize �û�������ڴ���ֽڴ�С.����ǲ�ɫ����������ڴ��С=��Ƶ��x��Ƶ��x3
���������
	pLeftImgbuf �����ͼ���ڴ�ָ�룬�û�������ͷ��ڴ�
	pRightImgbuf �����ͼ���ڴ�ָ�룬�û�������ͷ��ڴ�
	pThirdImage �������(��)ͼ���ڴ�ָ�룬�û�������ͷ��ڴ�
���أ�
	0 ʧ�ܡ�
	����ͼ�������ֽ���
*/
TJ3DSDK_API bool FLEXMATCHCatchImage(float fGain, unsigned int nMaxBufsize, unsigned char* pLeftImage, unsigned char* pRightImage, unsigned char* pThirdImage);

#ifdef UNICODE
#define TJST3DSaveBmpImage TJST3DSaveBmpImageW
#define TJST3DSaveTriggerImages TJST3DSaveTriggerImagesW
#define TJST3DCalibCameraGroup TJST3DCalibCameraGroupW
#define TJST3DAddCalibCameraGroup TJST3DAddCalibCameraGroupW
#define TJST3DCalibCameraGroupSingle TJST3DCalibCameraGroupSingleW
#define TJST3DCalibCameraGroupDoubleSingle	TJST3DCalibCameraGroupDoubleSingleW
#define TJST3DSetBox TJST3DSetBoxW
#define TJST3DPointcloudToMesh TJST3DPointcloudToMeshW
#define TJST3DSavePointcloudTofile TJST3DSavePointcloudTofileW
#define TJST3DLoadPointcloudFromfile TJST3DLoadPointcloudFromfileW
#define TJST3DCreate3DViewer TJST3DCreate3DViewerW
#define TJST3DCalibrateAxis TJST3DCalibrateAxisW
#define TJST3DCalibrationRobot3 TJST3DCalibrationRobot3W
#define TJST3DChangePointCloudToRobot TJST3DChangePointCloudToRobotW
#define TJST3DLoadSingleStdData TJST3DLoadSingleStdDataW
#define TJST3DSaveDepthdata TJST3DSaveDepthdataW
#define BINGLANGInit	BINGLANGInitW
#define BINGLANGCalculation BINGLANGCalculationW
#define BINGLANGCalculationDebug BINGLANGCalculationDebugW
#define BINGLANGLoadSegInfo BINGLANGLoadSegInfoW
#define BINGLANGCalibSeg BINGLANGCalibSegW
#define BINGLANGCalibSegNew BINGLANGCalibSegNewW
#define BINGLANGCreatePointCloudImage BINGLANGCreatePointCloudImageW
#define GOUQICalculation GOUQICalculationW
#define GOUQICalculationDebug GOUQICalculationDebugW
#define BINGLANGInit3C BINGLANGInit3CW
#define BINGLANGCalibCamera3C BINGLANGCalibCamera3CW
#define FLEXMATCHCalibCamera	FLEXMATCHCalibCameraW
#define FLEXMATCHCreatModel		FLEXMATCHCreatModelW
#define FLEXMATCHReadModel		FLEXMATCHReadModelW
#else
#define TJST3DSaveBmpImage TJST3DSaveBmpImageA
#define TJST3DSaveTriggerImages TJST3DSaveTriggerImagesA
#define TJST3DCalibCameraGroup TJST3DCalibCameraGroupA
#define TJST3DAddCalibCameraGroup TJST3DAddCalibCameraGroupA
#define TJST3DCalibCameraGroupSingle TJST3DCalibCameraGroupSingleA
#define TJST3DCalibCameraGroupDoubleSingle	TJST3DCalibCameraGroupDoubleSingleA
#define TJST3DSetBox TJST3DSetBoxA
#define TJST3DPointcloudToMesh TJST3DPointcloudToMeshA
#define TJST3DSavePointcloudTofile TJST3DSavePointcloudTofileA
#define TJST3DLoadPointcloudFromfile TJST3DLoadPointcloudFromfileA
#define TJST3DCreate3DViewer TJST3DCreate3DViewerA
#define TJST3DCalibrateAxis TJST3DCalibrateAxisA
#define TJST3DCalibrationRobot3 TJST3DCalibrationRobot3A
#define TJST3DChangePointCloudToRobot TJST3DChangePointCloudToRobotA
#define TJST3DLoadSingleStdData		TJST3DLoadSingleStdDataA
#define TJST3DSaveDepthdata			TJST3DSaveDepthdataA
#define BINGLANGInit	BINGLANGInitA
#define BINGLANGCalculation BINGLANGCalculationA
#define BINGLANGCalculationDebug BINGLANGCalculationDebugA
#define BINGLANGLoadSegInfo BINGLANGLoadSegInfoA
#define BINGLANGCalibSeg	BINGLANGCalibSegA
#define BINGLANGCalibSegNew BINGLANGCalibSegNewA
#define BINGLANGCreatePointCloudImage BINGLANGCreatePointCloudImageA
#define GOUQICalculation GOUQICalculationA
#define GOUQICalculationDebug GOUQICalculationDebugA
#define BINGLANGInit3C	BINGLANGInit3CA
#define BINGLANGCalibCamera3C BINGLANGCalibCamera3CA
#define FLEXMATCHCalibCamera	FLEXMATCHCalibCameraA
#define FLEXMATCHCreatModel		FLEXMATCHCreatModelA
#define FLEXMATCHReadModel		FLEXMATCHReadModelA
#endif
