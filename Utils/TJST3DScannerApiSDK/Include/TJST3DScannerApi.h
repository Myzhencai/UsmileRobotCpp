/*
* 文件名：[TJST3DScannerApi.h]
* 作者：〈杭州腾聚科技-JXB〉
* 描述：〈高速3维扫描设备软件开发包,只支持64位程序开发〉
* 修改人：〈JXB〉
* 修改时间：2019 - 04 - 03
* 修改内容：〈新建〉

* 修改人：〈JXB〉
* 修改时间：2019 - 05 - 31
* 修改内容：
	1.增加投影控制接口，包括LED的开关，电流（亮度）调节，部分投影的颜色控制
	2.支持点云保存.ply等格式支持
	3.支持按照相机名称来初始化设备以自动区分左右设备
* 修改人：〈JXB〉
* 修改时间：2019 - 07 - 09
* 修改内容：
	1.修改槟榔项目点云生成方式：1）点云生成 2）坐标系转换 3）包围盒滤波
	2.其他一般项目点云生成方式：1）点云生成 2）包围盒滤波 3）杂点去除
* 修改人：〈JXB〉
* 修改时间：2019 - 07 - 17
* 修改内容：
	1.增加导入3轴标定数据的函数接口
	2.增加3个角度取得变换矩阵的函数接口
	3.增加点云坐标系转换函数接口  TJST3DChangeCoordinate
* 修改人：〈JXB〉
* 修改时间：2019 - 07 - 30
* 修改内容：
	1.增加条纹适配接口：TJST3DSetRasterType(). 高分辨率相机需要切换条纹，扫描效果会更好。投影机需要下载新条纹.
	2.增加取得最少条纹接口 TJST3DGetMinTriggerImageNum
* 修改人：〈JXB〉
* 修改时间：2019 - 10 - 09
* 修改内容：
	1.增加从标志点取得转换矩阵的接口
	2.增加配置特征点云接口
	3.增加计算特征接口

* 修改人：〈JXB〉
* 修改时间：2019 - 10 - 12
* 修改内容：
	1.增加网络设备ip地址配置接口 TJST3DSetNetdevIP
	2.增加标定板点识别参数接口
* 修改人：〈JXB〉
* 修改时间：2019 - 10 - 15
* 修改内容：
	1.增加点云分割接口 TJST3DPointcloudDivision
	2.增加TJST3DRangeNew 和 TJST3DRangeDelete 接口以配合 C#编程

* 修改人：〈JXB〉
* 修改时间：2019 - 11 - 06
* 修改内容：
	1.修改模块初始化接口 TJST3DModuleInit
	2.增加机械臂标定函数 TJST3DCalibrationRobot

* 修改人：〈JXB〉
* 修改时间：2019 - 11 - 12
* 修改内容：
	1.增加机械臂标定函数 TJST3DCalibrationRobot2

* 修改人：〈JXB〉
* 修改时间：2020 - 01 - 17
* 修改内容：
	1.增加点云使用Mask接口 TJST3DUseMaskImage
	2.增加点云上色接口	   TJST3DAddColorImage
	3.增加取得左相机最后一幅扫描图(一般为投影白色)，可用作MaskImage 和 ColorImage ，接口函数为 TJST3DGetLeftCameraLastTriggerImagebuff
	4.增加取得右相机最后一幅扫描图，TJST3DGetRightCameraLastTriggerImagebuff

* 修改人：〈JXB〉
* 修改时间：2020 - 04 - 23
* 修改内容：
	1. 增加点云计算处理器选择接口 TJST3DSetProcessorUnit

* 修改人：〈JXB〉
* 修改时间：2020 - 07 - 28
* 修改内容：
	1.增加新的模块初始化接口--当有连接多套设备时，打开其中一套设备接口 TJST3DModuleInitOneDev

* 修改人：〈JXB〉
* 修改时间：2020 - 08 - 05
* 修改内容：
	1.增加单目模式功能函数：
		1)TJST3DSetModuleType
		2)TJST3DLoadSingleStdData
		3)TJST3DSetWorkMode
	2.增加扫描模式使用G命令触发配置  TJST3DSetTriggerGray

* 修改人：〈JXB〉
* 修改时间：2020 - 10 - 12
* 修改内容：
	1.增加取得错误码接口 TJST3DGetLastErrorCode

* 修改人：〈JXB〉
* 修改时间：2020 - 10 - 29
* 修改内容：
	1.增加深度数据保存接口 TJST3DGetDepthdata

* 修改人：〈JXB〉
* 修改时间：2021 - 04 - 21
	* 修改内容：
	1.增加双单目标定合并接口

* 修改人：〈JXB〉
* 修改时间：2021 - 09 - 09
	* 修改内容：
	1.增加散斑模式点云生成接口 TJST3DCreatePointcloudRasterSpeckle
	2.增加散斑模式设备控制接口 TJST3DContinueScan

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
	TJST3DERR_CONNECT_PRJ,		//1.投影机连接失败
	TJST3DERR_CONNECT_CAM,		//2.相机连接失败
	TJST3DERR_TRIGGER,			//3.投影机无法触发
	TJST3DERR_IMAGE_NOTENOUGH,	//4.条纹图片数据不足
	TJST3DERR_OUTBOX,			//5.超出包围盒
	TJST3DERR_REMOVE_ERRPAR,	//6.异常的去杂点参数
	TJST3DERR_FUNC_PAR,			//7.错误的函数参数
	TJST3DERR_DEV_OPEN,			//8.设备没有打开
	TJST3DERR_FRINGEIMAGE,		//9.条纹图像质量差
	TJST3DERR_NUM
}TJST3DError_t;

//回调函数原型
typedef void(__stdcall * TJST3DRasterSpeckleCallback)(sn3DCore::sn3DRangeData* pPointcloud, unsigned char* pColorImagedata,bool bResult, TJST3DError_t errCode);


/*取得最近调用接口异常错误时的代码(用于原因分析)
返回:
	错误代码
*/
TJ3DSDK_API TJST3DError_t TJST3DGetLastErrorCode();

/*设置点云生成处理器,在TJST3DModuleInit 或者 TJST3DCalibCameraGroup 前调用
参数说明：
	nMode：0 使用CPU；1 使用GPU；其他自动选择
*/
TJ3DSDK_API void TJST3DSetProcessorUnit(int nMode);

/*设置模块类型
参数说明：
	nModuleType：0 双目模式；1 单目双相机；2 单目单左相机；3 单目单右相机；4 三相机模式
	(相机左右区分按照相对于投影机，人眼方向同投影方向)
*/

TJ3DSDK_API void TJST3DSetModuleType(int nModuleType=0);

/*单目模式设置工作状态
参数说明：
	nWorkMode：0 扫描模式；1标定模式
*/

TJ3DSDK_API bool TJST3DSetWorkMode(int nWorkMode = 0);

/*单目模式导入标准数据文件
参数说明：
	strFileName:文件全路径名
	nVideoWidth:相机横向分辨率
	nVideoHeight:相机纵向分辨率
	nFringeNum:点云生成时需要的条纹数量
返回:
	成功true
	失败false
*/
TJ3DSDK_API bool TJST3DLoadSingleStdDataA(const char* strPathName, int nVideoWidth = 1280, int nVideoHeight = 1024, int nFringeNum = 24);
TJ3DSDK_API bool TJST3DLoadSingleStdDataW(const wchar_t* strPathName, int nVideoWidth = 1280, int nVideoHeight = 1024, int nFringeNum = 24);

/*模块初始化
参数说明：
	nMode: 0 USB设备，1 网络设备
	nDevNum: 设备数量(套),大于0.(1套为1台投影机，2台相机)
返回:
	成功true
	失败false
*/
TJ3DSDK_API bool TJST3DModuleInit(IN int nMode, IN int nDevNum);

/*模块初始化--当有连接多套设备时，打开其中一套设备
参数说明：
	nMode: 0 USB设备，1 网络设备
	nDevIndex: 设备序号，从0开始
返回:
	成功true
	失败false
*/
TJ3DSDK_API bool TJST3DModuleInitOneDev(IN int nMode, IN int nDevIndex);

/*释放资源,关闭设备*/
TJ3DSDK_API void TJST3DModuleUninit();

/*判断模块初始化是否成功
返回:
	成功true
	失败false
*/
TJ3DSDK_API bool TJST3DModuleIsInitedOK();


TJ3DSDK_API bool TJST3DSetVideoImageColor(bool bRGBColor);

/*读取图像分辨率及像素位数：在没有初始化或初始化失败的情况下返回true，数值为模块默认的数值
参数说明：
	pWidth 图像宽度
	pHeight 图像高度
	pBitCount 像素位数：灰度图像 = 8，BRG图像=24
返回:
	成功true
	失败false
*/
TJ3DSDK_API bool TJST3DGetImageFormat(OUT int* pWidth, OUT int* pHeight, OUT int* pBitCount);

/*读取视频分辨率及像素位数：与TJST3DGetImageFormat 不同的是，没有初始化或初始化失败的情况下返回 false
参数说明：
	pWidth 图像宽度
	pHeight 图像高度
	pBitCount 像素位数：灰度图像 = 8，BRG图像=24
返回:
	成功true
	失败false
*/
TJ3DSDK_API bool TJST3DVideoFormat(OUT int* pWidth, OUT int* pHeight, OUT int* pBitCount);

/*设置相机预览窗口
输入参数：（C++中为HWND 类型）
	hwndLeft 左相机串口指针
	hwndRight右相机窗口指针。
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetDisplayHwnd(IN void* hwndLeft, IN void* hwndRight);

TJ3DSDK_API bool TJST3DSetDisplayHwndThird(IN void* hwndThird);

/*重设相机预览窗口大小(部分相机支持）*/
TJ3DSDK_API bool TJST3DResizeDisplayHwnd();

/*开始视频,进入采集模式
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DVideoStart();

/*暂停视频*/
TJ3DSDK_API void TJST3DVideoPause();

/*判断是否在采集模式
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DVideoIsGrabbing();

/*切换左右相机,切换相机后需要重新标定
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DExchangeLeftRightCam();

/*设置当前相机组增益
输入参数:
	fGainLeft 左相机增益值0-15
	fGainRight 右相机增益值0-15
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetGainFloat(IN float fGainLeft, IN float fGainRight);

/*设置指定相机组增益
输入参数:
	nGroup 相机组ID。如果只有一组相机，设为0
	fGainLeft 左相机增益值0-15
	fGainRight 右相机增益值0-15
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetGainFloatEx(IN int nGroup,IN float fGainLeft, IN float fGainRight);

TJ3DSDK_API bool TJST3DSetAutoGain(IN int nGroup,IN bool bAuto);

//设置曝光时间：所有打开的相机设置同一个参数 (只有特殊情况可使用，一般无效)
TJ3DSDK_API bool TJST3DSetExposureTime(IN float fExposureTime); 
TJ3DSDK_API bool TJST3DSetTriggerDelaye(IN float fTrDelay);

/*取得支持的扫描FPS
输出参数：
	pFps 所有支持的值
输入参数:
	nMaxNum pFps指针(数组)数量，一般设为5就够了
返回：
	成功 支持的数量;
	失败 0;
*/
TJ3DSDK_API int TJST3DGetSupportScanFps(OUT int* pFps, IN int nMaxNum);

/*设置扫描帧率:固定曝光时间
输入参数:
	nFps 所有支持的FPS值。130W相机:120,60,30 ; 600W相机： 30,15
	nTriggerNum 触发条纹数
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetScanFps(IN int nFps,IN int nTriggerNum);

TJ3DSDK_API bool TJST3DSetScanerPar(IN float fExposureTime, IN float fTriggerDelay, IN int nRepeat, IN int nTriggerNum, IN int nStart, IN int nStartImg);


/*进入扫描模式(打开默认)。扫描后自动进入此模式
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DEnterScanMode();
/*进入视频模式
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DEnterContinuousMode();

/*读取所需最小条纹数
返回：
	成功 所需条纹数,正整数;
	失败 0;
*/
TJ3DSDK_API int TJST3DGetMinTriggerImageNum();

/*设置触发条纹数
输入参数:
	nTriggers 条纹数量。nTriggers>=12，所有打开的相机设置同一个参数
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetTriggerImageNum(int nTriggers);

/*设置使用G命令带灰度触发
输入参数：
	byteGray： 255 使用T 命令；其他 使用G xxx命令，xxx代表最后一张白屏灰度值
*/
TJ3DSDK_API void TJST3DSetTriggerGray(unsigned char byteGray);

/*保存所有的触发图像数据到指定目录
输入参数:
	strPath 指定的目录路径
	nCameraGroup 相机组ID。如果只有一组相机，设为0
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSaveTriggerImagesA(IN const char* strPath, IN int nCameraGroup=0);
TJ3DSDK_API bool TJST3DSaveTriggerImagesW(IN const wchar_t* strPath,IN int nCameraGroup=0);
TJ3DSDK_API bool TJST3DSaveTriggerImagesExA(IN const char* strPath, IN int nCameraGroup);

/*从左右相机中各抓取1幅图:TJST3DCatchImage
注意：
	只能在视频模式中抓图
输出参数：
	pLeftImgbuf 左相机图像内存指针，用户分配和释放内存
	pRightImgbuf 右相机图像内存指针，用户分配和释放内存
输入参数：
	nMaxBufsize 用户分配的内存的字节大小.如果是彩色相机，最新内存大小=视频长x视频宽x3
返回：
	0 失败。
	单幅图像数据字节数
*/
TJ3DSDK_API unsigned int TJST3DCatchImage(OUT unsigned char* pLeftImgbuf, OUT unsigned char* pRightImgbuf, IN unsigned int nMaxBufsize);
TJ3DSDK_API unsigned int TJST3DCatchImageEx(OUT unsigned char* pLeftImgbuf, OUT unsigned char* pRightImgbuf, IN unsigned int nMaxBufsize,IN bool bRGB);
TJ3DSDK_API unsigned int TJST3DCatchImage3Ex(OUT unsigned char* pLeftImgbuf, OUT unsigned char* pRightImgbuf, OUT unsigned char* pThirdImgbuf, IN unsigned int nMaxBufsize, IN bool bRGB);

TJ3DSDK_API bool TJST3DCatchLeftCameraImage(OUT unsigned char* pLeftImgbuf, IN unsigned int nMaxBufsize);
TJ3DSDK_API bool TJST3DCatchRightCameraImage(OUT unsigned char* pRightImgbuf, IN unsigned int nMaxBufsize);
/*保存图像数据到bmp图片
输出参数：
	strFileFullname 文件全路径(带扩展名.bmp)
	pImageBuf 图像指针
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSaveBmpImageA(IN const char* strFileFullname, IN unsigned char* pImageBuf, IN int bitCount = 8);
TJ3DSDK_API bool TJST3DSaveBmpImageW(IN const wchar_t* strFileFullname, IN unsigned char* pImageBuf, IN int bitCount = 8);
TJ3DSDK_API bool TJST3DSaveBmpImageRotate90A(IN const char* strFileFullname, IN unsigned char* pImageBuf, IN int bitCount = 8);

/*取得标定板圆点二维坐标:TJST3DGetCalibImage2DPoints
输入参数：
	pImageCalibLeft 左图像内存指针
	pImageCalibRight 右图像内存指针
输出参数：
	fXLeft 左图像圆点X坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
	fYLeft 左图像圆点Y坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
	fXRight 右图像圆点X坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
	fYRight 右图像圆点Y坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DGetCalibImage2DPoints(unsigned char* pImageCalibLeft, unsigned char* pImageCalibRight,
	double* fXLeft, double* fYLeft, double* fXRight, double* fYRight);

/*取得标定板圆点二维坐标(单目模式):TJST3DGetCalibImage2DPointsSingle
输入参数：
	mode 0 左相机；1 右相机
	pImageCalib 图像内存指针
	pImagePhaseArr 相机采集的横竖条纹图	大小为	24*nImageW*nImageH
输出参数：
	fXLeft 左图像圆点X坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
	fYLeft 左图像圆点Y坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
	fXRight 右图像圆点X坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
	fYRight 右图像圆点Y坐标数组，找不到的点用-1代替，用户分配和释放内存 17*14
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DGetCalibImage2DPointsSingle(int mode, unsigned char* pImageCalib, unsigned char* pImagePhaseArr,
	double* fXLeft, double* fYLeft, double* fXRight, double* fYRight);

/*标定参数设置:TJST3DSetCalibParameter
输入参数：
	fFocusLeft 左相机镜头焦距，单位mm
	fPixelSizeLeft 左相机像素尺寸，单位mm
	fFocusLeft 右相机镜头焦距，单位mm
	fPixelSizeLeft 右相机像素尺寸，单位mm
	fSizeX 标定板横向所有点间距总和，单位mm
	fSizeY 标定板纵向所有点间距总和，单位mm
*/
TJ3DSDK_API void TJST3DSetCalibParameter(double fFocusLeft, double fPixelSizeLeft, double fFocusRight, double fPixelSizeRight, double fSizeX, double fSizeY);
/*光机参数设置(单目算法使用):TJST3DSetLightProjectioParameter
输入参数：
	nImageW：光机横向分辨率
	nImageH：光机纵向分辨率
	fFocus：光机焦距，单位mm
	fPixelSize：光机像素尺寸，单位mm
*/
TJ3DSDK_API void TJST3DSetLightProjectioParameter(int nGroupId,int nImageW, int nImageH, double fFocus, double fPixelSize);

/*相机组标定:TJST3DCalibCameraGroup
输入参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	nImageNum 参与标定的图像数量
	nImageW 图像宽度
	nImageH 图像高度
	pLeftImageArr 左相机所有标定图像内存数据，长度与nImageNum参数匹配
	pRightImageArr 右相机所有标定图像内存数据，长度与nImageNum参数匹配
	ccfLeft 左相机标定参数文件名
	ccfRight 右相机标定参数文件名
	table bable文件名
	box box文件名
输出参数：
	fCalibErr 用户分配内存 输出标定误差(为0时也是标定成功)
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DCalibCameraGroupA(int nCameraGroupID, int nImageNum, int nImageW, int nImageH,unsigned char* pLeftImageArr, unsigned char* pRightImageArr, double* fCalibErr,
	const char* ccfLeft, const char* ccfRight, const char* table, const char* box);

TJ3DSDK_API bool TJST3DCalibCameraGroupW(int nCameraGroupID, int nImageNum, int nImageW, int nImageH,unsigned char* pLeftImageArr, unsigned char* pRightImageArr, double* fCalibErr,
	const wchar_t* ccfLeft, const wchar_t* ccfRight, const wchar_t* table, const wchar_t* box);

/*相机标定(单目模式):TJST3DCalibCameraGroupSingle
输入参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	nMode 0 左相机;1 右相机
	nImageNum 参与标定的图像数量
	nImageW 图像宽度
	nImageH 图像高度
	pImageArr 相机采集白屏图数据内存
	pImagePhaseArr 相机采集的横竖条纹图数据内存指针
	ccfLeft 左相机标定参数文件名
	ccfRight 右相机标定参数文件名
	table bable文件名
	box box文件名
输出参数：
	fCalibErr 用户分配内存 输出标定误差(为0时也是标定成功)
返回值：
	true 成功
	false 失败
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


/*增加相机组标定参数:TJST3DAddCalibCameraGroup
输入参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	nImageW 图像宽度
	nImageH 图像高度
	ccfLeft 左相机标定参数文件名
	ccfRight 右相机标定参数文件名
	table bable文件名
	box box文件名
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DAddCalibCameraGroupA(int nCameraGroupID, int nImageW, int nImageH, const char* ccfLeft, const char* ccfRight, const char* table, const char* box);
TJ3DSDK_API bool TJST3DAddCalibCameraGroupW(int nCameraGroupID, int nImageW, int nImageH, const wchar_t* ccfLeft, const wchar_t* ccfRight, const wchar_t* table, const wchar_t* box);

/*修改包围盒信息
输入参数：
	nCameraGroupID 相机组识别号
	minx,miny,minz 表示一个点
	maxx,maxy,maxz 表示另一个点
	box   保存路径
注意：
	计算点云是，当点坐标P(x,y,z)满足minx<x<maxx,miny<y<maxy,minz<z<maxz时点才会保留
	需要调用AddCalibCameraGroup以后才能修改对应相机组的box信息
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DSetBoxA(int nCameraGroupID, float minx, float miny, float minz, float maxx, float maxy, float maxz, const char* box);
TJ3DSDK_API bool TJST3DSetBoxW(int nCameraGroupID, float minx, float miny, float minz, float maxx, float maxy, float maxz, const wchar_t* box);

TJ3DSDK_API bool TJST3DReadBoxW(const wchar_t* boxFile, double x[], double y[], double z[]);
TJ3DSDK_API bool TJST3DReadBoxA(const char* boxFile, double x[], double y[], double z[]);

TJ3DSDK_API unsigned int TJST3DGetImageBuffpoints(unsigned int nStart,int nImages, unsigned char** ppLeft, unsigned char** ppRight, bool brgb);
TJ3DSDK_API unsigned int TJST3DGetImageBuffpointsEx(int nGroup,unsigned int nStart, int nImages, unsigned char** ppLeft, unsigned char** ppRight, bool brgb);
TJ3DSDK_API unsigned int TJST3DGetImageBuffpoints3C(unsigned int nStart, int nImages, unsigned char** ppLeft, unsigned char** ppRight, unsigned char** ppThird, bool brgb);
/*设置投影条纹类型
输入参数：
	nRasterType 条纹类型：0 一般条纹(默认值)； 1 适配高分辨率相机；2 格雷码
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DSetRasterType(int nRasterType);

/*设置是否给点云添加贴图(颜色) 
输入参数：
	bColor true 增加颜色（最后一张条纹图），生成点云是自动上色；false 不增加（默认）
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DSetColorPoints(bool bColor);

/*给点云添加贴图(颜色)
输入参数：
	pPointcloud 生成的点云指针
	pColorImagedata BGR(888)真彩色图像数据指针,分辨率必须与生成点云时扫描相机的分辨率一致
返回值：
	无
*/
TJ3DSDK_API void TJST3DAddColorImage(sn3DCore::sn3DRangeData* pPointcloud, unsigned char* pColorImagedata);

/*设置mask参数
*/
TJ3DSDK_API void TJST3DSetMinMaxMaskth(int nMinMaskth, int nMaxMaskth);

/*使用MaskImage删除点云数据
输入参数：
	pPointcloud 生成的点云指针
	pMaskImagedata 8位灰度图像数据指针,分辨率必须与生成点云时扫描相机的分辨率一致
	nMinMaskth 小于此灰度值所对应的点云数据删除
	nMaxMaskth 大于此灰度值所对应的点云数据删除
返回值：
	无
说明：
	如果 nMinMaskth=0，nMaxMaskth=255，那就是无意义的操作
	如果 nMinMaskth=1，nMaxMaskth=1，那就只保留灰度值为1所对应的点云数据
*/
TJ3DSDK_API void TJST3DUseMaskImage(sn3DCore::sn3DRangeData* pPointcloud, unsigned char* pMaskImagedata, unsigned char nMinMaskth, unsigned char nMaxMaskth);

/*取得左相机最后一幅扫描图(一般为投影白色)，可用作MaskImage 和 ColorImage
输入参数：
	bColor 为true,返回BRG(888)数据；为false,返回8位灰度数据
返回值：
	成功：图像数据指针，不需要用户释放。在下一次扫描之前都可以使用
	失败：NULL
说明：
	如果我们的相机为黑白相机，参数使用bColor=true，那返回的RGB数值是一样的
*/
TJ3DSDK_API unsigned char* TJST3DGetLeftCameraLastTriggerImagebuff(bool bColor = false);

/*取得右相机最后一幅扫描图(一般为投影白色)
输入参数：
	bColor 为true,返回BRG(888)数据；为false,返回8位灰度数据
返回值：
	成功：图像数据指针，不需要用户释放。在下一次扫描之前都可以使用
	失败：NULL
*/
TJ3DSDK_API unsigned char* TJST3DGetRightCameraLastTriggerImagebuff(bool bColor = false);


/*创建sn3DRangeData类指针
返回值：
	非空指针  成功
	NULL	  失败
*/
TJ3DSDK_API sn3DCore::sn3DRangeData* TJST3DRangeNew();

/*释放sn3DRangeData类指针
输入参数：
	pRangedata 确保为 sn3DCore::sn3DRangeData* 类型有效指针(非空，以前没有释放过)
*/
TJ3DSDK_API void TJST3DRangeDelete(sn3DCore::sn3DRangeData* pRangedata);

/*返回法向数量
输入参数：
	pRangedata 确保为 sn3DCore::sn3DRangeData* 类型有效指针
*/
TJ3DSDK_API unsigned int TJST3DGetRangeNors(sn3DCore::sn3DRangeData* pRangedata);

/*返回点云数量
输入参数：
	pRangedata 确保为 sn3DCore::sn3DRangeData* 类型有效指针
*/
TJ3DSDK_API unsigned int TJST3DGetRangePoints(sn3DCore::sn3DRangeData* pRangedata);

/*把点云数据结构指针中的三维点转换为浮点数组*/
/*
输入参数：
	pRangeData: 点云数据结构指针
	pPointArrayX：数组指针，存放三维点X值，由用户申请和释放内存
	pPointArrayY：数组指针，存放三维点Y值，由用户申请和释放内存
	pPointArrayZ：数组指针，存放三维点Z值，由用户申请和释放内存
	nArraySize：数据长度。长度最低要求：三维点数量
返回：
	实际转换的三维点数
*/
TJ3DSDK_API unsigned int TJST3DConvertRangePointToFloatArray(sn3DCore::sn3DRangeData* pRangeData, float* pPointArrayX, float* pPointArrayY, float* pPointArrayZ, unsigned int nArraySize);

/*把点云数据结构指针中的三维法向转换为浮点数组*/
/*
输入参数：
	pRangeData: 点云数据结构指针
	pNorArrayX：数组指针，存放三维法向X值，由用户申请和释放内存
	pNorArrayY：数组指针，存放三维法向Y值，由用户申请和释放内存
	pNorArrayZ：数组指针，存放三维法向Z值，由用户申请和释放内存
	nArraySize：数据长度。长度最低要求：三维点数量
返回：
	实际转换的三维点数
*/
TJ3DSDK_API unsigned int TJST3DConvertRangeNorToFloatArray(sn3DCore::sn3DRangeData* pRangeData, float* pNorArrayX, float* pNorArrayY, float* pNorArrayZ, unsigned int nArraySize);

/*返回三维点法向浮点值
输入参数：
	pRangedata 确保为 sn3DCore::sn3DRangeData* 类型有效指针
	nIndex 点序列号，确保数值不超范围
	xyz  0，代表x，1代表y，2 代表z。 其他无效
返回：
	正确返回 指定序号的扫描值
	错误返回 固定值 -0.1234
*/
TJ3DSDK_API float TJST3DGeNorDataFromRangedata(sn3DCore::sn3DRangeData* pRangedata, unsigned int nIndex, unsigned int xyz);


/*返回三维点浮点值
输入参数：
	pRangedata 确保为 sn3DCore::sn3DRangeData* 类型有效指针
	nIndex 点序列号，确保数值不超范围
	xyz  0，代表x，1代表y，2 代表z。 其他无效
返回：
	正确返回 指定序号的扫描值
	错误返回 固定值 -0.1234
*/
TJ3DSDK_API float TJST3DGetXYZDataFromRangedata(sn3DCore::sn3DRangeData* pRangedata, unsigned int nIndex, unsigned int xyz);

/**/
TJ3DSDK_API bool TJST3DGetTriggerBuffInfo(unsigned char** pLeft, unsigned char** pRight, int* pTrnum, int nOffset=0, bool bColor=false, int nGray=-1);
/*连续触发模式*/
TJ3DSDK_API bool TJST3DStartContinueTrigger(unsigned int nMaxImageNum, unsigned char* pLeftImageBuff, unsigned char* pRightImageBuff, unsigned int nMaxMemSize);
TJ3DSDK_API unsigned int TJST3DStopContinueTrigger();
/*根据图像数据生成点云：TJST3DCreatePointcloudFromImageMemery
输入参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	nImageNum 参与计算的条纹图像数量
	nImageW 图像宽度
	nImageH 图像高度
	pLeftImageArr 左相机所有条纹图像内存数据，长度与nImageNum参数匹配
	pRightImageArr 右相机所有条纹图像内存数据，长度与nImageNum参数匹配
	pPointcloud 生成的点云类，由用户申请和释放内存
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageMemery(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageMemeryNoProcess(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud);

TJ3DSDK_API bool TJST3DCreatePointcloudFromImageMemeryColor(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageGrayCode(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, sn3DCore::sn3DRangeData* pPointcloud,bool bColor);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImageGrayCodeP(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char* pLeftImageArr, unsigned char* pRightImageArr, unsigned char* pMaskImage, unsigned char* pColorImage, sn3DCore::sn3DRangeData* pPointcloud, bool bColor,int nMode);
TJ3DSDK_API bool TJST3DCreatePointcloudFromImage3CAll(unsigned char *color_image, unsigned char *imageL, unsigned char *imageT, unsigned char *imageR, sn3DCore::sn3DRangeData *p, sn3DCore::sn3DRangeData *p0, sn3DCore::sn3DRangeData *p1, sn3DCore::sn3DRangeData *p2, unsigned char *Mask,unsigned char *Color);
/*实时扫描生成点云：TJST3DCreatePointcloudFromScan
输入参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	pPointcloud 生成的点云类，由用户申请和释放内存
	bCreate 是否创建点云。true 创建点云；false 测试硬件扫描，不创建点云数据
	hEvent 如果为非空指针，扫描完成(不是点云创建完成,仅采集到足够的条纹数据)时会调用SetEvent(hEvent)
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DCreatePointcloudFromScan(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud,bool bCreate=true,void* hEvent=0);

/*散斑条纹连续扫描
参数：
	bEnable： true,开始扫描；false 停止扫描
	callBackfunc：点云生成后的回调函数
*/
TJ3DSDK_API bool TJST3DContinueScan(bool bEnable, TJST3DRasterSpeckleCallback callBackfunc = NULL);

/*散斑单次扫描
参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	pPointcloud 生成的点云类，由用户申请和释放内存
	bCreate 是否创建点云。true 创建点云；false 测试硬件扫描，不创建点云数据
*/
TJ3DSDK_API bool TJST3DCreatePointcloudRasterSpeckle(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud,bool bCreate=true);

/*多次扫描(采集不同亮度扫描图)生成点云:TJST3DCreatePointcloudFromMultScan
输入参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	pPointcloud 生成的点云类，由用户申请和释放内存
	nScanNums 扫描次数，范围：2--4
	nScanFps 扫描FPS指针(数组),可通过 TJST3DGetSupportScanFps 函数来取得支持的扫描FPS数据
	fGain    相机增益指针(数组)，值的范围为0.1-15.0
	nLedLight LED亮度指针(数组)，值的范围为10-255
	nMinMaskth minMaskth指针(数组)，值的范围为0-255
	nMaxMaskth maxMaskth指针(数组)，值的范围为0-255
	bCreate 是否创建点云。true 创建点云；false 测试硬件扫描，不创建点云数据
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DCreatePointcloudFromMultScan(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud, int nScanNums, int* nScanFps, float* fGain, int* nLedLight, int* nMinMaskth, int*nMaxMaskth, bool bCreate=true);
TJ3DSDK_API bool TJST3DCreatePointcloudFromMultScanNoProcess(int nCameraGroupID, sn3DCore::sn3DRangeData* pPointcloud, int nScanNums, int* nScanFps, float* fGain, int* nLedLight, int* nMinMaskth, int*nMaxMaskth, bool bCreate = true);

/*ICP自动拼接
输入参数：
	nNums 需要合并的点云数量
	pPointclouds 需要合并的点云指针
	fSearchRadFac 搜索半径
	iSampleSize 点数
	iniFile 参数配置文件
输出参数：
	diff 为全局平均误差
	rot 4x4转换矩阵 
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DMatchPointcloudA(int nNums, sn3DCore::sn3DRangeData* pPointclouds[], float fSearchRadFac, int iSampleSize, float *rot, float &diff, const char* iniFile);
TJ3DSDK_API bool TJST3DMatchPointcloudW(int nNums, sn3DCore::sn3DRangeData* pPointclouds[], float fSearchRadFac, int iSampleSize, float *rot, float &diff, const wchar_t* iniFile);
/*合并点云(三维点和法向)
输入参数：
	nNums 需要合并的点云数量
	pPointclouds 需要合并的点云指针
输出参数：
	pPointcloud_out 合并后的点云指针(由用户申请和释放内存)
返回值:
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DMergePointcloud(int nNums, sn3DCore::sn3DRangeData* pPointclouds[], sn3DCore::sn3DRangeData* pPointcloud_out);

/*点云分割
输入参数：
	pInput 需要分割的点云
	px py pz 3个点坐标，用于确定分割平面
输出参数：
	pOut1 分割后的点云1 点云指针(由用户申请和释放内存)
	pOut2 分割后的点云2 点云指针(由用户申请和释放内存)
返回值:
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DPointcloudDivision(const sn3DCore::sn3DRangeData* pInput, double px[3], double py[3], double pz[3], sn3DCore::sn3DRangeData* pOut1, sn3DCore::sn3DRangeData* pOut2);

/*对点云进行空间采样
输入参数：
	pointcloud_in 原始点云
	fSampleRadius 采样半径
输出参数:
	pointcloud_out 采样后点云
返回值:
	true 成功
	false 失败
*/
TJ3DSDK_API bool  TJST3DPointcloudSpaceSampling(sn3DCore::sn3DRangeData* pointcloud_in, float fSampleRadius,sn3DCore::sn3DRangeData *pointcloud_out);


/*生成网格(MESH)
输入参数：
	pointcloud_in 原始点云
	strParfile 参数文件全路径(含文件名)
输出参数：
	mesh_out mesh指针
返回值:
	true 成功
	false 失败
*/
TJ3DSDK_API bool  TJST3DPointcloudToMeshA(sn3DCore::sn3DRangeData* pointcloud_in,const char* strParfile, sn3DCore::sn3DRangeData *mesh_out);
TJ3DSDK_API bool  TJST3DPointcloudToMeshW(sn3DCore::sn3DRangeData* pointcloud_in, const wchar_t* strParfile, sn3DCore::sn3DRangeData *mesh_out);

/*设置去除杂点方法：TJST3DSetRemoveOutliersMethod
输入参数：
	nMethod 0 表示不计算，1 方式1计算 ，2 方式2计算(多线程加速)
*/
TJ3DSDK_API void TJST3DSetRemoveOutliersMethod(int nMethod);

/*设置去除杂点参数：TJST3DSetRemoveOutliersPar
输入参数：
	fGrowRad：半径
	nMinNumPts:半径内最少点数
*/
TJ3DSDK_API void TJST3DSetRemoveOutliersPar(double fGrowRad, int nMinNumPts);

/*保存点云文件：TJST3DSavePointcloudTofile
输入参数：
	pPointcloud 生成的点云类指针
	strFileFullname 点云文件全路径,支持.txt .ply
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DSavePointcloudTofileA(sn3DCore::sn3DRangeData* pPointcloud, const char* strFileFullname);
TJ3DSDK_API bool TJST3DSavePointcloudTofileW(sn3DCore::sn3DRangeData* pPointcloud, const wchar_t* strFileFullname);

/*打开点云文件：TJST3DLoadPointcloudFromfile
输入参数：
	pPointcloud 生成的点云类指针,由用户申请和释放内存
	strFileFullname 点云文件全路径,支持.txt .ply
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DLoadPointcloudFromfileA(sn3DCore::sn3DRangeData* pPointcloud, const char* strFileFullname);
TJ3DSDK_API bool TJST3DLoadPointcloudFromfileW(sn3DCore::sn3DRangeData* pPointcloud, const wchar_t* strFileFullname);

/*显示点云窗口：TJST3DCreate3DViewer
输入参数：
	strName 显示窗口的名称(暂时无效)
*/
TJ3DSDK_API void TJST3DCreate3DViewerA(const char* strName=NULL);
TJ3DSDK_API void TJST3DCreate3DViewerW(const wchar_t* strName=NULL);

/*关闭点云窗口：TJST3DCloseViewer*/
TJ3DSDK_API void TJST3DCloseViewer();

/*显示生成的点云：TJST3DShowPoint
输入参数:
	pPointcloud 生成的点云类指针
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DShowPoint(sn3DCore::sn3DRangeData * pPointcloud);

/*设置视点
输入参数:
	pos为摄像头的坐标
	view为摄像头观测方向
	up为摄像头的正上方坐标
说明：
	默认设置为观测z轴正方向   y轴正方向为上方
*/
TJ3DSDK_API void TJST3DSetViewPoint(double posX, double posY, double posZ, double viewX = 0.0, double viewY = 0.0, double viewZ = 1.0, double upX = 0.0, double upY = 1.0, double upZ = 0.0);

/*设置颜色渐变的范围：TJST3DSetSizeColorShow
输入参数:
	zmin Z坐标较小值
	zmax Z坐标较大值
说明：
	将点云再z轴方向渐变
	当zmin=zmax时，自动用点云的规模渐变。
	超出范围的点云显示为白色
*/
TJ3DSDK_API void TJST3DSetSizeColorShow(float zmin, float zmax);

/*切换当前相机组：TJST3DSelectCameraGroup
输入参数:
	nCamGroup 相机组序号,从0开始
返回值：
	true 成功
	false 失败
*/
TJ3DSDK_API bool TJST3DSelectCameraGroup(int nCamGroup);

/*读取当前相机组
返回值：
	相机组ID。如果只有一套相机，始终返回0
*/
TJ3DSDK_API int TJST3DGetCameraGroup();


/*设置投影模式
输入参数:
	nPrjMode 0 黑屏 1 白屏 2 十字 3 棋盘格
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetProject(IN int nPrjMode);


/*设置投影模式,指定相机组
输入参数:
	nCamGroup	相机组ID
	nPrjMode 0 黑屏 1 白屏 2 十字 3 棋盘格
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetProjectEx(IN int nCamGroup ,IN int nPrjMode);

/*设置投影亮度,断电后不会生效
输入参数:
	nLight 投影亮度值（10-255），常时间在高亮度运行时注意散热
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetLedLight(IN unsigned char nLight);

/*保存投影亮度设置，断电后会生效
输入参数:
	nLight 投影亮度值（10-255），常时间在高亮度运行时注意散热
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSaveLedLight(IN unsigned char nLight);

/*打开LED*/
TJ3DSDK_API void TJST3DLedOn();
/*关闭LED*/
TJ3DSDK_API void TJST3DLedOff();

/*设置投影亮度,断电后不会生效
输入参数:
	nColor 投影颜色,0 1 2 3 代表红 绿 蓝 白
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetLedColor(IN unsigned char nColor);

/*设置最后一张投影灰度(非条纹)
输入参数:
	nGray 0-255灰度值
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DSetPrjGray(IN unsigned char nGray);

/*标定转轴
输入参数:
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	nImageNum 参与计算的条纹图像数量
	nImageW 图像宽度
	nImageH 图像高度
	ImageL 左相机所有标定图像内存数据，长度与nImageNum参数匹配
	Imager 右相机所有标定图像内存数据，长度与nImageNum参数匹配
	saveName 保存的轴点路径
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DCalibrateAxisA(IN int nCamGroup, IN int nImageNum, IN int nImageW, IN int nImageH, IN unsigned char* ImageL, IN unsigned char* ImageR, IN OUT const char* saveName);
TJ3DSDK_API bool TJST3DCalibrateAxisW(IN int nCamGroup, IN int nImageNum,IN int nImageW, IN int nImageH ,IN unsigned char* ImageL, IN unsigned char* ImageR,IN OUT const wchar_t* saveName);

TJ3DSDK_API bool TJST3DCalibrateAxisExA(IN int nCamGroup, IN int nImageNum, IN int nImageW, IN int nImageH, IN unsigned char* ImageL, IN unsigned char* ImageR, IN float *Angle ,IN OUT const char* saveName);
TJ3DSDK_API bool TJST3DCalibrateAxisExW(IN int nCamGroup, IN int nImageNum, IN int nImageW, IN int nImageH, IN unsigned char* ImageL, IN unsigned char* ImageR, IN float *Angle ,IN OUT const wchar_t* saveName);

/*导入轴点数据
输入参数:
	pXAxisFilename 水平转轴轴点文件
	pYAxisFilename 垂直转轴轴点文件
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DLoadAxisCalibratePointsA(IN const char* pXAxisFilename, IN const char* pYAxisFilename);
TJ3DSDK_API bool TJST3DLoadAxisCalibratePointsW(IN const wchar_t* pXAxisFilename, IN const wchar_t* pYAxisFilename);

TJ3DSDK_API bool TJST3DLoadAxis3CalibratePointsA(IN const char* pXAxisFilename, IN const char* pYAxisFilename, IN const char* pZAxisFilename);
TJ3DSDK_API bool TJST3DLoadAxis3CalibratePointsW(IN const wchar_t* pXAxisFilename, IN const wchar_t* pYAxisFilename, IN const wchar_t* pZAxisFilename);

/*根据旋转的角度得到4x4刚体变换矩阵
输入参数:      
	xAngle 水平转轴旋转角度(-360--+360)
	yAngle 垂直转轴旋转角度(-360--+360)
输出参数：
	pRTMat 4x4刚体变换矩阵 由用户申请和释放内存
返回：
	成功 true;
	失败 false;
C/C++调用举例：
	{
		float RTMat [16];
		TJST3DAnglexToRotateTrans(60,80,RTMat);
	}	
*/
TJ3DSDK_API bool TJST3DAnglesToRotateTrans(IN float xAngle, IN float yAngle,OUT float* pRTMat);
TJ3DSDK_API bool TJST3DAnglesAxis3ToRotateTrans(IN float xAngle, IN float yAngle, IN float zAngle, OUT float* pRTMat);


//设置需要打开的网络相机IPv4地址
TJ3DSDK_API bool TJST3DSetNetdevIP(int nGroupIndex, const char* prjIP, const char* leftCamIP = NULL, const char* rightCamIP = NULL);
//设置标定板圆点识别参数
TJ3DSDK_API void TJST3DSetCalibImage2DPoints(double sigma, double tlow, double thigh, double pointQuality, double circularity, int minDiameter, int maxDiameter);
//清除标志点
TJ3DSDK_API void TJST3DClearMarkPoints();
/*从标志点图像数据得到旋转平移矩阵*/
TJ3DSDK_API bool TJST3DMarkPointsAlign(int nCameraGroupID, int nImageW, int nImageH, unsigned char* pImageMarkLeft, unsigned char* pImageMarkRight, double* fRotateTrans);
/*从扫描标志点图像数据(内存)得到旋转平移矩阵*/
TJ3DSDK_API bool TJST3DMarkPointsAlignEx(int nCameraGroupID,double* fRotateTrans);

TJ3DSDK_API bool TJST3DSaveTextureFileW(const wchar_t* filePathname);

/*初始化 工件识别模块 */
TJ3DSDK_API bool TJST3DWorkpieceModuleInitA(const char* strCadfile, int nDetectpoint, const char* strDetectpointFile[]);
TJ3DSDK_API bool TJST3DWorkpieceModuleInitW(const wchar_t* strCadfile, int nDetectpoint, const wchar_t* strDetectpointFile[]);
/*生成粗拼数据文件*/
TJ3DSDK_API unsigned int TJST3DCreateCoarseSstitchingA(const sn3DRangeData* pointcloud, unsigned int lastResult,const char* strCoarseFileUp, const char* strCoarseFileDown,const char* strDebugOut1=NULL, const char* strDebugOut2 = NULL);
TJ3DSDK_API unsigned int TJST3DCreateCoarseSstitchingW(const sn3DRangeData* pointcloud, unsigned int lastResult,const wchar_t* strCoarseFileUp, const wchar_t* strCoarseFileDown, const wchar_t* strDebugOut1 =NULL, const wchar_t* strDebugOut2 = NULL);
/*导入粗拼数据文件*/
TJ3DSDK_API bool TJST3DLoadCoarseSstitchingA(const char* strCoarseFileUp, const char* strCoarseFileDown);
TJ3DSDK_API bool TJST3LoadCoarseSstitchingW(const wchar_t* strCoarseFileUp, const wchar_t* strCoarseFileDown);
/*工件识别计算*/
TJ3DSDK_API unsigned int TJST3DWorkpieceModuleCalResult(const sn3DRangeData* pointcloud);

/*扫描时计算深度信息
输入参数：
	nUseDepth: 0 不计算，1 只计算Z，2计算XYZ
*/
TJ3DSDK_API void TJST3DSetCalDepthdata(int nUseDepth);

/*保存深度信息
输入参数：
	strFullPath：文件名
	bTxt：		true 保存文本格式；false 保存二进制格式
返回：
	成功:true
	失败:false
*/
TJ3DSDK_API bool TJST3DSaveDepthdataA(const char* strFullPath, bool bTxt=false);
TJ3DSDK_API bool TJST3DSaveDepthdataW(const wchar_t* strFullPath,bool bTxt=false);

/*取得深度数据指针（单精度浮点型）
返回：
	成功:数据指针地址。当使用深度模式1时，返回深度Z值(长度为图像长乘宽); 当使用深度模式2时，数据按XYZXYZ......XYZ排列(长度为图像长乘宽乘3)
	失败:NULL(0)
*/
TJ3DSDK_API const float* TJST3DGetDepthdata();
/*机器人模块*/

/*转换点云坐标系
输入参数：
	nCameraGroupID 相机组识别号，需要区分多组相机。nCameraGroupID=0，为主相机组
	pPointcloud 点云指针
	nMode 类型参数。说明：Mode = 1 原点变换到拍摄的第一幅标定板位置平面;  Mode = 2  原点(0,0,0)变换到左相机中心;
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DChangeCoordinate(IN int nCamGroup, IN OUT sn3DCore::sn3DRangeData* pPointcloud, IN int nMode);
/*机械手标定*/
TJ3DSDK_API bool TJST3DCalibrationRobot(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char * pLeftImageArr, unsigned char * pRightImageArr, float RobotPoint[][6], float * MatRT);

TJ3DSDK_API bool TJST3DCalibrationRobot2(int nCameraGroupID, int nPointNum, float PointL[][2], float PointR[][2], float RobotPoint[][3], float *MatRT);

TJ3DSDK_API bool TJST3DCalibrationRobot3A(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char * pLeftImageArr, unsigned char * pRightImageArr,float RobotPoint[][6],const char* pstrCalibFile, int mode);
TJ3DSDK_API bool TJST3DCalibrationRobot3W(int nCameraGroupID, int nImageNum, int nImageW, int nImageH, unsigned char * pLeftImageArr, unsigned char * pRightImageArr, float RobotPoint[][6], const wchar_t* pstrCalibFile, int mode);

/*点云转换标定板坐标系到机器人坐标系
输入参数：
	pointcloudin 标定板坐标系点云
	SaveFileName 机器人标定文件全路径
	mode 机器人工作方式。0 眼在手外；1 眼在手上 
	RobotPoint 机器人当前坐标。当mode=1 时有效
输出参数：
	pointcloudout 机器人坐标系点云
返回：
	成功 true;
	失败 false;
*/
TJ3DSDK_API bool TJST3DChangePointCloudToRobotA(sn3DRangeData *pointcloudin, sn3DRangeData *pointcloudout, const char* SaveFileName, int mode = 0, float RobotPoint[6] = NULL);
TJ3DSDK_API bool TJST3DChangePointCloudToRobotW(sn3DRangeData *pointcloudin, sn3DRangeData *pointcloudout, const wchar_t* SaveFileName, int mode = 0, float RobotPoint[6] = NULL);

/*绕坐标轴旋转*/
TJ3DSDK_API void TJST3DGetTransformMat(double fXAngle, double fYAngle, double fZAngle, float* pRT);
TJ3DSDK_API void TJST3DRangedataTransformMat(sn3DCore::sn3DRangeData* pPointcloud, float* pRT);
//用于测试，开发者不要调用，后果自负
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

//FlexMatch模块
/*相机(3)标定 FLEXMATCHCalibCamera
输入参数：
	nImageNum 标定图数量
	nImageW	  标定图横向分辨率
	nImageH	  标定图纵向分辨率
	pImgL     左相机标定图内存数据,8位灰度。内存空间大小为 nImageW*nImageH*nImageNum
	pImgM     中相机标定图内存数据,8位灰度。内存空间大小为 nImageW*nImageH*nImageNum
	pImgR     右相机标定图内存数据,8位灰度。内存空间大小为 nImageW*nImageH*nImageNum

	ccfLeft,ccfRight,table,box 输出的一组标定结果文件，共三组
返回：
	true 成功
	false 失败
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
输入参数：
	PointcloudSor 点云类指针
	ImageSor 二维图像数据指针
输出参数
	Tr 用SDK申请内存，用户释放
返回：
	数量 成功
	0 失败
*/
TJ3DSDK_API int FLEXMATCHMatchUseImage(sn3DCore::sn3DRangeData *PointcloudSor, unsigned char *ImageSor, float** Tr);

/*FLEXMATCHSetMatchParam
输入参数：
	npar 参数数量
	param 参数指针
返回：
	true 成功
	false 失败
*/
TJ3DSDK_API bool FLEXMATCHSetMatchParam(int npar,double* param);

/*扫描后生成点云:FLEXMATCHCreateRangeDataFromScan
输入参数：
	fGain 扫描时的相机增益，小于0时不做相机配置
输出参数：
	PointcloudSor 点云类指针，用户分配和释放内存
返回：
	true 成功
	false 失败
*/
TJ3DSDK_API bool FLEXMATCHCreateRangeDataFromScan(float fGain, sn3DCore::sn3DRangeData* PointcloudSor);


/*从相机中各抓取1幅图:FLEXMATCHCatchImage
注意：
	只能在视频模式中抓图
输入参数：
	fGain 抓图时的相机增益
	nMaxBufsize 用户分配的内存的字节大小.如果是彩色相机，最新内存大小=视频长x视频宽x3
输出参数：
	pLeftImgbuf 左相机图像内存指针，用户分配和释放内存
	pRightImgbuf 右相机图像内存指针，用户分配和释放内存
	pThirdImage 第三相机(中)图像内存指针，用户分配和释放内存
返回：
	0 失败。
	单幅图像数据字节数
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
