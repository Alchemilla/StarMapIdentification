#ifndef _GEOCAMERA
#define	_GEOCAMERA

#include "GeoDefine.h"
#include "GeoBase.h"

////////////////////////////////////////////////////////
// 相机基类
////////////////////////////////////////////////////////
class GeoCamera
{
public:
	GeoCamera(void);
	virtual ~GeoCamera(void);

public:
	StrCamParamInput m_Input;	// 相机参数输入结构体,这个不能被类外部改变

public:
	long get_xnum();			// 获取沿轨向像素个数
	long get_ynum();			// 获取垂轨向像素个数
	double get_OffX();			// 获得相机相对本体偏心距X
	double get_OffY();			// 获得相机相对本体偏心距Y
	double get_OffZ();			// 获得相机相对本体偏心距Z
	void get_ROff(double *R);	// 获取从相机坐标系到本体坐标系的旋转矩阵
	void set_ROff(double *R);	// 获取从相机坐标系到本体坐标系的旋转矩阵

	/////////////////////////////////////////////////////
	// 以下几个函数需要分别实现,但是注意不要用纯虚函数,否则无法实例化
	/////////////////////////////////////////////////////
	// 读取内方位元素
	virtual void ReadCamFile(string filepath, StrCamParamInput input);
	// 输出内方位元素文件
	virtual void WriteCamFile(string filepath);
	// 根据CCD排列获取初始化的内方位元素文件
	virtual void InitInnerFile(string outpath, StrCamParamInput input);
	// 获取内方位元素(探元指向角形式)
	virtual void GetInner(double x, double y, double &phiX, double &phiY);
	// 根据探元指向角的值取反求索引号
	virtual void GetIndexBaseInner(double phiY, double phiX, double &y, double &x);
};

#endif

