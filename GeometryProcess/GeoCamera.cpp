#include "GeoCamera.h"

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoCamera::GeoCamera(void){}
GeoCamera::~GeoCamera(void){}

// 获取沿轨向像素个数
long GeoCamera::get_xnum()
{
	return m_Input.Xnum;
}

// 获取垂轨向像素个数
long GeoCamera::get_ynum()
{
	return m_Input.Ynum;
}

// 获得相机相对本体偏心距X
double GeoCamera::get_OffX()
{
	return m_Input.m_Off[0];
}

// 获得相机相对本体偏心距Y
double GeoCamera::get_OffY()
{
	return m_Input.m_Off[1];
}

// 获得相机相对本体偏心距Z
double GeoCamera::get_OffZ()
{
	return m_Input.m_Off[2];
}

// 获取从相机坐标系到本体坐标系的旋转矩阵
void GeoCamera::get_ROff(double *R)
{
	memcpy(R, m_Input.ROff, sizeof(double)*9);
}
// 设置从相机坐标系到本体坐标系的旋转矩阵
void GeoCamera::set_ROff(double *R)
{
	memcpy( m_Input.ROff, R,sizeof(double) * 9);
}

/////////////////////////////////////////////////////
// 以下几个函数需要分别实现,但是注意不要用纯虚函数,否则无法实例化
/////////////////////////////////////////////////////
// 读取内方位元素
void GeoCamera::ReadCamFile(string filepath, StrCamParamInput input){}
// 输出内方位元素文件
void GeoCamera::WriteCamFile(string filepath){}
// 根据CCD排列获取初始化的内方位元素文件
void GeoCamera::InitInnerFile(string outpath, StrCamParamInput input){}
// 获取内方位元素(探元指向角形式)
void GeoCamera::GetInner(double x, double y, double &phiX, double &phiY){}
// 根据探元指向角的值取反求索引号
void GeoCamera::GetIndexBaseInner(double phiY, double phiX, double &y, double &x){	return;	};
