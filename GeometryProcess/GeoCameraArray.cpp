#include "GeoCameraArray.h"

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoCameraArray::GeoCameraArray(void){}
GeoCameraArray::~GeoCameraArray(void){}


//////////////////////////////////////
// 功能：根据CCD排列获取初始化的内方位元素文件
// 输入:
//		string outpath:				内方位元素输出文件,如果为"",则不输出
//		StrCamParamInput input：	内方位元素配置输入参数
// 返回值：
//		void
//////////////////////////////////////
void GeoCameraArray::InitInnerFile(string outpath, StrCamParamInput input)
{
	m_Input = input;
}


//////////////////////////////////////
// 功能：获取内方位元素(探元指向角形式)
// 输入:
//		string outpath:				内方位元素输出文件
//		StrCamParamInput input：	内方位元素配置输入参数
// 返回值：
//		void
//////////////////////////////////////
void GeoCameraArray::GetInner(double x, double y, double &phiX, double &phiY)
{
	// 进行内插
	//x = m_Input.Ynum - x;//仿真出来的y是反的
	phiX = (x-m_Input.Xnum/2)*m_Input.Xsize/m_Input.f;
	y = m_Input.Ynum - y;//仿真出来的y是反的
	phiY = (y-m_Input.Ynum/2)*m_Input.Ysize/m_Input.f;
}


//////////////////////////////////////
// 功能：根据探元指向角的值取反求索引号
// 输入:
//		double y:					探元指向角的值
// 返回值：
//		double:						返回的列坐标索引号
//////////////////////////////////////
void GeoCameraArray::GetIndexBaseInner(double phiY, double phiX, double &y, double &x)
{
	x = phiX*m_Input.f/m_Input.Xsize + m_Input.Xnum/2;
	y = phiY*m_Input.f/m_Input.Ysize + m_Input.Ynum/2;
}
