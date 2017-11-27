#ifndef _GEOTRANSLATION
#define	_GEOTRANSLATION
#include "GeoBase.h"
#include "triangle.h"
#include <vector>
using namespace std;

////////////////////////////////////////////////////////
// 各类变换：比如仿射
////////////////////////////////////////////////////////
class GeoTranslation
{
public:
	GeoTranslation(void);
	virtual ~GeoTranslation(void);

private:
	GeoBase m_Base;								  // 底层通用算法类对象

// 仿射变换所用到的参数
public:
	double m_aff[6];							// 仿射变换系数
	double xscale, yscale, uscale, vscale;		// 缩放参数
    double xoff, yoff, uoff, voff;				// 平移参数
	double minx, miny, minu, minv;				// 最小值
	double maxx, maxy, maxu, maxv;				// 最大值


// 仿射变换所用到的函数
public:
	// 计算仿射变换系数
	bool CalAffineParam(double *x, double *y, double *u, double *v, long num);
	// 根据仿射变换系数获得值
	void GetValueBaseAffine(double x, double y, double &u, double &v);

// 有理函数式
public:	
	vector<double> m_polyx, m_polyy;			// 有理函数系数
	int m_xnum, m_ynum, m_xynum;				// 系数个数
	// 根据多项式系数获得值
	void GetValueBasePoly(double x, double y, double &u, double &v);
	// 计算反向系数
	vector<double> m_polyxinv, m_polyyinv;		// 反向有理函数系数
	void CalPolyParam();						// 计算反向系数
	void GetValueBaseInvPoly(double u, double v, double &x, double &y);

	////////////////////////////////////////////////////////
	// CE5专用
	////////////////////////////////////////
	// 计算仿射变换系数_CE5专用
	void CalAffineParamCE5(double *x, double *y, double *u, double *v, long num);
	// 根据前一个仿射系数,修正仿射系数,CE5专用
	void ModifyAffineCE5(double aff[6]);
	// 仿射正算,从局部到全局
	void GetValueCE5(double x, double y, double &u, double &v);
	// 仿射反算,从全局到局部
	void GetValueInvCE5(double u, double v, double &x, double &y);


// 三角网所用到的参数
private:
	triangulateio m_in, m_out;
	int  m_nCurTriangle;
	int m_num;

// 三角网所用到的函数
public:
	// 构建三角网模型
	void CreateTriangleModel(long num, double *x, double *y, double *u, double *v);
	// 根据三角网进行内插
	void FromXY2UV(double x, double y, double& u, double& v);
private:
	// 释放三角网模型
	void ReleaseTriangleModel(triangulateio &m_tri, bool isdel=true);
	int area(int a,int b, double x3, double y3);
	int GetLocalTriangle(double x, double y);

// 归一化所用到的函数
private:
	// 计算平均值
	double Average(double *value, long num);
	// 计算最大最小值
	void MinMax(double *value, long num, double &max, double &min);
	// 数据归一化
	void DataNormalization(long num, double *x, double *y, double *u, double *v);
};


#endif
