#pragma once
#include <vector>
#include "GeoReadImage.h"
#include "GeoDefine.h"
#include "math.h" 
#include <string> 

//////////////////////////
//// 匹配结果
//////////////////////////
//struct StrCornerPoint
//{
//	double xl;	// 垂轨
//	double yl;	// 沿轨
//	double xr;
//	double yr;
//	double XYZ[3];
//	int type;
//	bool operator < (const StrCornerPoint &m)const 
//	{
//		return yl < m.yl;
//	}
//};
 

////////////////////////////////////////////////////////
// Harris算子
////////////////////////////////////////////////////////
class GeoHarris
{
public:
	GeoHarris(void);
	~GeoHarris(void);
	// Harris角点提取算子
	int Harris(double* I,int width,int height, int gw, double sigma, 
			int size, int thresh, vector<MatchPoint> &point);
	//im：输入图像  tp：模板参数 
	int Mbys(double* im, double* out, int imW, int imH, double *tp, int tpW, int tpH); 
};