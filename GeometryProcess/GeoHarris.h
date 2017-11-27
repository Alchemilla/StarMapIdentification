#pragma once
#include <vector>
#include "GeoReadImage.h"
#include "GeoDefine.h"
#include "math.h" 
#include <string> 

//////////////////////////
//// ƥ����
//////////////////////////
//struct StrCornerPoint
//{
//	double xl;	// ����
//	double yl;	// �ع�
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
// Harris����
////////////////////////////////////////////////////////
class GeoHarris
{
public:
	GeoHarris(void);
	~GeoHarris(void);
	// Harris�ǵ���ȡ����
	int Harris(double* I,int width,int height, int gw, double sigma, 
			int size, int thresh, vector<MatchPoint> &point);
	//im������ͼ��  tp��ģ����� 
	int Mbys(double* im, double* out, int imW, int imH, double *tp, int tpW, int tpH); 
};