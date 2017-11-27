#ifndef LS_MATCHING_H
#define LS_MATCHING_H
#include "GeoReadImage.h"

#if defined(_WIN32)
	#ifdef MATCHMETHOD_EXPORTS
		#define DLL_EXPORT __declspec(dllexport)
	#else
		#define DLL_EXPORT
	#endif
#else
	#define DLL_EXPORT
#endif

////////////////////////////////////////////////////////////
// 最小二乘匹配类
////////////////////////////////////////////////////////////
class DLL_EXPORT CLSMatching
{
private:
	// 影像参数
	int    m_LH, m_LW;
	int    m_RH, m_RW;
	GeoReadImage *m_imgL, *m_imgR;	// 整个影像的灰度
	// 窗口大小
	int	m_winH, m_winW;
	int m_winH2, m_winW2;
	
public:
	// 构造/析构函数
	CLSMatching();
	virtual ~CLSMatching();
	// 初始化单点最小二乘影像匹配的初值
	// gL(xL,yL) = h0 + h1gR(a0+a1*xL+a2*yL, b0+b1*xL+b2*yL)
	void Init(GeoReadImage *imgL, GeoReadImage *imgR, int winH = 21, int winW = 21);
	//xl,yl-待匹配影像上的点; xr,yr-参考影像上的初始位置,既是输入参数，也是输出参数; CcWindow_h为计算相关系数用到的窗口,CcWindow_v, MaxWindow表示搜索同名点的窗口；
	bool MatchCorrcoef(double xl,double yl,double &xr,double& yr,int CcWindow_h, int CcWindow_v,int MaxWindow_h,int MaxWindow_v,double fnr);
	bool CalcCorrcoef(double *pData1,double *pData2,int numPixels,double& Corrcoef);
	// 进行单点最小二乘影像匹配
	bool LSMatch(double &xL, double &yL, double &xR, double &yR, double threhold=0.9, bool isModify=true);

private:
	// 计算相关系数
	double CorrCoef(double *imgL, int imgLWid, double *imgR, int imgRWid, int wid, int hei);
	// 构建误差方程
	bool ErrorEquation(double xL, double yL, double xR, double yR, 
			double *pLImg, double *pRImg, double *m_t, double *t);
	// 获取最佳位置
	void GetBestPos(double *pLImg, double &bestx, double &besty);

private:
	////////////////////////////////////
	// 各类矩阵运算
	////////////////////////////////////
	int Gauss(double *A,double *b,int n);
	void pNormal(double *a,int n,double b,double *aa, double *ab,double p);
	int findlm(double *A,int ic,int n,double *maxx);
	void exlij(double *A,double *b,int li,int lj,int n);
	void eliminate(double *A,double *b,int ii,int n);
};

#endif