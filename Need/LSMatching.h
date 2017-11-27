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
// ��С����ƥ����
////////////////////////////////////////////////////////////
class DLL_EXPORT CLSMatching
{
private:
	// Ӱ�����
	int    m_LH, m_LW;
	int    m_RH, m_RW;
	GeoReadImage *m_imgL, *m_imgR;	// ����Ӱ��ĻҶ�
	// ���ڴ�С
	int	m_winH, m_winW;
	int m_winH2, m_winW2;
	
public:
	// ����/��������
	CLSMatching();
	virtual ~CLSMatching();
	// ��ʼ��������С����Ӱ��ƥ��ĳ�ֵ
	// gL(xL,yL) = h0 + h1gR(a0+a1*xL+a2*yL, b0+b1*xL+b2*yL)
	void Init(GeoReadImage *imgL, GeoReadImage *imgR, int winH = 21, int winW = 21);
	//xl,yl-��ƥ��Ӱ���ϵĵ�; xr,yr-�ο�Ӱ���ϵĳ�ʼλ��,�������������Ҳ���������; CcWindow_hΪ�������ϵ���õ��Ĵ���,CcWindow_v, MaxWindow��ʾ����ͬ����Ĵ��ڣ�
	bool MatchCorrcoef(double xl,double yl,double &xr,double& yr,int CcWindow_h, int CcWindow_v,int MaxWindow_h,int MaxWindow_v,double fnr);
	bool CalcCorrcoef(double *pData1,double *pData2,int numPixels,double& Corrcoef);
	// ���е�����С����Ӱ��ƥ��
	bool LSMatch(double &xL, double &yL, double &xR, double &yR, double threhold=0.9, bool isModify=true);

private:
	// �������ϵ��
	double CorrCoef(double *imgL, int imgLWid, double *imgR, int imgRWid, int wid, int hei);
	// ��������
	bool ErrorEquation(double xL, double yL, double xR, double yR, 
			double *pLImg, double *pRImg, double *m_t, double *t);
	// ��ȡ���λ��
	void GetBestPos(double *pLImg, double &bestx, double &besty);

private:
	////////////////////////////////////
	// �����������
	////////////////////////////////////
	int Gauss(double *A,double *b,int n);
	void pNormal(double *a,int n,double b,double *aa, double *ab,double p);
	int findlm(double *A,int ic,int n,double *maxx);
	void exlij(double *A,double *b,int li,int lj,int n);
	void eliminate(double *A,double *b,int ii,int n);
};

#endif