#ifndef MATCH_METHOD_H
#define MATCH_METHOD_H

#include <vector>
#include <string>
using namespace std;

#if defined(_WIN32)
	#ifdef MATCHMETHOD_EXPORTS
		#define DLL_EXPORT __declspec(dllexport)
	#else
		#define DLL_EXPORT
	#endif
#else
	#define DLL_EXPORT
#endif

#include "ModelFitMethod.h"

////////////////////////////////////////////////////////////////////////////
//	Ӱ��ƥ����
////////////////////////////////////////////////////////////////////////////
class DLL_EXPORT MatchMethod
{
public:
	MatchMethod(void);
	virtual ~MatchMethod(void);

public:
	// �ֿ���ȡSIFT����
	static void GetSiftFeatureBaseBlock(string imgpath, long &num, float* &keys, float* &des, 
									int blocksize = -1, int maxTexSize = -1, int extend = 0);
	// �������������Ӱ��,���SIFTƥ����
	static void MatchBaseSIFT(string imgpath1, string imgpath2, string ptspath, int blocksize = 2048);
	// �������������������ƥ��,������ƥ����
	static void MatchBaseSIFT_Data(unsigned char *ldata, unsigned char *rdata, long lwidth, long lheight, 
								long rwidth, long rheight,long &nmatch, float* &lx, float* &ly, float* &rx, 
								float* &ry, long leftx=0, long lefty=0, long rightx=0, long righty=0);
	// �������������������ƥ��,������ƥ����
	static bool MatchBaseSIFT_Data(unsigned char *ldata, unsigned char *rdata, long lwidth, long lheight, 
				long rwidth, long rheight, string ptspath, long leftx=0, long lefty=0, long rightx=0, long righty=0);
	// �������������Ӱ���Լ���ƥ���,�����λ���ƥ����
	static void MatchBasePhaseCorrelation(string imgpath1, string imgpath2, string inpts, string outpts, 
						int blocksize = 2048, int win = 32, double weight = 0.95, double SNR = 0.95);
	// �������Ӱ����в�����׼,�������Ӧ�ķ���任����
	static void MatchBaseSIFT(string imgpath, int refBand, int adjBand, vector<ModelFitMethod> &m_trans, int block = 2048);

public:
	// ���SIFT����
	static void SaveSiftFeature(string feapath, long num, float *keys, float *des);
	// ��ENVI��ʽ���ƥ����
	static void SaveMatchBaseEnvi(string ptspath, long nmatch, float *lx, float *ly, float *rx, float *ry);
	// ��ENVI��ʽ����ƥ����
	static void ReadMatchBaseEnvi(string ptspath, long &nmatch, float* &lx, float* &ly, float* &rx, float* &ry);
	// �Զ����ƶ������ƥ����
	static void SaveMatchBaseBinary(string ptspath, long nmatch, float *lx, float *ly, float *rx, float *ry);
	// �Զ����ƶ��Ƕ�ȡƥ����
	static void ReadMatchBaseBinary(string ptspath, long &nmatch, float* &lx, float* &ly, float* &rx, float* &ry);
	// ƥ������������,������ƽ������ת
	static void MatchAccuracyAnalyse(string ptspath, string accpath, float dx, float dy, float centerx, float centery, float phi);


public:
	///////////////////////////////
	//	CE5ר��
	///////////////////////////////
	// ��ȡ������
	static void GetSiftFeature_CE5(string imgpath, string feapath);
	// ����ƥ��
	static void GetMatch_CE5(string fea1, string fea2, long &nmatch, 
					float* &lx, float* &ly, float* &rx, float* &ry);
	// �������������Ӱ��,���SIFTƥ����(����CE-5�������ʹ��)
	static void MatchBaseSIFTforCE5(string imgpath1, string imgpath2, string ptspath);
	// �������������Ӱ��,���SIFTƥ����(����CE-5�������ʹ��)
	static void MatchBaseSIFTforCE5(string imgpath1, string imgpath2, long &nmatch, 
					float* &lx, float* &ly, float* &rx, float* &ry);
	// �������������Ӱ��,���SIFTƥ����(����CE-5�������ʹ��)
	static int MatchBaseSIFTforCE5(string imgpath1, string imgpath2, double &offx, double &offy);
};

#endif