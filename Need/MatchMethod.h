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
//	影像匹配类
////////////////////////////////////////////////////////////////////////////
class DLL_EXPORT MatchMethod
{
public:
	MatchMethod(void);
	virtual ~MatchMethod(void);

public:
	// 分块提取SIFT特征
	static void GetSiftFeatureBaseBlock(string imgpath, long &num, float* &keys, float* &des, 
									int blocksize = -1, int maxTexSize = -1, int extend = 0);
	// 根据输入的两张影像,输出SIFT匹配结果
	static void MatchBaseSIFT(string imgpath1, string imgpath2, string ptspath, int blocksize = 2048);
	// 根据输入的数据区进行匹配,并返回匹配结果
	static void MatchBaseSIFT_Data(unsigned char *ldata, unsigned char *rdata, long lwidth, long lheight, 
								long rwidth, long rheight,long &nmatch, float* &lx, float* &ly, float* &rx, 
								float* &ry, long leftx=0, long lefty=0, long rightx=0, long righty=0);
	// 根据输入的数据区进行匹配,并返回匹配结果
	static bool MatchBaseSIFT_Data(unsigned char *ldata, unsigned char *rdata, long lwidth, long lheight, 
				long rwidth, long rheight, string ptspath, long leftx=0, long lefty=0, long rightx=0, long righty=0);
	// 根据输入的两张影像以及初匹配点,输出相位相关匹配结果
	static void MatchBasePhaseCorrelation(string imgpath1, string imgpath2, string inpts, string outpts, 
						int blocksize = 2048, int win = 32, double weight = 0.95, double SNR = 0.95);
	// 对输入的影像进行波段配准,并输出对应的仿射变换参数
	static void MatchBaseSIFT(string imgpath, int refBand, int adjBand, vector<ModelFitMethod> &m_trans, int block = 2048);

public:
	// 输出SIFT特征
	static void SaveSiftFeature(string feapath, long num, float *keys, float *des);
	// 以ENVI格式输出匹配结果
	static void SaveMatchBaseEnvi(string ptspath, long nmatch, float *lx, float *ly, float *rx, float *ry);
	// 以ENVI格式输入匹配结果
	static void ReadMatchBaseEnvi(string ptspath, long &nmatch, float* &lx, float* &ly, float* &rx, float* &ry);
	// 以二进制而是输出匹配结果
	static void SaveMatchBaseBinary(string ptspath, long nmatch, float *lx, float *ly, float *rx, float *ry);
	// 以二进制而是读取匹配结果
	static void ReadMatchBaseBinary(string ptspath, long &nmatch, float* &lx, float* &ly, float* &rx, float* &ry);
	// 匹配结果精度评估,都是先平移再旋转
	static void MatchAccuracyAnalyse(string ptspath, string accpath, float dx, float dy, float centerx, float centery, float phi);


public:
	///////////////////////////////
	//	CE5专用
	///////////////////////////////
	// 获取特征点
	static void GetSiftFeature_CE5(string imgpath, string feapath);
	// 进行匹配
	static void GetMatch_CE5(string fea1, string fea2, long &nmatch, 
					float* &lx, float* &ly, float* &rx, float* &ry);
	// 根据输入的两张影像,输出SIFT匹配结果(对于CE-5特殊情况使用)
	static void MatchBaseSIFTforCE5(string imgpath1, string imgpath2, string ptspath);
	// 根据输入的两张影像,输出SIFT匹配结果(对于CE-5特殊情况使用)
	static void MatchBaseSIFTforCE5(string imgpath1, string imgpath2, long &nmatch, 
					float* &lx, float* &ly, float* &rx, float* &ry);
	// 根据输入的两张影像,输出SIFT匹配结果(对于CE-5特殊情况使用)
	static int MatchBaseSIFTforCE5(string imgpath1, string imgpath2, double &offx, double &offy);
};

#endif