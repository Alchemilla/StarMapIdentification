#ifndef POINT_EXTRACTION_H
#define POINT_EXTRACTION_H

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
// 特征点提取类
////////////////////////////////////////////////////////////
class DLL_EXPORT PointExtraction
{
public:
	PointExtraction(void);
	~PointExtraction(void);
	// 以ENVI格式输出匹配结果
	static void SaveBaseEnvi(string ptspath, long nmatch, float *lx, float *ly, float *rx, float *ry);
	// 采用Moverac算子提取点特征
	bool GetMoveracPoint(string imgpath, string ptspath, long block = 5000);
	// 采用Forstner算子提取点特征
	bool GetForstnerPoint(string imgpath, string ptspath, long block = 5000);
	// 采用Harris算子提取点特征
	bool GetHarrisPoint(string imgpath, string ptspath, long block = 5000);

//private:
//	void Moverac(GeoReadImage img, );
};

#endif

