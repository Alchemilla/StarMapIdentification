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
// ��������ȡ��
////////////////////////////////////////////////////////////
class DLL_EXPORT PointExtraction
{
public:
	PointExtraction(void);
	~PointExtraction(void);
	// ��ENVI��ʽ���ƥ����
	static void SaveBaseEnvi(string ptspath, long nmatch, float *lx, float *ly, float *rx, float *ry);
	// ����Moverac������ȡ������
	bool GetMoveracPoint(string imgpath, string ptspath, long block = 5000);
	// ����Forstner������ȡ������
	bool GetForstnerPoint(string imgpath, string ptspath, long block = 5000);
	// ����Harris������ȡ������
	bool GetHarrisPoint(string imgpath, string ptspath, long block = 5000);

//private:
//	void Moverac(GeoReadImage img, );
};

#endif

