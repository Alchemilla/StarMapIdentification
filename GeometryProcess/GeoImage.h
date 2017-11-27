// GeoImage.h : Declaration of the CGeoImage

#pragma once
#include <vector>
#include "GeoBase.h"
#include "GeoReadImage.h"
#include "GeoModelLine.h"
using namespace std;


class GeoImage 
{
public:	

private:
	//此结构体内部使用,存储影像的区域
	struct ImageRect
	{
		bool isread;
		string filepath;
		double sample[4];       // 在整体影像中的sample位置
		double line[4];         // 在整体影像中的line位置
		double lat[4];
		double lon[4];
		double ts;
		double tl;
		double tw;
		double th;
	};
	// 存储PSF模板的结构体
	struct StrPSF
	{
		double *psf;
		int num;
		StrPSF()	
		{
			psf = NULL;	
			num = 0;	
		}
		~StrPSF()
		{
			if(psf!=NULL)
				delete []psf;
			psf = NULL;
			num = 0;
		}
	};

	int m_step;                                  // 分块的步距
	int m_extend,m_extend2;                      // 读取影像块外扩的像素个数
	vector<vector<ImageRect>> m_ImageRect;       // 存储影像分块范围
	// 存储用户指定区域的最大和最小经纬度
	double m_maxeast, m_mineast, m_maxnorth, m_minnorth;

protected:
	GeoModelLine *m_model;
	double DEM_minvalue, DEM_maxvalue;



public:
	// 对影像的裁剪
	bool ImageWarp(string inpath, string outpath, GeoModelLine *model);
	
};
