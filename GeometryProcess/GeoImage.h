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
	//�˽ṹ���ڲ�ʹ��,�洢Ӱ�������
	struct ImageRect
	{
		bool isread;
		string filepath;
		double sample[4];       // ������Ӱ���е�sampleλ��
		double line[4];         // ������Ӱ���е�lineλ��
		double lat[4];
		double lon[4];
		double ts;
		double tl;
		double tw;
		double th;
	};
	// �洢PSFģ��Ľṹ��
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

	int m_step;                                  // �ֿ�Ĳ���
	int m_extend,m_extend2;                      // ��ȡӰ������������ظ���
	vector<vector<ImageRect>> m_ImageRect;       // �洢Ӱ��ֿ鷶Χ
	// �洢�û�ָ�������������С��γ��
	double m_maxeast, m_mineast, m_maxnorth, m_minnorth;

protected:
	GeoModelLine *m_model;
	double DEM_minvalue, DEM_maxvalue;



public:
	// ��Ӱ��Ĳü�
	bool ImageWarp(string inpath, string outpath, GeoModelLine *model);
	
};
