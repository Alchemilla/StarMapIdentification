#ifndef _WORKFLOW
#define	_WORKFLOW

#include "GeoBase.h"
#include "GeoModel.h"

////////////////////////////////////////////////////////
// ����������
////////////////////////////////////////////////////////
class WorkFlow
{
public:
	GeoBase m_base;
	GeoModel *pModel;

public:
	WorkFlow(void);
	virtual ~WorkFlow(void);
/*	// ��һ���ļ����а���ͨ�������ָ�����͵��ļ�
	bool AddFeaFileInDirectory(string dirpath, vector<string> &filelist);
	// Ӱ���໥ͶӰ
	void ImageProject(string imgsrc, GeoModel *modelsrc, string imgpro, 
					  GeoModel *modelpro, string imgdem, string imgout);
	// ǰ�����ṹ��ϡ��DSM
	void GenerateDSM(int num, GeoModel **model, string *imgpath, string *ptspath, 
					string dempath, string outpath, int win, string datumname);
	// ����Ӱ����������ƥ��
	void MatchBase_Data(string leftpath, string rightpath, long lwidth, long lheight, 
						long rwidth, long rheight, string ptspath, long leftx=0, 
						long lefty=0, long rightx=0, long righty=0);

public:
	// �̵߳����õ���Ӧ��������
	bool EleItera(GeoModel *model, double i, double j, GeoReadImage *pDEM, 
				double &lat, double &lon, double &h);
	// ��ENVI��ʽ����ƥ����
	void ReadMatchBaseEnvi(string ptspath, long &nmatch, double* &lx, double* &ly, double* &rx, double* &ry);
	// ��ENVI��ʽ���ƥ����
	void SaveMatchBaseEnvi(string ptspath, long nmatch, double *lx, double *ly, double *rx, double *ry);*/
};

#endif

