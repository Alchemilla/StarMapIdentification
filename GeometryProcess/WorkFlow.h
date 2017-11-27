#ifndef _WORKFLOW
#define	_WORKFLOW

#include "GeoBase.h"
#include "GeoModel.h"

////////////////////////////////////////////////////////
// 处理流程类
////////////////////////////////////////////////////////
class WorkFlow
{
public:
	GeoBase m_base;
	GeoModel *pModel;

public:
	WorkFlow(void);
	virtual ~WorkFlow(void);
/*	// 从一个文件夹中按照通配符搜索指定类型的文件
	bool AddFeaFileInDirectory(string dirpath, vector<string> &filelist);
	// 影像相互投影
	void ImageProject(string imgsrc, GeoModel *modelsrc, string imgpro, 
					  GeoModel *modelpro, string imgdem, string imgout);
	// 前方交会构建稀疏DSM
	void GenerateDSM(int num, GeoModel **model, string *imgpath, string *ptspath, 
					string dempath, string outpath, int win, string datumname);
	// 根据影像块进行特征匹配
	void MatchBase_Data(string leftpath, string rightpath, long lwidth, long lheight, 
						long rwidth, long rheight, string ptspath, long leftx=0, 
						long lefty=0, long rightx=0, long righty=0);

public:
	// 高程迭代得到对应地面坐标
	bool EleItera(GeoModel *model, double i, double j, GeoReadImage *pDEM, 
				double &lat, double &lon, double &h);
	// 以ENVI格式输入匹配结果
	void ReadMatchBaseEnvi(string ptspath, long &nmatch, double* &lx, double* &ly, double* &rx, double* &ry);
	// 以ENVI格式输出匹配结果
	void SaveMatchBaseEnvi(string ptspath, long nmatch, double *lx, double *ly, double *rx, double *ry);*/
};

#endif

