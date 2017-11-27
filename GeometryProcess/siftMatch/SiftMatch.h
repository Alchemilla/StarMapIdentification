// SiftMatch1.h: interface for the CSiftMatch class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
//GPU加速运行库
//#include "GL/gl.h"	
#include "SiftGPU.h"
#include "GL/glut.h"

#include"../GeoModelRFM.h"
#include "../GeoReadImage.h"
#include<omp.h>
#include"../GeoBase.h"
#include "../surfMatch.h"
#include "sift.h"
#include "kdtree.h"
#include "imgfeatures.h"
#include<string>
#include<vector>
using namespace std;

#ifdef _DEBUG
#pragma comment(lib,"SiftGPU_d.lib")
#else
#pragma comment(lib,"SiftGPU.lib")
#endif

//#ifndef _matchPts
//#define _matchPts
//typedef struct matchPts
//{
//	float xl;
//	float yl;
//	float xr;
//	float yr;
//}matchPts;
//#endif

class CSiftMatch  
{
public:
	CSiftMatch();
	virtual ~CSiftMatch();
	feature * m_feat1, * m_feat2;
	bool Init(float *block1,long block1width,long block1height,float *block2,long block2width,long block2height);
	bool ApplyImgMatch(matchPts *& pGCP,int &num);
private:
	int m_n1,m_n2;

protected:
	bool kdmatching(struct feature* *feat1, struct feature* feat2,int n1,int n2,int &matchednum);
	bool kdmatching_once(struct feature* *feat1, struct feature* feat2,int n1,int n2,int &matchednum);
};

int siftMatch(string sImg1,string sImg2,int nBlocks,double fSigmacondition,int numThread,string sRes);

//采用GPU加速的SIFT
bool siftGPUMatch(string sImg1, string sImg2, int nBlocks, bool usedGeoModel, double fCondition, string sRes);
//template<class T>
//bool siftGPUMatch(CRPCModel* pModel, string *sImgPath, int nBlocks, bool usedGeoModel, double fCondition, string sRes, T dataType);
bool siftMatch_core(void* pData1, int w1, int h1, void *pData2, int w2, int h2, int dataType, vector<matchPts>& cps);
