#pragma once
#include "UseOpenCV.h"
#include<string>
#include<vector>
using namespace std;
typedef struct matchPts
{
	float xl;
	float yl;
	float xr;
	float yr;
}matchPts;

struct ImgBlockInfo
{
	int sampleIndex;
	int lineIndex;
	int width;
	int height;
};

bool delMatchAffine(vector<matchPts> &allGCP,double fSigmacondition);

//Mat need to be 8U;
bool  surfMatch(Mat& srcImage1,Mat& srcImage2 ,matchPts*& cps,int& nCps);
//分块匹配
bool surfMatch_uchar(string sImg1,string sImg2,int nBlocks,double fSigmacondition,int numThread,string sRes);
//
bool BlocksForImgProcess(int bline, int bsample, int perBlockLine,
	int perBlockSample, int width, int height, ImgBlockInfo* &pBlock, int& num);
