#pragma once
#include<string>
#include"BaseFunc.h"
#include"SateBase.h"
#include"ParseSTG.h"
using namespace std;
class APScalibration
{
public:
	APScalibration();
	~APScalibration();
	BaseFunc mBase;
	ParseSTG mParse;
	string workpath;
	//�����������������ξ���
	void Calibrate3Param(vector<StarGCP> getGCP, int index);
	void Calibrate3ParamMultiImg(vector < vector<StarGCP>> getGCP, int index);
		void Calibrate3ParamKalman(vector<vector<StarGCP>>getGCP, int index);
	void Calibrate6Param(vector<StarGCP> getGCP, int index);
	//���ܼ��η�ʽ����
	void CalibrateRigorous3(vector<StarGCP> getGCP, int index);
	void CalibrateRigorous6(vector<StarGCP> getGCP, int index);
	void CalibrateRigorousRPY(vector<StarGCP> getGCP, int index);
	//void CalibrateRigorousQuat(vector<StarGCP> getGCP);
	//���Ƶ㾫��
	void OptimizeGCP(vector<StarGCP> &getGCP, int pixel);
	//�����һ�����Ŀ��Ƶ�
	void SimulateGCP(vector<StarGCP> &getGCP,vector<STGData> ZY3_02STGdata, int index);
	void SimulateGCP_PreRand(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int *gcpNum, int index, double *randx, double *randy);
	//���㾫��
	void CaliAccuracy(vector<StarGCP> getGCP, double *Xest, int num);
	//qMethod����
	bool qMethod(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	bool qMethod6(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	//����ļ�
	void OutputFile(double *Xest, int num, string description);
};

