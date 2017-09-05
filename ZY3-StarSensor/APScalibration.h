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
	StarCaliParam ZY302CaliParam;
	string workpath;
	//�����������������ξ���
	void Calibrate3Param(vector<StarGCP> getGCP, int index);
	void Calibrate5Param(vector<StarGCP>getGCP, int index);
	void Calibrate6Param(vector<StarGCP> getGCP, int index);
	void Calibrate3ParamMultiImg(vector < vector<StarGCP>> getGCP, int index);
	void Calibrate6ParamMultiImg(vector<vector<StarGCP>>getGCP, int index);
	void Calibrate5ParamMultiImg(vector<vector<StarGCP>>getGCP, int index);
	//�������˲�����
	void Calibrate3ParamKalman(vector<vector<StarGCP>>getGCP);
	void Calibrate5ParamKalman(vector<vector<StarGCP>>getGCP);
	void Calibrate5ParamKalman2(vector<vector<StarGCP>>getGCP);
	//���ܼ��η�ʽ����
	void CalibrateRigorous3(vector<StarGCP> getGCP, int index);
	void CalibrateRigorous6(vector<StarGCP> getGCP, int index);
	void CalibrateRigorousRPY(vector<StarGCP> getGCP, int index);
	//�ǵ�͹���Ǿ಻�䶨��
	void CalibrateOpticAxisMultiImg(vector<vector<StarGCP>>getGCP, int index);
	//void CalibrateRigorousQuat(vector<StarGCP> getGCP);
	//���Ƶ㾫��
	void OptimizeGCP(vector<StarGCP> &getGCP, int pixel,int index);
	//�����һ�����Ŀ��Ƶ�--������̬���ݷ���
	void SimulateGCP(vector<StarGCP> &getGCP,vector<STGData> ZY3_02STGdata, int index);
	void SimulateGCP_PreRand(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int *gcpNum, int index);
	void SimulateGCP_PreRand5Param(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int index);
	void SimulateGCP_PreRand6Param(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int *gcpNum, int index);
	//�����һ�����Ŀ��Ƶ�--���������
	void SimulateGCP_RandomXY(int index, vector<vector<StarGCP>>& getGCP);
	void SimulateGCP_RandomXY5Param(int index, vector<vector<StarGCP>>& getGCP);
	//���㾫��
	void CaliAccuracy(vector<StarGCP> getGCP, double *Xest, int num);
	void CaliAccuracy3Param(vector<vector<StarGCP>> getGCP, double *Xest, double &RMS);
	void CaliAccuracy5Param(vector<vector<StarGCP>> getGCP, double *Xest, double &RMS);
	//qMethod����
	bool qMethod(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	bool qMethod6(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	//����ļ�
	void OutputFile(double *Xest, int num, string description);
	void OutputGCP(vector<StarGCP> &getGCP, int index);
	void OutputAllGCP(vector<vector<StarGCP>> getGCP);
	void ReadAllGCP(vector<vector<StarGCP>> &getGCP);
	//����в��ļ�
	void OutputErr(vector<vector<StarGCP>> getGCP, vector<STGData>ZY302STG, int index);
};

