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
	//三参数或六参数几何纠正
	void Calibrate3Param(vector<StarGCP> getGCP, int index);
	void Calibrate3ParamMultiImg(vector < vector<StarGCP>> getGCP, int index);
		void Calibrate3ParamKalman(vector<vector<StarGCP>>getGCP, int index);
	void Calibrate6Param(vector<StarGCP> getGCP, int index);
	//严密几何方式定标
	void CalibrateRigorous3(vector<StarGCP> getGCP, int index);
	void CalibrateRigorous6(vector<StarGCP> getGCP, int index);
	void CalibrateRigorousRPY(vector<StarGCP> getGCP, int index);
	//void CalibrateRigorousQuat(vector<StarGCP> getGCP);
	//控制点精化
	void OptimizeGCP(vector<StarGCP> &getGCP, int pixel);
	//仿真带一定误差的控制点
	void SimulateGCP(vector<StarGCP> &getGCP,vector<STGData> ZY3_02STGdata, int index);
	void SimulateGCP_PreRand(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int *gcpNum, int index, double *randx, double *randy);
	//解算精度
	void CaliAccuracy(vector<StarGCP> getGCP, double *Xest, int num);
	//qMethod定姿
	bool qMethod(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	bool qMethod6(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	//输出文件
	void OutputFile(double *Xest, int num, string description);
};

