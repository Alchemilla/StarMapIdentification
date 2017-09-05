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
	//三参数或六参数几何纠正
	void Calibrate3Param(vector<StarGCP> getGCP, int index);
	void Calibrate5Param(vector<StarGCP>getGCP, int index);
	void Calibrate6Param(vector<StarGCP> getGCP, int index);
	void Calibrate3ParamMultiImg(vector < vector<StarGCP>> getGCP, int index);
	void Calibrate6ParamMultiImg(vector<vector<StarGCP>>getGCP, int index);
	void Calibrate5ParamMultiImg(vector<vector<StarGCP>>getGCP, int index);
	//卡尔曼滤波估计
	void Calibrate3ParamKalman(vector<vector<StarGCP>>getGCP);
	void Calibrate5ParamKalman(vector<vector<StarGCP>>getGCP);
	void Calibrate5ParamKalman2(vector<vector<StarGCP>>getGCP);
	//严密几何方式定标
	void CalibrateRigorous3(vector<StarGCP> getGCP, int index);
	void CalibrateRigorous6(vector<StarGCP> getGCP, int index);
	void CalibrateRigorousRPY(vector<StarGCP> getGCP, int index);
	//星点和光轴角距不变定标
	void CalibrateOpticAxisMultiImg(vector<vector<StarGCP>>getGCP, int index);
	//void CalibrateRigorousQuat(vector<StarGCP> getGCP);
	//控制点精化
	void OptimizeGCP(vector<StarGCP> &getGCP, int pixel,int index);
	//仿真带一定误差的控制点--根据姿态数据仿真
	void SimulateGCP(vector<StarGCP> &getGCP,vector<STGData> ZY3_02STGdata, int index);
	void SimulateGCP_PreRand(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int *gcpNum, int index);
	void SimulateGCP_PreRand5Param(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int index);
	void SimulateGCP_PreRand6Param(vector<vector<StarGCP>> &getGCP,
		vector<STGData> ZY3_02STGdata, int *gcpNum, int index);
	//仿真带一定误差的控制点--随机像点仿真
	void SimulateGCP_RandomXY(int index, vector<vector<StarGCP>>& getGCP);
	void SimulateGCP_RandomXY5Param(int index, vector<vector<StarGCP>>& getGCP);
	//解算精度
	void CaliAccuracy(vector<StarGCP> getGCP, double *Xest, int num);
	void CaliAccuracy3Param(vector<vector<StarGCP>> getGCP, double *Xest, double &RMS);
	void CaliAccuracy5Param(vector<vector<StarGCP>> getGCP, double *Xest, double &RMS);
	//qMethod定姿
	bool qMethod(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	bool qMethod6(vector<StarGCP> getGCP, double *X, int Param, double *quater);
	//输出文件
	void OutputFile(double *Xest, int num, string description);
	void OutputGCP(vector<StarGCP> &getGCP, int index);
	void OutputAllGCP(vector<vector<StarGCP>> getGCP);
	void ReadAllGCP(vector<vector<StarGCP>> &getGCP);
	//输出残差文件
	void OutputErr(vector<vector<StarGCP>> getGCP, vector<STGData>ZY302STG, int index);
};

