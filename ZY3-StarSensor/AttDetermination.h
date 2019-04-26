#pragma once
#include "SateBase.h"
#include "BaseFunc.h"
#include "AttSim.h"
#include "EOP_files.h"
class AttDetermination
{
public:
	AttDetermination();
	~AttDetermination();
	BaseFunc mbase;
	AttSim mSim;
	string workpath;
	void AttDeter(vector<STGData> AttData, vector<vector<StarGCP>>getGCP, StarCaliParam Param, vector<Quat>&Quater);
	void AttDeter2(vector<STGData> AttData, vector<vector<StarGCP>>getGCP);
	void AttDeter3(vector<STGData> AttData);
	void AttDeter4(vector<STGData> AttData);
	//static attitude determination
	bool q_Method(vector<StarGCP> getGCP, StarCaliParam Param, Quat &quater);
	void luojiaAlinFix(vector<Quat>LJCamera, Quat quater, SateEuler &ruEuler);//修正偏置矩阵

	//卡尔曼滤波
	void EKF6StateV2(vector<STGData> AttData, vector<Quat> &AttDet, int StarTag);
	void EKF6StateV3(vector<STGData> AttData, vector<Quat> APSdat, vector<Quat>&EKFres);
	void EKF6StateV4(vector<STGData> AttData, vector<Quat>&EKFres);
	//星敏陀螺时间不同步滤波
	void EKF6StateV5(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	//多星敏融合
	void EKF6StateV6(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	void EKF6StateV7(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	void EKF6StateV8(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	//双向滤波
	void EKF6StateV9(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	//针对仿真AB星敏进行姿态确定
	void EKF6StateForStarAB(int m, Quat *qA, Quat *qB, Quat * qTrue, Gyro *wMeas);
	void EKF6StateForStarAB2(int m, Quat *qA, Quat *qB, Quat * qTrue, Gyro *wMeas);
	void EKF6StateForStarAB3(int m, Quat *qA, Quat *qB, Quat * qTrue, Gyro *wMeas);
	//针对星图识别矢量进行姿态确定
	void EKF6StateForStarMap(vector < vector<BmImStar>>BmIm, vector<STGData>stg);
	void Measurement(vector<BmImStar> BmIm, double *Att, MatrixXd &matH, MatrixXd &matDetZ);
	void GetImBm(vector<vector<StarGCP >>getGCP, const StarCaliParam Param, vector<vector<BmImStar >>&BmIm);
	void GetImRm(vector<vector<BmImStar >>&BmIm);
	bool Aberration(vector<vector<BmImStar>>& BmIm, vector<Orbit_Ep> EpDat);
	//添加了陀螺尺度和安装
	void EKF15State(vector<STGData> AttData, Quat *EKFres, double *xest_store);

	//姿态仿真
	void simAttandEKF(double dt, int m, double sig_ST, string Res,
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	void simEKFForwardAndBackward(double dt, int m, double sig_ST, string Res,
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	void simGyroAndAcc(double dt, int m, string Res, double wBiasA[3],
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	void simGyroAndAccBack(double dt, int m, string Res, double wBiasA[3],
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	//得到正交安装矩阵
	void getInstall(double* Binstall, double* Cinstall);
	void getInstallErr(int m, Quat *qB, Quat *qC,double *Err);

	//KalmanFilter公式
	void predictQuat(MatrixXd wMeas, MatrixXd Qk, MatrixXd &Qk1, double dt);
	void crossMatrix(MatrixXd we, MatrixXd &cMat);

	//其他方式姿态确定
	bool SingleStar(vector<STGData> AttData, int StarTag, vector<Quat> &AttDet);
	void alinAPS(vector<Quat> &attAPS);//乘以APS的安装矩阵
	void alinBC(const vector<STGData> AttData, Quat* starB,Quat* starC);
	void DoubleStar(vector<STGData> StarDat, vector<Quat> &AttDet, int StarTag);
	void GyroAtt(vector<STGData> AttData, vector<Quat> &AttDet, string Res, int StarTag);
	//bool Aberration(vector<STGData> StarDat, vector<Orbit_Ep> EpDat, string Res, int StarTag);
	//读取定标参数文件
	void ReadCaliParam(vector<StarCaliParam>&caliParam);
	//输出文件
	void OutputFile(vector<Quat>Att, string FileName);
	void OutputFile(Quat *Att, int num, string FileName);
	void outputXest(int num, double * xest_store);
	void outputXest15(int num, double * xest_store, string res);
	//对比
	void compareEKFAndAOCC(Quat *StarDat,int n, string res);
	void compareEKFAndAOCC(vector<Quat>StarDat, string res);
	void compareAPSandStarMap();
private:
	const static double Ainstall[9], Binstall[9], Cinstall[9], GyroIns[9];
};

