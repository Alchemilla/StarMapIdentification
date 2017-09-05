#pragma once
#include "BaseFunc.h"
#include "SateBase.h"
#include "DateTime.h"
#include "matrix.h"
#include <iomanip>
class AttSim
{
public:
	AttSim();
	~AttSim();
	BaseFunc mBase;
	string workpath;
	void attitudeSimulation(double dt, double tf, int m, double qInitial[4], double sig_ST, double wBiasA[3],
		double sigu, double sigv, MatrixXd &qTure, MatrixXd &wTure, MatrixXd &qMeas, MatrixXd &wMeas);
	void attitudeVerify(double dt, int m, string path);
	void getQinit(double *qInit);
	void twoStarSimulation(double dt, int m, double alinN, double alinMent[3], double wBiasA[3],
		double sigu, double sigv, Quat *qA, Quat *qB, Quat * qTrue, Gyro *wMeas);
	//根据两颗星敏测量值得到星敏最优旋转矩阵
	void alinSimulation(double dt, int m, double alinN);
	//wahba问题仿真
	void wahbaSimulation(double dt, int m,double FOV, double alinN);
	void alinCalculation(const MatrixXd qA, const MatrixXd  qB);
	void alinCalculation2(const MatrixXd qA, const MatrixXd  qB);
	void alinCalculation3(const MatrixXd qA, const MatrixXd  qB);
	void alinCalculation4(const MatrixXd qA, const MatrixXd  qB);
	void alinCalculation4(const MatrixXd qA, const MatrixXd  qB, double *Err);
	void twoStarErr(int m, Quat *qB, Quat *qC, double *Err);
	void q_Method(const MatrixXd qA, const MatrixXd  qB);
	//得到正交安装矩阵
	void getInstall(double* Binstall,double* Cinstall);

private:
	double alinAB[3], qInitial[4];
	const static double Ainstall[9], Binstall[9], Cinstall[9];
};