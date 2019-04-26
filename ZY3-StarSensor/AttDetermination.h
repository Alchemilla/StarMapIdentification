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
	void luojiaAlinFix(vector<Quat>LJCamera, Quat quater, SateEuler &ruEuler);//����ƫ�þ���

	//�������˲�
	void EKF6StateV2(vector<STGData> AttData, vector<Quat> &AttDet, int StarTag);
	void EKF6StateV3(vector<STGData> AttData, vector<Quat> APSdat, vector<Quat>&EKFres);
	void EKF6StateV4(vector<STGData> AttData, vector<Quat>&EKFres);
	//��������ʱ�䲻ͬ���˲�
	void EKF6StateV5(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	//�������ں�
	void EKF6StateV6(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	void EKF6StateV7(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	void EKF6StateV8(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	//˫���˲�
	void EKF6StateV9(vector<STGData> AttData, Quat *quatEst, double *xest_store);
	//��Է���AB����������̬ȷ��
	void EKF6StateForStarAB(int m, Quat *qA, Quat *qB, Quat * qTrue, Gyro *wMeas);
	void EKF6StateForStarAB2(int m, Quat *qA, Quat *qB, Quat * qTrue, Gyro *wMeas);
	void EKF6StateForStarAB3(int m, Quat *qA, Quat *qB, Quat * qTrue, Gyro *wMeas);
	//�����ͼʶ��ʸ��������̬ȷ��
	void EKF6StateForStarMap(vector < vector<BmImStar>>BmIm, vector<STGData>stg);
	void Measurement(vector<BmImStar> BmIm, double *Att, MatrixXd &matH, MatrixXd &matDetZ);
	void GetImBm(vector<vector<StarGCP >>getGCP, const StarCaliParam Param, vector<vector<BmImStar >>&BmIm);
	void GetImRm(vector<vector<BmImStar >>&BmIm);
	bool Aberration(vector<vector<BmImStar>>& BmIm, vector<Orbit_Ep> EpDat);
	//��������ݳ߶ȺͰ�װ
	void EKF15State(vector<STGData> AttData, Quat *EKFres, double *xest_store);

	//��̬����
	void simAttandEKF(double dt, int m, double sig_ST, string Res,
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	void simEKFForwardAndBackward(double dt, int m, double sig_ST, string Res,
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	void simGyroAndAcc(double dt, int m, string Res, double wBiasA[3],
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	void simGyroAndAccBack(double dt, int m, string Res, double wBiasA[3],
		MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas);
	//�õ�������װ����
	void getInstall(double* Binstall, double* Cinstall);
	void getInstallErr(int m, Quat *qB, Quat *qC,double *Err);

	//KalmanFilter��ʽ
	void predictQuat(MatrixXd wMeas, MatrixXd Qk, MatrixXd &Qk1, double dt);
	void crossMatrix(MatrixXd we, MatrixXd &cMat);

	//������ʽ��̬ȷ��
	bool SingleStar(vector<STGData> AttData, int StarTag, vector<Quat> &AttDet);
	void alinAPS(vector<Quat> &attAPS);//����APS�İ�װ����
	void alinBC(const vector<STGData> AttData, Quat* starB,Quat* starC);
	void DoubleStar(vector<STGData> StarDat, vector<Quat> &AttDet, int StarTag);
	void GyroAtt(vector<STGData> AttData, vector<Quat> &AttDet, string Res, int StarTag);
	//bool Aberration(vector<STGData> StarDat, vector<Orbit_Ep> EpDat, string Res, int StarTag);
	//��ȡ��������ļ�
	void ReadCaliParam(vector<StarCaliParam>&caliParam);
	//����ļ�
	void OutputFile(vector<Quat>Att, string FileName);
	void OutputFile(Quat *Att, int num, string FileName);
	void outputXest(int num, double * xest_store);
	void outputXest15(int num, double * xest_store, string res);
	//�Ա�
	void compareEKFAndAOCC(Quat *StarDat,int n, string res);
	void compareEKFAndAOCC(vector<Quat>StarDat, string res);
	void compareAPSandStarMap();
private:
	const static double Ainstall[9], Binstall[9], Cinstall[9], GyroIns[9];
};

