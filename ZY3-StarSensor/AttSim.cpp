#include "AttSim.h"



AttSim::AttSim()
{
}


AttSim::~AttSim()
{
}
const double AttSim::Ainstall[] =//Crb
{
	cos(29.13212 / 180 * PI),cos(104.8425 / 180 * PI),cos(65.5449 / 180 * PI),
	cos(75.98529 / 180 * PI),cos(120.6005 / 180 * PI),cos(145.6867 / 180 * PI),
	cos(65.0185 / 180 * PI),cos(34.7422 / 180 * PI),cos(112.497 / 180 * PI)
};
const double AttSim::Binstall[9] =
{
	cos(21.68954 / 180 * PI),cos(92.60966 / 180 * PI),cos(111.5162 / 180 * PI),
	cos(102.7403 / 180 * PI),cos(149.8423 / 180 * PI),cos(116.833 / 180 * PI),
	cos(107.2508 / 180 * PI),cos(59.97562 / 180 * PI),cos(144.4336 / 180 * PI)
};
//const double AttSim::Binstall[9] =
//{
//	0.92920005755065446997807548906766,-0.04553141272876051376428470281838,-0.36676428144071355154444668102217,
//	-0.2205323093917142117167813087526,-0.86464593306966506067946037079859,-0.45139155840239752379660164403031,
//	-0.29655490954062675980392302520358,0.50036845829475349622876219119266,-0.81344199595378323807619437523358
//};
const double AttSim::Cinstall[9] =
{
	cos(63.63085 / 180 * PI),cos(92.72818 / 180 * PI),cos(26.53156 / 180 * PI),
	cos(68.95412 / 180 * PI),cos(154.8806 / 180 * PI),cos(103.0838 / 180 * PI),
	cos(34.83029 / 180 * PI),cos(65.0493 / 180 * PI),cos(112.6469 / 180 * PI)
};
//////////////////////////////////////////////////////////////////////////
//功能：姿态仿真程序
//输入：dt：姿态间隔时长（单位：秒）；		tf：总时长（单位：秒）
//			 qInitial[4]：初始姿态四元数，最后为标量；		sig_ST：星敏误差（单位：角秒）
//			 wBiasA[3]：陀螺漂移理论值（单位：°/Hr）；		sigu，sigv：陀螺常值漂移误差，陀螺误差
//输出：
//注意：
//作者：GZC
//日期：2017.06.08
//////////////////////////////////////////////////////////////////////////
void AttSim::attitudeSimulation(double dt, double tf, int m, double qInitial[4], double sig_ST,double wBiasA[3], 
	double sigu, double sigv, MatrixXd &qTure, MatrixXd &wTure, MatrixXd &qMeas, MatrixXd &wMeas)
{
	//MatrixXd qTure(m, 4), wTure(m, 3), qMeas(m, 4), wMeas(m, 3);
	qTure.row(0) << qInitial[0], qInitial[1], qInitial[2], qInitial[3];

	double sig_tracker = 0.5*sig_ST / 3600 * PI / 180;
	double *noise1 = new double[m]; double *noise2 = new double[m]; double *noise3 = new double[m];
	//根据程序启动时间得到一个随机数，作为种子放入星敏和陀螺的随机模型中
	int randcount;
	double randtmp[1];
	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = randtmp[0];
	mBase.RandomDistribution(0, sig_tracker, m, randcount + 0, noise1);
	mBase.RandomDistribution(0, sig_tracker, m, randcount + 1, noise2);
	mBase.RandomDistribution(0, sig_tracker, m, randcount + 2, noise3);
	//四元数在Matrix矩阵中顺序为1234, qTure(0,0)对应1,qTure(0,3)对应4，为标量
	double q1[] = { qTure(0,0), qTure(0,1),  qTure(0,2), qTure(0,3) };
	double q2[] = { noise1[0], noise2[0], noise3[0], 1};
	double q3[4];
	mBase.quatMult(q1,q2,q3);
	qMeas.row(0) << q3[0], q3[1], q3[2], q3[3];

	//设置常值漂移
	double wbias1 = wBiasA[0]; double wbias2 = wBiasA[1]; double wbias3 = wBiasA[2];
	double *bias1 = new double[m]; double *bias2 = new double[m]; double *bias3 = new double[m];
	double *wn1 = new double[m]; double *wn2 = new double[m]; double *wn3 = new double[m];
	mBase.RandomDistribution(wbias1*PI / 180 / 3600 * dt, sigu / sqrt(dt) , m, randcount + 3, bias1);//注意是*dt，matlab中是/dt
	mBase.RandomDistribution(wbias2*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 4, bias2);
	mBase.RandomDistribution(wbias3*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 5, bias3);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 6, wn1);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 7, wn2);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 8, wn3);

	for (size_t i = 0; i < m; i++)
	{		
		wTure(i,0) = 0.1*PI / 180 * sin(0.1*dt*i);
		wTure(i, 1) = 0.1*PI / 180 * sin(0.085*dt*i);
		wTure(i, 2) = 0.1*PI / 180 * cos(0.085*dt*i); 
		wMeas.row(i) << wTure(i, 0) + wn1[i] + bias1[i], wTure(i, 1) + wn2[i] + bias2[i], wTure(i, 2) + wn3[i] + bias3[i];
		if (i==m-1)	{break;}
		double ww = sqrt(pow(wTure(i, 0), 2) + pow(wTure(i, 1), 2) + pow(wTure(i, 2), 2));
		double co = cos(0.5*ww*dt);
		double si = sin(0.5*ww*dt);
		double n1 = wTure(i, 0) / ww; double n2= wTure(i, 1) / ww; double n3 = wTure(i, 2) / ww;		
		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
		Matrix4d om;
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		qTure.row(i+1) = (om*qTure.row(i).transpose()).transpose();
		q1[0] = qTure(i + 1, 0); q1[1] = qTure(i + 1, 1); q1[2] = qTure(i + 1, 2); q1[3] = qTure(i + 1, 3);
		q2[0] = noise1[i + 1]; q2[1] = noise2[i + 1]; q2[2] = noise3[i + 1], q2[3] = 1;
		mBase.quatMult(q1, q2, q3);
		double q3norm = sqrt(pow(q3[0], 2) + pow(q3[1], 2) + pow(q3[2], 2) + pow(q3[3], 2));
		q3[0] /= q3norm; q3[1] /= q3norm; q3[2] /= q3norm; q3[3] /= q3norm;
		qMeas.row(i+1)<< q3[0], q3[1], q3[2], q3[3];
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：根据四元数求姿态角速度
//输入：dt：姿态间隔时长（单位：秒）；		tf：总时长（单位：秒）
//			 qInitial[4]：初始姿态四元数，最后为标量；		
//输出：
//注意：
//作者：GZC
//日期：2017.07.25
//////////////////////////////////////////////////////////////////////////
void AttSim::attitudeVerify(double dt, int m, string path)
{
	MatrixXd qTure(m,4), wTure(m,3);
	//四元数在Matrix矩阵中顺序为1234, qTure(0,0)对应1,qTure(0,3)对应4，为标量
	qTure.row(0) << qInitial[0], qInitial[1], qInitial[2], qInitial[3];
	
	for (int i = 0; i < m-1; i++)
	{
		//wTure(i, 0) = 0.1*PI / 180 * sin(1*dt*i);
		//wTure(i, 1) = 0.6*PI / 180 * sin(0.85*dt*i);
		//wTure(i, 2) = 0.1*PI / 180 * cos(0.85*dt*i);
		wTure(i, 0) = 0.001;
		wTure(i, 1) = 0.0065;
		wTure(i, 2) = 0.001;
		double ww = sqrt(pow(wTure(i, 0), 2) + pow(wTure(i, 1), 2) + pow(wTure(i, 2), 2));
		double co = cos(0.5*ww*dt);
		double si = sin(0.5*ww*dt);
		double n1 = wTure(i, 0) / ww; double n2 = wTure(i, 1) / ww; double n3 = wTure(i, 2) / ww;
		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
		Matrix4d om;
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		qTure.row(i + 1) = (om*qTure.row(i).transpose()).transpose();		
	}

	string resPath = path + "\\simOmegaCompare.txt";
	FILE *fp = fopen(resPath.c_str(), "w");
	double omega[3], mTureL[9], mTureR[9], mTureRes[9], domega[3];
	for (int i = 0; i < m - 1; i++)
	{
		mBase.quat2matrix(qTure(i, 0), qTure(i, 1), qTure(i, 2), qTure(i, 3), mTureL);
		mBase.quat2matrix(qTure(i+1, 0), qTure(i+1, 1), qTure(i+1, 2), qTure(i+1, 3), mTureR);
		mBase.invers_matrix(mTureL, 3);
		mBase.Multi( mTureR, mTureL,mTureRes, 3, 3, 3);

		omega[0] = (mTureRes[5] - mTureRes[7]) / 2/ dt;
		omega[1] = (mTureRes[6] - mTureRes[2]) / 2 / dt;
		omega[2] = (mTureRes[1] - mTureRes[3]) / 2 / dt;
		domega[0] = omega[0] - wTure(i, 0);
		domega[1] = omega[1] - wTure(i, 1);
		domega[2] = omega[2] - wTure(i, 2);
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", wTure(i, 0), wTure(i, 1), wTure(i, 2),
			omega[0] , omega[1] , omega[2] ,domega[0], domega[1], domega[2]);
	}
	fclose(fp);
	resPath = path + "\\simOmegaCompare2.txt";
	fp = fopen(resPath.c_str(), "w");
	double qL[4], qR[4], qRes[4];
	for (int i = 0; i < m - 1; i++)
	{
		qL[0] = qTure(i, 0), qL[1] = qTure(i, 1), qL[2] = qTure(i, 2), qL[3] = qTure(i, 3);
		qR[0] = qTure(i+1, 0), qR[1] = qTure(i + 1, 1), qR[2] = qTure(i + 1, 2), qR[3] = -qTure(i + 1, 3);
		mBase.quatMult(qR, qL, qRes);

		omega[0] = qRes[0] * 2 / dt;
		omega[1] = qRes[1] * 2 / dt;
		omega[2] = qRes[2] * 2 / dt;
		domega[0] = omega[0] - wTure(i, 0);
		domega[1] = omega[1] - wTure(i, 1);
		domega[2] = omega[2] - wTure(i, 2);
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", wTure(i, 0), wTure(i, 1), wTure(i, 2),
			omega[0], omega[1], omega[2], domega[0], domega[1], domega[2]);
	}
	fclose(fp);
	resPath = path + "\\simOmegaCompare3.txt";
	fp = fopen(resPath.c_str(), "w");
	for (int i = 0; i < m - 1; i++)
	{
		mBase.quat2matrix(qTure(i, 0), qTure(i, 1), qTure(i, 2), qTure(i, 3), mTureL);
		mBase.quat2matrix(qTure(i + 1, 0), qTure(i + 1, 1), qTure(i + 1, 2), qTure(i + 1, 3), mTureR);
		mBase.invers_matrix(mTureL, 3);
		mBase.Multi(mTureR, mTureL, mTureRes, 3, 3, 3);
		mBase.Matrix2Eulor(mTureRes, 123, omega[0], omega[1], omega[2]);
		omega[0] = omega[0] / dt;
		omega[1] = omega[1] / dt;
		omega[2] = omega[2] / dt;
		domega[0] = omega[0] - wTure(i, 0);
		domega[1] = omega[1] - wTure(i, 1);
		domega[2] = omega[2] - wTure(i, 2);
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", wTure(i, 0), wTure(i, 1), wTure(i, 2),
			omega[0], omega[1], omega[2], domega[0], domega[1], domega[2]);
	}
	fclose(fp);
}

void AttSim::getQinit(double *qInit)
{
	qInitial[0] = qInit[0], qInitial[1] = qInit[1], qInitial[2] = qInit[2], qInitial[3] = qInit[3];
}

//////////////////////////////////////////////////////////////////////////
//功能：仿真两颗星敏的测量值
//输入：dt：姿态间隔时长（单位：秒），m：测量值数量
//输出：
//注意：
//作者：GZC
//日期：2017.08.07
//////////////////////////////////////////////////////////////////////////
void AttSim::twoStarSimulation(double dt, int m, double alinN, double alinMent[3], double wBiasA[3],
	double sigu, double sigv, Quat *qA, Quat *qB, Quat * qTrueOut, Gyro *wMeas)
{
	double *noise1 = new double[m]; double *noise2 = new double[m]; double *noise3 = new double[m];
	double *noise4 = new double[m]; double *noise5 = new double[m]; double *noise6 = new double[m];
	//根据程序启动时间得到一个随机数，作为种子放入星敏和陀螺的随机模型中
	int randcount;
	double randtmp[1];
	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = randtmp[0];
	mBase.RandomDistribution(0, alinN, m, randcount + 0, noise1);
	mBase.RandomDistribution(0, alinN, m, randcount + 1, noise2);
	mBase.RandomDistribution(0, alinN, m, randcount + 2, noise3);
	mBase.RandomDistribution(0, alinN, m, randcount + 3, noise4);
	mBase.RandomDistribution(0, alinN, m, randcount + 4, noise5);
	mBase.RandomDistribution(0, alinN, m, randcount + 5, noise6);

	double *bias1 = new double[m]; double *bias2 = new double[m]; double *bias3 = new double[m];
	double *wn1 = new double[m]; double *wn2 = new double[m]; double *wn3 = new double[m];
	mBase.RandomDistribution(wBiasA[0]*PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 13, bias1);//注意是*dt，matlab中是/dt
	mBase.RandomDistribution(wBiasA[1] *PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 14, bias2);
	mBase.RandomDistribution(wBiasA[2] *PI / 180 / 3600 * dt, sigu / sqrt(dt), m, randcount + 15, bias3);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 16, wn1);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 17, wn2);
	mBase.RandomDistribution(0, sqrt(sigv*sigv / dt + 1 / 12 * sigu *sigu * dt), m, randcount + 18, wn3);
	
	double qSim[4], wRand[3];
	mBase.RandomDistribution(0, 1, 4, randcount + 6, qSim);
	mBase.RandomDistribution(0, 0.1, 3, randcount + 7, wRand);
	double qSimAll = sqrt(pow(qSim[0], 2) + pow(qSim[1], 2) + pow(qSim[2], 2) + pow(qSim[3], 2));
	qInitial[0] = qSim[0] / qSimAll, qInitial[1] = qSim[1] / qSimAll, qInitial[2] = qSim[2] / qSimAll, qInitial[3] = qSim[3] / qSimAll;
	
	FILE *fp = fopen((workpath + "\\StarBC_TrueAtt.txt").c_str(), "w");
	MatrixXd qTrue(m,4), qMeas(m, 4), wTure(m, 3);
	qTrue.row(0) << qInitial[0], qInitial[1], qInitial[2], qInitial[3];
	qTrueOut[0].Q1 = qInitial[0], qTrueOut[0].Q2 = qInitial[1];
	qTrueOut[0].Q3 = qInitial[2], qTrueOut[0].Q0 = qInitial[3];
	fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\n", 0, qInitial[0], qInitial[1], qInitial[2]);
	//四元数在Matrix矩阵中顺序为1234, qTure(0,0)对应1,qTure(0,3)对应4，为标量
	double Ra[9], Rb[9], eulerB[3],eulerC[3],RaMeas[9], RbMeas[9], alinMeas[9],Bins[9], Cins[9], CinsErr[9];

	getInstall(Bins, Cins);//添加给定安装误差
	mBase.Eulor2Matrix(alinMent[0], alinMent[1], alinMent[2], 213, alinMeas);
	mBase.Matrix2Eulor(Bins, 213, eulerB[0], eulerB[1], eulerB[2]);
	mBase.Multi(alinMeas, Cins, CinsErr, 3, 3, 3);
	mBase.Matrix2Eulor(CinsErr, 213, eulerC[0], eulerC[1], eulerC[2]);

	mBase.Eulor2Matrix(eulerB[0] + noise1[0], eulerB[1] + noise2[0], eulerB[2] + noise3[0], 213, alinMeas);
	mBase.quat2matrix(qTrue(0, 0), qTrue(0, 1), qTrue(0, 2), qTrue(0, 3), Ra);
	mBase.Multi(alinMeas, Ra, RaMeas, 3, 3, 3);
	mBase.matrix2quat(RaMeas, qA[0].Q1, qA[0].Q2, qA[0].Q3, qA[0].Q0);
	qA[0].UTC = 0;

	mBase.Eulor2Matrix(eulerC[0] + noise4[0], eulerC[1] + noise5[0], eulerC[2] + noise6[0], 213, alinMeas);
	mBase.quat2matrix(qTrue(0, 0), qTrue(0, 1), qTrue(0, 2), qTrue(0, 3), Rb);
	mBase.Multi(alinMeas, Rb, RbMeas, 3, 3, 3);
	mBase.matrix2quat(RbMeas, qB[0].Q1, qB[0].Q2, qB[0].Q3, qB[0].Q0);
	qB[0].UTC = 0;

	for (int i = 0; i < m; i++)
	{
		//wTure(i, 0) = 0.1*PI / 180 * sin(abs(wRand[0]) * dt*i);
		//wTure(i, 1) = 0.1*PI / 180 * sin(abs(wRand[1]) * dt*i);
		//wTure(i, 2) = 0.1*PI / 180 * cos(abs(wRand[2]) *dt*i); 
		wTure(i, 0) = 0.01*PI / 180;
		wTure(i, 1) = 0.065*PI / 180 ;
		wTure(i, 2) = 0.01*PI / 180;
		wMeas[i].UTC = dt*i;
		wMeas[i].x = wTure(i, 0) + wn1[i] + bias1[i];
		wMeas[i].y = wTure(i, 1) + wn2[i] + bias2[i];
		wMeas[i].z = wTure(i, 2) + wn3[i] + bias3[i];
		if (i == m - 1) { break; }

		double ww = sqrt(pow(wTure(i, 0), 2) + pow(wTure(i, 1), 2) + pow(wTure(i, 2), 2));
		double co = cos(0.5*ww*dt);
		double si = sin(0.5*ww*dt);
		double n1 = wTure(i, 0) / ww; double n2 = wTure(i, 1) / ww; double n3 = wTure(i, 2) / ww;
		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
		Matrix4d om;
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		qTrue.row(i + 1) = (om*qTrue.row(i).transpose()).transpose();//Cbj
		fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\n", dt*(i+1), qTrue(i + 1, 0), qTrue(i + 1, 1), qTrue(i + 1, 2));
		qTrueOut[i + 1].Q1 = qTrue(i + 1, 0), qTrueOut[i + 1].Q2 = qTrue(i + 1, 1);
		qTrueOut[i + 1].Q3 = qTrue(i + 1, 2), qTrueOut[i + 1].Q0 = qTrue(i + 1, 3);

		mBase.Eulor2Matrix(eulerB[0] + noise1[i+1], eulerB[1] + noise2[i + 1], eulerB[2] + noise3[i + 1], 213, alinMeas);
		mBase.quat2matrix(qTrue(i+1, 0), qTrue(i+1, 1), qTrue(i+1, 2), qTrue(i+1, 3), Ra);//Cbj
		mBase.Multi(alinMeas, Ra, RaMeas, 3, 3, 3);//Crj
		mBase.matrix2quat(RaMeas, qA[i+1].Q1, qA[i + 1].Q2, qA[i + 1].Q3, qA[i + 1].Q0);
		qA[i + 1].UTC = dt*(i + 1);

		mBase.Eulor2Matrix(eulerC[0] + noise4[i+1], eulerC[1] + noise5[i+1], eulerC[2] + noise6[i+1], 213, alinMeas);
		mBase.quat2matrix(qTrue(i+1, 0), qTrue(i+1, 1), qTrue(i+1, 2), qTrue(i+1, 3), Rb);
		mBase.Multi(alinMeas, Rb, RbMeas, 3, 3, 3);
		mBase.matrix2quat(RbMeas, qB[i + 1].Q1, qB[i + 1].Q2, qB[i + 1].Q3, qB[i + 1].Q0);
		qB[i + 1].UTC = dt*(i + 1);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：根据两颗星敏测量值得到星敏最优旋转矩阵
//输入：dt：姿态间隔时长（单位：秒）；	
//			 qInitial[4]：初始姿态四元数，最后为标量；	alinN：安装噪声
//输出：
//注意：
//作者：GZC
//日期：2017.07.28
//////////////////////////////////////////////////////////////////////////
void AttSim::alinSimulation(double dt, int m, double alinN)
{
	double *noise1 = new double[m]; double *noise2 = new double[m]; double *noise3 = new double[m];
	//根据程序启动时间得到一个随机数，作为种子放入星敏和陀螺的随机模型中
	int randcount;
	double randtmp[1];
	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = randtmp[0];
	mBase.RandomDistribution(0, alinN, m, randcount + 0, noise1);
	mBase.RandomDistribution(0, alinN, m, randcount + 1, noise2);
	mBase.RandomDistribution(0, alinN, m, randcount + 2, noise3);
	double qSim[4], alinSim[3],wRand[3];
	mBase.RandomDistribution(0, 1, 4, randcount + 3, qSim);
	mBase.RandomDistribution(0, 90, 3, randcount + 4, alinAB);
	mBase.RandomDistribution(0, 0.1, 3, randcount + 5, wRand);
	double qSimAll = sqrt(pow(qSim[0], 2) + pow(qSim[1], 2) + pow(qSim[2], 2) + pow(qSim[3], 2));
	qInitial[0] = qSim[0] / qSimAll, qInitial[1] = qSim[1] / qSimAll, qInitial[2] = qSim[2] / qSimAll, qInitial[3] = qSim[3] / qSimAll;
	MatrixXd qA(m, 4), wTure(m, 3), qB(m, 4);
	qA.row(0) << qInitial[0], qInitial[1], qInitial[2], qInitial[3];
	cout << setiosflags(ios::fixed);
	cout << setprecision(15)<< "初始四元数是：" << qInitial[0] << " " << qInitial[1] << " " << qInitial[2] << " " << qInitial[3] << endl;
	//四元数在Matrix矩阵中顺序为1234, qTure(0,0)对应1,qTure(0,3)对应4，为标量
	//string path = workpath+"\\alin_set.txt";
	//FILE *fp = fopen(path.c_str(), "w");
	double alinMeas[9], Ra[9], Rb[9];
	cout << "初始安装矩阵：" << alinAB[0] << " " << alinAB[1] << " " << alinAB[2] << endl;
	alinAB[0] = alinAB[0] / 180 * PI, alinAB[1] = alinAB[1] / 180 * PI, alinAB[2] = alinAB[2] / 180 * PI;
	mBase.Eulor2Matrix(alinAB[0], alinAB[1], alinAB[2] , 213, alinMeas);
	//cout << alinMeas[0] << " " << alinMeas[1] << " " << alinMeas[2] << endl;
	//cout << alinMeas[3] << " " << alinMeas[4] << " " << alinMeas[5] << endl;
	//cout << alinMeas[6] << " " << alinMeas[7] << " " << alinMeas[8] << endl;

	mBase.Eulor2Matrix(alinAB[0] + noise1[0], alinAB[1] + noise2[0], alinAB[2] + noise3[0], 213, alinMeas);
	//fprintf(fp, "%.5f\t%.5f\t%.5f\n", (alinAB[0] + noise1[0]) / PI * 180, (alinAB[1] + noise2[0]) / PI * 180, (alinAB[2] + noise3[0]) / PI * 180);
	mBase.quat2matrix(qA(0, 0), qA(0, 1), qA(0, 2), qA(0, 3),Ra);
	mBase.Multi(alinMeas, Ra, Rb, 3, 3, 3);
	mBase.matrix2quat(Rb, qB(0, 0), qB(0, 1), qB(0, 2), qB(0, 3));

	for (int i = 0; i < m-1; i++)
	{
		wTure(i, 0) = 0.1*PI / 180 * sin(abs(wRand[0]) * dt*i);
		wTure(i, 1) = 0.1*PI / 180 * sin(abs(wRand[1]) * dt*i);
		wTure(i, 2) = 0.1*PI / 180 * cos(abs(wRand[2]) *dt*i);
		double ww = sqrt(pow(wTure(i, 0), 2) + pow(wTure(i, 1), 2) + pow(wTure(i, 2), 2));
		double co = cos(0.5*ww*dt);
		double si = sin(0.5*ww*dt);
		double n1 = wTure(i, 0) / ww; double n2 = wTure(i, 1) / ww; double n3 = wTure(i, 2) / ww;
		double qw1 = n1*si; double qw2 = n2*si; double qw3 = n3*si; double qw4 = co;
		Matrix4d om;
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		qA.row(i + 1) = (om*qA.row(i).transpose()).transpose();//Cbj
		mBase.Eulor2Matrix(alinAB[0] + noise1[i + 1], alinAB[1] + noise2[i + 1], alinAB[2] + noise3[i + 1], 213, alinMeas);
		//fprintf(fp, "%.5f\t%.5f\t%.5f\n", (alinAB[0] + noise1[i + 1]) / PI * 180,
			//(alinAB[1] + noise2[i + 1]) / PI * 180, (alinAB[2] + noise3[i + 1]) / PI * 180);
		mBase.quat2matrix(qA(i + 1, 0), qA(i + 1, 1), qA(i + 1, 2), qA(i + 1, 3), Ra);
		mBase.Multi(alinMeas, Ra, Rb, 3, 3, 3);//Cbb
		mBase.matrix2quat(Rb, qB(i + 1,0), qB(i + 1, 1), qB(i + 1, 2), qB(i + 1, 3));//Cbj
	}
	//fclose(fp);
	alinCalculation(qA, qB);
	alinCalculation2(qA, qB);
	alinCalculation4(qA, qB);
	q_Method(qA, qB);
}

//////////////////////////////////////////////////////////////////////////
//功能：根据两颗星敏测量值得到星敏最优旋转矩阵
//输入：dt：姿态间隔时长（单位：秒）；	
//			 qInitial[4]：初始姿态四元数，最后为标量；	alinN：安装噪声；FOV：星敏视场
//输出：
//注意：
//作者：GZC
//日期：2017.07.31
//////////////////////////////////////////////////////////////////////////
void AttSim::wahbaSimulation(double dt, int m, double FOV, double alinN)
{
	MatrixXd qA(m, 4), wTure(m, 3), qB(m, 4);
	double *noise1 = new double[m]; double *noise2 = new double[m]; double *noise3 = new double[m];
	double *fov1 = new double[m]; double *fov2= new double[m]; double *fov3 = new double[m];
	
	//根据程序启动时间得到一个随机数，作为种子放入星敏和陀螺的随机模型中
	int randcount;
	double randtmp[1];
	mBase.RandomDistribution(0, 30000, 1, 0, randtmp);
	randcount = randtmp[0];
	mBase.RandomDistribution(0, alinN, m, randcount + 0, noise1);
	mBase.RandomDistribution(0, alinN, m, randcount + 1, noise2);
	mBase.RandomDistribution(0, alinN, m, randcount + 2, noise3);
	mBase.RandomDistribution(0, FOV, m, randcount + 3, fov1);
	mBase.RandomDistribution(0, FOV, m, randcount + 4, fov2);
	mBase.RandomDistribution(0, FOV, m, randcount + 5, fov3);
	double qSim[4],alinSim[3];
	mBase.RandomDistribution(0, 1, 4, randcount + 6, qSim);
	mBase.RandomDistribution(0, 90, 3, randcount + 7, alinAB);
	alinAB[0] = alinAB[0]/180*PI, alinAB[1] = alinAB[1] / 180 * PI, alinAB[2] = alinAB[2] / 180 * PI;
	double qSimAll = sqrt(pow(qSim[0], 2) + pow(qSim[1], 2) + pow(qSim[2], 2) + pow(qSim[3], 2));
	qInitial[0] = qSim[0] / qSimAll, qInitial[1] = qSim[1] / qSimAll, qInitial[2] = qSim[2] / qSimAll, qInitial[3] = qSim[3] / qSimAll;
	//四元数在Matrix矩阵中顺序为1234, qTure(0,0)对应1,qTure(0,3)对应4，为标量
	double alinMeas[9], Ra[9], Rb[9],RqA[9],qAtrue[4],qRand[4];
	string path = workpath + "\\alin_set.txt";
	FILE *fp = fopen(path.c_str(), "w");
	for (int i = 0; i < m; i++)
	{
		mBase.Eulor2Matrix(fov1[i], fov2[i], fov3[i], 213, RqA);
		mBase.matrix2quat(RqA, qRand[0], qRand[1], qRand[2], qRand[3]);
		mBase.quatMult(qInitial, qRand, qAtrue);
		qA.row(i) << qAtrue[0], qAtrue[1], qAtrue[2], qAtrue[3];	
		mBase.Eulor2Matrix(alinAB[0] + noise1[i], alinAB[1] + noise2[i], alinAB[2] + noise3[i], 213, alinMeas);
		fprintf(fp, "%.5f\t%.5f\t%.5f\n", (alinAB[0] + noise1[i + 1]) / PI * 180,
			(alinAB[1] + noise2[i + 1]) / PI * 180, (alinAB[2] + noise3[i + 1]) / PI * 180);
		mBase.quat2matrix(qA(i, 0), qA(i, 1), qA(i, 2), qA(i, 3), Ra);
		mBase.Multi(alinMeas, Ra, Rb, 3, 3, 3);//Cbb
		mBase.matrix2quat(Rb, qB(i, 0), qB(i, 1), qB(i, 2), qB(i, 3));//Cbj
	}
	for (int i=0;i<m;i++)
	{		if (qA(i,3)<0)
		{			qA(i, 0) = -qA(i, 0), qA(i, 1) = -qA(i, 1), qA(i, 2) = -qA(i, 2), qA(i, 3) = -qA(i, 3);		}
			if (qB(i, 3) < 0)
			{		qB(i, 0) = -qB(i, 0), qB(i, 1) = -qB(i, 1), qB(i, 2) = -qB(i, 2), qB(i, 3) = -qB(i, 3);			}
			if (i == 0)
			{
				cout << setiosflags(ios::fixed);
				cout << setprecision(15) << "初始四元数： " << qA(i, 0) << ' ' << qA(i, 1) << ' ' << qA(i, 2) << ' ' << qA(i, 3) << endl;
				cout << "安装矩阵为： " << alinAB[0] * 180 / PI << ' ' << alinAB[1] * 180 / PI << ' ' << alinAB[2] * 180 / PI << endl;			}
	}
	alinCalculation(qA, qB);
	alinCalculation2(qA, qB);
	alinCalculation4(qA, qB);
	q_Method(qA, qB);
}

//////////////////////////////////////////////////////////////////////////
//功能：根据两颗星敏测量值求解星敏最优旋转矩阵
//输入：dt：姿态间隔时长（单位：秒）；	
//			 qInitial[4]：初始姿态四元数，最后为标量；	alinN：安装噪声
//输出：
//注意：
//作者：GZC
//日期：2017.07.28
//////////////////////////////////////////////////////////////////////////
void AttSim::alinCalculation(const MatrixXd qA, const MatrixXd  qB)
{
	int m = qA.size()/4;
	double RA[9], RB[9],Ru[9];
	double phi = 0, omg = 0, kap = 0, sinphi, sinomg, sinkap, cosphi, cosomg, coskap;

	//验证数据正确性
	string path = workpath + "\\alin_test.txt";
	FILE *fp = fopen(path.c_str(), "w");
	for (int i = 0; i < m; i++)
	{
		mBase.quat2matrix(qA(i, 0), qA(i, 1), qA(i, 2), qA(i, 3), RA);
		mBase.quat2matrix(qB(i, 0), qB(i, 1), qB(i, 2), qB(i, 3), RB);
		mBase.invers_matrix(RA, 3);
		mBase.Multi(RB, RA, Ru, 3, 3, 3);
		mBase.Matrix2Eulor(Ru, 213, phi, omg, kap);
		phi = phi / PI * 180, omg = omg / PI * 180, kap = kap / PI * 180;
		fprintf(fp, "%.5f\t%.5f\t%.5f\n", phi, omg, kap);
	}
	fclose(fp);

	mBase.quat2matrix(qA(0, 0), qA(0, 1), qA(0, 2), qA(0, 3), RA);
	mBase.quat2matrix(qB(0, 0), qB(0, 1), qB(0, 2), qB(0, 3), RB);
	mBase.invers_matrix(RA, 3);
	mBase.Multi(RB, RA, Ru, 3, 3, 3);
	mBase.Matrix2Eulor(Ru, 213, phi, omg, kap);

	//有关法化计算的数组
	double ATA[9], ATL[3], L;
	//设置迭代判断初值
	double iter_phi = PI, iter_omg = PI, iter_kap = PI, iter_count = 0;

	while (true)
	{
		memset(ATA, 0, sizeof(double) * 9);	memset(ATL, 0, sizeof(double) * 3);
		sinphi = sin(phi); sinomg = sin(omg); sinkap = sin(kap);
		cosphi = cos(phi); cosomg = cos(omg); coskap = cos(kap);
		for (int i = 0; i < m; i++)
		{
			double optic[] = { 0,0,1 };
			mBase.quat2matrix(qA(i, 0), qA(i, 1), qA(i, 2), qA(i, 3), RA);
			mBase.quat2matrix(qB(i, 0), qB(i, 1), qB(i, 2), qB(i, 3), RB);
			double optA[3], optB[3];
			//mBase.invers_matrix(RA, 3);
			//mBase.invers_matrix(RB, 3);
			mBase.Multi(RA, optic, optA, 3, 3, 1);
			mBase.Multi(RB, optic, optB, 3, 3, 1);

			mBase.Eulor2Matrix(phi, omg, kap, 213, Ru);
			//mBase.invers_matrix(Ru, 3);
			//组合为X_ Y_ Z_
			double Xbar, Ybar, Zbar;
			Xbar = Ru[0] * optA[0] + Ru[1] * optA[1] + Ru[2] * optA[2];
			Ybar = Ru[3] * optA[0] + Ru[4] * optA[1] + Ru[5] * optA[2];
			Zbar = Ru[6] * optA[0] + Ru[7] * optA[1] + Ru[8] * optA[2];
			//mBase.invers_matrix(Ru, 3);
			//旋转矩阵对各项的偏微分
			//对phi角的偏微分
			double partial_a1phi = -sinphi*coskap + cosphi*sinomg*sinkap;
			double partial_a2phi = 0;
			double partial_a3phi = -cosphi*coskap - sinphi*sinomg*sinkap;
			double partial_b1phi = sinphi*sinkap + cosphi*sinomg*coskap;
			double partial_b2phi = 0;
			double partial_b3phi = cosphi*sinkap - sinphi*sinomg*coskap;
			double partial_c1phi = cosphi*cosomg;
			double partial_c2phi = 0;
			double partial_c3phi = -sinphi*cosomg;
			//对omega角的偏微分
			double partial_a1omg = sinphi*cosomg*sinkap;
			double partial_a2omg = -sinomg*sinkap;
			double partial_a3omg = cosphi*cosomg*sinkap;
			double partial_b1omg = sinphi*cosomg*coskap;
			double partial_b2omg = -sinomg*coskap;
			double partial_b3omg = cosphi*cosomg*coskap;
			double partial_c1omg = -sinphi*sinomg;
			double partial_c2omg = -cosomg;
			double partial_c3omg = -cosphi*sinomg;
			//对kappa角的偏微分
			double partial_a1kap = -cosphi*sinkap + sinphi*sinomg*coskap;
			double partial_a2kap = cosomg*coskap;
			double partial_a3kap = sinphi*sinkap + cosphi*sinomg*coskap;
			double partial_b1kap = -cosphi*coskap - sinphi*sinomg*sinkap;
			double partial_b2kap = -cosomg*sinkap;
			double partial_b3kap = sinphi*coskap - cosphi*sinomg*sinkap;
			double partial_c1kap = 0;
			double partial_c2kap = 0;
			double partial_c3kap = 0;
			//求Xbar,Ybar,Zbar三者的偏导数
			double partial_Xbar_phi = optA[0] * partial_a1phi + optA[1] * partial_a2phi + optA[2] * partial_a3phi;
			double partial_Xbar_omg = optA[0] * partial_a1omg + optA[1] * partial_a2omg + optA[2] * partial_a3omg;
			double partial_Xbar_kap = optA[0] * partial_a1kap + optA[1] * partial_a2kap + optA[2] * partial_a3kap;
			double partial_Ybar_phi = optA[0] * partial_b1phi + optA[1] * partial_b2phi + optA[2] * partial_b3phi;
			double partial_Ybar_omg = optA[0] * partial_b1omg + optA[1] * partial_b2omg + optA[2] * partial_b3omg;
			double partial_Ybar_kap = optA[0] * partial_b1kap + optA[1] * partial_b2kap + optA[2] * partial_b3kap;
			double partial_Zbar_phi = optA[0] * partial_c1phi + optA[1] * partial_c2phi + optA[2] * partial_c3phi;
			double partial_Zbar_omg = optA[0] * partial_c1omg + optA[1] * partial_c2omg + optA[2] * partial_c3omg;
			double partial_Zbar_kap = optA[0] * partial_c1kap + optA[1] * partial_c2kap + optA[2] * partial_c3kap;
			
			//方程Vx法化
			double Ax[3], Ay[3], L;
			Ax[0] = partial_Xbar_phi;
			Ax[1] = partial_Xbar_omg;
			Ax[2] = partial_Xbar_kap;
			L = optB[0] - Xbar;
			mBase.pNormal(Ax, 3, L, ATA, ATL, 1.0);
			//方程Vy法化
			Ax[0] = partial_Ybar_phi;
			Ax[1] = partial_Ybar_omg;
			Ax[2] = partial_Ybar_kap;
			L = optB[1] - Ybar;
			mBase.pNormal(Ax, 3, L, ATA, ATL, 1.0);
			//方程Vz法化
			Ax[0] = partial_Zbar_phi;
			Ax[1] = partial_Zbar_omg;
			Ax[2] = partial_Zbar_kap;
			L = optB[2] - Zbar;
			mBase.pNormal(Ax, 3, L, ATA, ATL, 1.0);
		}
		//迭代求解
		mBase.solve33(ATA, ATL);
		if (abs(ATL[0]) > iter_phi&&abs(ATL[1]) > iter_omg&&abs(ATL[3]) > iter_kap || iter_count > 20)
			break;
		iter_count++;
		phi += ATL[0]; omg += ATL[1]; 	kap += ATL[2];
		//mBase.Eulor2Matrix(phi, omg, kap, 213, Ru);
		iter_phi = abs(ATL[0]), iter_omg = abs(ATL[1]), iter_kap = abs(ATL[2]);
	}
	cout<< "phi = " << phi*180/PI << " omg = " << omg * 180 / PI << " kap = " << kap * 180 / PI << endl;
}

//////////////////////////////////////////////////////////////////////////
//功能：根据两颗星敏测量值求解星敏最优旋转矩阵
//输入：dt：姿态间隔时长（单位：秒）；	
//			 qInitial[4]：初始姿态四元数，最后为标量；	alinN：安装噪声
//输出：
//注意：
//作者：GZC
//日期：2017.07.31
//////////////////////////////////////////////////////////////////////////
void AttSim::alinCalculation2(const MatrixXd qA, const MatrixXd  qB)
{
	int m = qA.size()/4;
	double RA[9], RB[9], Ru[9];
	double phi = 0, omg = 0, kap = 0, sinphi, sinomg, sinkap, cosphi, cosomg, coskap;

	//验证数据正确性
	string path = workpath + "\\alin_test.txt";
	FILE *fp = fopen(path.c_str(), "w");
	for (int i = 0; i < m; i++)
	{
		mBase.quat2matrix(qA(i, 0), qA(i, 1), qA(i, 2), qA(i, 3), RA);
		mBase.quat2matrix(qB(i, 0), qB(i, 1), qB(i, 2), qB(i, 3), RB);
		mBase.invers_matrix(RA, 3);
		mBase.Multi(RB, RA, Ru, 3, 3, 3);
		mBase.Matrix2Eulor(Ru, 213, phi, omg, kap);
		phi = phi / PI * 180, omg = omg / PI * 180, kap = kap / PI * 180;
		fprintf(fp, "%.5f\t%.5f\t%.5f\n", phi, omg, kap);
	}
	fclose(fp);

	//有关法化计算的数组
	//double*  matrixB = new double[16 * m];
	//double *L = new double[4 * m];
	double matrixB[16] = { 0 };
	double L[4] = { 0 };
	double correction[4], w[1], matrixB1[4], C[4];
	double qAm[4], qBm[4], qInit[4], qM[4];

	mBase.quat2matrix(qA(0, 0), qA(0, 1), qA(0, 2), qA(0, 3), RA);
	mBase.quat2matrix(qB(0, 0), qB(0, 1), qB(0, 2), qB(0, 3), RB);
	mBase.invers_matrix(RA, 3);
	mBase.Multi(RB, RA, Ru, 3, 3, 3);
	mBase.matrix2quat(Ru, qInit[0], qInit[1], qInit[2], qInit[3]);
	double Rres[9];
	mBase.Matrix2Eulor(Ru, 213, phi, omg, kap);
	//cout << "phi = " << phi * 180 / PI << " omg = " << omg * 180 / PI << " kap = " << kap * 180 / PI << endl;


	int N = 4, U = 4, S = 1;
	double Rba_Q1[] = { 0,0,0,1,  0,0,1,0,  0,-1,0,0,  -1,0,0,0 };
	double Rba_Q2[] = { 0,0,-1,0,  0,0,0,1,  1,0,0,0,  0,-1,0,0 };
	double Rba_Q3[] = { 0,1,0,0,  -1,0,0,0,  0,0,0,1,  0,0,-1,0 };
	double Rba_Q4[] = { 1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 };
	double iter_q1 = 1, iter_q2 = 2, iter_q3 = 3, iter_q4 = 4, iter_count=0;
	while (true)
	{
		for (int i=0;i<m;i++)
		{
			qAm[0] = qA(i, 0), qAm[1] = qA(i, 1), qAm[2] = qA(i, 2), qAm[3] = qA(i, 3);
			qBm[0] = qB(i, 0), qBm[1] = qB(i, 1), qBm[2] = qB(i, 2), qBm[3] = qB(i, 3);
			mBase.Multi(Rba_Q1, qAm, matrixB1, 4, 4, 1);
			matrixB[0] += matrixB1[0], matrixB[4] += matrixB1[1], matrixB[8] += matrixB1[2], matrixB[12] += matrixB1[3];
			mBase.Multi(Rba_Q2, qAm, matrixB1, 4, 4, 1);
			matrixB[1] += matrixB1[0], matrixB[5] += matrixB1[1], matrixB[9] += matrixB1[2], matrixB[13] += matrixB1[3];
			mBase.Multi(Rba_Q3, qAm, matrixB1, 4, 4, 1);
			matrixB[2] += matrixB1[0], matrixB[6] += matrixB1[1], matrixB[10] += matrixB1[2], matrixB[14] += matrixB1[3];
			mBase.Multi(Rba_Q4, qAm, matrixB1, 4, 4, 1);
			matrixB[3] += matrixB1[0], matrixB[7] += matrixB1[1], matrixB[11] += matrixB1[2], matrixB[15] += matrixB1[3];
			mBase.quatMult3(qInit, qAm, qM);
			L[0] += qM[0] - qBm[0], L[1] += qM[1] - qBm[1], L[2] += qM[2] - qBm[2], L[3] += qM[3] - qBm[3];
			//mBase.Multi(Rba_Q1, qAm, matrixB1, 4, 4, 1);
			//matrixB[16 * i + 0] = matrixB1[0], matrixB[16 * i + 4] = matrixB1[1], matrixB[16 * i + 8] = matrixB1[2], matrixB[16 * i + 12] = matrixB1[3];
			//mBase.Multi(Rba_Q2, qAm, matrixB1, 4, 4, 1);
			//matrixB[16 * i + 1] = matrixB1[0], matrixB[16 * i + 5] = matrixB1[1], matrixB[16 * i + 9] = matrixB1[2], matrixB[16 * i + 13] = matrixB1[3];
			//mBase.Multi(Rba_Q3, qAm, matrixB1, 4, 4, 1);
			//matrixB[16 * i + 2] = matrixB1[0], matrixB[16 * i + 6] = matrixB1[1], matrixB[16 * i + 10] = matrixB1[2], matrixB[16 * i + 14] = matrixB1[3];
			//mBase.Multi(Rba_Q4, qAm, matrixB1, 4, 4, 1);
			//matrixB[16 * i + 3] = matrixB1[0], matrixB[16 * i + 7] = matrixB1[1], matrixB[16 * i + 11] = matrixB1[2], matrixB[16 * i + 15] = matrixB1[3];
			//mBase.quatMult3(qInit, qAm, qM);
			//L[4 * i + 0] = qM[0] - qBm[0], L[4 * i + 1] = qM[1] - qBm[1], L[4 * i + 2] = qM[2] - qBm[2], L[4 * i + 3] = qM[3] - qBm[3];
			C[0] = 2 * qInit[0], C[1] = 2 * qInit[1], C[2] = 2 * qInit[2], C[3] = 2 * qInit[3];
			w[0] = pow(qInit[0], 2) + pow(qInit[1], 2) + pow(qInit[2], 2) + pow(qInit[3], 2) - 1;
		}
		GetCorrectionWithCondition<double>(correction, C, w, matrixB, L, N, U, S);
		if (abs(correction[0]) > iter_q1&&abs(correction[1]) > iter_q2&&abs(correction[2]) > iter_q3&&
			abs(correction[3]) > iter_q4 || iter_count > 20)
			break;
		iter_count++; 		
		qInit[0] -= correction[0], qInit[1] -= correction[1], qInit[2] -= correction[2], qInit[3] -= correction[3];
		mBase.NormVector(qInit, 4);
		//mBase.Eulor2Matrix(phi, omg, kap, 213, Ru);
		iter_q1 = abs(qInit[0]), iter_q2 = abs(qInit[1]), iter_q3 = abs(qInit[2]), iter_q4 = abs(qInit[3]);
	}
	//delete[]matrixB, L; matrixB = L = NULL;
	//double Rres[9];
	mBase.quat2matrix(qInit[0], qInit[1], qInit[2], qInit[3], Rres);
	mBase.Matrix2Eulor(Rres, 213, phi, omg, kap);
	cout << "phi = " << phi * 180 / PI << " omg = " << omg * 180 / PI << " kap = " << kap * 180 / PI << endl;
}

//////////////////////////////////////////////////////////////////////////
//功能：根据两颗星敏测量值求解星敏最优旋转矩阵
//输入：dt：姿态间隔时长（单位：秒）；	
//			 qInitial[4]：初始姿态四元数，最后为标量；	alinN：安装噪声
//输出：
//注意：
//作者：GZC
//日期：2017.08.02
//////////////////////////////////////////////////////////////////////////
void AttSim::alinCalculation3(const MatrixXd qA, const MatrixXd  qB)
{
	int m = qA.size() / 4;
	double RA[9], RB[9],Ru[9],qInit[4];
	mBase.quat2matrix(qA(0, 0), qA(0, 1), qA(0, 2), qA(0, 3), RA);
	mBase.quat2matrix(qB(0, 0), qB(0, 1), qB(0, 2), qB(0, 3), RB);
	mBase.invers_matrix(RA, 3);
	mBase.Multi(RB, RA, Ru, 3, 3, 3);
	mBase.matrix2quat(Ru, qInit[0], qInit[1], qInit[2], qInit[3]);

	double qAA[4], qBB[4];
	double Rba_Q1[] = { 0,0,0,1,  0,0,1,0,  0,-1,0,0,  -1,0,0,0 };
	double Rba_Q2[] = { 0,0,-1,0,  0,0,0,1,  1,0,0,0,  0,-1,0,0 };
	double Rba_Q3[] = { 0,1,0,0,  -1,0,0,0,  0,0,0,1,  0,0,-1,0 };
	double Rba_Q4[] = { 1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 };
	double iter_q1 = 1, iter_q2 = 1, iter_q3 = 1, iter_q4 = 1, iter_count = 0;
	//有关法化计算的数组
	double ATA[16], ATL[4], L;
	while (true)
	{
		double qL[4], Bx[4], Bxx[16];
		memset(Bxx, 0, sizeof(double) * 16), memset(Bx, 0, sizeof(double) * 4);
		for (int i = 0; i < m; i++)
		{
			qAA[0] = qA(i, 0), qAA[1] = qA(i, 1), qAA[2] = qA(i, 2), qAA[3] = qA(i, 3);
			qBB[0] = qB(i, 0), qBB[1] = qB(i, 1), qBB[2] = qB(i, 2), qBB[3] = qB(i, 3);
			mBase.quatMult(qAA, qInit, qL);
			qL[0] += qBB[0] - qL[0], qL[1] += qBB[1] - qL[1], qL[2] += qBB[2] - qL[2], qL[3] += qBB[3] - qL[3];
			
			mBase.Multi(Rba_Q1, qAA, Bx, 4, 4, 1);
			Bxx[0] += Bx[0], Bxx[4] += Bx[1], Bxx[8] += Bx[2], Bxx[12] += Bx[3];
			mBase.Multi(Rba_Q2, qAA, Bx, 4, 4, 1);
			Bxx[1] += Bx[0], Bxx[5] += Bx[1], Bxx[9] += Bx[2], Bxx[13] += Bx[3];
			mBase.Multi(Rba_Q3, qAA, Bx, 4, 4, 1);
			Bxx[2] += Bx[0], Bxx[6] += Bx[1], Bxx[10] += Bx[2], Bxx[14] += Bx[3];
			mBase.Multi(Rba_Q4, qAA, Bx, 4, 4, 1);
			Bxx[3] += Bx[0], Bxx[7] += Bx[1], Bxx[11] += Bx[2], Bxx[15] += Bx[3];
		}
		//迭代求解
		mBase.solve33(Bxx, qL);
		if (abs(qL[0]) > iter_q1&&abs(qL[1]) > iter_q2&&abs(qL[2]) > iter_q3&&
			abs(qL[3]) > iter_q4 || iter_count > 20)
			break;
		iter_count++;
		qInit[0] -= ATL[0], qInit[1] -= ATL[1], qInit[2] -= ATL[2], qInit[3] -= ATL[3];
		mBase.NormVector(qInit, 4);
		//mBase.Eulor2Matrix(phi, omg, kap, 213, Ru);
		iter_q1 = abs(qInit[0]), iter_q2 = abs(qInit[1]), iter_q3 = abs(qInit[2]), iter_q4 = abs(qInit[3]);
	}
	double Rres[9],phi,omg,kap;
	mBase.quat2matrix(qInit[0], qInit[1], qInit[2], qInit[3], Rres);
	mBase.Matrix2Eulor(Rres, 213, phi, omg, kap);
	cout << "phi = " << phi * 180 / PI << " omg = " << omg * 180 / PI << " kap = " << kap * 180 / PI << endl;
}

void  AttSim::alinCalculation4(const MatrixXd qA, const MatrixXd  qB)
{
	int m = qA.size() / 4;	
	MatrixXd RA(3, 3 * m), RB(3, 3 * m);
	for (int a = 0; a < m; a++)
	{
		double RAtmp[9], RBtmp[9];
		double optic[] = { 0,0,1 };
		mBase.quat2matrix(qA(a, 0), qA(a, 1), qA(a, 2), qA(a, 3), RAtmp);
		mBase.quat2matrix(qB(a, 0), qB(a, 1), qB(a, 2), qB(a, 3), RBtmp);
		Map<rMatrixXd> RAtmp2(RAtmp, 3, 3);
		Map<rMatrixXd> RBtmp2(RBtmp, 3, 3);
		RA.block<3, 3>(0, 3 * a) = RAtmp2;
		//cout << RA.block<3, 3>(0, 3 * a) << endl;
		RB.block<3, 3>(0, 3 * a) = RBtmp2;
	}
	MatrixXd Cba(3,3);
	Cba = RB*RA.transpose()*(RA*RA.transpose()).inverse();
	//cout << Cba << endl;
	double Cm[] = { Cba(0),Cba(3),Cba(6),Cba(1),Cba(4),Cba(7),  Cba(2),Cba(5) , Cba(8)};
	double phi, omg, kap;
	mBase.Matrix2Eulor(Cm, 213, phi, omg, kap);
	cout << "phi = " << phi * 180 / PI << " omg = " << omg * 180 / PI << " kap = " << kap * 180 / PI << endl;
}
//求星敏B相对于星敏A的安装Cba
void  AttSim::alinCalculation4(const MatrixXd qA, const MatrixXd  qB,double *Err)
{
	int m = qA.size() / 4;
	MatrixXd RA(3, 3 * m), RB(3, 3 * m);
	for (int a = 0; a < m; a++)
	{
		double RAtmp[9], RBtmp[9];
		double optic[] = { 0,0,1 };
		mBase.quat2matrix(qA(a, 0), qA(a, 1), qA(a, 2), qA(a, 3), RAtmp);
		mBase.quat2matrix(qB(a, 0), qB(a, 1), qB(a, 2), qB(a, 3), RBtmp);
		Map<rMatrixXd> RAtmp2(RAtmp, 3, 3);
		Map<rMatrixXd> RBtmp2(RBtmp, 3, 3);
		RA.block<3, 3>(0, 3 * a) = RAtmp2;
		RB.block<3, 3>(0, 3 * a) = RBtmp2;
	}
	MatrixXd Cba(3, 3);
	Cba = RB*RA.transpose()*(RA*RA.transpose()).inverse();
	double Cm[] = { Cba(0),Cba(3),Cba(6),Cba(1),Cba(4),Cba(7),  Cba(2),Cba(5) , Cba(8) };
	memcpy(Err, Cm, sizeof(double) * 9);
}
//求星敏C相对于星敏B的安装Ccb
void AttSim::twoStarErr(int m, Quat *qB, Quat *qC, double *Err)
{
	MatrixXd starB(m, 4), starC(m, 4);
	for (int i=0;i<m;i++)
	{
		starB(i, 0) = qB[i].Q1, starB(i, 1) = qB[i].Q2, starB(i, 2) = qB[i].Q3, starB(i, 3) = qB[i].Q0;
		starC(i, 0) = qC[i].Q1, starC(i, 1) = qC[i].Q2, starC(i, 2) = qC[i].Q3, starC(i, 3) = qC[i].Q0;
	}
	alinCalculation4(starB, starC, Err);
}
void  AttSim::q_Method(const MatrixXd qA, const MatrixXd  qB)
{
	int m = qA.size()/4;
	MatrixXd obs_in_startrackerframe(m, 3), stars_in_celestialframeV(m, 3);
	for (int a = 0; a < m; a++)
	{	
		double RA[9], RB[9];
		double optic[] = { 0,0,1 };
		mBase.quat2matrix(qA(a, 0), qA(a, 1), qA(a, 2), qA(a, 3), RA);
		mBase.quat2matrix(qB(a, 0), qB(a, 1), qB(a, 2), qB(a, 3), RB);
		double optA[3], optB[3];
		mBase.Multi(RA, optic, optA, 3, 3, 1);
		mBase.Multi(RB, optic, optB, 3, 3, 1);
		
		obs_in_startrackerframe.row(a) << optB[0], optB[1], optB[2];
		stars_in_celestialframeV.row(a) << optA[0], optA[1], optA[2];
	}
	MatrixXd W(3, m), V(3, m);
	W = obs_in_startrackerframe.transpose();
	V = stars_in_celestialframeV.transpose();
	Matrix3d B, S;
	B = W*V.transpose();
	S = B + B.transpose();
	Vector3d Z;
	Z << B(1, 2) - B(2, 1), B(2, 0) - B(0, 2), B(0, 1) - B(1, 0);
	double SIGMA = B.trace();
	Matrix<double, 4, 4>K;
	K << S - MatrixXd::Identity(3, 3)*SIGMA, Z, Z.transpose(), SIGMA;
	EigenSolver<Matrix<double, 4, 4>> es(K);
	MatrixXcd evecs = es.eigenvectors();
	MatrixXcd evals = es.eigenvalues();
	MatrixXd evalsReal;
	evalsReal = evals.real();
	MatrixXf::Index evalsMax;
	evalsReal.rowwise().sum().maxCoeff(&evalsMax);
	Vector4d q;
	q << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax), evecs.real()(3, evalsMax);
	//输出顺序为0123，标量为0
	double Rres[9],phi,omg,kap;
	mBase.quat2matrix(q(0), q(1), q(2), q(3), Rres);
	mBase.Matrix2Eulor(Rres, 213, phi, omg, kap);
	cout << "phi = " << phi * 180 / PI << " omg = " << omg * 180 / PI << " kap = " << kap * 180 / PI << endl;
	double dphi, domg, dkap;
	dphi = (phi - alinAB[0]) * 180 / PI, domg = (omg	- alinAB[1]) * 180 / PI, dkap = (kap - alinAB[2]) * 180 / PI;
	//cout << "dphi = " << dphi<< " domg = " << domg << " dkap = " << dkap << endl;

}

//将选择矩阵正交化
void AttSim::getInstall(double* Binstall, double* Cinstall)
{
	double eulerB[3], eulerC[3];
	mBase.Matrix2Eulor(const_cast<double*>(this->Binstall), 213, eulerB[0], eulerB[1], eulerB[2]);
	mBase.Matrix2Eulor(const_cast<double*>(this->Cinstall), 213, eulerC[0], eulerC[1], eulerC[2]);
	mBase.Eulor2Matrix(eulerB[0], eulerB[1], eulerB[2], 213, Binstall);
	mBase.Eulor2Matrix(eulerC[0], eulerC[1], eulerC[2], 213, Cinstall);
}
