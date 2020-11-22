#include "AttDetermination.h"



AttDetermination::AttDetermination()
{
}


AttDetermination::~AttDetermination()
{
}

//////////////////////////////////////////////////////////////////////////
//功能：姿态确定
//输入：AttData：从STG文件读取的数据，主要是陀螺数据
//			 getGCP：星图识别得到的所有星点控制
//			 Param：相机内畸变参数，num：参数数量
//输出：Quater：Kalman滤波结果四元数
//注意：采用的是星图的时标
//作者：GZC
//日期：2017.03.16
//////////////////////////////////////////////////////////////////////////
void AttDetermination::AttDeter(vector<STGData> AttData,
	vector<vector<StarGCP>>getGCP, StarCaliParam Param, vector<Quat>&Quater)
{
	vector<StarGCP>OneGCP;
	Quat QuaterTmp;
	vector<vector<StarGCP>>getGCPcopy(getGCP);
	vector<vector<StarGCP>>getGCPcopy2(getGCP);
	for (int a = 0; a < getGCP.size(); a++)
	{
		OneGCP.swap(getGCP[a]);
		q_Method(OneGCP, Param, QuaterTmp);
		Quater.push_back(QuaterTmp);
		OneGCP.clear();
	}
	OutputFile(Quater, "APS星敏四元数.txt");

	Quater.clear();
	Param.f = 2884.8;
	Param.x0 = 526.146;
	Param.y0 = 534.714;
	Param.k1 = 0;
	Param.k2 = 0;
	for (int a = 0; a < getGCPcopy.size(); a++)
	{
		OneGCP.swap(getGCPcopy[a]);
		q_Method(OneGCP, Param, QuaterTmp);
		Quater.push_back(QuaterTmp);
		OneGCP.clear();
	}
	OutputFile(Quater, "APS星敏四元数2.txt");

	Quater.clear();
	Param.f = 2886.667;
	Param.x0 = 512;
	Param.y0 = 512;
	Param.k1 = 0;
	Param.k2 = 0;
	for (int a = 0; a < getGCPcopy2.size(); a++)
	{
		OneGCP.swap(getGCPcopy2[a]);
		q_Method(OneGCP, Param, QuaterTmp);
		Quater.push_back(QuaterTmp);
		OneGCP.clear();
	}
	OutputFile(Quater, "APS星敏四元数3.txt");

	vector<Quat>EKFres;
	EKF6StateV3(AttData, Quater, EKFres);
	OutputFile(EKFres, "EKF滤波结果.txt");
	Quater.swap(EKFres);
}

//////////////////////////////////////////////////////////////////////////
//功能：姿态确定
//输入：AttData：从STG文件读取的数据，主要是陀螺数据
//			 getGCP：星图识别得到的所有星点控制
//输出：Quater：Kalman滤波结果四元数
//注意：会读取相机内畸变参数，采用的是星图的时标
//作者：GZC
//日期：2017.04.27
//////////////////////////////////////////////////////////////////////////
void AttDetermination::AttDeter2(vector<STGData> AttData, vector<vector<StarGCP>>getGCP)
{
	vector<StarCaliParam>caliParam;
	ReadCaliParam(caliParam);
	//StarCaliParam caliParam;
	//caliParam.x0 = 512; caliParam.y0 = 512; caliParam.f = 43.3/0.015; caliParam.k1 = 0; caliParam.k2 = 0;
	vector<StarGCP>OneGCP;
	Quat QuaterTmp;
	vector<Quat>Quater;
	for (int a = 0; a < getGCP.size(); a++)
	{
		OneGCP.swap(getGCP[a]);
		q_Method(OneGCP, caliParam[a], QuaterTmp);
		//q_Method(OneGCP, caliParam, QuaterTmp);
		Quater.push_back(QuaterTmp);
		OneGCP.clear();
	}
	OutputFile(Quater, "APS星敏四元数.txt");

	vector<Quat>EKFres;
	EKF6StateV3(AttData, Quater, EKFres);
	OutputFile(EKFres, "EKF滤波结果.txt");
	Quater.swap(EKFres);
}

//////////////////////////////////////////////////////////////////////////
//功能：姿态确定
//输入：AttData：从STG文件读取的数据
//输出：Quater：Kalman滤波结果四元数
//注意：
//作者：GZC
//日期：2017.05.02
//////////////////////////////////////////////////////////////////////////
void AttDetermination::AttDeter3(vector<STGData> AttData)
{
	//int m = AttData.size();
	//double *xest_store = new double[15 * m];
	//Quat *quatEst = new Quat[15 * m];
	//EKF15State(AttData, quatEst, xest_store);
	//outputXest15(m, xest_store);

	//int m = AttData.size();
	//double *xest_store = new double[15 * m];
	//Quat *quatEst = new Quat[m];
	//EKF15State(AttData, quatEst, xest_store);
	//OutputFile(quatEst, m, "ekf_aps.txt");
	//compareEKFAndAOCC(quatEst, m, "compare_ekf_aps.txt");
	//outputXest15(m, xest_store,"xest_store.txt");

	int m = AttData.size();
	Quat *quatEst = new Quat[m];
	double *xest_store = new double[6 * m];
	EKF6StateV8(AttData, quatEst, xest_store);
	OutputFile(quatEst, m, "ekf.txt");
	compareEKFAndAOCC(quatEst, m, "compare_ekf_aocc.txt");
	EKF6StateV9(AttData, quatEst, xest_store);
	OutputFile(quatEst, m, "ekf_bid.txt");
	outputXest(m, xest_store);
	compareEKFAndAOCC(quatEst, m, "compare_ekf__bid_aocc.txt");
	delete[] quatEst; quatEst = NULL;

	/*vector<Quat>attEst;
	EKF6StateV2(AttData, attEst, 23);
	OutputFile(attEst, "ekf_old.txt");
	compareEKFAndAOCC(attEst, "compare_ekf_old.txt");*/

	/*vector<Quat>EKFres;
	EKF6StateV4(AttData, EKFres);
	OutputFile(EKFres, "APS星上四元数EKF滤波结果.txt");*/
}

//////////////////////////////////////////////////////////////////////////
//功能：（APS）星敏姿态确定
//输入：AttData：从STG文件读取的数据
//输出：Quater：Kalman滤波结果四元数
//注意：
//作者：GZC
//日期：2017.11.29
//////////////////////////////////////////////////////////////////////////
void AttDetermination::AttDeter4(vector<STGData> AttData)
{
	vector<Quat>Quater, EKFres;
	for (int i = 0; i < AttData.size(); i++)
	{
		Quater.push_back(AttData[i].StarA);
	}
	EKF6StateV3(AttData, Quater, EKFres);
	OutputFile(EKFres, "EKF滤波结果.txt");
}
//////////////////////////////////////////////////////////////////////////
//功能：珞珈一号Ru参数更新
//输入：
//输出：
//注意：
//作者：GZC
//日期：2019.04.26
//////////////////////////////////////////////////////////////////////////
void AttDetermination::luojiaAlinFix(vector<Quat>LJCamera, Quat quater, SateEuler &ruEuler)
{
	Quat quater2 = LJCamera[1];
	double r1[9], r2[9], r3[9], r4[9], Ru[9];
	mbase.quat2matrix(quater.Q1, quater.Q2, quater.Q3, quater.Q0, r1);//恒星观测的Ccj
	mbase.quat2matrix(quater2.Q1, quater2.Q2, quater2.Q3, quater2.Q0, r2);//星敏测量转换的Ccj
	mbase.rot(-0.005949811481223, 0.015002138143471, 0.003740215940200, r3);//偏置矩阵Cbc
	//mbase.rot(-0, 0, 0, r3);//偏置矩阵Cbc
	mbase.Multi(r3, r2, r4, 3, 3, 3);//星敏测量的Cbj
	mbase.invers_matrix(r1, 3);//恒星观测的Cjc
	mbase.Multi(r4, r1, Ru, 3, 3, 3);//新的Ru参数
	mbase.invers_matrix(Ru, 3);//
	mbase.Matrix2Eulor(Ru, 213, ruEuler.R, ruEuler.P, ruEuler.Y);
}

//////////////////////////////////////////////////////////////////////////
//功能：吉林一号106偏置参数更新
//输入：
//输出：
//注意：
//作者：GZC
//日期：2020.07.28
//////////////////////////////////////////////////////////////////////////
void AttDetermination::jl106AlinFix(double R, double P, double Y, Quat starsensor, Quat camera, SateEuler& ruEuler)
{
	double r1[9], r2[9], r3[9], r4[9], Ru[9];
	mbase.quat2matrix(camera.Q1, camera.Q2, camera.Q3, camera.Q0, r1);//恒星观测的Cbj
	mbase.quat2matrix(starsensor.Q1, starsensor.Q2, starsensor.Q3, starsensor.Q0, r2);//星敏测量转换的Ccj
	//mbase.rot(0.0081118233393375015, -0.0058250636154377755, 2.3823500922100580, r3);//偏置矩阵Cbc,第一组参数计算所得
	//mbase.rot(0.0077724624168520199, -0.0064798179838863662, 2.6272696832905043, r3);//偏置矩阵Cbc,第三组参数计算所得
	mbase.rot(R, P, Y, r3);
	//mbase.rot(0, 0, 0, r3);//偏置矩阵Cbc
	mbase.Multi(r3, r2, r4, 3, 3, 3);//星敏测量的Cbj
	mbase.invers_matrix(r1, 3);//恒星观测的Cjb
	mbase.Multi(r4, r1, Ru, 3, 3, 3);//新的Ru参数
	mbase.Matrix2Eulor(Ru, 213, ruEuler.R, ruEuler.P, ruEuler.Y);
	//夹角
	double opt[3] = { 0,0,1 };
	double vstar[3], vcam[3];
	mbase.invers_matrix(r4, 3);//这里用Cjc也就是星敏光轴
	mbase.Multi(r1, opt, vcam, 3, 3, 1);
	mbase.Multi(r4, opt, vstar, 3, 3, 1);
	double ab = vcam[0] * vstar[0] + vcam[1] * vstar[1] + vcam[2] * vstar[2];
	double abm = sqrt(vcam[0] * vcam[0] + vcam[1] * vcam[1] + vcam[2] * vcam[2]) * sqrt(vstar[0] * vstar[0] + vstar[1] * vstar[1] + vstar[2] * vstar[2]);
	double alpha = ab / abm;
	double theta = acos((alpha > 0.999999999999999) ? 0.999999999999999 : (alpha < -0.999999999999999) ? -0.999999999999999 : alpha);
	ruEuler.UTC = theta / PI * 180 * 3600;//用utc替代一下
	//printf("%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ruEuler.R, ruEuler.P, ruEuler.Y, ruEuler.R / PI * 180 * 3600, ruEuler.P / PI * 180 * 3600, ruEuler.Y / PI * 180 * 3600, theta);
}
//////////////////////////////////////////////////////////////////////////
//功能：吉林一号106相机指向四元数计算
//输入：
//输出：
//注意：
//作者：GZC
//日期：2020.10.21
//////////////////////////////////////////////////////////////////////////
void AttDetermination::jl106CamQuat(double R, double P, double Y, Quat starsensor, Quat &camera,double &ra,double &dec)
{
	double r1[9], r2[9], r3[9], r4[9], Ru[9];
	mbase.quat2matrix(starsensor.Q1, starsensor.Q2, starsensor.Q3, starsensor.Q0, r2);//星敏测量转换的Ccj
	mbase.rot(R, P, Y, r3);
	mbase.Multi(r3, r2, r4, 3, 3, 3);//星敏测量的Cbj
	mbase.matrix2quat(r4, camera.Q1, camera.Q2, camera.Q3, camera.Q0);
	mbase.invers_matrix(r4, 3);
	double opt[3] = { 0,0,1 };
	double vcam[3];
	mbase.Multi(r4, opt, vcam, 3, 3, 1);
	dec = asin(vcam[2]);
	ra = asin(vcam[1]/cos(dec))/PI*180;
	dec = dec / PI * 180;
}

//////////////////////////////////////////////////////////////////////////
//功能：吉林一号106偏置参数更新
//输入：
//输出：
//注意：
//作者：GZC
//日期：2020.07.28
//更新：2020.11.21吉林一号107星
//////////////////////////////////////////////////////////////////////////
void AttDetermination::CalOptAngle(Quat starsensor, Quat camera, SateEuler& ruEuler)
{
	double r1[9], r2[9], r3[9], r4[9], Ru[9];
	double opt[3] = { 0,0,1 };
	double vstar[3], vcam[3];
	mbase.quat2matrix(camera.Q1, camera.Q2, camera.Q3, camera.Q0, r1);//恒星观测的Ccj
	mbase.quat2matrix(starsensor.Q1, starsensor.Q2, starsensor.Q3, starsensor.Q0, r2);//星敏测量转换的Crj
	mbase.invers_matrix(r1, 3);
	mbase.invers_matrix(r2, 3);
	//夹角
	mbase.Multi(r1,opt, vcam, 3, 3, 1);
	mbase.Multi(r2, opt, vstar, 3, 3, 1);
	double ab = vcam[0] * vstar[0] + vcam[1] * vstar[1] + vcam[2] * vstar[2];
	double abm = sqrt(vcam[0] * vcam[0] + vcam[1] * vcam[1] + vcam[2] * vcam[2]) * sqrt(vstar[0] * vstar[0] + vstar[1] * vstar[1] + vstar[2] * vstar[2]);
	double alpha = ab / abm;
	double theta = acos((alpha > 0.999999999999999) ? 0.999999999999999 : (alpha < -0.999999999999999) ? -0.999999999999999 : alpha);
	ruEuler.UTC = theta / PI * 180 * 3600;//用utc替代一下

	//欧拉角
	mbase.invers_matrix(r2,3);
	mbase.Multi(r2, r1, Ru, 3, 3, 3);//r2*r1t=Ru ---> r2=Ru*r1 ---> Rut*r2=r1
	mbase.Matrix2Eulor(Ru, 123, ruEuler.R, ruEuler.P, ruEuler.Y);
	//ruEuler.R = ruEuler.R / PI * 180 * 3600;
	//ruEuler.P = ruEuler.P / PI * 180 * 3600;
	//ruEuler.Y = ruEuler.Y / PI * 180 * 3600;


	//printf("%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ruEuler.R, ruEuler.P, ruEuler.Y, ruEuler.R / PI * 180 * 3600, ruEuler.P / PI * 180 * 3600, ruEuler.Y / PI * 180 * 3600, theta);
}
//////////////////////////////////////////////////////////////////////////
//功能：吉林一号107光轴角度计算
//输入：
//输出：
//注意：
//作者：GZC
//日期：2020.10.13
//////////////////////////////////////////////////////////////////////////
void AttDetermination::CalOptAngleforJL107(StarGCP cam, Quat starsensor, double &theta)
{
	double r2[9];
	double opt[3] = { 0,0,1 };
	double vstar[3], vcam[3];
	mbase.quat2matrix(starsensor.Q1, starsensor.Q2, starsensor.Q3, starsensor.Q0, r2);//星敏测量转换的Ccj
	mbase.invers_matrix(r2, 3);
	//夹角
	vcam[0] = cam.V[0]; vcam[1] = cam.V[1]; vcam[2] = cam.V[2];
	mbase.Multi(r2, opt, vstar, 3, 3, 1);
	double ab = vcam[0] * vstar[0] + vcam[1] * vstar[1] + vcam[2] * vstar[2];
	double abm = sqrt(vcam[0] * vcam[0] + vcam[1] * vcam[1] + vcam[2] * vcam[2]) * sqrt(vstar[0] * vstar[0] + vstar[1] * vstar[1] + vstar[2] * vstar[2]);
	double alpha = ab / abm;
	theta = acos((alpha > 0.999999999999999) ? 0.999999999999999 : (alpha < -0.999999999999999) ? -0.999999999999999 : alpha);
	theta = theta / PI * 180 * 3600;
}
void AttDetermination::compareRes(vector<Quat> attTrue, vector<Quat> attMeas, string resPath)
{
	//添加RMS指标(正确做法,2017.11.02)
	double rmsQ1, rmsQ2, rmsQ3;
	rmsQ1 = rmsQ2 = rmsQ3 = 0;
	double aveQ1, aveQ2, aveQ3;
	aveQ1 = aveQ2 = aveQ3 = 0;

	int num2 = attTrue.size();
	Quat dq1, dq2;
	Quat* dq3 = new Quat[num2];

	FILE* fp = fopen(resPath.c_str(), "w");
	//fprintf(fp, "%d\n", num2);
	for (int i = 0; i < num2; i++)
	{
		dq1.Q0 = -attTrue[i].Q0; dq1.Q1 = attTrue[i].Q1; dq1.Q2 = attTrue[i].Q2; dq1.Q3 = attTrue[i].Q3;
		dq2.Q0 = attMeas[i].Q0; dq2.Q1 = attMeas[i].Q1; dq2.Q2 = attMeas[i].Q2; dq2.Q3 = attMeas[i].Q3;
		mbase.quatMult(dq1, dq2, dq3[i]);
		dq3[i].Q1 = dq3[i].Q1 * 2 / PI * 180 * 3600;
		dq3[i].Q2 = dq3[i].Q2 * 2 / PI * 180 * 3600;
		dq3[i].Q3 = dq3[i].Q3 * 2 / PI * 180 * 3600;
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			attTrue[i].UTC, attTrue[i].Q1, attTrue[i].Q2, attTrue[i].Q3, attTrue[i].Q0,
			attMeas[i].Q1, attMeas[i].Q2, attMeas[i].Q3, attMeas[i].Q0, dq3[i].Q1, dq3[i].Q2, dq3[i].Q3);
		aveQ1 += dq3[i].Q1 / num2; aveQ2 += dq3[i].Q2 / num2; aveQ3 += dq3[i].Q3 / num2;
	}
	for (int i = 0; i < num2; i++)
	{
		rmsQ1 += pow(dq3[i].Q1 - aveQ1, 2);
		rmsQ2 += pow(dq3[i].Q2 - aveQ2, 2);
		rmsQ3 += pow(dq3[i].Q3 - aveQ3, 2);
	}
	rmsQ1 = sqrt(rmsQ1 / (num2 - 1)); rmsQ2 = sqrt(rmsQ2 / (num2 - 1)); rmsQ3 = sqrt(rmsQ3 / (num2 - 1));
	double rmsAll = sqrt(rmsQ1 * rmsQ1 + rmsQ2 * rmsQ2 + rmsQ3 * rmsQ3);
	fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\n", rmsQ1, rmsQ2, rmsQ3, rmsAll);
	fclose(fp);
	delete[] dq3; dq3 = NULL;
}
//////////////////////////////////////////////////////////////////////////
//功能：读取定标参数
//输入：定义类的对象时得到workplace的路径
//输出：StarCaliParam结构体的定标参数
//注意：这是5参数情况
//作者：GZC
//日期：2017.04.27
//////////////////////////////////////////////////////////////////////////
void AttDetermination::ReadCaliParam(vector<StarCaliParam>&caliParam)
{
	string caliPath = workpath.substr(0, workpath.rfind('\\'));
	caliPath = caliPath.substr(0, caliPath.rfind('\\'));
	caliPath = caliPath + "\\星图\\定标结果！！！.txt";
	FILE *fp = fopen(caliPath.c_str(), "r");
	StarCaliParam caliParamTmp;
	while (!feof(fp))
	{
		fscanf(fp, "%*s\t%lf\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\n", &caliParamTmp.x0, &caliParamTmp.y0,
			&caliParamTmp.f, &caliParamTmp.k1, &caliParamTmp.k2);
		caliParam.push_back(caliParamTmp);
	}
}


//////////////////////////////////////////////////////////////////////////
//功能：q_Method定姿方式
//输入：getGCP：获取的星点控制，Param：相机内畸变参数
//输出：四元数quater
//注意：根据相机模型的不同选择不同参数(5参数或者6参数)
//作者：GZC
//日期：2017.01.12，更新：2017.03.16
//			 2019.04.23更新，根据珞珈一号更改Xp和Yp计算方式
//////////////////////////////////////////////////////////////////////////
bool AttDetermination::q_Method(vector<StarGCP> getGCP, StarCaliParam Param, Quat &quater)
{
	MatrixXd obs_in_startrackerframe(getGCP.size(), 3), stars_in_celestialframeV(getGCP.size(), 3);
	double Wob[3];
	double X, Y, DetX, DetY;
	for (int a = 0; a < getGCP.size(); a++)
	{
		double Xp, Yp;
		//Xp = Param.x0*2 - getGCP[a].y;		Yp = getGCP[a].x;
		Xp = getGCP[a].x;		Yp = getGCP[a].y;
		//畸变模型(5参数情况)
		double r2 = (Xp - Param.x0)*(Xp - Param.x0) + (Yp - Param.y0)*(Yp - Param.y0);
		double xreal = -(Xp - Param.x0)*(1 - Param.k1 * r2 - Param.k2 * r2*r2);
		double yreal = -(Yp - Param.y0)*(1 - Param.k1 * r2 - Param.k2 * r2*r2);
		double freal = Param.f;

		//赋值，将像方坐标系转到星敏坐标系，星敏坐标系为-(X-X0)，-(Y-Y0)，f；然后
		double D = sqrt(pow(xreal, 2) + pow(yreal, 2) + pow(freal, 2));
		//double Wob[3];
		Wob[0] = xreal / D;
		Wob[1] = yreal / D;
		Wob[2] = freal / D;
		obs_in_startrackerframe.row(a) << Wob[0], Wob[1], Wob[2];
		stars_in_celestialframeV.row(a) << getGCP[a].V[0], getGCP[a].V[1], getGCP[a].V[2];
	}
	MatrixXd W(3, getGCP.size()), V(3, getGCP.size());
	//这里的V和W对应关系需要注意，如果是随机星点仿星敏控制点然后乘以R，就是星敏观测量就是V，如果是任意恒星赤经赤纬经过R投影到星图，则星敏观测量是W
	V = obs_in_startrackerframe.transpose();
	W = stars_in_celestialframeV.transpose();
	Matrix3d B, S;
	B = W * V.transpose();
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
	quater.Q0 = q(3); quater.Q1 = q(0); quater.Q2 = q(1); quater.Q3 = q(2);
	quater.UTC = getGCP[0].UTC;
	//if (quater.Q0 < 0)
	//{
	//	quater.Q0 = -quater.Q0;
	//	quater.Q1 = -quater.Q1;
	//	quater.Q2 = -quater.Q2;
	//	quater.Q3 = -quater.Q3;
	//}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//功能：q_Method定姿方式
//输入：getGCP：获取的星点控制，Param：相机内畸变参数
//输出：四元数quater
//注意：根据相机模型的不同选择不同参数(5参数或者6参数)
//作者：GZC
//日期：2017.01.12，更新：2017.03.16
//			 2019.04.23更新，根据珞珈一号更改Xp和Yp计算方式
//////////////////////////////////////////////////////////////////////////
bool AttDetermination::q_MethodforJL106(vector<StarGCP> getGCP, StarCaliParam Param, Quat& quater)
{
	MatrixXd obs_in_startrackerframe(getGCP.size(), 3), stars_in_celestialframeV(getGCP.size(), 3);
	double Wob[3];
	double X, Y, DetX, DetY;
	for (int a = 0; a < getGCP.size(); a++)
	{
		double Xp, Yp;
		//Xp = Param.x0*2 - getGCP[a].x;		
		//Yp = getGCP[a].y;
		Xp = getGCP[a].x;  Yp = Param.y0 * 2 - getGCP[a].y;//y坐标是反的
		//畸变模型(5参数情况)
		double r2 = (Xp - Param.x0) * (Xp - Param.x0) + (Yp - Param.y0) * (Yp - Param.y0);
		double xreal = -(Xp - Param.x0) * (1 - Param.k1 * r2 - Param.k2 * r2 * r2);
		double yreal = -(Yp - Param.y0) * (1 - Param.k1 * r2 - Param.k2 * r2 * r2);
		double freal = Param.f;

		//赋值，将像方坐标系转到星敏坐标系，星敏坐标系为-(X-X0)，-(Y-Y0)，f；然后
		double D = sqrt(pow(xreal, 2) + pow(yreal, 2) + pow(freal, 2));
		//double Wob[3];
		Wob[0] = xreal / D;
		Wob[1] = yreal / D;
		Wob[2] = freal / D;
		obs_in_startrackerframe.row(a) << Wob[0], Wob[1], Wob[2];
		stars_in_celestialframeV.row(a) << getGCP[a].V[0], getGCP[a].V[1], getGCP[a].V[2];
	}
	MatrixXd W(3, getGCP.size()), V(3, getGCP.size());
	W = obs_in_startrackerframe.transpose();
	V = stars_in_celestialframeV.transpose();
	Matrix3d B, S;
	B = W * V.transpose();
	S = B + B.transpose();
	Vector3d Z;
	Z << B(1, 2) - B(2, 1), B(2, 0) - B(0, 2), B(0, 1) - B(1, 0);
	double SIGMA = B.trace();
	Matrix<double, 4, 4>K;
	K << S - MatrixXd::Identity(3, 3) * SIGMA, Z, Z.transpose(), SIGMA;
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
	quater.Q0 = q(3); quater.Q1 = q(0); quater.Q2 = q(1); quater.Q3 = q(2);
	quater.UTC = getGCP[0].UTC;
	if (quater.Q0 < 0)
	{
		quater.Q0 = -quater.Q0;
		quater.Q1 = -quater.Q1;
		quater.Q2 = -quater.Q2;
		quater.Q3 = -quater.Q3;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//功能：q_Method定姿方式，添加了光行差部分
//输入：getGCP：获取的星点控制，Param：相机内畸变参数
//输出：四元数quater
//注意：根据相机模型的不同选择不同参数(5参数或者6参数)
//作者：GZC
//日期：2017.01.12，更新：2017.03.16
//			 2019.04.23更新，根据珞珈一号更改Xp和Yp计算方式
//			 2019.06.10更新，添加了光行差修正
//////////////////////////////////////////////////////////////////////////
bool AttDetermination::q_MethodForLuojia(vector<StarGCP> getGCP, StarCaliParam Param, Quat &quater, vector<Orbit_Ep>imgOrb)
{
	MatrixXd obs_in_startrackerframe(getGCP.size(), 3), stars_in_celestialframeV(getGCP.size(), 3);
	double Wob[3];
	double X, Y, DetX, DetY;
	for (int a = 0; a < getGCP.size(); a++)
	{
		double Xp, Yp;
		//Xp = Param.x0*2 - getGCP[a].y;		Yp = getGCP[a].x;
		Xp = getGCP[a].x;		Yp = getGCP[a].y;
		//畸变模型(5参数情况)
		double r2 = (Xp - Param.x0)*(Xp - Param.x0) + (Yp - Param.y0)*(Yp - Param.y0);
		double xreal = -(Xp - Param.x0)*(1 - Param.k1 * r2 - Param.k2 * r2*r2);
		double yreal = -(Yp - Param.y0)*(1 - Param.k1 * r2 - Param.k2 * r2*r2);
		double freal = Param.f;

		//赋值，将像方坐标系转到星敏坐标系，星敏坐标系为-(X-X0)，-(Y-Y0)，f；然后
		double D = sqrt(pow(xreal, 2) + pow(yreal, 2) + pow(freal, 2));
		//double Wob[3];
		Wob[0] = xreal / D;
		Wob[1] = yreal / D;
		Wob[2] = freal / D;
		obs_in_startrackerframe.row(a) << Wob[0], Wob[1], Wob[2];

		//光行差修正
		StarGCP vTemp = getGCP[a];
		//AberrationForLuojia(vTemp,imgOrb);
		stars_in_celestialframeV.row(a) << vTemp.V[0], vTemp.V[1], vTemp.V[2];
		//stars_in_celestialframeV.row(a) << getGCP[a].V[0], getGCP[a].V[1], getGCP[a].V[2];
	}

	//q_Method
	MatrixXd W(3, getGCP.size()), V(3, getGCP.size());
	W = obs_in_startrackerframe.transpose();
	V = stars_in_celestialframeV.transpose();
	Matrix3d B, S;
	B = W * V.transpose();
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
	quater.Q0 = q(3); quater.Q1 = q(0); quater.Q2 = q(1); quater.Q3 = q(2);
	quater.UTC = getGCP[0].UTC;
	if (quater.Q0 < 0)
	{
		quater.Q0 = -quater.Q0;
		quater.Q1 = -quater.Q1;
		quater.Q2 = -quater.Q2;
		quater.Q3 = -quater.Q3;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//功能：输出姿态数据
//输入：Att：四元数
//输出：FileName：txt文件名
//注意：输出的四元数顺序为0123，第一个为标量
//作者：GZC
//日期：2017.03.23 updata 2017.04.27 updata 2017.07.12（重载）
//////////////////////////////////////////////////////////////////////////
void AttDetermination::OutputFile(vector<Quat>Att, string FileName)
{
	string path = workpath;
	path = path.substr(0, path.rfind('\\') + 1) + FileName;
	FILE *fp = fopen(path.c_str(), "w");
	fprintf(fp, "%s\n", "ZY302");
	fprintf(fp, "%s\n", "ZY302");
	fprintf(fp, "%d\n", Att.size());
	for (int a = 0; a < Att.size(); a++)
	{
		fprintf(fp, "%s\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\n", "Quater", Att[a].UTC, Att[a].Q0, Att[a].Q1, Att[a].Q2, Att[a].Q3);
	}
	fclose(fp);
}
void AttDetermination::OutputFile(Quat *Att, int num, string FileName)
{
	string path = workpath + FileName;
	FILE *fp = fopen(path.c_str(), "w");
	fprintf(fp, "%s\n", "ZY302");
	fprintf(fp, "%s\n", "ZY302");
	fprintf(fp, "%d\n", num);
	for (int a = 0; a < num; a++)
	{
		fprintf(fp, "%s\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\n", "Quater", Att[a].UTC, Att[a].Q0, Att[a].Q1, Att[a].Q2, Att[a].Q3);
	}
	fclose(fp);
}
//////////////////////////////////////////////////////////////////////////
//功能：输出状态估值
//输入：
//输出：
//注意：
//作者：GZC
//日期：2017.07.10
//////////////////////////////////////////////////////////////////////////
void AttDetermination::outputXest(int num, double * xest_store)
{
	int npos = workpath.rfind('\\');
	string filetmp;
	if (npos != string::npos)
	{
		filetmp = workpath.substr(0, npos);
		filetmp = filetmp + "\\ekf_bias.txt";
	}
	FILE *fp = fopen(filetmp.c_str(), "w");
	for (int i = 0; i < num; i++)
	{
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\n",
			xest_store[6 * i + 0], xest_store[6 * i + 3] / PI * 180 * 3600,
			xest_store[6 * i + 4] / PI * 180 * 3600, xest_store[6 * i + 5] / PI * 180 * 3600);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：输出状态估值
//输入：
//输出：
//注意：
//作者：GZC
//日期：2017.07.10
//////////////////////////////////////////////////////////////////////////
void AttDetermination::outputXest15(int num, double * xest_store, string res)
{
	string path = workpath;
	path = path.substr(0, path.rfind('\\') + 1) + res;
	FILE *fp = fopen(path.c_str(), "w");
	for (int i = 0; i < num; i++)
	{
		fprintf(fp, "%.6f\t%.9f\t%.9f\t%.9f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\n",
			xest_store[15 * i + 0], xest_store[15 * i + 3],
			xest_store[15 * i + 4], xest_store[15 * i + 5], xest_store[15 * i + 6], xest_store[15 * i + 7]
			, xest_store[15 * i + 8], xest_store[15 * i + 9], xest_store[15 * i + 10], xest_store[15 * i + 11]
			, xest_store[15 * i + 12], xest_store[15 * i + 13], xest_store[15 * i + 14]);
	}
	fclose(fp);
}
const double AttDetermination::Ainstall[] =//Crb
{
	cos(29.13212 / 180 * PI),cos(104.8425 / 180 * PI),cos(65.5449 / 180 * PI),
	cos(75.98529 / 180 * PI),cos(120.6005 / 180 * PI),cos(145.6867 / 180 * PI),
	cos(65.0185 / 180 * PI),cos(34.7422 / 180 * PI),cos(112.497 / 180 * PI)
};
const double AttDetermination::Binstall[9] =
{
	cos(21.68954 / 180 * PI),cos(92.60966 / 180 * PI),cos(111.5162 / 180 * PI),
	cos(102.7403 / 180 * PI),cos(149.8423 / 180 * PI),cos(116.833 / 180 * PI),
	cos(107.2508 / 180 * PI),cos(59.97562 / 180 * PI),cos(144.4336 / 180 * PI)
};
const double AttDetermination::Cinstall[9] =
{
	cos(63.63085 / 180 * PI),cos(92.72818 / 180 * PI),cos(26.53156 / 180 * PI),
	cos(68.95412 / 180 * PI),cos(154.8806 / 180 * PI),cos(103.0838 / 180 * PI),
	cos(34.83029 / 180 * PI),cos(65.0493 / 180 * PI),cos(112.6469 / 180 * PI)
};
//const double AttDetermination::GyroIns[9]=//Crb,ZY3-01星陀螺安装矩阵
//{
//	cos(54.7604/180*PI),	cos(90.0868/180*PI),cos(35.2731/180*PI),
//	cos(54.6596/180*PI),	cos(134.9521/180*PI),cos(114.0841/180*PI),
//	cos(54.8202/180*PI),	cos(45.0095/180*PI),cos(114.2353/180*PI)
//};
const double AttDetermination::GyroIns[9] =//Crb,ZY3-02星陀螺安装矩阵
{
	cos(54.72148 / 180 * PI),	cos(89.9605 / 180 * PI),cos(35.25769 / 180 * PI),
	cos(54.72435 / 180 * PI),	cos(135.0281 / 180 * PI),cos(114.0297 / 180 * PI),
	cos(54.77499 / 180 * PI),	cos(44.97068 / 180 * PI),cos(114.0914 / 180 * PI)
};

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼主程序
//输入：AttData:STG解析出的姿态数据	Res:结果输出路径		StarTag:星敏标识
//输出：AttDet:姿态确定结果
//注意：先进行双矢量再定姿
//作者：GZC
//日期：2015.11.15
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV2(vector<STGData> AttData, vector<Quat> &AttDet, int StarTag)
{
	int i;
	vector<Quat> AttDet2Vec;
	DoubleStar(AttData, AttDet2Vec, StarTag);
	int m = AttDet2Vec.size();
	MatrixXd StarDat(m, 4), Wgm(m, 3), Qest(m, 4);
	double GyDat[3], GyTran[3];
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	for (i = 0; i < m; i++)
	{
		StarDat(i, 0) = AttDet2Vec.at(i).Q1, StarDat(i, 1) = AttDet2Vec.at(i).Q2;
		StarDat(i, 2) = AttDet2Vec.at(i).Q3, StarDat(i, 3) = AttDet2Vec.at(i).Q0;
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		Wgm(i, 0) = GyTran[0] / 180 * PI, Wgm(i, 1) = GyTran[1] / 180 * PI, Wgm(i, 2) = GyTran[2] / 180 * PI;
	}
	double dt = 0.25;
	Qest(0, 0) = StarDat(0, 0), Qest(0, 1) = StarDat(0, 1), Qest(0, 2) = StarDat(0, 2), Qest(0, 3) = StarDat(0, 3);
	double a1 = Qest(0, 0), a2 = Qest(0, 1), a3 = Qest(0, 2), a4 = Qest(0, 3);

	double sig = 1e-6;
	Matrix3d zero33, eye33, poa, pog, r, sigu, sigv;
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu << 2e-20*eye33;//陀螺漂移噪声
	sigv << 2e-14*eye33;//陀螺噪声
	poa << 1e-7*eye33;//初始姿态误差协方差
	pog << 1e-5*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), be(m, 3), we(m, 3), xest(m, 6);
	eye66 << eye33, zero33, zero33, eye33;
	be.row(0) << 0, 0, 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv, zero33, zero33, sigu;//过程噪声

	string path = workpath;
	path = path.substr(0, path.rfind('\\'));
	string strpath = path + "GyroBiasEstimate.txt";
	string strpath1 = path + "EKF.txt";
	FILE *fpres = fopen(strpath.c_str(), "w");
	FILE *fpEKF = fopen(strpath1.c_str(), "w");

	int j = 0;
	while (j < 2)
	{
		for (i = 0; i < m - 1; i++)
		{
			double qmm1, qmm2, qmm3;
			qmm1 = -StarDat(i, 3)*Qest(i, 0) - StarDat(i, 2)*Qest(i, 1) + StarDat(i, 1)*Qest(i, 2) + StarDat(i, 0)*Qest(i, 3);
			qmm2 = StarDat(i, 2)*Qest(i, 0) - StarDat(i, 3)*Qest(i, 1) - StarDat(i, 0)*Qest(i, 2) + StarDat(i, 1)*Qest(i, 3);
			qmm3 = -StarDat(i, 1)*Qest(i, 0) + StarDat(i, 0)*Qest(i, 1) - StarDat(i, 3)*Qest(i, 2) + StarDat(i, 2)*Qest(i, 3);
			MatrixXd z(3, 1);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			//cout<<"观测残差："<<z.transpose()<<endl;
			MatrixXd h(3, 6), k(6, 3);
			h << eye33, zero33;
			k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
			//cout<<k<<endl;
			p = (eye66 - k * h)*p;
			//cout<<"p"<<p<<endl;
			xest.row(i) = xest.row(i) + (k*z).transpose();
			//cout<<xest.row(i);

			MatrixXd xe(1, 3);
			xe = 0.5*xest.row(i).head(3);
			double qe11, qe22, qe33, qe44;
			qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
			qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
			qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
			qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
			MatrixXd tempqe(4, 1);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			//cout<<Qest.row(i)<<endl;

			//Propagate Covariance
			//cout<<Wgm.row(i)<<endl;
			//cout<<xest.row(i).tail(3)<<endl;
			we.row(i) = Wgm.row(i) - xest.row(i).tail(3);
			double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
			Matrix3d wa;
			//wa<<0,we(i,2),-we(i,1),-we(i,2),0,we(i,0),we(i,1),-we(i,0),0;
			wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
			//cout<<wa<<endl;
			MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			//gamma=gmat*dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//cout<<phi<<endl;
			//cout<<gamma<<endl;

			//Propagate State
			double qw1, qw2, qw3, qw4;
			qw1 = we(i, 0) / w * sin(0.5*w*dt);
			qw2 = we(i, 1) / w * sin(0.5*w*dt);
			qw3 = we(i, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			MatrixXd om(4, 4);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			//cout<<om<<endl;
			Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();
			//cout<<Qest.row(i+1)<<endl;

			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(i + 1) = xest.row(i);
			xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;
			//cout<<xest.row(i)<<endl;

			fprintf(fpres, "%lf\t%lf\t%lf\n", xest(i, 3) * 180 / PI * 3600,
				xest(i, 4) * 180 / PI * 3600, xest(i, 5) * 180 / PI * 3600);
		}
		j++;
		xest(0, 3) = xest(m - 1, 3), xest(0, 4) = xest(m - 1, 4), xest(0, 5) = xest(m - 1, 5);
	}

	Quat EKFres;
	fprintf(fpEKF, "%d\n", m);
	for (i = 0; i < m; i++)
	{
		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", AttData[i].utgyro, Qest(i, 3), Qest(i, 0), Qest(i, 1), Qest(i, 2));
		EKFres.UTC = AttData[i].utgyro, EKFres.Q0 = Qest(i, 3), EKFres.Q1 = Qest(i, 0), EKFres.Q2 = Qest(i, 1), EKFres.Q3 = Qest(i, 2);
		AttDet.push_back(EKFres);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波主程序
//输入：AttData：STG解析出的姿态数据	
//			 APSdat：APS星敏定姿四元数
//输出：EKFatt：姿态确定结果
//注意：
//作者：GZC
//日期：2017.03.19
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV3(vector<STGData> AttData, vector<Quat> APSdat, vector<Quat>&EKFres)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	double *UT = new double[m];
	MatrixXd StarDat(m, 4), Wgm(m, 3), Qest(m, 4);
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		Wgm(i, 0) = GyTran[0] / 180 * PI, Wgm(i, 1) = GyTran[1] / 180 * PI, Wgm(i, 2) = GyTran[2] / 180 * PI;
		UT[i] = AttData[i].utgyro;
	}
	Quat *Quat_inter = new Quat[m];
	alinAPS(APSdat);//乘以安装矩阵
	mbase.QuatInterpolation(APSdat, UT, m, Quat_inter);

	for (size_t i = 0; i < m; i++)
	{
		StarDat(i, 0) = Quat_inter[i].Q1, StarDat(i, 1) = Quat_inter[i].Q2;
		StarDat(i, 2) = Quat_inter[i].Q3, StarDat(i, 3) = Quat_inter[i].Q0;
	}
	double dt = 0.25;
	Qest(0, 0) = StarDat(0, 0), Qest(0, 1) = StarDat(0, 1), Qest(0, 2) = StarDat(0, 2), Qest(0, 3) = StarDat(0, 3);
	double a1 = Qest(0, 0), a2 = Qest(0, 1), a3 = Qest(0, 2), a4 = Qest(0, 3);

	double sig = 1e-6;
	Matrix3d zero33, eye33, poa, pog, r, sigu, sigv;
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu << 2e-20*eye33;//陀螺漂移噪声
	sigv << 2e-14*eye33;//陀螺噪声
	poa << 1e-7*eye33;//初始姿态误差协方差
	pog << 1e-5*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), be(m, 3), we(m, 3), xest(m, 6);
	eye66 << eye33, zero33, zero33, eye33;
	be.row(0) << 0, 0, 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv, zero33, zero33, sigu;//过程噪声

	int j = 0;
	while (j < 2)
	{
		for (size_t i = 0; i < m - 1; i++)
		{
			double qmm1, qmm2, qmm3;
			qmm1 = -StarDat(i, 3)*Qest(i, 0) - StarDat(i, 2)*Qest(i, 1) + StarDat(i, 1)*Qest(i, 2) + StarDat(i, 0)*Qest(i, 3);
			qmm2 = StarDat(i, 2)*Qest(i, 0) - StarDat(i, 3)*Qest(i, 1) - StarDat(i, 0)*Qest(i, 2) + StarDat(i, 1)*Qest(i, 3);
			qmm3 = -StarDat(i, 1)*Qest(i, 0) + StarDat(i, 0)*Qest(i, 1) - StarDat(i, 3)*Qest(i, 2) + StarDat(i, 2)*Qest(i, 3);
			MatrixXd z(3, 1);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			//cout<<"观测残差："<<z.transpose()<<endl;
			MatrixXd h(3, 6), k(6, 3);
			h << eye33, zero33;
			k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
			//cout<<k<<endl;
			p = (eye66 - k * h)*p;
			//cout<<"p"<<p<<endl;
			xest.row(i) = xest.row(i) + (k*z).transpose();
			//cout<<xest.row(i);

			MatrixXd xe(1, 3);
			xe = 0.5*xest.row(i).head(3);
			double qe11, qe22, qe33, qe44;
			qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
			qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
			qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
			qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
			MatrixXd tempqe(4, 1);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			//cout<<Qest.row(i)<<endl;

			//Propagate Covariance
			//cout<<Wgm.row(i)<<endl;
			//cout<<xest.row(i).tail(3)<<endl;
			we.row(i) = Wgm.row(i) - xest.row(i).tail(3);
			double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
			Matrix3d wa;
			//wa<<0,we(i,2),-we(i,1),-we(i,2),0,we(i,0),we(i,1),-we(i,0),0;
			wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
			//cout<<wa<<endl;
			MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			//gamma=gmat*dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//cout<<phi<<endl;
			//cout<<gamma<<endl;

			//Propagate State
			double qw1, qw2, qw3, qw4;
			qw1 = we(i, 0) / w * sin(0.5*w*dt);
			qw2 = we(i, 1) / w * sin(0.5*w*dt);
			qw3 = we(i, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			MatrixXd om(4, 4);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			//cout<<om<<endl;
			Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();
			//cout<<Qest.row(i+1)<<endl;

			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(i + 1) = xest.row(i);
			xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;
			//cout<<xest.row(i)<<endl;

		}
		j++;
		xest(0, 3) = xest(m - 1, 3), xest(0, 4) = xest(m - 1, 4), xest(0, 5) = xest(m - 1, 5);
	}

	Quat EKFresTmp;
	for (size_t i = 0; i < m; i++)
	{
		EKFresTmp.UTC = AttData[i].utgyro, EKFresTmp.Q0 = Qest(i, 3), EKFresTmp.Q1 = Qest(i, 0),
			EKFresTmp.Q2 = Qest(i, 1), EKFresTmp.Q3 = Qest(i, 2);
		EKFres.push_back(EKFresTmp);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波主程序
//输入：AttData：STG解析出的姿态数据	
//输出：EKFatt：姿态确定结果
//注意：
//作者：GZC
//日期：2017.05.02
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV4(vector<STGData> AttData, vector<Quat>&EKFres)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	double *UT = new double[m];
	MatrixXd StarDat(m, 4), Wgm(m, 3), Qest(m, 4);
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		Wgm(i, 0) = GyTran[0] / 180 * PI, Wgm(i, 1) = GyTran[1] / 180 * PI, Wgm(i, 2) = GyTran[2] / 180 * PI;
		UT[i] = AttData[i].utgyro;
	}
	vector<Quat>APSdat(m);
	for (size_t i = 0; i < m; i++)
	{
		APSdat[i].UTC = AttData[i].StarA.UTC;
		APSdat[i].Q0 = AttData[i].StarA.Q0; APSdat[i].Q1 = AttData[i].StarA.Q1;
		APSdat[i].Q2 = AttData[i].StarA.Q2; APSdat[i].Q3 = AttData[i].StarA.Q3;
	}
	alinAPS(APSdat);//乘以安装矩阵
	Quat *Quat_inter = new Quat[m];
	mbase.QuatInterpolation(APSdat, UT, m, Quat_inter);

	for (size_t i = 0; i < m; i++)
	{
		StarDat(i, 0) = Quat_inter[i].Q1, StarDat(i, 1) = Quat_inter[i].Q2;
		StarDat(i, 2) = Quat_inter[i].Q3, StarDat(i, 3) = Quat_inter[i].Q0;
	}
	double dt = 0.25;
	Qest(0, 0) = StarDat(0, 0), Qest(0, 1) = StarDat(0, 1), Qest(0, 2) = StarDat(0, 2), Qest(0, 3) = StarDat(0, 3);
	double a1 = Qest(0, 0), a2 = Qest(0, 1), a3 = Qest(0, 2), a4 = Qest(0, 3);

	double sig = 1e-6;
	Matrix3d zero33, eye33, poa, pog, r, sigu, sigv;
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu << 2e-20*eye33;//陀螺漂移噪声
	sigv << 2e-14*eye33;//陀螺噪声
	poa << 1e-7*eye33;//初始姿态误差协方差
	pog << 1e-5*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), be(m, 3), we(m, 3), xest(m, 6);
	eye66 << eye33, zero33, zero33, eye33;
	be.row(0) << 0, 0, 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv, zero33, zero33, sigu;//过程噪声

	int j = 0;
	while (j < 2)
	{
		for (size_t i = 0; i < m - 1; i++)
		{
			double qmm1, qmm2, qmm3;
			qmm1 = -StarDat(i, 3)*Qest(i, 0) - StarDat(i, 2)*Qest(i, 1) + StarDat(i, 1)*Qest(i, 2) + StarDat(i, 0)*Qest(i, 3);
			qmm2 = StarDat(i, 2)*Qest(i, 0) - StarDat(i, 3)*Qest(i, 1) - StarDat(i, 0)*Qest(i, 2) + StarDat(i, 1)*Qest(i, 3);
			qmm3 = -StarDat(i, 1)*Qest(i, 0) + StarDat(i, 0)*Qest(i, 1) - StarDat(i, 3)*Qest(i, 2) + StarDat(i, 2)*Qest(i, 3);
			MatrixXd z(3, 1);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			//cout<<"观测残差："<<z.transpose()<<endl;
			MatrixXd h(3, 6), k(6, 3);
			h << eye33, zero33;
			k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
			//cout<<k<<endl;
			p = (eye66 - k * h)*p;
			//cout<<"p"<<p<<endl;
			xest.row(i) = xest.row(i) + (k*z).transpose();
			//cout<<xest.row(i);

			MatrixXd xe(1, 3);
			xe = 0.5*xest.row(i).head(3);
			double qe11, qe22, qe33, qe44;
			qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
			qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
			qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
			qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
			MatrixXd tempqe(4, 1);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			//cout<<Qest.row(i)<<endl;

			//Propagate Covariance
			//cout<<Wgm.row(i)<<endl;
			//cout<<xest.row(i).tail(3)<<endl;
			we.row(i) = Wgm.row(i) - xest.row(i).tail(3);
			double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
			Matrix3d wa;
			//wa<<0,we(i,2),-we(i,1),-we(i,2),0,we(i,0),we(i,1),-we(i,0),0;
			wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
			//cout<<wa<<endl;
			MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			//gamma=gmat*dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//cout<<phi<<endl;
			//cout<<gamma<<endl;

			//Propagate State
			double qw1, qw2, qw3, qw4;
			qw1 = we(i, 0) / w * sin(0.5*w*dt);
			qw2 = we(i, 1) / w * sin(0.5*w*dt);
			qw3 = we(i, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			MatrixXd om(4, 4);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			//cout<<om<<endl;
			Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();
			//cout<<Qest.row(i+1)<<endl;

			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(i + 1) = xest.row(i);
			xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;
			//cout<<xest.row(i)<<endl;

		}
		j++;
		xest(0, 3) = xest(m - 1, 3), xest(0, 4) = xest(m - 1, 4), xest(0, 5) = xest(m - 1, 5);
	}

	Quat EKFresTmp;
	for (size_t i = 0; i < m; i++)
	{
		EKFresTmp.UTC = AttData[i].utgyro, EKFresTmp.Q0 = Qest(i, 3), EKFresTmp.Q1 = Qest(i, 0),
			EKFresTmp.Q2 = Qest(i, 1), EKFresTmp.Q3 = Qest(i, 2);
		EKFres.push_back(EKFresTmp);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波主程序
//输入：AttData：STG解析出的姿态数据	
//输出：quatEst：姿态确定结果
//注意：解决了星敏陀螺时间不同步
//作者：GZC
//日期：2017.07.11
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV5(vector<STGData> AttData, Quat *quatEst, double *xest_store)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	int nGyro = m, nQuat = m;
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	Quat *qMeas = new Quat[m];
	Gyro *wMeas = new Gyro[m];
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		wMeas[i].x = GyTran[0] / 180 * PI, wMeas[i].y = GyTran[1] / 180 * PI, wMeas[i].z = GyTran[2] / 180 * PI;
		wMeas[i].UTC = AttData[i].utgyro;
	}
	vector<Quat>APSdat(m);
	for (size_t i = 0; i < m; i++)
	{
		APSdat[i].UTC = AttData[i].StarA.UTC;
		APSdat[i].Q0 = AttData[i].StarA.Q0; APSdat[i].Q1 = AttData[i].StarA.Q1;
		APSdat[i].Q2 = AttData[i].StarA.Q2; APSdat[i].Q3 = AttData[i].StarA.Q3;
	}
	alinAPS(APSdat);//乘以安装矩阵
	for (size_t i = 0; i < m; i++)
	{
		qMeas[i].UTC = APSdat[i].UTC;
		qMeas[i].Q0 = APSdat[i].Q0; qMeas[i].Q1 = APSdat[i].Q1;
		qMeas[i].Q2 = APSdat[i].Q2; qMeas[i].Q3 = APSdat[i].Q3;
	}
	double sig = 8. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量
	double utStart = qMeas[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = qMeas[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = qMeas[0].UTC;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = qMeas[0].Q1, Qest(0, 1) = qMeas[0].Q2;
	Qest(0, 2) = qMeas[0].Q3, Qest(0, 3) = qMeas[0].Q0;
	quatEst[0].UTC = qMeas[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	xest_store[0] = wMeas[0].UTC; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = qMeas[a].UTC - utStart;
			utStart = qMeas[a].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].Q0*Qest(b, 0) - qMeas[a].Q3*Qest(b, 1) + qMeas[a].Q2*Qest(b, 2) + qMeas[a].Q1*Qest(b, 3);
			qmm2 = qMeas[a].Q3*Qest(b, 0) - qMeas[a].Q0*Qest(b, 1) - qMeas[a].Q1*Qest(b, 2) + qMeas[a].Q2*Qest(b, 3);
			qmm3 = -qMeas[a].Q2*Qest(b, 0) + qMeas[a].Q1*Qest(b, 1) - qMeas[a].Q0*Qest(b, 2) + qMeas[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, zero33;
				k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye66 - k * h)*p;
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);
			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b++;
			i++;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波主程序
//输入：AttData：STG解析出的姿态数据	
//输出：quatEst：姿态确定结果
//注意：解决多星敏联合滤波的问题
//作者：GZC
//日期：2017.07.11
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV6(vector<STGData> AttData, Quat *quatEst, double *xest_store)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	int nGyro = m, nQuat = m;
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	Gyro *wMeas = new Gyro[m];
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		wMeas[i].x = GyTran[0] / 180 * PI, wMeas[i].y = GyTran[1] / 180 * PI, wMeas[i].z = GyTran[2] / 180 * PI;
		wMeas[i].UTC = AttData[i].utgyro;
	}
	//星敏B和C转坐标系，由Crj变为Cbj；
	Quat *starB = new Quat[m];
	Quat *starC = new Quat[m];
	alinBC(AttData, starB, starC);

	double	Lb1[3], Lb2[3];//星敏1和2的Z轴在卫星本体的坐标
	double	Lb1X[9], Lb2X[9];//星敏1和2的Z轴在卫星本体的坐标
	double Lmj1[3], Lmj2[3];//星敏1和2的Z轴在惯性系的坐标
	double optic[] = { 0,0,1 };
	double Install1[9], Install2[9], Cbi[9], Crj[9];
	memcpy(Install1, Binstall, sizeof(double) * 9);//复制星敏B到星敏1
	memcpy(Install2, Cinstall, sizeof(double) * 9);//复制星敏C到星敏2
	mbase.invers_matrix(Install1, 3);//Cbr
	mbase.Multi(Install1, optic, Lb1, 3, 3, 1);//星敏1在本体系中的坐标
	Lb1X[0] = 1; Lb1X[1] = 2 * Lb1[2]; Lb1X[2] = -2 * Lb1[1];
	Lb1X[3] = -2 * Lb1[2]; Lb1X[4] = 1; Lb1X[5] = 2 * Lb1[0];
	Lb1X[6] = 2 * Lb1[1]; Lb1X[7] = -2 * Lb1[0]; Lb1X[8] = 1;
	mbase.invers_matrix(Install2, 3);//Cbr
	mbase.Multi(Install2, optic, Lb2, 3, 3, 1);//星敏2在本体系中的坐标
	Lb2X[0] = 1; Lb2X[1] = 2 * Lb2[2]; Lb2X[2] = -2 * Lb2[1];
	Lb2X[3] = -2 * Lb2[2]; Lb2X[4] = 1; Lb2X[5] = 2 * Lb2[0];
	Lb2X[6] = 2 * Lb2[1]; Lb2X[7] = -2 * Lb2[0]; Lb2X[8] = 1;

	double sig = 8. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 6), r(6, 6), z(6, 1), h(6, 6), k(6, 6), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	eye66 = MatrixXd::Identity(6, 6);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye66;//星敏噪声	

	//预先计算估计四元数的数量
	double utStart = starB[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = starB[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = starB[0].UTC;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = starB[0].Q1, Qest(0, 1) = starB[0].Q2;
	Qest(0, 2) = starB[0].Q3, Qest(0, 3) = starB[0].Q0;
	quatEst[0].UTC = starB[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	xest_store[0] = wMeas[0].UTC; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = starB[a].UTC - utStart;
			utStart = starB[a].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			mbase.quat2matrix(Qest(b, 0), Qest(b, 1), Qest(b, 2), Qest(b, 3), Cbi);//Cbj
			mbase.invers_matrix(Cbi, 3);//Cjb
			double CTmp[9], Cbj[9], Ztmp1[3], Ztmp2[3], Ztmp3[3], Ztmp4[3];
			Matrix3d hTmp1, hTmp2;
			mbase.Multi(Cbi, Lb1X, CTmp, 3, 3, 3);
			hTmp1 << CTmp[0], CTmp[1], CTmp[2], CTmp[3],
				CTmp[4], CTmp[5], CTmp[6], CTmp[7], CTmp[8];
			mbase.Multi(Cbi, Lb2X, CTmp, 3, 3, 3);
			hTmp2 << CTmp[0], CTmp[1], CTmp[2], CTmp[3],
				CTmp[4], CTmp[5], CTmp[6], CTmp[7], CTmp[8];
			h << hTmp1, zero33, hTmp2, zero33;//h(6*6)
			mbase.quat2matrix(AttData[a].StarB.Q1, AttData[a].StarB.Q2,
				AttData[a].StarB.Q3, AttData[a].StarB.Q0, Crj);//Crj
			mbase.invers_matrix(Crj, 3);//Cjr
			mbase.Multi(Crj, optic, Ztmp1, 3, 3, 1);
			mbase.Multi(Cbi, Lb1, Ztmp3, 3, 3, 1);
			mbase.quat2matrix(AttData[a].StarC.Q1, AttData[a].StarC.Q2,
				AttData[a].StarC.Q3, AttData[a].StarC.Q0, Crj);//Crj
			mbase.invers_matrix(Crj, 3);//Cjr
			mbase.Multi(Crj, optic, Ztmp2, 3, 3, 1);
			mbase.Multi(Cbi, Lb2, Ztmp4, 3, 3, 1);
			z << Ztmp1[0] - Ztmp3[0], Ztmp1[1] - Ztmp3[1], Ztmp1[2] - Ztmp3[2],
				Ztmp2[0] - Ztmp4[0], Ztmp2[1] - Ztmp4[1], Ztmp2[2] - Ztmp4[2];//z(6*1)
			//cout << z << endl;
			k = p * h.transpose()*(h*p*h.transpose() + r).inverse();//k(6*6)
			p = (eye66 - k * h)*p;
			xest.row(b) = xest.row(b) + (k*(z - h * xest.row(b).transpose())).transpose();
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);
			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b++;
			i++;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波（多星敏联合滤波）
//输入：AttData：STG解析出的姿态数据	
//输出：quatEst：姿态确定结果
//策略：直接扩到6维
//作者：GZC
//日期：2017.07.12
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV7(vector<STGData> AttData, Quat *quatEst, double *xest_store)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	int nGyro = m, nQuat = m;
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	Gyro *wMeas = new Gyro[m];
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		wMeas[i].x = GyTran[0] / 180 * PI, wMeas[i].y = GyTran[1] / 180 * PI, wMeas[i].z = GyTran[2] / 180 * PI;
		wMeas[i].UTC = AttData[i].utgyro;
	}
	//星敏B和C转坐标系，由Crj变为Cbj；
	Quat *starB = new Quat[m];
	Quat *starC = new Quat[m];
	alinBC(AttData, starB, starC);

	double sig = 8. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qmm4, qmm5, qmm6, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 6), r(6, 6), z(6, 1), h(6, 6), k(6, 6), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	eye66 = MatrixXd::Identity(6, 6);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye66;//星敏噪声	

						   //预先计算估计四元数的数量
	double utStart = starB[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = starB[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = starB[0].UTC;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = starB[0].Q1, Qest(0, 1) = starB[0].Q2;
	Qest(0, 2) = starB[0].Q3, Qest(0, 3) = starB[0].Q0;
	quatEst[0].UTC = starB[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	xest_store[0] = wMeas[0].UTC; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = starB[a].UTC - utStart;
			utStart = starB[a].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -starB[a].Q0*Qest(b, 0) - starB[a].Q3*Qest(b, 1) + starB[a].Q2*Qest(b, 2) + starB[a].Q1*Qest(b, 3);
			qmm2 = starB[a].Q3*Qest(b, 0) - starB[a].Q0*Qest(b, 1) - starB[a].Q1*Qest(b, 2) + starB[a].Q2*Qest(b, 3);
			qmm3 = -starB[a].Q2*Qest(b, 0) + starB[a].Q1*Qest(b, 1) - starB[a].Q0*Qest(b, 2) + starB[a].Q3*Qest(b, 3);
			qmm4 = -starC[a].Q0*Qest(b, 0) - starC[a].Q3*Qest(b, 1) + starC[a].Q2*Qest(b, 2) + starC[a].Q1*Qest(b, 3);
			qmm5 = starC[a].Q3*Qest(b, 0) - starC[a].Q0*Qest(b, 1) - starC[a].Q1*Qest(b, 2) + starC[a].Q2*Qest(b, 3);
			qmm6 = -starC[a].Q2*Qest(b, 0) + starC[a].Q1*Qest(b, 1) - starC[a].Q0*Qest(b, 2) + starC[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3, 2 * qmm4, 2 * qmm5, 2 * qmm6;
			h << eye33, zero33, eye33, zero33;
			k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
			p = (eye66 - k * h)*p;
			xest.row(b) = xest.row(b) + (k*z).transpose();
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);
			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b++;
			i++;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波主程序（先双星敏定姿）
//输入：AttData：STG解析出的姿态数据	
//输出：quatEst：姿态确定结果
//注意：解决了星敏陀螺时间不同步
//作者：GZC
//日期：2017.07.12
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV8(vector<STGData> AttData, Quat *quatEst, double *xest_store)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	int nGyro = m, nQuat = m;
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	Quat *qMeas = new Quat[m];
	Gyro *wMeas = new Gyro[m];
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		wMeas[i].x = GyTran[0] / 180 * PI, wMeas[i].y = GyTran[1] / 180 * PI, wMeas[i].z = GyTran[2] / 180 * PI;
		wMeas[i].UTC = AttData[i].utgyro;
	}
	vector <Quat>qMeasTmp;
	DoubleStar(AttData, qMeasTmp, 23);
	//OutputFile(qMeasTmp, "\\DoubleStar.txt");
	for (int i = 0; i < m; i++)
	{
		qMeas[i].UTC = qMeasTmp[i].UTC;
		qMeas[i].Q0 = qMeasTmp[i].Q0; qMeas[i].Q1 = qMeasTmp[i].Q1;
		qMeas[i].Q2 = qMeasTmp[i].Q2; qMeas[i].Q3 = qMeasTmp[i].Q3;
	}

	double sig = 5. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量
	double utStart = qMeas[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = qMeas[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = qMeas[0].UTC;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = qMeas[0].Q1, Qest(0, 1) = qMeas[0].Q2;
	Qest(0, 2) = qMeas[0].Q3, Qest(0, 3) = qMeas[0].Q0;
	quatEst[0].UTC = qMeas[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	xest_store[0] = wMeas[0].UTC; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = qMeas[a].UTC - utStart;
			utStart = qMeas[a].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].Q0*Qest(b, 0) - qMeas[a].Q3*Qest(b, 1) + qMeas[a].Q2*Qest(b, 2) + qMeas[a].Q1*Qest(b, 3);
			qmm2 = qMeas[a].Q3*Qest(b, 0) - qMeas[a].Q0*Qest(b, 1) - qMeas[a].Q1*Qest(b, 2) + qMeas[a].Q2*Qest(b, 3);
			qmm3 = -qMeas[a].Q2*Qest(b, 0) + qMeas[a].Q1*Qest(b, 1) - qMeas[a].Q0*Qest(b, 2) + qMeas[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, zero33;
				k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye66 - k * h)*p;
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);

			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b++;
			i++;
		}
	}

}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼滤波主程序（先双星敏定姿）
//输入：AttData：STG解析出的姿态数据	
//输出：quatEst：姿态确定结果
//注意：解决了星敏陀螺时间不同步
//作者：GZC
//日期：2017.07.12
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateV9(vector<STGData> AttData, Quat *quatEst, double *xest_store)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	int nGyro = m, nQuat = m;
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	Quat *qMeas = new Quat[m];
	Gyro *wMeas = new Gyro[m];
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		wMeas[i].x = GyTran[0] / 180 * PI, wMeas[i].y = GyTran[1] / 180 * PI, wMeas[i].z = GyTran[2] / 180 * PI;
		wMeas[i].UTC = AttData[i].utgyro;
	}
	vector <Quat>qMeasTmp;
	DoubleStar(AttData, qMeasTmp, 23);
	//OutputFile(qMeasTmp, "\\DoubleStar.txt");
	for (int i = 0; i < m; i++)
	{
		qMeas[i].UTC = qMeasTmp[i].UTC;
		qMeas[i].Q0 = qMeasTmp[i].Q0; qMeas[i].Q1 = qMeasTmp[i].Q1;
		qMeas[i].Q2 = qMeasTmp[i].Q2; qMeas[i].Q3 = qMeasTmp[i].Q3;
	}

	double sig = 5. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量
	double utStart = qMeas[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = qMeas[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);


	/************************************************************************/
	/*									卡尔曼滤波正向递推过程	                                  */
	/************************************************************************/

	//设置递推初始值
	a = 1, b = 0;
	utStart = qMeas[0].UTC;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = qMeas[0].Q1, Qest(0, 1) = qMeas[0].Q2;
	Qest(0, 2) = qMeas[0].Q3, Qest(0, 3) = qMeas[0].Q0;
	quatEst[0].UTC = 0;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = qMeas[a].UTC - utStart;
			utStart = qMeas[a].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].Q0*Qest(b, 0) - qMeas[a].Q3*Qest(b, 1) + qMeas[a].Q2*Qest(b, 2) + qMeas[a].Q1*Qest(b, 3);
			qmm2 = qMeas[a].Q3*Qest(b, 0) - qMeas[a].Q0*Qest(b, 1) - qMeas[a].Q1*Qest(b, 2) + qMeas[a].Q2*Qest(b, 3);
			qmm3 = -qMeas[a].Q2*Qest(b, 0) + qMeas[a].Q1*Qest(b, 1) - qMeas[a].Q0*Qest(b, 2) + qMeas[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, zero33;
				k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye66 - k * h)*p;
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			b++;
			i++;
		}
	}
	/************************************************************************/
	/*									卡尔曼滤波逆向递推过程	                                  */
	/************************************************************************/
	quatEst[nGyro - 1].UTC = utStart;
	quatEst[nGyro - 1].Q1 = Qest(b, 0); quatEst[nGyro - 1].Q2 = Qest(b, 1);
	quatEst[nGyro - 1].Q3 = Qest(b, 2); quatEst[nGyro - 1].Q0 = Qest(b, 3);
	xest_store[6 * (nGyro - 1) + 0] = wMeas[m - 1].UTC; xest_store[6 * (nGyro - 1) + 1] = xest(b, 1);
	xest_store[6 * (nGyro - 1) + 2] = xest(b, 2);	xest_store[6 * (nGyro - 1) + 3] = xest(b, 3);
	xest_store[6 * (nGyro - 1) + 4] = xest(b, 4); xest_store[6 * (nGyro - 1) + 5] = xest(b, 5);
	a = nQuat - 2;
	for (int i = nGyro - 2; i >= 0;)
	{
		if (a >= 0 && (qMeas[a].UTC - utStart) >= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = utStart - qMeas[a].UTC;
			utStart = qMeas[a].UTC;
			we(b, 0) = -wMeas[i].x + xest(b, 3);
			we(b, 1) = -wMeas[i].y + xest(b, 4);
			we(b, 2) = -wMeas[i].z + xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
			b--;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].Q0*Qest(b, 0) - qMeas[a].Q3*Qest(b, 1) + qMeas[a].Q2*Qest(b, 2) + qMeas[a].Q1*Qest(b, 3);
			qmm2 = qMeas[a].Q3*Qest(b, 0) - qMeas[a].Q0*Qest(b, 1) - qMeas[a].Q1*Qest(b, 2) + qMeas[a].Q2*Qest(b, 3);
			qmm3 = -qMeas[a].Q2*Qest(b, 0) + qMeas[a].Q1*Qest(b, 1) - qMeas[a].Q0*Qest(b, 2) + qMeas[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, zero33;
				k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye66 - k * h)*p;
				xest.row(b) = xest.row(b) + (k*z).transpose();
				double aa = xest(b, 3); double bb = xest(b, 4); double cc = xest(b, 5);
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a--;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = utStart - wMeas[i].UTC;
			utStart = wMeas[i].UTC;
			we(b, 0) = -wMeas[i].x + xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = -wMeas[i].y + xest(b, 4);
			we(b, 2) = -wMeas[i].z + xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b - 1, 0), quatEst[i].Q2 = Qest(b - 1, 1);
			quatEst[i].Q3 = Qest(b - 1, 2), quatEst[i].Q0 = Qest(b - 1, 3);

			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b--;
			i--;
		}
	}
	memcpy(xest_store, &xest_store[6], sizeof(double) * 6);
}

//////////////////////////////////////////////////////////////////////////
//功能：针对仿真的AB星敏进行姿态确定
//输入：
//输出：
//注意：解决多星敏联合滤波的问题
//作者：GZC
//日期：2017.08.07
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateForStarAB(int m, Quat *qB, Quat *qC, Quat * qTrue, Gyro *wMeas)
{
	//星敏B和C转坐标系，由Crj变为Cbj；
	Quat *starB = new Quat[m];
	Quat *starC = new Quat[m];
	double Balin[9], Calin[9];
	getInstall(Balin, Calin);
	double Crj[9], Cbj[9], q[4];

	mbase.invers_matrix(Balin, 3);//转换为Cbr	
	mbase.invers_matrix(Calin, 3);//转换为Cbr	
	for (int i = 0; i < m; i++)
	{
		mbase.quat2matrix(qB[i].Q1, qB[i].Q2, qB[i].Q3, qB[i].Q0, Crj);
		mbase.Multi(Balin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starB[i].Q1, starB[i].Q2, starB[i].Q3, starB[i].Q0);
		starB[i].UTC = qB[i].UTC;
		mbase.quat2matrix(qC[i].Q1, qC[i].Q2, qC[i].Q3, qC[i].Q0, Crj);
		mbase.Multi(Calin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starC[i].Q1, starC[i].Q2, starC[i].Q3, starC[i].Q0);
		starC[i].UTC = qC[i].UTC;
	}

	//获取两颗星敏相对安装误差
	double Err[9];
	getInstallErr(m, qB, qC, Err);

	double	Lb1[3], Lb2[3];//星敏1和2的Z轴在卫星本体的坐标
	double	Lb1X[9], Lb2X[9];//星敏1和2的Z轴在卫星本体的叉乘
	double Lmj1[3], Lmj2[3];//星敏1和2的Z轴在惯性系的坐标
	double optic[] = { 0,0,1 };
	double Install1[9], Install2[9], Cbi[9];
	memcpy(Install1, Balin, sizeof(double) * 9);//复制星敏B到星敏1,Cbr
	memcpy(Install2, Calin, sizeof(double) * 9);//复制星敏C到星敏2,Cbr
	mbase.Multi(Install1, optic, Lb1, 3, 3, 1);//星敏1在本体系中的坐标
	Lb1X[0] = 0; Lb1X[1] = -Lb1[2]; Lb1X[2] = Lb1[1];
	Lb1X[3] = Lb1[2]; Lb1X[4] = 0; Lb1X[5] = -Lb1[0];
	Lb1X[6] = -Lb1[1]; Lb1X[7] = Lb1[0]; Lb1X[8] = 0;
	mbase.Multi(Install2, optic, Lb2, 3, 3, 1);//星敏2在本体系中的坐标
	Lb2X[0] = 0; Lb2X[1] = -Lb2[2]; Lb2X[2] = Lb2[1];
	Lb2X[3] = Lb2[2]; Lb2X[4] = 0; Lb2X[5] = -Lb2[0];
	Lb2X[6] = -Lb2[1]; Lb2X[7] = Lb2[0]; Lb2X[8] = 0;

	double sig = 0.8 / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 6), r(6, 6), z(6, 1), h(6, 6), k(6, 6), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	eye66 = MatrixXd::Identity(6, 6);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye66;//星敏噪声	

	//预先计算估计四元数的数量
	double utStart = starB[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < m;)
	{
		if (a < m && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = starB[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	Quat *quatEst = new Quat[m];
	//设置递推初始值
	a = 1, b = 0;
	utStart = starB[0].UTC;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = starB[0].Q1, Qest(0, 1) = starB[0].Q2;
	Qest(0, 2) = starB[0].Q3, Qest(0, 3) = starB[0].Q0;
	quatEst[0].UTC = starB[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	FILE *fp = fopen((workpath + "\\StarBC_EKFAtt.txt").c_str(), "w");
	fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\n", 0, quatEst[0].Q1, quatEst[0].Q2, quatEst[0].Q3, 0, 0, 0);

	MatrixXd g(6, 6), qcc(6, 6);
	g << -eye33, zero33, zero33, eye33;
	qcc << 1.00000033333333e-13, 0, 0, -5.00000000000000e-20, 0, 0,
		0, 1.00000033333333e-13, 0, 0, -5.00000000000000e-20, 0,
		0, 0, 1.00000033333333e-13, 0, 0, -5.00000000000000e-20,
		-5.00000000000000e-20, 0, 0, 1.00000000000000e-19, 0, 0,
		0, -5.00000000000000e-20, 0, 0, 1.00000000000000e-19, 0,
		0, 0, -5.00000000000000e-20, 0, 0, 1.00000000000000e-19;

	for (int i = 1; i < m;)
	{
		if (a < m && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = starB[a].UTC - utStart;
			utStart = starB[a].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			//wa<< 0, we(b, 2), -we(b, 1), -we(b, 2), 0, we(b, 0), we(b, 1), -we(b, 0), 0;
			//MatrixXd phi11 = eye33 + wa*sin(w*dt) / w + wa*wa*(1 - cos(w*dt)) / w / w;
			//MatrixXd	phi12 = -(eye33*dt + wa*(1 - cos(w*dt)) / w / w + wa*wa*(w*dt - sin(w*dt)) / w / w / w);
			//MatrixXd phi(6, 6);
			//phi << phi11, phi12, zero33, eye33;
			//p = phi*p*phi.transpose() + qcc;

			fmat << -wa, eye33, zero33, zero33;
			gmat << eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			mbase.quat2matrix(Qest(b, 0), Qest(b, 1), Qest(b, 2), Qest(b, 3), Cbi);//Cbj
			mbase.invers_matrix(Cbi, 3);//Cjb
			double CTmp[9], Ztmp1[3], Ztmp2[3], Ztmp3[3], Ztmp4[3];
			mbase.Multi(Cbi, Lb1, CTmp, 3, 3, 1);
			Lb1X[0] = 0; Lb1X[1] = -CTmp[2]; Lb1X[2] = CTmp[1];
			Lb1X[3] = CTmp[2]; Lb1X[4] = 0; Lb1X[5] = -CTmp[0];
			Lb1X[6] = -CTmp[1]; Lb1X[7] = CTmp[0]; Lb1X[8] = 0;
			Map<rMatrixXd>hTmp1(Lb1X, 3, 3);

			mbase.Multi(Cbi, Lb2, CTmp, 3, 3, 1);
			Lb2X[0] = 0; Lb2X[1] = -CTmp[2]; Lb2X[2] = CTmp[1];
			Lb2X[3] = CTmp[2]; Lb2X[4] = 0; Lb2X[5] = -CTmp[0];
			Lb2X[6] = -CTmp[1]; Lb2X[7] = CTmp[0]; Lb2X[8] = 0;
			Map<rMatrixXd>hTmp2(Lb2X, 3, 3);
			h << hTmp1, zero33, hTmp2, zero33;//h(6*6)
			mbase.quat2matrix(qB[a].Q1, qB[a].Q2, qB[a].Q3, qB[a].Q0, Crj);//Crj
			mbase.invers_matrix(Crj, 3);//Cjr
			mbase.Multi(Crj, optic, Ztmp1, 3, 3, 1);
			mbase.Multi(Cbi, Lb1, Ztmp3, 3, 3, 1);
			mbase.quat2matrix(qC[a].Q1, qC[a].Q2, qC[a].Q3, qC[a].Q0, Crj);//Crj
			mbase.invers_matrix(Crj, 3);//Cjr
			mbase.Multi(Crj, optic, Ztmp2, 3, 3, 1);
			mbase.Multi(Cbi, Lb2, Ztmp4, 3, 3, 1);
			z << Ztmp1[0] - Ztmp3[0], Ztmp1[1] - Ztmp3[1], Ztmp1[2] - Ztmp3[2],
				Ztmp2[0] - Ztmp4[0], Ztmp2[1] - Ztmp4[1], Ztmp2[2] - Ztmp4[2];//z(6*1)		
			k = p * h.transpose()*(h*p*h.transpose() + r).inverse();//k(6*6)
			//p = (eye66 - k*h)*p*(eye66 - k*h).transpose() + k*r*k.transpose();
			p = (eye66 - k * h)*p;
			//xest.row(b) = xest.row(b) + (k*(z - h*xest.row(b).transpose())).transpose();
			xest.row(b) = xest.row(b) + (k*z).transpose();
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			/*	qe11 =  Qest(b, 0) - xe(2) * Qest(b, 1) + xe(1) * Qest(b, 2) + xe(0) * Qest(b, 3);
				qe22 = xe(2) * Qest(b, 0) +  Qest(b, 1) - xe(0) * Qest(b, 2) + xe(1) * Qest(b, 3);
				qe33 = -xe(1) * Qest(b, 0) + xe(0) * Qest(b, 1) +  Qest(b, 2) + xe(2) * Qest(b, 3);
				qe44 = -xe(0) * Qest(b, 0) - xe(1) * Qest(b, 1) - xe(2) * Qest(b, 2) +  Qest(b, 3);*/
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			/*	wa << 0, we(b, 2), -we(b, 1), -we(b, 2), 0, we(b, 0), we(b, 1), -we(b, 0), 0;
				MatrixXd phi11 = eye33 + wa*sin(w*dt) / w + wa*wa*(1 - cos(w*dt)) / w / w;
				MatrixXd	phi12 = -(eye33*dt + wa*(1 - cos(w*dt)) / w / w + wa*wa*(w*dt - sin(w*dt)) / w / w / w);
				MatrixXd phi(6, 6);
				phi << phi11, phi12, zero33, eye33;
				p = phi*p*phi.transpose() + qcc;*/
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			/*fmat << -wa, -0.5*eye33, zero33, zero33;
			gmat << -0.5*eye33, zero33, zero33, eye33;*/
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);
			fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\n",
				quatEst[i].UTC, quatEst[i].Q1, quatEst[i].Q2, quatEst[i].Q3,
				xest(b, 3) / PI * 180 * 3600 / 0.25, xest(b, 4) / PI * 180 * 3600 / 0.25, xest(b, 5) / PI * 180 * 3600 / 0.25);
			//保存xest值
			//xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			//xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b++;
			i++;
		}
	}
	fclose(fp);

	fp = fopen((workpath + "\\StarBC_Err.txt").c_str(), "w");
	double dq1[4], dq2[4], dq3[4];
	for (int i = 0; i < m; i++)
	{
		dq1[0] = -quatEst[i].Q0, dq1[1] = quatEst[i].Q1, dq1[2] = quatEst[i].Q2, dq1[3] = quatEst[i].Q3;
		dq2[0] = qTrue[i].Q0, dq2[1] = qTrue[i].Q1, dq2[2] = qTrue[i].Q2, dq2[3] = qTrue[i].Q3;
		mbase.quatMult2(dq1, dq2, dq3);
		dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600, dq3[3] = dq3[3] * 2 / PI * 180 * 3600;
		fprintf(fp, "%.9f\t%.9f\t%.9f\n", dq3[1], dq3[2], dq3[3]);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：针对仿真的AB星敏进行姿态确定，
//输入：
//输出：
//策略：1、在原来经典方法基础上进行扩维处理：经典方法先双星敏光轴组成测量值，然后滤波
//			 2、将观测值z扩展为6*1矩阵，利用了两颗星敏的四元数和估计四元数进行比较
//作者：GZC
//日期：2017.08.09
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateForStarAB2(int m, Quat *qB, Quat *qC, Quat * qTrue, Gyro *wMeas)
{
	//星敏B和C转坐标系，由Crj变为Cbj；
	Quat *starB = new Quat[m];
	Quat *starC = new Quat[m];
	double Balin[9], Calin[9];
	getInstall(Balin, Calin);
	double Crj[9], Cbj[9], q[4];

	mbase.invers_matrix(Balin, 3);//转换为Cbr	
	mbase.invers_matrix(Calin, 3);//转换为Cbr	
	for (int i = 0; i < m; i++)
	{
		mbase.quat2matrix(qB[i].Q1, qB[i].Q2, qB[i].Q3, qB[i].Q0, Crj);
		mbase.Multi(Balin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starB[i].Q1, starB[i].Q2, starB[i].Q3, starB[i].Q0);
		starB[i].UTC = qB[i].UTC;
		mbase.quat2matrix(qC[i].Q1, qC[i].Q2, qC[i].Q3, qC[i].Q0, Crj);
		mbase.Multi(Calin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starC[i].Q1, starC[i].Q2, starC[i].Q3, starC[i].Q0);
		starC[i].UTC = qC[i].UTC;
	}

	//获取两颗星敏相对安装误差
	double Err[9];
	getInstallErr(m, qB, qC, Err);

	double	Lb1[3], Lb2[3];//星敏1和2的Z轴在卫星本体的坐标
	double	Lb1X[9], Lb2X[9];//星敏1和2的Z轴在卫星本体的叉乘
	double Lmj1[3], Lmj2[3];//星敏1和2的Z轴在惯性系的坐标
	double optic[] = { 0,0,1 };
	double Install1[9], Install2[9], Cbi[9];
	memcpy(Install1, Balin, sizeof(double) * 9);//复制星敏B到星敏1,Cbr
	memcpy(Install2, Calin, sizeof(double) * 9);//复制星敏C到星敏2,Cbr
	mbase.Multi(Install1, optic, Lb1, 3, 3, 1);//星敏1在本体系中的坐标
	Lb1X[0] = 1; Lb1X[1] = 2 * Lb1[2]; Lb1X[2] = -2 * Lb1[1];
	Lb1X[3] = -2 * Lb1[2]; Lb1X[4] = 1; Lb1X[5] = 2 * Lb1[0];
	Lb1X[6] = 2 * Lb1[1]; Lb1X[7] = -2 * Lb1[0]; Lb1X[8] = 1;
	mbase.Multi(Install2, optic, Lb2, 3, 3, 1);//星敏2在本体系中的坐标
	Lb2X[0] = 1; Lb2X[1] = 2 * Lb2[2]; Lb2X[2] = -2 * Lb2[1];
	Lb2X[3] = -2 * Lb2[2]; Lb2X[4] = 1; Lb2X[5] = 2 * Lb2[0];
	Lb2X[6] = 2 * Lb2[1]; Lb2X[7] = -2 * Lb2[0]; Lb2X[8] = 1;

	double sig = 8. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qmm4, qmm5, qmm6, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 6), r(6, 6), z(6, 1), h(6, 6), k(6, 6), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	eye66 = MatrixXd::Identity(6, 6);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye66;//星敏噪声	

						   //预先计算估计四元数的数量
	double utStart = starB[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < m;)
	{
		if (a < m && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = starB[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	Quat *quatEst = new Quat[m];
	//设置递推初始值
	a = 1, b = 0;
	utStart = starB[0].UTC;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = starB[0].Q1, Qest(0, 1) = starB[0].Q2;
	Qest(0, 2) = starB[0].Q3, Qest(0, 3) = starB[0].Q0;
	quatEst[0].UTC = starB[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	FILE *fp = fopen((workpath + "\\StarBC_EKFAtt.txt").c_str(), "w");
	fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\n", 0, quatEst[0].Q1, quatEst[0].Q2, quatEst[0].Q3, 0, 0, 0);

	for (int i = 1; i < m;)
	{
		if (a < m && (starB[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = starB[a].UTC - utStart;
			utStart = starB[a].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();//p(6*6)
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			///****************星敏测量值更新***************/
			//mbase.quat2matrix(Qest(b, 0), Qest(b, 1), Qest(b, 2), Qest(b, 3), Cbi);//Cbj
			//mbase.invers_matrix(Cbi, 3);//Cjb
			//double CTmp[9], Ztmp1[3], Ztmp2[3], Ztmp3[3], Ztmp4[3];
			//mbase.Multi(Cbi, Lb1X, CTmp, 3, 3, 3);
			//Map<rMatrixXd>hTmp1(CTmp, 3, 3);
			//mbase.Multi(Cbi, Lb2X, CTmp, 3, 3, 3);
			//Map<rMatrixXd>hTmp2(CTmp, 3, 3);
			//h << hTmp1, zero33;//h(3*6)
			//mbase.quat2matrix(qB[a].Q1, qB[a].Q2, qB[a].Q3, qB[a].Q0, Crj);//Crj
			//mbase.invers_matrix(Crj, 3);//Cjr
			//mbase.Multi(Crj, optic, Ztmp1, 3, 3, 1);
			//mbase.Multi(Cbi, Lb1, Ztmp3, 3, 3, 1);
			//z << Ztmp1[0] - Ztmp3[0], Ztmp1[1] - Ztmp3[1], Ztmp1[2] - Ztmp3[2];//z(3*1)	

			/****************星敏测量值更新***************/
			qmm1 = -starB[a].Q0*Qest(b, 0) - starB[a].Q3*Qest(b, 1) + starB[a].Q2*Qest(b, 2) + starB[a].Q1*Qest(b, 3);
			qmm2 = starB[a].Q3*Qest(b, 0) - starB[a].Q0*Qest(b, 1) - starB[a].Q1*Qest(b, 2) + starB[a].Q2*Qest(b, 3);
			qmm3 = -starB[a].Q2*Qest(b, 0) + starB[a].Q1*Qest(b, 1) - starB[a].Q0*Qest(b, 2) + starB[a].Q3*Qest(b, 3);
			qmm4 = -starC[a].Q0*Qest(b, 0) - starC[a].Q3*Qest(b, 1) + starC[a].Q2*Qest(b, 2) + starC[a].Q1*Qest(b, 3);
			qmm5 = starC[a].Q3*Qest(b, 0) - starC[a].Q0*Qest(b, 1) - starC[a].Q1*Qest(b, 2) + starC[a].Q2*Qest(b, 3);
			qmm6 = -starC[a].Q2*Qest(b, 0) + starC[a].Q1*Qest(b, 1) - starC[a].Q0*Qest(b, 2) + starC[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3, 2 * qmm4, 2 * qmm5, 2 * qmm6;
			h << eye33, zero33, eye33, zero33;
			k = p * h.transpose()*(h*p*h.transpose() + r).inverse();//k(6*3)
			p = (eye66 - k * h)*p;//等效于：p = (eye66 - k*h)*p*(eye66 - k*h).transpose() + k*r*k.transpose();			
			xest.row(b) = xest.row(b) + (k*z).transpose();//等效于：xest.row(b) = xest.row(b) + (k*(z - h*xest.row(b).transpose())).transpose();			
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			fmat << -wa, -0.5*eye33, zero33, zero33;
			gmat << -0.5*eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);
			fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\n",
				quatEst[i].UTC, quatEst[i].Q1, quatEst[i].Q2, quatEst[i].Q3,
				xest(b, 3) / PI * 180 * 3600 / 0.25, xest(b, 4) / PI * 180 * 3600 / 0.25, xest(b, 5) / PI * 180 * 3600 / 0.25);
			//保存xest值
			//xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			//xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			b++;
			i++;
		}
	}
	fclose(fp);

	fp = fopen((workpath + "\\StarBC_Err.txt").c_str(), "w");
	double dq1[4], dq2[4], dq3[4];
	for (int i = 0; i < m; i++)
	{
		dq1[0] = -quatEst[i].Q0, dq1[1] = quatEst[i].Q1, dq1[2] = quatEst[i].Q2, dq1[3] = quatEst[i].Q3;
		dq2[0] = qTrue[i].Q0, dq2[1] = qTrue[i].Q1, dq2[2] = qTrue[i].Q2, dq2[3] = qTrue[i].Q3;
		mbase.quatMult2(dq1, dq2, dq3);
		dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600, dq3[3] = dq3[3] * 2 / PI * 180 * 3600;
		fprintf(fp, "%.9f\t%.9f\t%.9f\n", dq3[1], dq3[2], dq3[3]);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：针对仿真的AB星敏进行姿态确定
//输入：
//输出：
//注意：解决多星敏联合滤波的问题
//作者：GZC
//日期：2017.08.10
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateForStarAB3(int m, Quat *qB, Quat *qC, Quat * qTrue, Gyro *wMeas)
{
	//星敏B和C转坐标系，由Crj变为Cbj；
	Quat *starB = new Quat[m];
	Quat *starC = new Quat[m];
	double Balin[9], Calin[9];
	getInstall(Balin, Calin);
	double Crj[9], Cbj[9], q[4];

	//获取两颗星敏相对安装误差
	double Err[9];
	getInstallErr(m, qB, qC, Calin);

	mbase.invers_matrix(Balin, 3);//转换为Cbr	
	mbase.invers_matrix(Calin, 3);//转换为Cbr	
	for (int i = 0; i < m; i++)
	{
		mbase.quat2matrix(qB[i].Q1, qB[i].Q2, qB[i].Q3, qB[i].Q0, Crj);
		mbase.Multi(Balin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starB[i].Q1, starB[i].Q2, starB[i].Q3, starB[i].Q0);
		starB[i].UTC = qB[i].UTC;
		mbase.quat2matrix(qC[i].Q1, qC[i].Q2, qC[i].Q3, qC[i].Q0, Crj);
		mbase.Multi(Calin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starC[i].Q1, starC[i].Q2, starC[i].Q3, starC[i].Q0);
		starC[i].UTC = qC[i].UTC;
	}

	double sig = 0.8 / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, sigu33, sigv33;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 6), r(6, 6), z(6, 1), h(6, 6), k(6, 6), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	eye66 = MatrixXd::Identity(6, 6);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye66;//星敏噪声	

	MatrixXd Qest(m + 1, 4), we(m + 1, 3), xest(m + 1, 6);

	Quat *quatEst = new Quat[m];
	//设置递推初始值
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = starB[0].Q1, Qest(0, 1) = starB[0].Q2;
	Qest(0, 2) = starB[0].Q3, Qest(0, 3) = starB[0].Q0;
	quatEst[0].UTC = starB[0].UTC;
	quatEst[0].Q1 = Qest(0, 0), quatEst[0].Q2 = Qest(0, 1);
	quatEst[0].Q3 = Qest(0, 2), quatEst[0].Q0 = Qest(0, 3);
	FILE *fp = fopen((workpath + "\\StarBC_EKFAtt.txt").c_str(), "w");
	fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\n", 0, quatEst[0].Q1, quatEst[0].Q2, quatEst[0].Q3, 0, 0, 0);

	for (int i = 0; i < m - 1; i++)
	{
		dt = 0.25;
		double Cbj[9]; double optic[] = { 0,0,1 };
		mbase.quat2matrix(Qest(i, 0), Qest(i, 1), Qest(i, 2), Qest(i, 3), Cbj);//Cbj
		double im1[3], im2[3];
		mbase.quat2matrix(qB[i].Q1, qB[i].Q2, qB[i].Q3, qB[i].Q0, Crj);//Crj
		mbase.invers_matrix(Crj, 3);//Cjr
		mbase.Multi(Crj, optic, im1, 3, 3, 1);//
		double pbe1[3], pbe2[3];
		mbase.Multi(Cbj, im1, pbe1, 3, 3, 1);
		MatrixXd pbe_cr1(3, 3);
		pbe_cr1 << 0, -pbe1[2], pbe1[1], pbe1[2], 0, -pbe1[0], -pbe1[1], pbe1[0], 0;
		mbase.quat2matrix(qC[i].Q1, qC[i].Q2, qC[i].Q3, qC[i].Q0, Crj);//Crj
		mbase.invers_matrix(Crj, 3);//Cjr
		mbase.Multi(Crj, optic, im2, 3, 3, 1);//
		mbase.Multi(Cbj, im2, pbe2, 3, 3, 1);
		MatrixXd pbe_cr2(3, 3);
		pbe_cr2 << 0, -pbe2[2], pbe2[1], pbe2[2], 0, -pbe2[0], -pbe2[1], pbe2[0], 0;
		h << pbe_cr1, zero33, pbe_cr2, zero33;

		double	Lb1[3], Lb2[3];//星敏1和2的Z轴在卫星本体的坐标
		mbase.Multi(Balin, optic, Lb1, 3, 3, 1);
		mbase.Multi(Calin, optic, Lb2, 3, 3, 1);
		z << Lb1[0] - pbe1[0], Lb1[1] - pbe1[1], Lb1[2] - pbe1[2], Lb2[0] - pbe2[0], Lb2[1] - pbe2[1], Lb2[2] - pbe2[2];
		k = p * h.transpose()*(h*p*h.transpose() + r).inverse();//k(6*6)
		p = (eye66 - k * h)*p;
		xest.row(i) = xest.row(i) + (k*z).transpose();
		xe = 0.5*xest.row(i).head(3);
		qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
		qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
		qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
		qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
		tempqe << qe11, qe22, qe33, qe44;
		tempqe.normalize();
		Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

		//Propagate State
		we(i, 0) = wMeas[i + 1].x - xest(i, 3);//
		we(i, 1) = wMeas[i + 1].y - xest(i, 4);
		we(i, 2) = wMeas[i + 1].z - xest(i, 5);
		w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
		qw1 = we(i, 0) / w * sin(0.5*w*dt);
		qw2 = we(i, 1) / w * sin(0.5*w*dt);
		qw3 = we(i, 2) / w * sin(0.5*w*dt);
		qw4 = cos(0.5*w*dt);
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();

		//Propagate Covariance
		Matrix3d wa;
		wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
		fmat << -wa, -eye33, zero33, zero33;
		gmat << -eye33, zero33, zero33, eye33;
		phi = eye66 + fmat * dt;
		gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
		p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();

		xest.row(i + 1) = xest.row(i);
		xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;

		quatEst[i].UTC = wMeas[i].UTC;
		quatEst[i].Q1 = Qest(i, 0), quatEst[i].Q2 = Qest(i, 1);
		quatEst[i].Q3 = Qest(i, 2), quatEst[i].Q0 = Qest(i, 3);
		fprintf(fp, "%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\n",
			quatEst[i].UTC, quatEst[i].Q1, quatEst[i].Q2, quatEst[i].Q3,
			xest(i, 3) / PI * 180 * 3600 / 0.25, xest(i, 4) / PI * 180 * 3600 / 0.25, xest(i, 5) / PI * 180 * 3600 / 0.25);
	}
	fclose(fp);

	fp = fopen((workpath + "\\StarBC_Err.txt").c_str(), "w");
	double dq1[4], dq2[4], dq3[4];
	for (int i = 0; i < m; i++)
	{
		dq1[0] = -quatEst[i].Q0, dq1[1] = quatEst[i].Q1, dq1[2] = quatEst[i].Q2, dq1[3] = quatEst[i].Q3;
		dq2[0] = qTrue[i].Q0, dq2[1] = qTrue[i].Q1, dq2[2] = qTrue[i].Q2, dq2[3] = qTrue[i].Q3;
		mbase.quatMult2(dq1, dq2, dq3);
		dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600, dq3[3] = dq3[3] * 2 / PI * 180 * 3600;
		fprintf(fp, "%.9f\t%.9f\t%.9f\n", dq3[1], dq3[2], dq3[3]);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：针对恒星和像点对应关系进行姿态确定
//输入：恒星矢量、像点矢量
//输出：估计四元数
//注意：相当于多光轴联合滤波
//作者：GZC
//日期：2017.11.28
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF6StateForStarMap(vector < vector<BmImStar>>BmIm, vector<STGData>stg)
{
	//删掉四元数之前的陀螺数据
	double utStart = BmIm[0][0].UT;
	int ii = 0;
	while ((stg[ii].utgyro - utStart) < 0)
	{
		ii++;
	}
	stg.erase(stg.begin(), stg.begin() + ii);

	//乘以陀螺安装
	int nGyro = stg.size();
	double GyDat[3], GyTran[3];
	Gyro *wMeas = new Gyro[nGyro];
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	vector<Quat>APSdat;
	for (int i = 0; i < nGyro; i++)
	{
		GyDat[0] = stg.at(i).g1, GyDat[1] = stg.at(i).g3, GyDat[2] = stg.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		wMeas[i].x = GyTran[0] / 180 * PI, wMeas[i].y = GyTran[1] / 180 * PI, wMeas[i].z = GyTran[2] / 180 * PI;
		wMeas[i].UTC = stg.at(i).utgyro;
		Quat aps;
		aps.UTC = stg.at(i).StarA.UTC;
		aps.Q0 = stg.at(i).StarA.Q0; aps.Q1 = stg.at(i).StarA.Q1;
		aps.Q2 = stg.at(i).StarA.Q2; aps.Q3 = stg.at(i).StarA.Q3;
		APSdat.push_back(aps);
	}
	alinAPS(APSdat);//乘以安装矩阵

	int nQuat = BmIm.size();
	double *UT = new double[nQuat];
	for (int i = 0; i < nQuat; i++)
	{
		UT[i] = BmIm[i][0].UT;
	}
	Quat *Quat_inter = new Quat[nQuat];
	mbase.QuatInterpolation(APSdat, UT, nQuat, Quat_inter);

	double sig = 5. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu33 << 1e-19*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量	
	int a = 1, b = 0;
	for (int i = 0; i < nGyro;)
	{
		if (a < nQuat && (BmIm[a][0].UT - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = BmIm[a][0].UT;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = BmIm[0][0].UT;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = Quat_inter[0].Q1, Qest(0, 1) = Quat_inter[0].Q2;
	Qest(0, 2) = Quat_inter[0].Q3, Qest(0, 3) = Quat_inter[0].Q0;
	Quat *quatEst = new Quat[nGyro];
	double *xest_store = new double[6 * nGyro];
	quatEst[0].UTC = Quat_inter[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	xest_store[0] = wMeas[0].UTC; xest_store[1] = 0; xest_store[2] = 0;
	xest_store[3] = 0; xest_store[4] = 0, xest_store[5] = 0;

	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (BmIm[a][0].UT - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = BmIm[a][0].UT - utStart;
			utStart = BmIm[a][0].UT;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			double Cbj[9];
			mbase.quat2matrix(Qest(i, 0), Qest(i, 1), Qest(i, 2), Qest(i, 3), Cbj);//Cbj
			int num = BmIm[a].size();
			MatrixXd mH(3 * num, 6), mDetZ(3 * num, 1), k(6, 3 * num);
			MatrixXd r1 = pow(sig, 2)*MatrixXd::Identity(3 * num, 3 * num);
			Measurement(BmIm[a], Cbj, mH, mDetZ);
			k = p * mH.transpose()*(mH*p*mH.transpose() + r1).inverse();//k(6*6)
			p = (eye66 - k * mH)*p;
			xest.row(i) = xest.row(i) + (k*mDetZ).transpose();
			xe = 0.5*xest.row(b).head(3);
			qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
			qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
			qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
			qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
			tempqe << qe11, qe22, qe33, qe44;
			tempqe.normalize();
			Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			we(b, 0) = wMeas[i - 1].x - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].y - xest(b, 4);
			we(b, 2) = wMeas[i - 1].z - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat * dt;
			gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w * sin(0.5*w*dt);
			qw2 = we(b, 1) / w * sin(0.5*w*dt);
			qw3 = we(b, 2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);

			//保存xest值
			xest_store[6 * i + 0] = wMeas[i].UTC; xest_store[6 * i + 1] = xest(b, 1); xest_store[6 * i + 2] = xest(b, 2);
			xest_store[6 * i + 3] = xest(b, 3); xest_store[6 * i + 4] = xest(b, 4); xest_store[6 * i + 5] = xest(b, 5);

			if (i % 500 == 0)
			{
				printf("已运行到第%d个陀螺值\n", i);
			}
			b++;
			i++;
		}
	}

	OutputFile(quatEst, nGyro, "\\StarMap滤波结果.txt");
}

//////////////////////////////////////////////////////////////////////////
//功能：得到测量值，测量协方差
//输入：
//输出：
//注意：相当于多光轴联合滤波
//作者：GZC
//日期：2017.11.29
//////////////////////////////////////////////////////////////////////////
void AttDetermination::Measurement(vector<BmImStar> BmIm, double *Att,
	MatrixXd &mH, MatrixXd &mDetZ)
{
	int num = BmIm.size();
	MatrixXd pbe(3, num);
	Matrix3d pbe_cr(3, 3);
	Map<rMatrixXd>mAtt(Att, 3, 3);
	for (int a = 0; a < num; a++)
	{
		Map<MatrixXd>im(BmIm[a].Im, 3, 1);
		pbe.block<3, 1>(0, a) = mAtt * im;
		pbe_cr << 0, -pbe(2, a), pbe(1, a), pbe(2, a), 0, -pbe(0, a), -pbe(1, a), pbe(0, a), 0;
		mH.block<3, 6>(3 * a, 0) << pbe_cr, MatrixXd::Zero(3, 3);
		Map<MatrixXd>bm(BmIm[a].Bm, 3, 1);
		mDetZ.block<3, 1>(3 * a, 0) << pbe.block<3, 1>(0, a) - bm;
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：得到光轴在惯性系和星敏坐标系矢量
//输入：星点控制点，标定参数
//输出：
//注意：相当于多光轴联合滤波
//作者：GZC
//日期：2017.11.29
//////////////////////////////////////////////////////////////////////////
void AttDetermination::GetImBm(vector<vector<StarGCP>> getGCP,
	const StarCaliParam Param, vector<vector<BmImStar>> &BmIm)
{
	double Wob[3];
	double X, Y, DetX, DetY;
	for (int a = 0; a < getGCP.size(); a++)
	{
		vector<BmImStar>BmImTmp;
		for (int b = 0; b < getGCP[a].size(); b++)
		{
			double Xp, Yp;
			Xp = 1024 - getGCP[a][b].y;		Yp = getGCP[a][b].x;
			//畸变模型(5参数情况)
			double r2 = (Xp - Param.x0)*(Xp - Param.x0) + (Yp - Param.y0)*(Yp - Param.y0);
			double xreal = -(Xp - Param.x0)*(1 - Param.k1 * r2 - Param.k2 * r2*r2);
			double yreal = -(Yp - Param.y0)*(1 - Param.k1 * r2 - Param.k2 * r2*r2);
			double freal = Param.f;

			//赋值，将像方坐标系转到星敏坐标系，星敏坐标系为-(X-X0)，-(Y-Y0)，f；然后
			double D = sqrt(pow(xreal, 2) + pow(yreal, 2) + pow(freal, 2));
			//double Wob[3];
			Wob[0] = xreal / D;
			Wob[1] = yreal / D;
			Wob[2] = freal / D;
			BmImStar BmImTmp2;
			BmImTmp2.Bm[0] = Wob[0];
			BmImTmp2.Bm[1] = Wob[1];
			BmImTmp2.Bm[2] = Wob[2];
			BmImTmp2.Im[0] = getGCP[a][b].V[0];
			BmImTmp2.Im[1] = getGCP[a][b].V[1];
			BmImTmp2.Im[2] = getGCP[a][b].V[2];
			BmImTmp2.UT = getGCP[a][b].UTC;
			BmImTmp.push_back(BmImTmp2);
		}
		BmIm.push_back(BmImTmp);
	}
	GetImRm(BmIm);
}

//////////////////////////////////////////////////////////////////////////
//功能：将光轴从星敏坐标系转换到本体坐标系
//输入：光轴在星敏坐标系和惯性系矢量
//输出：光轴在本体坐标系和惯性系矢量
//注意：
//作者：GZC
//日期：2017.12.04
//////////////////////////////////////////////////////////////////////////
void AttDetermination::GetImRm(vector<vector<BmImStar>>& BmIm)
{
	double Aalin[9];
	memcpy(Aalin, Ainstall, sizeof(Ainstall));
	mbase.invers_matrix(Aalin, 3);//转换为Cbr	
	for (int a = 0; a < BmIm.size(); a++)
	{
		for (int b = 0; b < BmIm[a].size(); b++)
		{
			double Rm[3], Bm[3];
			Rm[0] = BmIm[a][b].Bm[0]; Rm[1] = BmIm[a][b].Bm[1]; Rm[2] = BmIm[a][b].Bm[2];
			mbase.Multi(Aalin, Rm, Bm, 3, 3, 1);
			BmIm[a][b].Bm[0] = Bm[0]; BmIm[a][b].Bm[1] = Bm[1]; BmIm[a][b].Bm[2] = Bm[2];
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：ZY3根据星敏和轨道得到光行差修正后，星敏之间的角度
//输入：星敏数据，轨道数据
//输出：星敏光行差修正结果
//注意：
//作者：GZC
//日期：2016.12.06	更新：2017.12.04
//////////////////////////////////////////////////////////////////////////
bool AttDetermination::Aberration(vector<vector<BmImStar>>& BmIm, vector<Orbit_Ep> EpDat)
{
	int m = EpDat.size();
	Orbit_Ep *Epdata = new Orbit_Ep[m];
	Orbit_Ep Epdatainter;
	for (int i = 0; i < m; i++)
	{
		memcpy(&Epdata[i], &EpDat[i], sizeof(EpDat[i]));
	}
	double jd0, mjd, second, PosEarth[6], GCRS2ITRS[9], SatePos[3], SateVel[3], LightVelocity = 299792458.0;
	int year, month, day, hour, minute;
	memset(PosEarth, 0, sizeof(double) * 6);
	Cal2JD(2009, 1, 1, 0., &jd0, &mjd);
	//设置星历参数路径和EOP参数路径
	char *JPLPath = "C:\\Users\\wcsgz\\Documents\\2-CProject\\9-ZY3\\Need\\2000_2020_421.txt";
	char* EOPPath = "C:\\Users\\wcsgz\\Documents\\2-CProject\\9-ZY3\\Need\\EOP00.txt";

	for (int a = 0; a < BmIm.size(); a++)
	{
		vector<BmImStar>BmImTmp;
		for (int b = 0; b < BmIm[a].size(); b++)
		{
			double za[3];//星敏光轴相关定义
			za[0] = BmIm[a][b].Im[0], za[1] = BmIm[a][b].Im[1], za[2] = BmIm[a][b].Im[2];
			mbase.LagrangianInterpolation(Epdata, m, BmIm[a][b].UT, Epdatainter, 7);
			SatePos[0] = Epdatainter.X, SatePos[1] = Epdatainter.Y, SatePos[2] = Epdatainter.Z;
			SateVel[0] = Epdatainter.Xv, SateVel[1] = Epdatainter.Yv, SateVel[2] = Epdatainter.Zv;
			FromSecondtoYMD(mjd, BmIm[a][b].UT - 28800, year, month, day, hour, minute, second);
			PlanetEph(year, month, day, hour, minute, second, JPLPath, EOPPath, 2, 11, PosEarth);
			IAU2000ABaseCIOTerToCel(year, month, day, hour, minute, second, EOPPath, 8, GCRS2ITRS, SatePos, SateVel);
			//卫星在GCRS坐标系下的速度(考虑了地球相对太阳的速度)
			SateVel[0] = SateVel[0] + PosEarth[3], SateVel[1] = SateVel[1] + PosEarth[4], SateVel[2] = SateVel[2] + PosEarth[5];
			double VelRa[3], SateVelocity, costhetaA, sinthetaA;
			mbase.crossmultnorm(za, SateVel, VelRa);//修正旋转轴
			SateVelocity = sqrt(pow(SateVel[0], 2) + pow(SateVel[1], 2) + pow(SateVel[2], 2));
			costhetaA = (za[0] * SateVel[0] + za[1] * SateVel[1] + za[2] * SateVel[2]) / sqrt(za[0] * za[0] + za[1] * za[1] + za[2] * za[2]) / SateVelocity;
			sinthetaA = sqrt(1 - pow(costhetaA, 2));//星敏A光轴与速度夹角			
			double angleA, quatA[4], RotA[9];
			angleA = -(SateVelocity / LightVelocity)*sinthetaA;
			//修正星敏光轴的旋转四元数
			quatA[0] = cos(angleA / 2);
			quatA[1] = VelRa[0] * sin(angleA / 2), quatA[2] = VelRa[1] * sin(angleA / 2), quatA[3] = VelRa[2] * sin(angleA / 2);
			//转换为旋转矩阵
			mbase.quat2matrix(quatA[1], quatA[2], quatA[3], quatA[0], RotA);
			double zafix[3], zbfix[3], StarSensorAngle, detza, detzb;
			mbase.Multi(RotA, za, zafix, 3, 3, 1);
			BmIm[a][b].Im[0] = zafix[0], BmIm[a][b].Im[1] = zafix[1], BmIm[a][b].Im[2] = zafix[2];
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：ZY3根据星敏和轨道得到光行差修正后，星敏之间的角度
//输入：星敏数据，轨道数据
//输出：星敏光行差修正结果
//注意：
//作者：GZC
//日期：2016.12.06	更新：2017.12.04  更新2：2019.06.10
//////////////////////////////////////////////////////////////////////////
bool AttDetermination::AberrationForLuojia(StarGCP &Im, vector<Orbit_Ep> EpDat)
{
	int m = EpDat.size();
	Orbit_Ep *Epdata = new Orbit_Ep[m];
	Orbit_Ep Epdatainter;
	for (int i = 0; i < m; i++)
	{
		memcpy(&Epdata[i], &EpDat[i], sizeof(EpDat[i]));
	}
	double jd0, mjd, second, PosEarth[6], GCRS2ITRS[9], SatePos[3], SateVel[3], LightVelocity = 299792458.0;
	int year, month, day, hour, minute;
	memset(PosEarth, 0, sizeof(double) * 6);
	Cal2JD(2000, 1, 1, 0.5, &jd0, &mjd);
	//设置星历参数路径和EOP参数路径	
	char* JPLpath = "C:\\Users\\wcsgz\\Documents\\OneDrive\\2-CProject\\13-Luojia-1\\LJ01_GeoProcess\\ExtDlls\\2000_2020_421";
	char* EOPpath = "C:\\Users\\wcsgz\\Documents\\OneDrive\\2-CProject\\13-Luojia-1\\LJ01_GeoProcess\\ExtDlls\\EOP00.txt";

	double za[3];//星敏光轴相关定义
	za[0] = Im.V[0], za[1] = Im.V[1], za[2] = Im.V[2];
	mbase.LagrangianInterpolation(Epdata, m, Im.UTC, Epdatainter, 7);
	SatePos[0] = Epdatainter.X, SatePos[1] = Epdatainter.Y, SatePos[2] = Epdatainter.Z;
	SateVel[0] = Epdatainter.Xv, SateVel[1] = Epdatainter.Yv, SateVel[2] = Epdatainter.Zv;
	FromSecondtoYMD(mjd, Im.UTC, year, month, day, hour, minute, second);
	PlanetEph(year, month, day, hour, minute, second, JPLpath, EOPpath, 2, 11, PosEarth);
	IAU2000ABaseCIOTerToCel(year, month, day, hour, minute, second, EOPpath, 8, GCRS2ITRS, SatePos, SateVel);
	//卫星在GCRS坐标系下的速度(考虑了地球相对太阳的速度)
	SateVel[0] = SateVel[0] + PosEarth[3], SateVel[1] = SateVel[1] + PosEarth[4], SateVel[2] = SateVel[2] + PosEarth[5];
	double VelRa[3], SateVelocity, costhetaA, sinthetaA;
	mbase.crossmultnorm(za, SateVel, VelRa);//修正旋转轴
	double tmp[3];
	tmp[0] = za[1] * SateVel[2] - za[2] * SateVel[1];
	tmp[1] = za[2] * SateVel[0] - za[0] * SateVel[2];
	tmp[2] = za[0] * SateVel[1] - za[1] * SateVel[0];
	double xx = sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1] + tmp[2] * tmp[2])/LightVelocity;
	SateVelocity = sqrt(pow(SateVel[0], 2) + pow(SateVel[1], 2) + pow(SateVel[2], 2));
	costhetaA = (za[0] * SateVel[0] + za[1] * SateVel[1] + za[2] * SateVel[2]) / sqrt(za[0] * za[0] + za[1] * za[1] + za[2] * za[2]) / SateVelocity;
	sinthetaA = sqrt(1 - pow(costhetaA, 2));//星敏A光轴与速度夹角			
	double angleA, quatA[4], RotA[9];
	angleA = -(SateVelocity / LightVelocity)*sinthetaA;
	//修正星敏光轴的旋转四元数
	quatA[0] = cos(angleA / 2);
	quatA[1] = VelRa[0] * sin(angleA / 2), quatA[2] = VelRa[1] * sin(angleA / 2), quatA[3] = VelRa[2] * sin(angleA / 2);
	//转换为旋转矩阵
	mbase.quat2matrix(quatA[1], quatA[2], quatA[3], quatA[0], RotA);
	double zafix[3], zbfix[3], StarSensorAngle, detza, detzb;
	mbase.Multi(RotA, za, zafix, 3, 3, 1);
	Im.V[0] = zafix[0], Im.V[1] = zafix[1], Im.V[2] = zafix[2];
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：ZY3根据星敏和轨道得到光行差修正后，星敏之间的角度
//输入：星敏数据，轨道数据
//输出：星敏光行差修正结果
//注意：
//作者：GZC
//日期：2016.12.06	更新：2017.12.04  更新2：2019.06.11
//////////////////////////////////////////////////////////////////////////
bool AttDetermination::AberrationForLuojia2(StarGCP &Im, vector<Orbit_Ep> EpDat)
{
	int m = EpDat.size();
	Orbit_Ep *Epdata = new Orbit_Ep[m];
	Orbit_Ep Epdatainter;
	for (int i = 0; i < m; i++)
	{
		memcpy(&Epdata[i], &EpDat[i], sizeof(EpDat[i]));
	}
	double jd0, mjd, second, PosEarth[6], GCRS2ITRS[9], SatePos[3], SateVel[3], LightVelocity = 299792458.0;
	int year, month, day, hour, minute;
	memset(PosEarth, 0, sizeof(double) * 6);
	Cal2JD(2000, 1, 1, 0.5, &jd0, &mjd);
	//设置星历参数路径和EOP参数路径	
	char* JPLpath = "C:\\Users\\wcsgz\\Documents\\OneDrive\\2-CProject\\13-Luojia-1\\LJ01_GeoProcess\\ExtDlls\\2000_2020_421";
	char* EOPpath = "C:\\Users\\wcsgz\\Documents\\OneDrive\\2-CProject\\13-Luojia-1\\LJ01_GeoProcess\\ExtDlls\\EOP00.txt";

	double za[3];//星敏光轴相关定义
	za[0] = Im.V[0], za[1] = Im.V[1], za[2] = Im.V[2];
	mbase.LagrangianInterpolation(Epdata, m, Im.UTC, Epdatainter, 7);
	SatePos[0] = Epdatainter.X, SatePos[1] = Epdatainter.Y, SatePos[2] = Epdatainter.Z;
	SateVel[0] = Epdatainter.Xv, SateVel[1] = Epdatainter.Yv, SateVel[2] = Epdatainter.Zv;
	FromSecondtoYMD(mjd, Im.UTC, year, month, day, hour, minute, second);
	PlanetEph(year, month, day, hour, minute, second, JPLpath, EOPpath, 2, 11, PosEarth);
	IAU2000ABaseCIOTerToCel(year, month, day, hour, minute, second, EOPpath, 8, GCRS2ITRS, SatePos, SateVel);
	//卫星在GCRS坐标系下的速度(考虑了地球相对太阳的速度)
	SateVel[0] = SateVel[0] + PosEarth[3], SateVel[1] = SateVel[1] + PosEarth[4], SateVel[2] = SateVel[2] + PosEarth[5];
	double VelRa[3], SateVelocity, costhetaA, sinthetaA;
	mbase.crossmultnorm(za, SateVel, VelRa);//修正旋转轴
	SateVelocity = sqrt(pow(SateVel[0], 2) + pow(SateVel[1], 2) + pow(SateVel[2], 2));
	costhetaA = (za[0] * SateVel[0] + za[1] * SateVel[1] + za[2] * SateVel[2]) / sqrt(za[0] * za[0] + za[1] * za[1] + za[2] * za[2]) / SateVelocity;
	sinthetaA = sqrt(1 - pow(costhetaA, 2));//星敏A光轴与速度夹角			
	double angleA, quatA[4], RotA[9];
	angleA = -(SateVelocity / LightVelocity)*sinthetaA;
	//修正星敏光轴的旋转四元数
	quatA[0] = cos(angleA / 2);
	quatA[1] = VelRa[0] * sin(angleA / 2), quatA[2] = VelRa[1] * sin(angleA / 2), quatA[3] = VelRa[2] * sin(angleA / 2);
	//转换为旋转矩阵
	mbase.quat2matrix(quatA[1], quatA[2], quatA[3], quatA[0], RotA);
	double zafix[3], zbfix[3], StarSensorAngle, detza, detzb;
	mbase.Multi(RotA, za, zafix, 3, 3, 1);
	Im.V[0] = zafix[0], Im.V[1] = zafix[1], Im.V[2] = zafix[2];
	return true;
}
//////////////////////////////////////////////////////////////////////////
//功能：计算恒星标定后精度
//输入：starCatlog，恒星参数，包括J2000系下的单位矢量V[3]
//		R，星敏坐标系与J2000系的旋转关系，Crj
//输出：
//日期：2019.06.13
//////////////////////////////////////////////////////////////////////////
void AttDetermination::CalcXYaccuracy(vector<StarGCP> starCatlog, Quat quater, vector<Orbit_Ep>imgOrb)
{
	double x, y, R[9];
	FrameDistortion param;
	string fpath = workpath + "StarPointAccuracy.txt";
	string cbrPath = workpath +"2018-06-28.cbr";
	FILE *fp = fopen(fpath.c_str(), "w");
	mbase.quat2matrix(quater.Q1, quater.Q2, quater.Q3, quater.Q0, R);
	readCompensate(cbrPath,param);
	for (int i = 0; i < starCatlog.size(); i++)
	{
		double V[3], W[3];
		V[0] = starCatlog[i].V[0]; V[1] = starCatlog[i].V[1]; V[2] = starCatlog[i].V[2];
		StarGCP tmp;
		tmp.UTC = starCatlog[i].UTC;
		tmp.V[0] = V[0]; tmp.V[1] = V[1]; tmp.V[2] = V[2];
		//AberrationForLuojia(tmp, imgOrb);
		//V[0] = tmp.V[0]; V[1] = tmp.V[1]; V[2] = tmp.V[2];
		mbase.Multi(R, V, W, 3, 3, 1);
		//x0=y0=512,f=43.3mm,像元大小0.015mm
		double x0 = 2048 / 2, y0 = 2048 / 2;
		double f = 0.055086;
		double pixel = 11 / 1.e6;
		if (W[2] > 0)
		{
			x = (x0 - W[0] / W[2] * f / pixel);
			y = (y0 - W[1] / W[2] * f / pixel);
		}
		else
		{
			x = -x0 - 1, y = -y0 - 1;
		}//这个是用来判断是否和星敏指向的半球方向一致
		double cx, cy;
		//getCoorInCamera(param,starCatlog[i].x, starCatlog[i].y, cx,cy);
		//starCatlog[i].x = 1024 - cx * f / pixel;
		//starCatlog[i].y = 1024 - cy * f / pixel;
		double xx = x - starCatlog[i].x;
		double yy = y - starCatlog[i].y;
		double zz = sqrt(xx*xx + yy * yy);
		fprintf(fp, "%d\t%d\t%.4f\t%.4f\t%.4f\n", (int)x,(int)y,xx, yy, zz);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：计算星点提取精度
//输入：starCatlog，恒星参数，包括J2000系下的单位矢量V[3]
//		R，星敏坐标系与J2000系的旋转关系，Crj
//输出：
//日期：2019.06.13
//////////////////////////////////////////////////////////////////////////
void AttDetermination::CalcStarExtractAccuracy(vector<StarGCP> starCatlog)
{
	double x, y, cx, cy;
	double V[3], W[3],VV[3],WW[3];
	double angle1, angle2, angle;
	FrameDistortion param;
	string fpath = workpath + "StarPointExtractAccuracy.txt";
	string cbrPath = workpath + "2018-06-28.cbr";
	readCompensate(cbrPath, param);
	FILE *fp = fopen(fpath.c_str(), "w");
	for (int i = 0; i < starCatlog.size()-1; i++)
	{
		V[0] = starCatlog[i].V[0]; V[1] = starCatlog[i].V[1]; V[2] = starCatlog[i].V[2];
		getCoorInCamera(param, starCatlog[i].x, starCatlog[i].y, cx, cy);
		W[0] = cx; W[1] = cy; W[2] = 1;
		for (int j =i+1 ; j < starCatlog.size(); j++)
		{
			VV[0] = starCatlog[j].V[0]; VV[1] = starCatlog[j].V[1]; VV[2] = starCatlog[j].V[2];
			getCoorInCamera(param, starCatlog[j].x, starCatlog[j].y, cx, cy);
			WW[0] = cx; WW[1] = cy; WW[2] = 1;
			angle1 = (V[0] * VV[0] + V[1] * VV[1] + V[2] * VV[2])/sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2])/sqrt(VV[0] * VV[0] + VV[1] * VV[1] + VV[2] * VV[2]);
			angle2 = (W[0] * WW[0] + W[1] * WW[1] + W[2] * WW[2]) / sqrt(W[0] * W[0] + W[1] * W[1] + W[2] * W[2]) / sqrt(WW[0] * WW[0] + WW[1] * WW[1] + WW[2] * WW[2]);
			angle = (angle1 - angle2)/PI*180*3600;
			fprintf(fp, "%d\t%d\t%.9f\t%.9f\t%.9f\n",i,j,angle1, angle2, angle);
		}
	}
	fclose(fp);
}
//读取珞珈内定标参数
bool AttDetermination::readCompensate(string sPath, FrameDistortion& param)
{
	if (sPath.empty() == true)
		return false;

	FILE *fp = fopen(sPath.c_str(), "r");
	if (!fp)
	{
		return false;
	}
	fscanf(fp, "%d%d", &param.xOrder, &param.yOrder);

	if (param.xOrder < 1 || param.yOrder < 1)
	{
		fclose(fp);
		return false;
	}

	int nParams1 = (param.xOrder + 1)*(param.xOrder + 2) / 2;
	int nParams2 = (param.yOrder + 1)*(param.yOrder + 2) / 2;
	for (int i = 0; i < nParams1; i++)
	{
		fscanf(fp, "%lf", &param.px[i]);
		//add by wjy
		//fscanf(fp,"%lf",&param.px[i] + i);
	}
	for (int i = 0; i < nParams2; i++)
	{
		fscanf(fp, "%lf", &param.py[i]);
		//fscanf(fp,"%lf",&param.py[i] + i);
	}

	fclose(fp);
	return true;
}
void AttDetermination::getCoorInCamera(FrameDistortion param, double x, double y, double &cx, double &cy)
{
		cx = mbase.getValue_poly(param.px, param.xOrder, x, y);
		cy = mbase.getValue_poly(param.py, param.yOrder, x, y);
}
//////////////////////////////////////////////////////////////////////////
//功能：15状态卡尔曼滤波主程序
//输入：AttData：STG解析出的姿态数据	
//输出：EKFatt：姿态确定结果
//注意：
//作者：GZC
//日期：2017.07.10
//////////////////////////////////////////////////////////////////////////
void AttDetermination::EKF15State(vector<STGData> AttData, Quat *quatEst, double *xest_store)
{
	double GyDat[3], GyTran[3];
	int m = AttData.size();
	int nGyro = m, nQuat = m;
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	Quat *qMeas = new Quat[m];
	Gyro *wMeas = new Gyro[m];
	for (size_t i = 0; i < m; i++)
	{
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		wMeas[i].x = GyTran[0] / 180 * PI, wMeas[i].y = GyTran[1] / 180 * PI, wMeas[i].z = GyTran[2] / 180 * PI;
		wMeas[i].UTC = AttData[i].utgyro;
	}
	vector<Quat>APSdat(m);
	for (size_t i = 0; i < m; i++)
	{
		APSdat[i].UTC = AttData[i].StarA.UTC;
		APSdat[i].Q0 = AttData[i].StarA.Q0; APSdat[i].Q1 = AttData[i].StarA.Q1;
		APSdat[i].Q2 = AttData[i].StarA.Q2; APSdat[i].Q3 = AttData[i].StarA.Q3;
	}
	alinAPS(APSdat);//乘以安装矩阵
	for (size_t i = 0; i < m; i++)
	{
		qMeas[i].UTC = APSdat[i].UTC;
		qMeas[i].Q0 = APSdat[i].Q0; qMeas[i].Q1 = APSdat[i].Q1;
		qMeas[i].Q2 = APSdat[i].Q2; qMeas[i].Q3 = APSdat[i].Q3;
	}

	double sig = 8. / 3600 * PI / 180;//星敏噪声，角秒转弧度
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d poa, pog, pos, poku, pokl, r, sigu, sigv, wa, sest, uhat, lhat, wec, diagwe_nos;
	MatrixXd p(15, 15), qcov(6, 6), qcovd(15, 15), xe(1, 3), z(3, 1), h(3, 15), k(15, 3),
		we(3, 1), wetmp(3, 1), we_nos(3, 1), tempqe(4, 1), om(4, 4),
		fmat(15, 15), gmat(15, 6), phi(6, 6), gamma(6, 6);
	MatrixXd eye33 = MatrixXd::Identity(3, 3);
	MatrixXd zero33 = MatrixXd::Zero(3, 3);
	MatrixXd eye15 = MatrixXd::Identity(15, 15);
	poa << pow((0.1*PI / 180), 2)*eye33;//初始姿态误差协方差0.1°
	pog << pow((0.2*PI / 180 / 3600), 2)*eye33;//初始陀螺误差协方差0.2°/Hr
	pos << pow((2 * 1e-9 / 3), 2)* eye33;//初始尺度因子
	poku << pow((2 * 1e-9 / 3), 2) * eye33;//初始上三角安装误差
	pokl << pow((2 * 1e-9 / 3), 2) *eye33;//初始下三角安装误差
	r << pow(sig, 2)*eye33;//星敏噪声	

	//预先计算估计四元数的数量
	double utStart = qMeas[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			utStart = qMeas[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UTC;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), xest(b + 1, 15);

	/************************************************************************/
	/*									卡尔曼滤波正向递推过程	                                  */
	/************************************************************************/
	a = 1, b = 0;
	utStart = qMeas[0].UTC;
	//初始四元数估计和漂移估计
	xest.row(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;//状态初始值15维
															   //初始协方差
	sigv << pow(1e-7, 2)*eye33;//陀螺噪声
	sigu << pow(1e-10, 2)*eye33;//陀螺漂移噪声
	qcov << sigv, zero33, zero33, sigu;//过程噪声协方差
	p << poa, zero33, zero33, zero33, zero33,
		zero33, pog, zero33, zero33, zero33,
		zero33, zero33, pos, zero33, zero33,
		zero33, zero33, zero33, poku, zero33,
		zero33, zero33, zero33, zero33, pokl;//过程协方差
	Qest(0, 0) = qMeas[0].Q1, Qest(0, 1) = qMeas[0].Q2;
	Qest(0, 2) = qMeas[0].Q3, Qest(0, 3) = qMeas[0].Q0;
	quatEst[0].UTC = qMeas[0].UTC;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = qMeas[a].UTC - utStart;
			utStart = qMeas[a].UTC;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = wMeas[i - 1].x - xest(b, 3);
			we_nos(1) = wMeas[i - 1].y - xest(b, 4);
			we_nos(2) = wMeas[i - 1].z - xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat * dt;
			qcovd = dt * gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w * sin(0.5*w*dt);
			qw2 = we(1) / w * sin(0.5*w*dt);
			qw3 = we(2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi * p*phi.transpose() + qcovd;
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].Q0*Qest(b, 0) - qMeas[a].Q3*Qest(b, 1) + qMeas[a].Q2*Qest(b, 2) + qMeas[a].Q1*Qest(b, 3);
			qmm2 = qMeas[a].Q3*Qest(b, 0) - qMeas[a].Q0*Qest(b, 1) - qMeas[a].Q1*Qest(b, 2) + qMeas[a].Q2*Qest(b, 3);
			qmm3 = -qMeas[a].Q2*Qest(b, 0) + qMeas[a].Q1*Qest(b, 1) - qMeas[a].Q0*Qest(b, 2) + qMeas[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, MatrixXd::Zero(3, 12);
				k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye15 - k * h)*p*(eye15 - k * h).transpose() + k * r*k.transpose();
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UTC - utStart;
			utStart = wMeas[i].UTC;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = wMeas[i - 1].x - xest(b, 3);
			we_nos(1) = wMeas[i - 1].y - xest(b, 4);
			we_nos(2) = wMeas[i - 1].z - xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat * dt;
			qcovd = dt * gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w * sin(0.5*w*dt);
			qw2 = we(1) / w * sin(0.5*w*dt);
			qw3 = we(2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi * p*phi.transpose() + qcovd;
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;

			b++;
			i++;
		}
	}
	/************************************************************************/
	/*									卡尔曼滤波逆向递推过程	                                  */
	/************************************************************************/
	quatEst[nGyro - 1].UTC = utStart;
	quatEst[nGyro - 1].Q1 = Qest(b, 0); quatEst[nGyro - 1].Q2 = Qest(b, 1);
	quatEst[nGyro - 1].Q3 = Qest(b, 2); quatEst[nGyro - 1].Q0 = Qest(b, 3);
	xest_store[15 * (nGyro - 1) + 0] = wMeas[m - 1].UTC; xest_store[15 * (nGyro - 1) + 1] = xest(b, 1); xest_store[15 * (nGyro - 1) + 2] = xest(b, 2);
	xest_store[15 * (nGyro - 1) + 3] = xest(b, 3); xest_store[15 * (nGyro - 1) + 4] = xest(b, 4); xest_store[15 * (nGyro - 1) + 5] = xest(b, 5);
	xest_store[15 * (nGyro - 1) + 6] = xest(b, 6); xest_store[15 * (nGyro - 1) + 7] = xest(b, 7); xest_store[15 * (nGyro - 1) + 8] = xest(b, 8);
	xest_store[15 * (nGyro - 1) + 9] = xest(b, 9); xest_store[15 * (nGyro - 1) + 10] = xest(b, 10); xest_store[15 * (nGyro - 1) + 11] = xest(b, 11);
	xest_store[15 * (nGyro - 1) + 12] = xest(b, 12); xest_store[15 * (nGyro - 1) + 13] = xest(b, 13); xest_store[15 * (nGyro - 1) + 14] = xest(b, 14);
	a = nQuat - 2;
	for (int i = nGyro - 2; i > 0;)
	{
		if (a > 0 && (qMeas[a].UTC - utStart) >= (wMeas[i].UTC - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = utStart - qMeas[a].UTC;
			utStart = qMeas[a].UTC;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = -wMeas[i].x + xest(b, 3);
			we_nos(1) = -wMeas[i].y + xest(b, 4);
			we_nos(2) = -wMeas[i].z + xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat * dt;
			qcovd = dt * gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w * sin(0.5*w*dt);
			qw2 = we(1) / w * sin(0.5*w*dt);
			qw3 = we(2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();

			//Propagate Covariance
			p = phi * p*phi.transpose() + qcovd;
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;
			b--;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].Q0*Qest(b, 0) - qMeas[a].Q3*Qest(b, 1) + qMeas[a].Q2*Qest(b, 2) + qMeas[a].Q1*Qest(b, 3);
			qmm2 = qMeas[a].Q3*Qest(b, 0) - qMeas[a].Q0*Qest(b, 1) - qMeas[a].Q1*Qest(b, 2) + qMeas[a].Q2*Qest(b, 3);
			qmm3 = -qMeas[a].Q2*Qest(b, 0) + qMeas[a].Q1*Qest(b, 1) - qMeas[a].Q0*Qest(b, 2) + qMeas[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			if (qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, MatrixXd::Zero(3, 12);
				k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye15 - k * h)*p*(eye15 - k * h).transpose() + k * r*k.transpose();
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a--;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = utStart - wMeas[i].UTC;
			utStart = wMeas[i].UTC;
			sest << xest(b, 6), xest(b, 9), xest(b, 10),
				xest(b, 12), xest(b, 7), xest(b, 11),
				xest(b, 13), xest(b, 14), xest(b, 8);
			we_nos(0) = -wMeas[i].x + xest(b, 3);
			we_nos(1) = -wMeas[i].y + xest(b, 4);
			we_nos(2) = -wMeas[i].z + xest(b, 5);
			we = (eye33 - sest)*we_nos;
			uhat << we_nos(1), we_nos(2), 0,
				0, 0, we_nos(2),
				0, 0, 0;
			lhat << 0, 0, 0,
				we_nos(0), 0, 0,
				0, we_nos(0), we_nos(1);
			wec << 0, -we(2), we(1),
				we(2), 0, -we(0),
				-we(1), we(0), 0;
			diagwe_nos = we_nos.asDiagonal();
			fmat << -wec, -(eye33 - sest), -diagwe_nos, -uhat, -lhat, MatrixXd::Zero(12, 15);
			gmat << -(eye33 - sest), zero33, zero33, eye33, MatrixXd::Zero(9, 6);
			phi = eye15 + fmat * dt;
			qcovd = dt * gmat*qcov*gmat.transpose();
			//gamma = (eye15*dt + fmat*dt*dt / 2)*gmat;

			//Propagate State
			w = sqrt(we(0)*we(0) + we(1)*we(1) + we(2)*we(2));
			qw1 = we(0) / w * sin(0.5*w*dt);
			qw2 = we(1) / w * sin(0.5*w*dt);
			qw3 = we(2) / w * sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b - 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi * p*phi.transpose() + qcovd;
			xest.row(b - 1) = xest.row(b);
			xest(b - 1, 0) = 0; xest(b - 1, 1) = 0; xest(b - 1, 2) = 0;

			quatEst[i].UTC = wMeas[i].UTC;
			quatEst[i].Q1 = Qest(b - 1, 0), quatEst[i].Q2 = Qest(b - 1, 1);
			quatEst[i].Q3 = Qest(b - 1, 2), quatEst[i].Q0 = Qest(b - 1, 3);
			//保存xest值
			xest_store[15 * i + 0] = wMeas[i].UTC; xest_store[15 * i + 1] = xest(b, 1); xest_store[15 * i + 2] = xest(b, 2);
			xest_store[15 * i + 3] = xest(b, 3); xest_store[15 * i + 4] = xest(b, 4); xest_store[15 * i + 5] = xest(b, 5);
			xest_store[15 * i + 6] = xest(b, 6); xest_store[15 * i + 7] = xest(b, 7); xest_store[15 * i + 8] = xest(b, 8);
			xest_store[15 * i + 9] = xest(b, 9); xest_store[15 * i + 10] = xest(b, 10); xest_store[15 * i + 11] = xest(b, 11);
			xest_store[15 * i + 12] = xest(b, 12); xest_store[15 * i + 13] = xest(b, 13); xest_store[15 * i + 14] = xest(b, 14);

			b--;
			i--;
		}
	}
	memcpy(xest_store, &xest_store[15], sizeof(double) * 15);
	delete[]qMeas, wMeas; qMeas = NULL, wMeas = NULL;
}

//////////////////////////////////////////////////////////////////////////
//功能：姿态仿真卡尔曼滤波
//输入：	
//输出：
//注意：
//作者：GZC
//日期：2017.06.09
//////////////////////////////////////////////////////////////////////////
void AttDetermination::simAttandEKF(double dt, int m, double sig_ST, string Res,
	MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas)
{
	MatrixXd Qest(m, 4);
	Qest(0, 0) = qMeas(0, 0), Qest(0, 1) = qMeas(0, 1), Qest(0, 2) = qMeas(0, 2), Qest(0, 3) = qMeas(0, 3);
	double a1 = Qest(0, 0), a2 = Qest(0, 1), a3 = Qest(0, 2), a4 = Qest(0, 3);

	double sig = sig_ST / 3600 * PI / 180;
	Matrix3d zero33, eye33, poa, pog, r, sigu, sigv;
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu << 1e-19*eye33;//陀螺漂移噪声
	sigv << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), be(m, 3), we(m, 3), xest(m, 6);
	eye66 << eye33, zero33, zero33, eye33;
	be.row(0) << 0, 0, 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv, zero33, zero33, sigu;//过程噪声

	string path = string(Res);
	string strpath = path + "\\GyroBiasEstimate.txt";
	string strpath1 = path + "\\EKF.txt";
	FILE *fpres = fopen(strpath.c_str(), "w");
	FILE *fpEKF = fopen(strpath1.c_str(), "w");

	int j = 0;
	/*while (j<2)
	{*/
	for (int i = 0; i < m - 1; i++)
	{
		double qmm1, qmm2, qmm3;
		qmm1 = -qMeas(i, 3)*Qest(i, 0) - qMeas(i, 2)*Qest(i, 1) + qMeas(i, 1)*Qest(i, 2) + qMeas(i, 0)*Qest(i, 3);
		qmm2 = qMeas(i, 2)*Qest(i, 0) - qMeas(i, 3)*Qest(i, 1) - qMeas(i, 0)*Qest(i, 2) + qMeas(i, 1)*Qest(i, 3);
		qmm3 = -qMeas(i, 1)*Qest(i, 0) + qMeas(i, 0)*Qest(i, 1) - qMeas(i, 3)*Qest(i, 2) + qMeas(i, 2)*Qest(i, 3);
		MatrixXd z(3, 1);
		z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
		//cout<<"观测残差："<<z.transpose()<<endl;
		MatrixXd h(3, 6), k(6, 3);
		h << eye33, zero33;
		k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
		//cout<<k<<endl;
		p = (eye66 - k * h)*p;
		//cout<<"p"<<p<<endl;
		xest.row(i) = xest.row(i) + (k*z).transpose();
		//cout<<xest.row(i);

		MatrixXd xe(1, 3);
		xe = 0.5*xest.row(i).head(3);
		double qe11, qe22, qe33, qe44;
		qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
		qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
		qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
		qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
		MatrixXd tempqe(4, 1);
		tempqe << qe11, qe22, qe33, qe44;
		tempqe.normalize();
		Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
		//cout<<Qest.row(i)<<endl;

		//Propagate Covariance
		//cout<<Wgm.row(i)<<endl;
		//cout<<xest.row(i).tail(3)<<endl;
		we.row(i) = wMeas.row(i) - xest.row(i).tail(3);
		double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
		Matrix3d wa;
		//wa<<0,we(i,2),-we(i,1),-we(i,2),0,we(i,0),we(i,1),-we(i,0),0;
		wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
		//cout<<wa<<endl;
		MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
		fmat << -wa, -eye33, zero33, zero33;
		gmat << -eye33, zero33, zero33, eye33;
		phi = eye66 + fmat * dt;
		//gamma=gmat*dt;
		gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;
		//cout<<phi<<endl;
		//cout<<gamma<<endl;

		//Propagate State
		double qw1, qw2, qw3, qw4;
		qw1 = we(i, 0) / w * sin(0.5*w*dt);
		qw2 = we(i, 1) / w * sin(0.5*w*dt);
		qw3 = we(i, 2) / w * sin(0.5*w*dt);
		qw4 = cos(0.5*w*dt);
		MatrixXd om(4, 4);
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		//cout<<om<<endl;
		Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();
		//cout<<Qest.row(i+1)<<endl;

		//Propagate Covariance
		p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
		xest.row(i + 1) = xest.row(i);
		xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;
		//cout<<xest.row(i)<<endl;

		fprintf(fpres, "%lf\t%lf\t%lf\n", xest(i, 3) * 180 / PI * 3600 / dt,
			xest(i, 4) * 180 / PI * 3600 / dt, xest(i, 5) * 180 / PI * 3600 / dt);
	}
	/*j++;
	xest(0, 3) = xest(m - 1, 3), xest(0, 4) = xest(m - 1, 4), xest(0, 5) = xest(m - 1, 5);
}*/

	double dq1[4], dq2[4], dq3[4];
	fprintf(fpEKF, "%d\n", m);
	for (int i = 0; i < m; i++)
	{
		dq1[0] = qTure(i, 0); dq1[1] = qTure(i, 1); dq1[2] = qTure(i, 2); dq1[3] = qTure(i, 3);
		dq2[0] = -Qest(i, 0); dq2[1] = -Qest(i, 1); dq2[2] = -Qest(i, 2); dq2[3] = Qest(i, 3);
		mbase.quatMult(dq1, dq2, dq3);
		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTure(i, 3), qTure(i, 0), qTure(i, 1), qTure(i, 2), Qest(i, 3), Qest(i, 0), Qest(i, 1), Qest(i, 2), dq3[0], dq3[1], dq3[2]);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：姿态仿真卡尔曼滤波
//输入：	
//输出：
//注意：
//作者：GZC
//日期：2017.06.26
//////////////////////////////////////////////////////////////////////////
void AttDetermination::simEKFForwardAndBackward(double dt, int m, double sig_ST, string Res,
	MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas)
{
	MatrixXd Qest(m, 4);
	Qest(0, 0) = qMeas(0, 0), Qest(0, 1) = qMeas(0, 1), Qest(0, 2) = qMeas(0, 2), Qest(0, 3) = qMeas(0, 3);
	double a1 = Qest(0, 0), a2 = Qest(0, 1), a3 = Qest(0, 2), a4 = Qest(0, 3);

	double sig = sig_ST / 3600 * PI / 180;
	Matrix3d zero33, eye33, poa, pog, r, sigu, sigv;
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu << 1e-19*eye33;//陀螺漂移噪声
	sigv << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), be(m, 3), we(m, 3), xest(m, 6);
	eye66 << eye33, zero33, zero33, eye33;
	be.row(0) << 0, 0, 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv, zero33, zero33, sigu;//过程噪声

	string path = string(Res);
	string strpath = path + "\\GyroBiasEstimate.txt";
	string strpath1 = path + "\\EKF.txt";
	FILE *fpres = fopen(strpath.c_str(), "w");
	FILE *fpEKF = fopen(strpath1.c_str(), "w");

	////////////////////////////////////////////////////////////
	////////////////////前向卡尔曼滤波///////////////////////
	////////////////////////////////////////////////////////////
	for (int i = 0; i < m - 1; i++)
	{
		double qmm1, qmm2, qmm3;
		qmm1 = -qMeas(i, 3)*Qest(i, 0) - qMeas(i, 2)*Qest(i, 1) + qMeas(i, 1)*Qest(i, 2) + qMeas(i, 0)*Qest(i, 3);
		qmm2 = qMeas(i, 2)*Qest(i, 0) - qMeas(i, 3)*Qest(i, 1) - qMeas(i, 0)*Qest(i, 2) + qMeas(i, 1)*Qest(i, 3);
		qmm3 = -qMeas(i, 1)*Qest(i, 0) + qMeas(i, 0)*Qest(i, 1) - qMeas(i, 3)*Qest(i, 2) + qMeas(i, 2)*Qest(i, 3);
		MatrixXd z(3, 1);
		z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
		MatrixXd h(3, 6), k(6, 3);
		h << eye33, zero33;
		k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
		p = (eye66 - k * h)*p;
		xest.row(i) = xest.row(i) + (k*z).transpose();

		MatrixXd xe(1, 3);
		xe = 0.5*xest.row(i).head(3);
		double qe11, qe22, qe33, qe44;
		qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
		qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
		qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
		qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
		MatrixXd tempqe(4, 1);
		tempqe << qe11, qe22, qe33, qe44;
		tempqe.normalize();
		Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

		//Propagate Covariance
		we.row(i) = wMeas.row(i) - xest.row(i).tail(3);
		double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
		Matrix3d wa;
		wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
		MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
		fmat << -wa, -eye33, zero33, zero33;
		gmat << -eye33, zero33, zero33, eye33;
		phi = eye66 + fmat * dt;
		gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;

		//Propagate State
		double qw1, qw2, qw3, qw4;
		qw1 = we(i, 0) / w * sin(0.5*w*dt);
		qw2 = we(i, 1) / w * sin(0.5*w*dt);
		qw3 = we(i, 2) / w * sin(0.5*w*dt);
		qw4 = cos(0.5*w*dt);
		MatrixXd om(4, 4);
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();

		//Propagate Covariance
		p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
		xest.row(i + 1) = xest.row(i);
		xest(i + 1, 0) = 0; xest(i + 1, 1) = 0; xest(i + 1, 2) = 0;

		//fprintf(fpres, "%lf\t%lf\t%lf\n", xest(i, 3) * 180 / PI * 3600/dt,
		//	xest(i, 4) * 180 / PI * 3600 / dt, xest(i, 5) * 180 / PI * 3600 / dt);
	}
	////////////////////////////////////////////////////////////
	////////////////////逆向卡尔曼滤波///////////////////////
	////////////////////////////////////////////////////////////
	for (int i = m - 1; i > 0; i--)
	{
		double qmm1, qmm2, qmm3;
		qmm1 = -qMeas(i, 3)*Qest(i, 0) - qMeas(i, 2)*Qest(i, 1) + qMeas(i, 1)*Qest(i, 2) + qMeas(i, 0)*Qest(i, 3);
		qmm2 = qMeas(i, 2)*Qest(i, 0) - qMeas(i, 3)*Qest(i, 1) - qMeas(i, 0)*Qest(i, 2) + qMeas(i, 1)*Qest(i, 3);
		qmm3 = -qMeas(i, 1)*Qest(i, 0) + qMeas(i, 0)*Qest(i, 1) - qMeas(i, 3)*Qest(i, 2) + qMeas(i, 2)*Qest(i, 3);
		MatrixXd z(3, 1);
		z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
		MatrixXd h(3, 6), k(6, 3);
		h << eye33, zero33;
		k = p * h.transpose()*(h*p*h.transpose() + r).inverse();
		p = (eye66 - k * h)*p;
		xest.row(i) = xest.row(i) + (k*z).transpose();

		MatrixXd xe(1, 3);
		xe = 0.5*xest.row(i).head(3);
		double qe11, qe22, qe33, qe44;
		qe11 = Qest(i, 0) + xe(2)*Qest(i, 1) - xe(1)*Qest(i, 2) + xe(0)*Qest(i, 3);
		qe22 = -xe(2)*Qest(i, 0) + Qest(i, 1) + xe(0)*Qest(i, 2) + xe(1)*Qest(i, 3);
		qe33 = xe(1)*Qest(i, 0) - xe(0)*Qest(i, 1) + Qest(i, 2) + xe(2)*Qest(i, 3);
		qe44 = -xe(0)*Qest(i, 0) - xe(1)*Qest(i, 1) - xe(2)*Qest(i, 2) + Qest(i, 3);
		MatrixXd tempqe(4, 1);
		tempqe << qe11, qe22, qe33, qe44;
		tempqe.normalize();
		Qest.row(i) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);

		//Propagate Covariance
		we.row(i) = -wMeas.row(i - 1) + xest.row(i).tail(3);
		double w = sqrt(we(i, 0)*we(i, 0) + we(i, 1)*we(i, 1) + we(i, 2)*we(i, 2));
		Matrix3d wa;
		wa << 0, -we(i, 2), we(i, 1), we(i, 2), 0, -we(i, 0), -we(i, 1), we(i, 0), 0;
		MatrixXd fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
		fmat << -wa, -eye33, zero33, zero33;
		gmat << -eye33, zero33, zero33, eye33;
		phi = eye66 + fmat * dt;
		gamma = (eye66*dt + fmat * dt*dt / 2)*gmat;

		//Propagate State
		double qw1, qw2, qw3, qw4;
		qw1 = we(i, 0) / w * sin(0.5*w*dt);
		qw2 = we(i, 1) / w * sin(0.5*w*dt);
		qw3 = we(i, 2) / w * sin(0.5*w*dt);
		qw4 = cos(0.5*w*dt);
		MatrixXd om(4, 4);
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		Qest.row(i - 1) = (om*Qest.row(i).transpose()).transpose();

		//Propagate Covariance
		p = phi * p*phi.transpose() + gamma * Q*gamma.transpose();
		xest.row(i - 1) = xest.row(i);
		xest(i - 1, 0) = 0; xest(i - 1, 1) = 0; xest(i - 1, 2) = 0;

		fprintf(fpres, "%lf\t%lf\t%lf\n", xest(i, 3) * 180 / PI * 3600 / dt,
			xest(i, 4) * 180 / PI * 3600 / dt, xest(i, 5) * 180 / PI * 3600 / dt);
	}

	double dq1[4], dq2[4], dq3[4];
	fprintf(fpEKF, "%d\n", m);
	for (int i = 0; i < m; i++)
	{
		dq1[0] = qTure(i, 0); dq1[1] = qTure(i, 1); dq1[2] = qTure(i, 2); dq1[3] = qTure(i, 3);
		dq2[0] = -Qest(i, 0); dq2[1] = -Qest(i, 1); dq2[2] = -Qest(i, 2); dq2[3] = Qest(i, 3);
		mbase.quatMult(dq1, dq2, dq3);
		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
		fprintf(fpEKF, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTure(i, 3), qTure(i, 0), qTure(i, 1), qTure(i, 2), Qest(i, 3), Qest(i, 0), Qest(i, 1), Qest(i, 2), dq3[0], dq3[1], dq3[2]);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：用陀螺测量数据递推，验证初始四元数不准对后续递推的影响
//输入：	
//输出：
//注意：
//作者：GZC
//日期：2017.06.25
//////////////////////////////////////////////////////////////////////////
void AttDetermination::simGyroAndAcc(double dt, int m, string Res, double wBiasA[3],
	MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas)
{
	MatrixXd Qest(m, 4);
	Qest(0, 0) = qMeas(0, 0), Qest(0, 1) = qMeas(0, 1), Qest(0, 2) = qMeas(0, 2), Qest(0, 3) = qMeas(0, 3);
	for (int i = 0; i < m - 1; i++)
	{
		wMeas(i, 0) = wMeas(i, 0) - wBiasA[0] / 3600 / 180 * PI*dt;
		wMeas(i, 1) = wMeas(i, 1) - wBiasA[1] / 3600 / 180 * PI*dt;
		wMeas(i, 2) = wMeas(i, 2) - wBiasA[2] / 3600 / 180 * PI*dt;
		double w = sqrt(wMeas(i, 0)*wMeas(i, 0) + wMeas(i, 1)*wMeas(i, 1) + wMeas(i, 2)*wMeas(i, 2));
		//Propagate State
		double qw1, qw2, qw3, qw4;
		qw1 = wMeas(i, 0) / w * sin(0.5*w*dt);
		qw2 = wMeas(i, 1) / w * sin(0.5*w*dt);
		qw3 = wMeas(i, 2) / w * sin(0.5*w*dt);
		qw4 = cos(0.5*w*dt);
		MatrixXd om(4, 4);
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		Qest.row(i + 1) = (om*Qest.row(i).transpose()).transpose();
	}

	string path = string(Res);
	string strpath = path + "\\初始四元数直接陀螺积分.txt";
	FILE *fp = fopen(strpath.c_str(), "w");
	double dq1[4], dq2[4], dq3[4];
	for (int i = 0; i < m; i++)
	{
		dq1[0] = qTure(i, 0); dq1[1] = qTure(i, 1); dq1[2] = qTure(i, 2); dq1[3] = qTure(i, 3);
		dq2[0] = -Qest(i, 0); dq2[1] = -Qest(i, 1); dq2[2] = -Qest(i, 2); dq2[3] = Qest(i, 3);
		mbase.quatMult(dq1, dq2, dq3);
		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTure(i, 3), qTure(i, 0), qTure(i, 1), qTure(i, 2), Qest(i, 3), Qest(i, 0), Qest(i, 1), Qest(i, 2), dq3[0], dq3[1], dq3[2]);
	}
	fclose(fp);
	strpath = path + "\\陀螺真实与仿真数据.txt";
	fp = fopen(strpath.c_str(), "w");
	for (int i = 0; i < m; i++)
	{
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			wTure(i, 0), wTure(i, 1), wTure(i, 2), wMeas(i, 0), wMeas(i, 1), wMeas(i, 2));
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：用陀螺测量数据递推，验证初始四元数不准对后续递推的影响
//输入：	
//输出：
//注意：
//作者：GZC
//日期：2017.06.25
//////////////////////////////////////////////////////////////////////////
void AttDetermination::simGyroAndAccBack(double dt, int m, string Res, double wBiasA[3],
	MatrixXd qTure, MatrixXd wTure, MatrixXd qMeas, MatrixXd wMeas)
{
	MatrixXd Qest(m, 4);
	Qest(m - 1, 0) = qMeas(m - 1, 0), Qest(m - 1, 1) = qMeas(m - 1, 1),
		Qest(m - 1, 2) = qMeas(m - 1, 2), Qest(m - 1, 3) = qMeas(m - 1, 3);
	/*Qest(m - 1, 0) = qTure(m - 1, 0), Qest(m - 1, 1) = qTure(m - 1, 1),
		Qest(m - 1, 2) = qTure(m - 1, 2), Qest(m - 1, 3) = qTure(m - 1, 3);*/
	for (int i = m - 1; i > 0; i--)
	{
		wMeas(i, 0) = -wMeas(i - 1, 0) + wBiasA[0] / 3600 / 180 * PI*dt;
		wMeas(i, 1) = -wMeas(i - 1, 1) + wBiasA[1] / 3600 / 180 * PI*dt;
		wMeas(i, 2) = -wMeas(i - 1, 2) + wBiasA[2] / 3600 / 180 * PI*dt;
		double w = sqrt(wMeas(i, 0)*wMeas(i, 0) + wMeas(i, 1)*wMeas(i, 1) + wMeas(i, 2)*wMeas(i, 2));
		//Propagate State
		double qw1, qw2, qw3, qw4;
		qw1 = wMeas(i, 0) / w * sin(0.5*w*dt);
		qw2 = wMeas(i, 1) / w * sin(0.5*w*dt);
		qw3 = wMeas(i, 2) / w * sin(0.5*w*dt);
		qw4 = cos(0.5*w*dt);
		MatrixXd om(4, 4);
		om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
		//cout << om << endl;
		//cout << om.inverse() << endl;
		Qest.row(i - 1) = (om*Qest.row(i).transpose()).transpose();
	}

	string path = string(Res);
	string strpath = path + "\\初始四元数直接陀螺积分.txt";
	FILE *fp = fopen(strpath.c_str(), "w");
	double dq1[4], dq2[4], dq3[4];
	for (int i = 0; i < m; i++)
	{
		dq1[0] = qTure(i, 0); dq1[1] = qTure(i, 1); dq1[2] = qTure(i, 2); dq1[3] = qTure(i, 3);
		dq2[0] = -Qest(i, 0); dq2[1] = -Qest(i, 1); dq2[2] = -Qest(i, 2); dq2[3] = Qest(i, 3);
		mbase.quatMult(dq1, dq2, dq3);
		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
			qTure(i, 3), qTure(i, 0), qTure(i, 1), qTure(i, 2), Qest(i, 3), Qest(i, 0), Qest(i, 1), Qest(i, 2), dq3[0], dq3[1], dq3[2]);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：乘以安装，使APS星敏从Crj转为Cbj
//输入：attAPS，Quat结构体，原始四元数
//输出：attAPS，Quat结构体，乘以安装后四元数
//注意：仅为APS星敏
//作者：GZC
//日期：2017.04.27
//////////////////////////////////////////////////////////////////////////
void AttDetermination::alinAPS(vector<Quat> &attAPS)
{
	int i, m;
	m = attAPS.size();
	double Aalin[9];
	memcpy(Aalin, Ainstall, sizeof(Ainstall));
	double k, Crj[9], Cbj[9], q[4];
	vector<Quat> attAPScopy(attAPS);
	attAPS.clear();
	Quat attAPStmp;

	mbase.invers_matrix(Aalin, 3);//转换为Cbr	
	for (i = 0; i < m; i++)
	{
		mbase.quat2matrix(attAPScopy[i].Q1, attAPScopy[i].Q2, attAPScopy[i].Q3, attAPScopy[i].Q0, Crj);
		mbase.Multi(Aalin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, attAPStmp.Q1, attAPStmp.Q2, attAPStmp.Q3, attAPStmp.Q0);
		attAPStmp.UTC = attAPScopy[i].UTC;
		attAPS.push_back(attAPStmp);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：乘以安装，使BC星敏从Crj转为Cbj
//输入：attAPS，Quat结构体，原始四元数
//输出：attAPS，Quat结构体，乘以安装后四元数
//注意：仅为APS星敏
//作者：GZC
//日期：2017.07.11
//////////////////////////////////////////////////////////////////////////
void AttDetermination::alinBC(const vector<STGData> AttData, Quat* starB, Quat* starC)
{
	int i, m;
	m = AttData.size();
	double Balin[9], Calin[9];
	memcpy(Balin, Binstall, sizeof(Binstall));
	memcpy(Calin, Cinstall, sizeof(Cinstall));
	vector <STGData>ATT(AttData);
	double k, Crj[9], Cbj[9], q[4];

	mbase.invers_matrix(Balin, 3);//转换为Cbr	
	mbase.invers_matrix(Calin, 3);//转换为Cbr	
	for (i = 0; i < m; i++)
	{
		mbase.quat2matrix(ATT[i].StarB.Q1, ATT[i].StarB.Q2, ATT[i].StarB.Q3, ATT[i].StarB.Q0, Crj);
		mbase.Multi(Balin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starB[i].Q1, starB[i].Q2, starB[i].Q3, starB[i].Q0);
		starB[i].UTC = ATT[i].StarB.UTC;
		mbase.quat2matrix(ATT[i].StarC.Q1, ATT[i].StarC.Q2, ATT[i].StarC.Q3, ATT[i].StarC.Q0, Crj);
		mbase.Multi(Calin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, starC[i].Q1, starC[i].Q2, starC[i].Q3, starC[i].Q0);
		starC[i].UTC = ATT[i].StarC.UTC;
	}
}

//将BC安装矩阵正交化
void AttDetermination::getInstall(double* Binstall, double* Cinstall)
{
	double eulerB[3], eulerC[3];
	mbase.Matrix2Eulor(const_cast<double*>(this->Binstall), 213, eulerB[0], eulerB[1], eulerB[2]);
	mbase.Matrix2Eulor(const_cast<double*>(this->Cinstall), 213, eulerC[0], eulerC[1], eulerC[2]);
	mbase.Eulor2Matrix(eulerB[0], eulerB[1], eulerB[2], 213, Binstall);
	mbase.Eulor2Matrix(eulerC[0], eulerC[1], eulerC[2], 213, Cinstall);
}

//得到安装误差
void AttDetermination::getInstallErr(int m, Quat *qB, Quat *qC, double *starCmeas)
{
	double starB[9], starC[9];
	getInstall(starB, starC);
	double Err1[9];
	//星敏C相对于B的安装Ccb
	mSim.twoStarErr(m, qB, qC, Err1);
	//以B为基础，纠正C相机安装
	mbase.Multi(Err1, starB, starCmeas, 3, 3, 3);
	//mbase.invers_matrix(starCmeas, 3);
	//mbase.Multi(starCmeas, starC, Err, 3, 3, 3);
}

//////////////////////////////////////////////////////////////////////////
//功能：根据角速度预测四元数
//输入：wMeas：减去漂移的陀螺测量值，Qk：四元数初值，dt：间隔时间
//输出：Qk1：四元数预测值
//注意：给定上一刻Qest的指针
//作者：GZC
//日期：2017.08.10
//////////////////////////////////////////////////////////////////////////
void AttDetermination::predictQuat(MatrixXd wMeas, MatrixXd Qk, MatrixXd &Qk1, double dt)
{
	double we[3];
	we[0] = wMeas(0);//
	we[1] = wMeas(1);
	we[2] = wMeas(2);
	double w = sqrt(we[0] * we[0] + we[1] * we[1] + we[2] * we[2]);
	double qw1, qw2, qw3, qw4;
	qw1 = we[0] / w * sin(0.5*w*dt);
	qw2 = we[1] / w * sin(0.5*w*dt);
	qw3 = we[2] / w * sin(0.5*w*dt);
	qw4 = cos(0.5*w*dt);
	MatrixXd om(4, 4);
	om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
	Qk1 = (om*Qk.transpose()).transpose();
}

//////////////////////////////////////////////////////////////////////////
//功能：求反斜对称矩阵
//输入：
//输出：
//注意：
//作者：GZC
//日期：2017.08.10
//////////////////////////////////////////////////////////////////////////
void AttDetermination::crossMatrix(MatrixXd we, MatrixXd &cMat)
{
	cMat << 0, -we(2), we(1),
		we(2), 0, -we(0),
		-we(1), we(0), 0;
}
//////////////////////////////////////////////////////////////////////////
//功能：单星敏定姿
//输入：AttData:STG解析出的姿态数据，StarTag:星敏标识
//输出：AttDet:姿态确定结果
//注意：可以用来检测数据的有效性
//作者：GZC
//日期：2015.03.01
//////////////////////////////////////////////////////////////////////////
bool AttDetermination::SingleStar(vector<STGData> AttData, int StarTag, vector<Quat> &AttDet)
{
	int i, m;
	m = AttData.size();
	double Aalin[9], Balin[9], Calin[9];
	memcpy(Aalin, Ainstall, sizeof(Ainstall));
	memcpy(Balin, Binstall, sizeof(Binstall));
	memcpy(Calin, Cinstall, sizeof(Cinstall));
	double k, Crj[9], Cbj[9], q[4];
	Quat EKFres;
	if (StarTag == 1)//星敏1或A
	{
		mbase.invers_matrix(Aalin, 3);//转换为Cbr	
		for (i = 0; i < m; i++)
		{
			mbase.quat2matrix(AttData[i].StarA.Q1, AttData[i].StarA.Q2, AttData[i].StarA.Q3, AttData[i].StarA.Q0, Crj);
			mbase.Multi(Aalin, Crj, Cbj, 3, 3, 3);
			mbase.matrix2quat(Cbj, q[1], q[2], q[3], q[0]);
			k = pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2) + pow(q[0], 2) - 1;
			if (k > 0.01) { return 0; }
			EKFres.UTC = AttData[i].StarA.UTC, EKFres.Q0 = q[0], EKFres.Q1 = q[1], EKFres.Q2 = q[2], EKFres.Q3 = q[3];
			AttDet.push_back(EKFres);
		}
	}
	else if (StarTag == 2)//星敏2或B
	{
		mbase.invers_matrix(Balin, 3);//转换为Cbr
		for (i = 0; i < m; i++)
		{
			mbase.quat2matrix(AttData[i].StarB.Q1, AttData[i].StarB.Q2, AttData[i].StarB.Q3, AttData[i].StarB.Q0, Crj);
			mbase.Multi(Balin, Crj, Cbj, 3, 3, 3);
			mbase.matrix2quat(Cbj, q[1], q[2], q[3], q[0]);
			k = pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2) + pow(q[0], 2) - 1;
			if (k > 0.01) { return 0; }
		}
	}
	else if (StarTag == 3)//星敏3或B
	{
		mbase.invers_matrix(Calin, 3);//转换为Cbr
		for (i = 0; i < m; i++)
		{
			mbase.quat2matrix(AttData[i].StarC.Q1, AttData[i].StarC.Q2, AttData[i].StarC.Q3, AttData[i].StarC.Q0, Crj);
			mbase.Multi(Calin, Crj, Cbj, 3, 3, 3);
			mbase.matrix2quat(Cbj, q[1], q[2], q[3], q[0]);
			k = pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2) + pow(q[0], 2) - 1;
			if (k > 0.01) { return 0; }
		}
	}
	return 1;
}

//////////////////////////////////////////////////////////////////////////
//功能：双星敏定姿
//输入：AttData:STG解析出的姿态数据	Res:结果输出路径		StarTag:星敏标识
//输出：AttDet:姿态确定结果
//注意：
//作者：GZC
//日期：2015.03.16
//////////////////////////////////////////////////////////////////////////
void AttDetermination::DoubleStar(vector<STGData> StarDat, vector<Quat> &AttDet, int StarTag)
{
	int i, m;
	m = StarDat.size();
	double Balin[9], Calin[9];//双星敏各自的安装矩阵
	Quat *StarX = new Quat[m];
	Quat *StarY = new Quat[m];
	Quat *StarXi = new Quat[m];
	Quat *StarYi = new Quat[m];
	double *UTC = new double[m];

	double mz[3] = { 0,0,1 }, zc[3], zb[3], x[3], y[3], z[3], RC[9], RB[9], Cinstallz[3], Binstallz[3];
	Quat Q2Vec;
	if (StarTag == 23)
	{
		//根据选择的星敏确定安装矩阵
		memcpy(Balin, Binstall, sizeof(Binstall));
		memcpy(Calin, Cinstall, sizeof(Cinstall));
		mbase.invers_matrix(Balin, 3);
		mbase.invers_matrix(Calin, 3);
		for (int i = 0; i < m; i++)
		{
			//内插星敏数据,此处为星敏2和星敏3
			UTC[i] = StarDat.at(i).utgyro;
			StarX[i].UTC = StarDat.at(i).StarB.UTC;
			StarX[i].Q1 = StarDat.at(i).StarB.Q1; StarX[i].Q2 = StarDat.at(i).StarB.Q2; StarX[i].Q3 = StarDat.at(i).StarB.Q3, StarX[i].Q0 = StarDat.at(i).StarB.Q0;
			StarY[i].UTC = StarDat.at(i).StarC.UTC;
			StarY[i].Q1 = StarDat.at(i).StarC.Q1; StarY[i].Q2 = StarDat.at(i).StarC.Q2; StarY[i].Q3 = StarDat.at(i).StarC.Q3, StarY[i].Q0 = StarDat.at(i).StarC.Q0;
		}
		mbase.QuatInterpolation(StarX, m, UTC, m, StarXi);
		mbase.QuatInterpolation(StarY, m, UTC, m, StarYi);

		mbase.Multi(Balin, mz, Binstallz, 3, 3, 1);//Cbr,星敏光轴在卫星本体系中的坐标值
		mbase.Multi(Calin, mz, Cinstallz, 3, 3, 1);
		mbase.normalvect(Binstallz, x);
		mbase.crossmultnorm(Binstallz, Cinstallz, y);//此时产生的X和Y均是本体系下的值
		mbase.crossmultnorm(x, y, z);//Crb,以本体系下的星敏A和B的Z轴作为新坐标系的X和Y轴，此坐标系组成的旋转矩阵也在本体系下
		double Cbr[9], Crj[9], Cbj[9];
		Cbr[0] = x[0], Cbr[1] = x[1], Cbr[2] = x[2];
		Cbr[3] = y[0], Cbr[4] = y[1], Cbr[5] = y[2];
		Cbr[6] = z[0], Cbr[7] = z[1], Cbr[8] = z[2];
		mbase.invers_matrix(Cbr, 3);
		for (int i = 0; i < m; i++)
		{
			mbase.quat2matrix(StarXi[i].Q1, StarXi[i].Q2, StarXi[i].Q3, StarXi[i].Q0, RB);//Crj
			mbase.quat2matrix(StarYi[i].Q1, StarYi[i].Q2, StarYi[i].Q3, StarYi[i].Q0, RC);
			mbase.invers_matrix(RC, 3);
			mbase.invers_matrix(RB, 3);//Cjr
			mbase.Multi(RC, mz, zc, 3, 3, 1);
			mbase.Multi(RB, mz, zb, 3, 3, 1);//星敏光轴在惯性系中的坐标值
			mbase.normalvect(zb, x);
			mbase.crossmultnorm(zb, zc, y);
			mbase.crossmultnorm(x, y, z);//Crj
			Crj[0] = x[0], Crj[1] = x[1], Crj[2] = x[2];
			Crj[3] = y[0], Crj[4] = y[1], Crj[5] = y[2];
			Crj[6] = z[0], Crj[7] = z[1], Crj[8] = z[2];
			mbase.Multi(Cbr, Crj, Cbj, 3, 3, 3);
			mbase.matrix2quat(Cbj, Q2Vec.Q1, Q2Vec.Q2, Q2Vec.Q3, Q2Vec.Q0);
			Q2Vec.UTC = UTC[i];
			AttDet.push_back(Q2Vec);
		}
	}
	else if (StarTag == 12)
	{
	}
	else if (StarTag == 13)
	{
	}
	delete StarX, StarXi, StarY, StarYi, UTC;
	StarX = StarXi = StarY = StarYi = NULL; UTC = NULL;
}

//////////////////////////////////////////////////////////////////////////
//功能：单星敏陀螺定姿(无滤波)
//输入：AttData:STG解析出的姿态数据	Res:结果输出路径		StarTag:星敏标识
//输出：AttDet:姿态确定结果
//注意：主要是为了验证陀螺和星敏数据安装矩阵的一致性
//作者：GZC
//日期：2015.07.10
//////////////////////////////////////////////////////////////////////////
void AttDetermination::GyroAtt(vector<STGData> AttData, vector<Quat> &AttDet, string Res, int StarTag)
{
	int i, num = 0;
	int m = AttData.size();
	MatrixXd Wgm(m, 3), W_Att(m, 4);
	double Aalin[9], Balin[9], Calin[9], q[4];
	memcpy(Aalin, Ainstall, sizeof(Ainstall));
	memcpy(Balin, Binstall, sizeof(Binstall));
	memcpy(Calin, Cinstall, sizeof(Cinstall));
	mbase.invers_matrix(Balin, 3);//转换为Cbr
	double GyDat[3], GyTran[3], Cbj[9], Crj[9];
	double gyIns[9];
	memcpy(gyIns, GyroIns, sizeof(GyroIns));
	mbase.invers_matrix(gyIns, 3);
	double *UTC = new double[m];
	Quat *StarX = new Quat[m];
	Quat *StarXi = new Quat[m];

	for (i = 0; i < m; i++)
	{
		mbase.quat2matrix(AttData[i].StarB.Q1, AttData[i].StarB.Q2, AttData[i].StarB.Q3, AttData[i].StarB.Q0, Crj);
		mbase.Multi(Balin, Crj, Cbj, 3, 3, 3);
		mbase.matrix2quat(Cbj, q[1], q[2], q[3], q[0]);
		StarX[i].UTC = AttData[i].StarB.UTC, StarX[i].Q0 = q[0], StarX[i].Q1 = q[1], StarX[i].Q2 = q[2], StarX[i].Q3 = q[3];
		GyDat[0] = AttData.at(i).g1, GyDat[1] = AttData.at(i).g3, GyDat[2] = AttData.at(i).g5;
		mbase.Multi(gyIns, GyDat, GyTran, 3, 3, 1);
		Wgm(i, 0) = GyTran[0] / 180 * PI, Wgm(i, 1) = GyTran[1] / 180 * PI, Wgm(i, 2) = GyTran[2] / 180 * PI;
		UTC[i] = AttData.at(i).utgyro;
	}
	mbase.QuatInterpolation(StarX, m, UTC, m, StarXi);

	string path = string(Res);
	path = path.substr(0, path.rfind('.'));
	string strpath = path + "_GyroAtt.txt";
	FILE *fpres = fopen(strpath.c_str(), "w");

	MatrixXd SAm1(4, 1);
	SAm1 << StarXi[0].Q1, StarXi[0].Q2, StarXi[0].Q3, StarXi[0].Q0;
	double dt = 0.25;
	for (i = 0; i < m; i++)
	{
		MatrixXd F(4, 4);
		F << 0, Wgm(i, 2), -Wgm(i, 1), Wgm(i, 0),
			-Wgm(i, 2), 0, Wgm(i, 0), Wgm(i, 1),
			Wgm(i, 1), -Wgm(i, 0), 0, Wgm(i, 2),
			-Wgm(i, 0), -Wgm(i, 1), -Wgm(i, 2), 0;
		double w0 = sqrt(Wgm(i, 0)*Wgm(i, 0) + Wgm(i, 1)*Wgm(i, 1) + Wgm(i, 2)*Wgm(i, 2));
		MatrixXd eye44(4, 4);
		eye44 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		if (num == 0)
		{
			W_Att.row(i).transpose() = (eye44*cos(w0*dt / 2) + F * sin(w0*dt / 2) / w0)*SAm1;
			//cout<<"Gyro Deter:"<<W_Att.row(i)<<endl;
			fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", UTC[i], W_Att(i, 3), W_Att(i, 0), W_Att(i, 1), W_Att(i, 2));
			num++;
		}
		else
		{
			W_Att.row(i).transpose() = (eye44*cos(w0*dt / 2) + F * sin(w0*dt / 2) / w0)*W_Att.row(i - 1).transpose();
			fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", UTC[i], W_Att(i, 3), W_Att(i, 0), W_Att(i, 1), W_Att(i, 2));
			//cout<<"Gyro Deter:"<<W_Att.row(i)<<endl;
		}

	}

}

//////////////////////////////////////////////////////////////////////////
//功能：对比地面和星上滤波结果
//输入：AttData：STG解析出的姿态数据	
//输出：EKFatt：姿态确定结果
//注意：
//作者：GZC
//日期：2017.07.11
//////////////////////////////////////////////////////////////////////////
void AttDetermination::compareEKFAndAOCC(Quat *StarDat, int n, string res)
{
	int i, m;
	string path = workpath;
	path = path.substr(0, path.rfind('.')) + ".ATT";
	FILE *fp = fopen(path.c_str(), "r");
	fscanf(fp, "%*f\t%*f\n%*s\n%d\n", &m);//星上滤波的姿态数量
	Quat *AOCCatt = new Quat[m];
	double *UTC = new double[m];
	for (i = 0; i < m; i++)
	{
		fscanf(fp, "%*s\t%lf\t%lf\t%lf\t%lf\t%lf\n", &AOCCatt[i].UTC, &AOCCatt[i].Q0, &AOCCatt[i].Q1, &AOCCatt[i].Q2, &AOCCatt[i].Q3);
		UTC[i] = AOCCatt[i].UTC;
	}
	fclose(fp);
	Quat *StarDatInt = new Quat[m];
	mbase.QuatInterpolation(StarDat, n, UTC, m, StarDatInt);

	path = path.substr(0, path.rfind('\\') + 1) + res;
	FILE *fpres = fopen(path.c_str(), "w");

	double dq1[4], dq2[4], dq3[4];
	for (i = 0; i < m; i++)
	{
		dq1[0] = -StarDatInt[i].Q0, dq1[1] = StarDatInt[i].Q1, dq1[2] = StarDatInt[i].Q2, dq1[3] = StarDatInt[i].Q3;
		dq2[0] = AOCCatt[i].Q0, dq2[1] = AOCCatt[i].Q1, dq2[2] = AOCCatt[i].Q2, dq2[3] = AOCCatt[i].Q3;
		mbase.quatMult2(dq1, dq2, dq3);
		dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600, dq3[3] = dq3[3] * 2 / PI * 180 * 3600;
		fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\n", UTC[i], dq3[1], dq3[2], dq3[3]);
		//注意dq1里的负号
		/*dq1[3] = -StarDatInt[i].Q0, dq1[0] = StarDatInt[i].Q1, dq1[1] = StarDatInt[i].Q2, dq1[2] = StarDatInt[i].Q3;
		dq2[3] = AOCCatt[i].Q0, dq2[0] = AOCCatt[i].Q1, dq2[1] = AOCCatt[i].Q2, dq2[2] = AOCCatt[i].Q3;
		mbase.quatMult(dq1, dq2, dq3);
		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
		fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\n",UTC[i],dq3[0],dq3[1],dq3[2]);	*/
	}
	fclose(fpres);
	delete[] UTC; UTC = NULL;
	delete[] StarDatInt; StarDatInt = NULL;
}
void AttDetermination::compareEKFAndAOCC(vector<Quat>StarDat, string res)
{
	int i, m;
	int n = StarDat.size();
	string path = workpath;
	path = path.substr(0, path.rfind('.')) + ".ATT";
	FILE *fp = fopen(path.c_str(), "r");
	fscanf(fp, "%*f\t%*f\n%*s\n%d\n", &m);//星上滤波的姿态数量
	Quat *AOCCatt = new Quat[m];
	double *UTC = new double[m];
	for (i = 0; i < m; i++)
	{
		fscanf(fp, "%*s\t%lf\t%lf\t%lf\t%lf\t%lf\n", &AOCCatt[i].UTC, &AOCCatt[i].Q0, &AOCCatt[i].Q1, &AOCCatt[i].Q2, &AOCCatt[i].Q3);
		UTC[i] = AOCCatt[i].UTC;
	}
	fclose(fp);
	Quat *StarDatInt = new Quat[m];
	mbase.QuatInterpolation(StarDat, UTC, m, StarDatInt);

	path = workpath;
	path = path.substr(0, path.rfind('\\') + 1) + res;
	FILE *fpres = fopen(path.c_str(), "w");

	double dq1[4], dq2[4], dq3[4];
	for (i = 0; i < m; i++)
	{
		//注意dq1里的负号
		dq1[0] = -StarDatInt[i].Q0, dq1[1] = StarDatInt[i].Q1, dq1[2] = StarDatInt[i].Q2, dq1[3] = StarDatInt[i].Q3;
		dq2[0] = AOCCatt[i].Q0, dq2[1] = AOCCatt[i].Q1, dq2[2] = AOCCatt[i].Q2, dq2[3] = AOCCatt[i].Q3;
		mbase.quatMult2(dq1, dq2, dq3);
		dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600, dq3[3] = dq3[3] * 2 / PI * 180 * 3600;
		fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\n", UTC[i], dq3[1], dq3[2], dq3[3]);
		//注意dq1里的负号
		/*dq1[3] = -StarDatInt[i].Q0, dq1[0] = StarDatInt[i].Q1, dq1[1] = StarDatInt[i].Q2, dq1[2] = StarDatInt[i].Q3;
		dq2[3] = AOCCatt[i].Q0, dq2[0] = AOCCatt[i].Q1, dq2[1] = AOCCatt[i].Q2, dq2[2] = AOCCatt[i].Q3;
		mbase.quatMult(dq1, dq2, dq3);
		dq3[0] = dq3[0] * 2 / PI * 180 * 3600, dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600;
		fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\n", UTC[i], dq3[0], dq3[1], dq3[2]);*/
	}
	fclose(fpres);
	delete[] UTC; UTC = NULL;
	delete[] StarDatInt; StarDatInt = NULL;
}

void AttDetermination::compareAPSandStarMap()
{
	string strStg = workpath + "EKF滤波结果.txt";
	string strSMap = workpath + "StarMap滤波结果.txt";
	string strRes = workpath + "APS和StarMap结果对比.txt";
	FILE *fp1 = fopen(strStg.c_str(), "r");
	FILE *fp2 = fopen(strSMap.c_str(), "r");
	FILE *fp3 = fopen(strRes.c_str(), "w");
	int m, n;
	fscanf(fp1, "%*s\n%*s\n%d\n", &m);
	vector<Quat> stgVec(m);
	fscanf(fp2, "%*s\n%*s\n%d\n", &n);
	vector<Quat> mapVec(n);
	for (int a = 0; a < m; a++)
	{
		fscanf(fp1, "%*s\t%lf\t%lf\t%lf\t%lf\t%lf\n", &stgVec[a].UTC, &stgVec[a].Q0,
			&stgVec[a].Q1, &stgVec[a].Q2, &stgVec[a].Q3);
	}
	for (int a = 0; a < n; a++)
	{
		fscanf(fp2, "%*s\t%lf\t%lf\t%lf\t%lf\t%lf\n", &mapVec[a].UTC, &mapVec[a].Q0,
			&mapVec[a].Q1, &mapVec[a].Q2, &mapVec[a].Q3);
	}
	int ii = 0;
	if (stgVec[0].UTC < mapVec[0].UTC)
	{
		while (stgVec[ii].UTC < mapVec[0].UTC)
		{
			ii++;
		}
		stgVec.erase(stgVec.begin(), stgVec.begin() + ii);
	}
	else
	{
		while (stgVec[0].UTC > mapVec[ii].UTC)
		{
			ii++;
		}
		mapVec.erase(mapVec.begin(), mapVec.begin() + ii);
	}
	int num = stgVec.size() > mapVec.size() ? mapVec.size() : stgVec.size();
	double dq1[4], dq2[4], dq3[4];
	for (int i = 0; i < num; i++)
	{
		//注意dq1里的负号
		dq1[0] = -stgVec[i].Q0, dq1[1] = stgVec[i].Q1, dq1[2] = stgVec[i].Q2, dq1[3] = stgVec[i].Q3;
		dq2[0] = mapVec[i].Q0, dq2[1] = mapVec[i].Q1, dq2[2] = mapVec[i].Q2, dq2[3] = mapVec[i].Q3;
		mbase.quatMult2(dq1, dq2, dq3);
		dq3[1] = dq3[1] * 2 / PI * 180 * 3600, dq3[2] = dq3[2] * 2 / PI * 180 * 3600, dq3[3] = dq3[3] * 2 / PI * 180 * 3600;
		fprintf(fp3, "%.9f\t%.9f\t%.9f\t%.9f\n", stgVec[i].UTC, dq3[1], dq3[2], dq3[3]);
	}
	fcloseall();
}