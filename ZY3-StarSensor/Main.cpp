#include "StarIdentify.h"
#include "StarExtract.h"
#include "ParseSTG.h"
#include "APScalibration.h"
#include "AttDetermination.h"
#include "AttSim.h"
//程序结束调用这段音乐
#include <windows.h>
#pragma comment(lib, "winmm.lib")


int main(int argc, char* argv[])
{
	if (atoi(argv[2]) == 1)
	{
		//////////////////////////////////////////////////////////////////////////
		//功能：不定标，直接用原始姿态定位
		//日期：2017.04.27
		//////////////////////////////////////////////////////////////////////////	
		ParseSTG ZY3_STGSTI;
		AttDetermination ZY3_AD;
		string argv1 = argv[1];
		vector<STGData> ZY3_02STGdata;
		ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata);
		ZY3_AD.workpath = argv1;
		ZY3_AD.AttDeter3(ZY3_02STGdata);
	}
	else if (atoi(argv[2]) == 2)
	{
		//////////////////////////////////////////////////////////////////////////
		//功能：姿态仿真与滤波
		//日期：2017.06.08
		//////////////////////////////////////////////////////////////////////////	
		string simPath = argv[1];
		AttSim attSim1;	AttDetermination attDeter1;
		double dt = 0.25;
		int tf = 1000;
		int m = tf / dt;
		double qInitial[] = { 0.5,0.5,0.5,0.5 };
		double sig_ST = 15; //星敏误差(单位：角秒)
		double wBiasA[] = { 0.5,0.1,-0.1 };
		//double wBiasA[] = { 0,0,0 };
		double sigu = sqrt(10)*1e-10;
		double sigv = sqrt(10)*1e-7;
		MatrixXd qTure(m, 4), wTure(m, 3), qMeas(m, 4), wMeas(m, 3);
		attSim1.getQinit(qInitial);
		attSim1.attitudeVerify(dt, m, simPath);
		//attSim1.attitudeSimulation(dt, tf, m, qInitial, sig_ST, wBiasA, sigu, sigv, qTure, wTure, qMeas, wMeas);
		//attDeter1.simAttandEKF(dt, m, sig_ST, simPath, qTure, wTure, qMeas, wMeas);
		//attDeter1.simEKFForwardAndBackward(dt, m, sig_ST, simPath, qTure, wTure, qMeas, wMeas);
		//attDeter1.simGyroAndAccBack(dt, m, simPath, wBiasA, qTure, wTure, qMeas, wMeas);
	}
	else if (atoi(argv[2]) == 21)
	{
		//////////////////////////////////////////////////////////////////////////
		//功能：姿态仿真
		//日期：2017.07.28
		//////////////////////////////////////////////////////////////////////////	
		AttSim attSim1;	AttDetermination attDeter1;
		attSim1.workpath = argv[1];
		double dt = 0.25;
		int tf = 10;
		int m = tf / dt;
		//double qInitial[] = { 0.5,0.5,0.5,0.5 };
		double qInitial[] = { 1./sqrt(3), 0, 1. / sqrt(3), 1. / sqrt(3)};
		//double qInitial[] = { 1. / sqrt(9), 1. / sqrt(9./4), 1. / sqrt(9. / 2), 1. / sqrt(9./2) };
		double alinAB[] = { 50./180*PI,75. / 180 * PI,60. / 180 * PI };
		double alinN = 0.1 / 180 * PI;
		double FOV =  5. / 180 * PI;
		while(true)
		{
			//attSim1.alinSimulation(dt, m, alinN);
			attSim1.wahbaSimulation(dt, m, FOV,alinN);
		}
	}
	else if (atoi(argv[2]) == 22)
	{
		//////////////////////////////////////////////////////////////////////////
		//功能：姿态仿真
		//日期：2017.07.28
		//////////////////////////////////////////////////////////////////////////	
		AttSim attSim1;	AttDetermination attDeter1;
		attSim1.workpath = argv[1];
		attDeter1.workpath = argv[1];
		double dt = 0.25;
		int tf = 1000;
		int m = tf / dt;	
		double alinN =  0.003 / 180 * PI;
		double wBiasA[] = { 0.5,0.1,-0.1 };
		double alin[] = { 0.03/180*PI,0.02 / 180 * PI,-0.04 / 180 * PI };
		double sigu =  sqrt(10)*1e-10;
		double sigv =  sqrt(10)*1e-7;
		Quat * starB = new Quat[m];
		Quat *starC = new Quat[m];
		Quat *starTrue = new Quat[m];
		Gyro *wMeas = new Gyro[m];
		attSim1.twoStarSimulation(dt, m,alinN, alin,wBiasA,sigu,sigv,starB,starC, starTrue,wMeas);
		attDeter1.EKF6StateForStarAB3(m,starB, starC, starTrue, wMeas);
		delete[] starB, starC, wMeas; starB = starC = NULL;
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星敏多景混合定标流程
	//日期：2017.02.21
	//////////////////////////////////////////////////////////////////////////
	//ParseSTG ZY3_STGSTI;
	//StarExtract ZY3_STMap;
	//StarIdentify ZY3_ST;
	//APScalibration ZY3_calibrate;
	//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\";	
	//vector<STGData> ZY3_02STGdata0702;
	//vector<STGData> ZY3_02STGdata0707;
	//vector<STGData> ZY3_02STGdata0712;
	//vector<STGData> ZY3_02STGdata0830;
	//vector<vector<StarGCP>>getGCPall;
	//vector<StarGCP > getGCPalltmp;
	//string argv1;
	//argv1 = workpath_ST + "0702\\0702.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0702);
	//argv1 = workpath_ST + "0707\\0707.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0707);
	//argv1 = workpath_ST + "0712\\0712.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0712);
	//argv1 = workpath_ST + "0830\\0830.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0830);
	//for (int index = 1100; index >=5; )
	//{
	//	printf("\r开始定标，累加%d景...", index);

	//	ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\星图\\";
	//	ZY3_ST.workpath = ZY3_STMap.workpath;
	//	ZY3_calibrate.workpath = ZY3_STMap.workpath;
	//	ZY3_STMap.StarPointExtraction(index);
	//	ZY3_ST.GetStarGCP0702(ZY3_02STGdata0702, ZY3_STMap.StarPointExtract, index);
	//	ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 50, index);//精化控制点
	//	getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
	//	getGCPall.push_back(getGCPalltmp);//累加控制点
	//	ZY3_STMap.StarPointExtract.clear();//记得清楚vector里的内容，不然内容会叠加
	//	ZY3_ST.getGCP.clear();
	//	//ZY3_calibrate.Calibrate3ParamMultiImg(getGCPall, index);
	//	ZY3_calibrate.Calibrate5ParamMultiImg(getGCPall, index);
		//ZY3_calibrate.Calibrate6ParamMultiImg(getGCPall, index);
		/*	if (index % 10 == 0)
			ZY3_calibrate.OutputErr(getGCPall, ZY3_02STGdata0702, index);*/

		//ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
		//ZY3_ST.workpath = ZY3_STMap.workpath;
		//ZY3_calibrate.workpath = ZY3_STMap.workpath;
		//ZY3_STMap.StarPointExtraction(index);
		//ZY3_ST.GetStarGCP0707(ZY3_02STGdata0707, ZY3_STMap.StarPointExtract, index);
		//ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 50, index);//精化控制点
		//getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
		//getGCPall.push_back(getGCPalltmp);//累加控制点
		//ZY3_STMap.StarPointExtract.clear();//记得清楚vector里的内容，不然内容会叠加
		//ZY3_ST.getGCP.clear();
		////ZY3_calibrate.Calibrate3ParamMultiImg(getGCPall, index);
		//ZY3_calibrate.Calibrate5ParamMultiImg(getGCPall, index);

		//ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\星图\\";
		//ZY3_ST.workpath = ZY3_STMap.workpath; 
		//ZY3_calibrate.workpath = ZY3_STMap.workpath;
		//ZY3_STMap.StarPointExtraction(index);
		//ZY3_ST.GetStarGCP0712(ZY3_02STGdata0712, ZY3_STMap.StarPointExtract, index);
		//ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 50, index);//精化控制点
		//getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
		//getGCPall.push_back(getGCPalltmp);//累加控制点
		//ZY3_STMap.StarPointExtract.clear();//记得清楚vector里的内容，不然内容会叠加
		//ZY3_ST.getGCP.clear();
		////ZY3_calibrate.Calibrate3ParamMultiImg(getGCPall, index);
		//ZY3_calibrate.Calibrate5ParamMultiImg(getGCPall, index);

		//ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\星图\\";
		//ZY3_ST.workpath = ZY3_STMap.workpath;
		//ZY3_calibrate.workpath = ZY3_STMap.workpath;
		//ZY3_STMap.StarPointExtraction(index);
		//ZY3_ST.GetStarGCP(ZY3_02STGdata0830, ZY3_STMap.StarPointExtract, index);
		//ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 50, index);//精化控制点
		//getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
		//getGCPall.push_back(getGCPalltmp);//累加控制点
		//ZY3_STMap.StarPointExtract.clear();//记得清楚vector里的内容，不然内容会叠加
		//ZY3_ST.getGCP.clear();
		////ZY3_calibrate.Calibrate3ParamMultiImg(getGCPall, index);
		//ZY3_calibrate.Calibrate5ParamMultiImg(getGCPall, index);

	//	index = index -5;//输出间隔
	//}
	//ZY3_calibrate.Calibrate5ParamMultiImg(getGCPall, 5);
	//ZY3_calibrate.OutputAllGCP(getGCPall);

	//////////////////////////////////////////////////////////////////////////
	//功能：以下为姿态确定过程
	//日期：2017.03.16
	//////////////////////////////////////////////////////////////////////////
	//ParseSTG ZY3_STGSTI;
	//APScalibration ZY3_calibrate;
	//AttDetermination ZY3_AD;
	//StarCaliParam Paramer;
	//Paramer.f = 2884.8;
	//Paramer.x0 = 526.146;
	//Paramer.y0 = 534.714;
	//Paramer.k1 = -0.0000000106;
	//Paramer.k2 = 0.00000000000000854;
	//vector<STGData> ZY3_02STGdata0702;
	//vector<vector<StarGCP>>getGCPall;
	//vector<Quat>Quater;
	//string argv1;
	//argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\0702.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0702);
	//ZY3_calibrate.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\";
	//ZY3_calibrate.ReadAllGCP(getGCPall);
	//ZY3_AD.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\姿态确定\\";
	//ZY3_AD.AttDeter(ZY3_02STGdata0702, getGCPall, Paramer, Quater);


	//////////////////////////////////////////////////////////////////////////
	//功能：以下为生产MATLAB定标格式影像
	//日期：2017.03.15
	//////////////////////////////////////////////////////////////////////////
	//ParseSTG ZY3_STGSTI;
	//StarExtract ZY3_STMap;
	//StarIdentify ZY3_ST;
	//APScalibration ZY3_calibrate;
	//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\";
	//vector<STGData> ZY3_02STGdata0702;
	//vector<STGData> ZY3_02STGdata0707;
	//vector<STGData> ZY3_02STGdata0712;
	//vector<STGData> ZY3_02STGdata0830;
	//vector<vector<StarGCP>>getGCPall;
	//vector<StarGCP > getGCPalltmp;
	//string argv1;
	//argv1 = workpath_ST + "0702\\0702.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0702);
	//argv1 = workpath_ST + "0707\\0707.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0707);
	//argv1 = workpath_ST + "0712\\0712.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0712);
	//argv1 = workpath_ST + "0830\\0830.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0830);
	//for (int index = 0; index <= 214; index++)
	//{
	//	printf("\r开始定标，累加%d景...", index);
	//	ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\Bouguet\\";
	//	ZY3_ST.workpath = ZY3_STMap.workpath;
	//	ZY3_calibrate.workpath = ZY3_STMap.workpath;
	//	ZY3_STMap.StarPointExtraction(index+1);
	//	ZY3_ST.GetStarGCP0712(ZY3_02STGdata0712, ZY3_STMap.StarPointExtract, index*5);
	//	ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 50, index+1);//精化控制点
	//	getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
	//	getGCPall.push_back(getGCPalltmp);//累加控制点
	//	ZY3_STMap.StarPointExtract.clear();//记得清楚vector里的内容，不然内容会叠加
	//	ZY3_ST.getGCP.clear();
	//}

	//////////////////////////////////////////////////////////////////////////
	//功能：以下为每隔一定景进行标定
	//日期：2017.02.21
	//////////////////////////////////////////////////////////////////////////
	//ParseSTG ZY3_STGSTI;
	//StarExtract ZY3_STMap;
	//StarIdentify ZY3_ST;
	//APScalibration ZY3_calibrate;
	//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\";	
	//vector<STGData> ZY3_02STGdata0702;
	//vector<STGData> ZY3_02STGdata0707;
	//vector<STGData> ZY3_02STGdata0712;
	//vector<STGData> ZY3_02STGdata0830;
	//vector<vector<StarGCP>>getGCPall;
	//vector<vector<StarGCP>>getGCPallresize;
	//vector<StarGCP > getGCPalltmp;
	//string argv1;
	//argv1 = workpath_ST + "0702\\0702.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0702);
	//argv1 = workpath_ST + "0707\\0707.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0707);
	//argv1 = workpath_ST + "0712\\0712.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0712);
	//argv1 = workpath_ST + "0830\\0830.STG";
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata0830);
	//const int label = 500;
	//for (int index = 1000; index >=3; )
	//{
	//	printf("\r开始定标，累加%d景...", index);
	//	ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\星图\\";
	//	ZY3_ST.workpath = ZY3_STMap.workpath;
	//	ZY3_calibrate.workpath = ZY3_STMap.workpath;
	//	ZY3_STMap.StarPointExtraction(index);
	//	ZY3_ST.GetStarGCP0702(ZY3_02STGdata0702, ZY3_STMap.StarPointExtract, index);
	//	ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 50,index);//精化控制点
	//	getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
	//	getGCPall.push_back(getGCPalltmp);//累加控制点
	//	ZY3_STMap.StarPointExtract.clear();//记得清楚vector里的内容，不然内容会叠加
	//	ZY3_ST.getGCP.clear();
	//	if (getGCPall.size()==label+1)
	//	{
	//		getGCPallresize.assign(getGCPall.begin()+1,getGCPall.end());
	//		getGCPall.swap(getGCPallresize);
	//		//ZY3_calibrate.Calibrate3ParamMultiImg(getGCPall, index);
	//		ZY3_calibrate.Calibrate5ParamMultiImg(getGCPall, index);
	//	}		
	//	index = index -5;//输出间隔
	//}

	//////////////////////////////////////////////////////////////////////////
	//功能：定义工作路径，STG、STI的路径，解析STI数据
	//日期：2017.02.20
	//////////////////////////////////////////////////////////////////////////
	//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
	//string workpath_STG = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\";
	//string workpath_STI = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\0712.STI";

	//////////////////////////////////////////////////////////////////////////
	//功能：以下为读取STG、STI数据流程
	//日期：2017.01.07
	//////////////////////////////////////////////////////////////////////////
	//ParseSTG ZY3_STGSTI;
	//ZY3_STGSTI.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\";
	//string workpath_STI = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\0830.STI";
	//vector<STGData> ZY3_02STGdata;
	////ZY3_STGSTI.ParseZY302_STG(argv[1], ZY3_02STGdata);
	//ZY3_STGSTI.ParseZY302_STItime(workpath_STI);
	//ZY3_STGSTI.ParseZY302_STI(workpath_STI);
	//ZY3_STGSTI.StarMap(ZY3_02STGdata);
	//ZY3_STGSTI.StarAngle(ZY3_02STGdata, 12);
	//ZY3_STGSTI.StarAngle(ZY3_02STGdata, 13);
	//ZY3_STGSTI.StarAngle(ZY3_02STGdata, 23);

	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星点提取流程
	//日期：2017.01.04
	//////////////////////////////////////////////////////////////////////////
	//StarExtract ZY3_STMap;
	//ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
	////ZY3_STMap.StarPointExtraction(1);
	//ZY3_STMap.StarPointMulti(1000);
	//ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\星图\\";
	//ZY3_STMap.StarPointMulti(1000); 
	//ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\星图\\";
	//ZY3_STMap.StarPointMulti(1000);
	

	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星敏定标流程
	//日期：2017.01.09
	//////////////////////////////////////////////////////////////////////////
	//StarIdentify ZY3_ST;
	//ZY3_ST.workpath = ZY3_STMap.workpath;
	//ZY3_ST.GetStarGCP(ZY3_02STGdata, ZY3_STMap.StarPointExtract);
	//APScalibration ZY3_calibrate;
	//ZY3_calibrate.workpath = workpath_ST;
	/*ZY3_calibrate.Calibrate3Param(ZY3_ST.getGCP);
	ZY3_calibrate.Calibrate6Param(ZY3_ST.getGCP);
	ZY3_calibrate.CalibrateRigorous3(ZY3_ST.getGCP);
	ZY3_calibrate.CalibrateRigorous6(ZY3_ST.getGCP);
	ZY3_calibrate.CalibrateRigorousRPY(ZY3_ST.getGCP);*/

	
	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星敏多景定标流程
	//日期：2017.01.14
	//////////////////////////////////////////////////////////////////////////
	//vector<vector<StarGCP>>getGCPall;
	//vector<StarGCP > getGCPalltmp;
	//for (int index = 1000; index >= 3; )
	//{
	//	printf("\r开始定标，累加%d景...", index);
	//	ZY3_STMap.StarPointExtraction(index);
	//	ZY3_ST.GetStarGCP(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
	//	ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 50);//精化控制点
	//	//ZY3_calibrate.OutputGCP(ZY3_ST.getGCP, index);//输出精化后的控制点
	//	getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
	//	getGCPall.push_back(getGCPalltmp);//累加控制点
	//	ZY3_STMap.StarPointExtract.clear();//记得清楚vector里的内容，不然内容会叠加
	//	ZY3_ST.getGCP.clear();
	//	ZY3_calibrate.Calibrate3ParamMultiImg(getGCPall, index);
	//	index = index-5;//输出间隔
	//}
	
	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星敏多景定标流程仿真
	//日期：2017.02.16
	//////////////////////////////////////////////////////////////////////////
	//vector<vector<StarGCP>>getGCPall;
	//vector<StarGCP > getGCPalltmp;
	//int index, gcpNumAll = 0;
	//const int nImg = 300;
	//int gcpNum[nImg];
	//BaseFunc mBase;
	//for (index = 1; index <= nImg; )
	//{
	//	printf("\r开始生成控制点，累加%d景...", index);
	//	ZY3_STMap.StarPointExtraction(index);
	//	ZY3_ST.GetStarGCP(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
	//	ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 100);//精化控制点
	//	gcpNum[index-1] = ZY3_ST.getGCP.size();
	//	getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
	//	getGCPall.push_back(getGCPalltmp);//累加控制点
	//	ZY3_STMap.StarPointExtract.clear();//记得清除vector里的内容，不然内容会叠加
	//	ZY3_ST.getGCP.clear();
	//	gcpNumAll = gcpNumAll + gcpNum[index-1];
	//	index = index++;//输出间隔
	//}
	//printf("开始定标\n", index);
	//double *randx = new double[gcpNumAll];
	//double *randy = new double[gcpNumAll];
	////设置x和y方向的噪声，并且加在真实数据上
	//mBase.RandomDistribution(1, 0.05, gcpNumAll, 0, randx);
	//mBase.RandomDistribution(3, 0.05, gcpNumAll, 0, randy);	
	//ZY3_calibrate.SimulateGCP_PreRand(getGCPall, ZY3_02STGdata, gcpNum, nImg, randx,randy);
	//vector<vector<StarGCP>>getGCPaccu;
	//for (index = 1; index <= nImg; )
	//{
	//	getGCPaccu.assign(getGCPall.begin(), getGCPall.begin() +index);
	//	printf("\r定标中，累加%d景...", index);
	//	ZY3_calibrate.Calibrate3ParamMultiImg(getGCPaccu, index);
	//	index = index++;//输出间隔
	//}
	
	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星敏多景定标流程仿真，星点位置跟随真实星图随机
	//日期：2017.03.27
	//////////////////////////////////////////////////////////////////////////
	//ParseSTG ZY3_STGSTI;
	//StarExtract ZY3_STMap;
	//StarIdentify ZY3_ST;
	//APScalibration ZY3_calibrate;
	//ZY3_calibrate.ZY302CaliParam.f = 2879;
	//ZY3_calibrate.ZY302CaliParam.x0 = 512 + 20;
	//ZY3_calibrate.ZY302CaliParam.y0 = 512 - 10;
	//ZY3_calibrate.ZY302CaliParam.k1 = -1e-8;
	//ZY3_calibrate.ZY302CaliParam.k2 = 8e-15;
	////ZY3_calibrate.ZY302CaliParam.k1 = 0;
	////ZY3_calibrate.ZY302CaliParam.k2 = 0;
	////string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\星图\\";
	//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
	////string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\星图\\";
	////string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\星图\\";
	//ZY3_STMap.workpath = workpath_ST;
	//ZY3_ST.workpath = ZY3_STMap.workpath;
	//ZY3_calibrate.workpath = ZY3_STMap.workpath;
	////string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\0702.STG";
	//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\0707.STG";
	////string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\0712.STG";
	////string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\0830.STG";
	//vector<STGData> ZY3_02STGdata;
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata);
	//vector<vector<StarGCP>>getGCPall;
	//vector<StarGCP > getGCPalltmp(0);
	//int index;
	//const int nImg = 1000;
	//for (index = 5; index <nImg; )
	//{
	//	printf("\r开始生成控制点，累加%d景...", index);
	//	ZY3_STMap.StarPointExtraction(index);
	//	//ZY3_ST.GetStarGCP0702(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
	//	ZY3_ST.GetStarGCP0707(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
	//	//ZY3_ST.GetStarGCP0712(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
	//	//ZY3_ST.GetStarGCP(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
	//	ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 100, index);//精化控制点
	//	getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
	//	getGCPall.push_back(getGCPalltmp);//累加控制点
	//	ZY3_STMap.StarPointExtract.clear();//记得清除vector里的内容，不然内容会叠加
	//	ZY3_ST.getGCP.clear();
	//	index = index++;//输出间隔
	//}	
	////星点按实际情况分布的控制点仿真
	////ZY3_calibrate.SimulateGCP_PreRand5Param(getGCPall, ZY3_02STGdata, nImg); 
	////星点随机分布的控制点仿真
	////ZY3_calibrate.SimulateGCP_RandomXY5Param(nImg, getGCPall);
	//printf("\n开始定标\n");
	////Kalman Filter定标
	////ZY3_calibrate.Calibrate3ParamKalman(getGCPall);
	//ZY3_calibrate.Calibrate5ParamKalman(getGCPall);
	//ZY3_calibrate.OutputAllGCP(getGCPall);
	//getGCPall.clear();
	//ZY3_calibrate.ReadAllGCP(getGCPall);

	//////////////////////////////////////////////////////////////////////////////
	//////功能：接上一步，为姿态确定过程
	//////日期：2017.04.27
	//////////////////////////////////////////////////////////////////////////////	
	//AttDetermination ZY3_AD;
	//ZY3_AD.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\姿态确定\\";
	//ZY3_AD.AttDeter2(ZY3_02STGdata, getGCPall);

	//vector<vector<StarGCP>>getGCPaccu;
	//for (index = 0; index < getGCPall.size(); )
	//{
	//	//ZY3_calibrate.Calibrate3Param(getGCPall[index], index);
	//	//ZY3_calibrate.Calibrate5Param(getGCPall[index], index);
	//	//getGCPaccu.assign(getGCPall[index].begin(), getGCPall[index].end());
	//	getGCPaccu.assign(getGCPall.begin(), getGCPall.begin() +index +1);
	//	printf("\r定标中，累加%d景...", index+1);
	//	ZY3_calibrate.Calibrate5ParamMultiImg(getGCPaccu, index + 1);
	//	//ZY3_calibrate.Calibrate3ParamMultiImg(getGCPaccu, index + 1);
	//	//ZY3_calibrate.CalibrateOpticAxisMultiImg(getGCPaccu, index + 1);
	//	//
	//	index++;//输出间隔
	//}

	//////////////////////////////////////////////////////////////////////////
	//功能：不定标，直接用原始姿态定位
	//日期：2017.04.27
	//////////////////////////////////////////////////////////////////////////	
	//APScalibration ZY3_calibrate;
	//ParseSTG ZY3_STGSTI;
	//AttDetermination ZY3_AD;
	//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\0707.STG"; 
	//vector<STGData> ZY3_02STGdata;
	//ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata);
	//ZY3_calibrate.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
	//vector<vector<StarGCP>>getGCPall;
	//ZY3_calibrate.ReadAllGCP(getGCPall);
	//ZY3_AD.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\姿态确定\\";
	//ZY3_AD.AttDeter2(ZY3_02STGdata, getGCPall);


	


	/*string path = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\星图\\控制点\\Allgcp.txt";
	FILE *fp = fopen(path.c_str(), "w");
	int n = ZY3_STall.getGCP.size();
	for (int i = 0; i < n ; i++)
	{
		fprintf(fp,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ZY3_STall.getGCP[i].V[0], ZY3_STall.getGCP[i].V[1],
			ZY3_STall.getGCP[i].V[2], ZY3_STall.getGCP[i].x, ZY3_STall.getGCP[i].y);
	}
	ZY3_STall.getGCP.clear();*/



	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星图识别流程
	//日期：2016.12.25
	//////////////////////////////////////////////////////////////////////////
	/*double array[] = { -0.00414330123660008, -0.0811396492167516, 0.996694130804353, 3.70000000000000, 10366, -0.531404221740299, -0.804297837296644, 0.265922060078752,	3.62000000000000,
		0.166822334948948, -0.160419053405453,	0.972849441520464,	3.90000000000000,	10330, -0.505400115551280, -0.740484871360861,	0.443004377502573,	3.83000000000000,
		-0.0213161783700601, -0.120677074320824,	0.992462928412473,	3.90000000000000,	10479, -0.494281082201171, -0.826336449536374,	0.269915327363341,	3.56000000000000,
		-0.0539625060505756,	0.00345913500324202,	0.998536970935864,	4.20000000000000,	10231, -0.582019990091462, -0.792337919825955,	0.182902574995030,	3.65000000000000 };
	for (size_t i = 0; i < 4; i++)
	{
		vector<double>tmp(array + 9 * i, array + 9 + 9 * i);
		ZY3_ST.id_list.push_back(tmp);
	}
	ZY3_ST.Q_Method();*/

	/*ZY3_ST.Load_Star_Data(argv[1]);
	double Optical_axistmp[] = {-0.59551143828591213, -0.76970238188778783, 0.23005297256699414};
	memcpy(ZY3_ST.optical_axis,Optical_axistmp,sizeof(double)*3);
	double star_tracker_output[6][3] = { -397,	11,		2.8,
										-12,	-235,	3.7,
										495,	-476,	3.9,
										334,	494,	3.9,
										-62,	-351,	3.9,
										-156,	10,		4.2	};
	double tmpob[3];
	for (int c1=0; c1<6; c1++)
	{
		tmpob[0] = star_tracker_output[c1][0]*0.015;
		tmpob[1] = star_tracker_output[c1][1]*0.015;
		tmpob[2] = 43.3;
		double normob = sqrt(pow(tmpob[0],2) + pow(tmpob[1],2) + pow(tmpob[2],2));
		strStar obstmp;
		obstmp.x = tmpob[0]/normob;
		obstmp.y = tmpob[1]/normob;
		obstmp.z = tmpob[2]/normob;
		obstmp.Mag = star_tracker_output[c1][2];
		ZY3_ST.obs.push_back(obstmp);
	}
	ZY3_ST.fov_radius = 8*PI/180;
	ZY3_ST.Create_Spherical_Polygon_Candidate_Set();
	ZY3_ST.candidate_radius = 25*5e-6;
	ZY3_ST.match_tolerance = 500*5e-6;
	ZY3_ST.Identify_Basis_Pair();
	ZY3_ST.match_tolerance = 500 * 5e-6;
	ZY3_ST.Match_Stars_Relative_To_Basis_Pair();
	ZY3_ST.Q_Method();*/
	PlaySound(TEXT("C:\\WINDOWS\\Media\\Alarm01.wav"), NULL, SND_FILENAME);
	return 0;
}