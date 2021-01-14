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
	//////////////////////////////////////////////////////////////////////////
	//功能：定标仿真
	//参数：sim
	//日期：2020.08.01
	//////////////////////////////////////////////////////////////////////////	
	if (argc == 2 && strcmp(argv[1], "sim") == 0)
	{
		APScalibration ZY3_calibrate;
		AttDetermination mDeter;
		vector<vector<StarGCP>> realGCPall;
		vector<vector<StarGCP>> getGCPall;
		int nImg = 200;
		ZY3_calibrate.workpath = "F:\\星图仿真\\";
		ZY3_calibrate.ZY302CaliParam.f = 43.3 / 0.015;
		ZY3_calibrate.ZY302CaliParam.x0 = 512;
		ZY3_calibrate.ZY302CaliParam.y0 = 512;
		ZY3_calibrate.ZY302CaliParam.k1 = 0;
		ZY3_calibrate.ZY302CaliParam.k2 = 0;
		//星点随机分布的控制点仿真
		//ZY3_calibrate.SimulateGCP_RandomXY5Param(nImg, getGCPall);
		ZY3_calibrate.SimulateGCP_RandomXY(nImg, realGCPall, getGCPall);//这个反而是5参数
		//printf("\n开始定标\n");
		//Kalman Filter定标
		//ZY3_calibrate.Calibrate3ParamKalman(getGCPall);
		//ZY3_calibrate.Calibrate5ParamKalman(getGCPall);

		string out = "F:\\星图仿真\\comRealEst.txt";
		FILE* fp = fopen(out.c_str(), "w");
		vector<Quat>vecEstQ, vecRealQ;
		for (int i = 0; i < nImg; i++)
		{
			Quat realQuat, estQuat;
			mDeter.q_Method(realGCPall[i], ZY3_calibrate.ZY302CaliParam, realQuat);
			vecRealQ.push_back(realQuat);
			StarCaliParam estParam;
			estParam.f = 43.3 / 0.015+0.1;
			estParam.x0 = 512.1;
			estParam.y0 = 511.9;
			estParam.k1 = 0;
			estParam.k2 = 0;
			//estParam = ZY3_calibrate.ZY302CaliParam;
			mDeter.q_Method(getGCPall[i], estParam, estQuat);
			vecEstQ.push_back(estQuat);
			SateEuler ruEuler;
			mDeter.CalOptAngle(estQuat, realQuat, ruEuler);
			fprintf(fp,"%.6f\t%.6f\t%.6f\t%.6f\n",ruEuler.R,ruEuler.P,ruEuler.Y,ruEuler.UTC);
		}
		string out2 = "F:\\星图仿真\\comRealEstByQ.txt";
		mDeter.compareRes(vecRealQ,vecEstQ,out2);
		fclose(fp);
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：吉林一号视频07星恒星拍摄处理
	//参数：F:\视频07星
	//日期：2020.10.13
	//////////////////////////////////////////////////////////////////////////	
	if (argc == 3 && atoi(argv[2]) == 7)
	{
		BaseFunc mBase;
		ParseSTG mStarMap;
		StarIdentify mStarid;
		AttDetermination mDeter;

		vector<string>csvPath;
		string tmp = string(argv[1]) + "\\";
		mBase.search_directory(tmp, "csv", csvPath);


		StarCaliParam param; Quat camQuat; SateEuler ruEuler;
		//param.f = 6500 / tan(0.6360615424140929 / 180 * PI);//焦距长度以像素单位表示，这是星图识别后的估值
		param.f = 3.2 / 5.5 * 1000000;
		param.x0 = 6000, param.y0 = 2500;

		string fresult = (string)argv[1] + "\\result.txt";
		FILE* fp = fopen(fresult.c_str(), "w");
		for (int j = 0; j < csvPath.size(); j++)
		{
			vector<string>ptsPath;
			tmp = csvPath[j].substr(0, csvPath[j].rfind('\\') + 1);
			mBase.search_directory(tmp, "pts", ptsPath);
			vector<img> imgJL107; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
			mStarMap.ReadJL107csv(csvPath[j], imgJL107, att, sa, sb, sc);

			for (int i = 0; i < ptsPath.size(); i++)
			{
				vector<StarGCP> camGCP; img imgLat;
				mStarid.GetStarGCPforJL106(ptsPath[i], camGCP, imgJL107, imgLat);//建立相机与恒星控制点
				mDeter.q_MethodforJL106(camGCP, param, camQuat);//根据qMethod来建立相机在J2000下姿态

				//相机和J2000姿态
				Quat q_inter, qa, qb, qc;
				mBase.QuatInterpolation(att, camQuat.UTC, q_inter);
				mDeter.CalOptAngle(q_inter, camQuat, ruEuler);
				fprintf(fp, "%.9f\t%.9f\t%.9f\t", camQuat.UTC, imgLat.lat, ruEuler.UTC);

				//相机和星敏A
				mBase.QuatInterpolation(sa, camQuat.UTC, qa);
				mDeter.CalOptAngle(qa, camQuat, ruEuler);
				fprintf(fp, "%.9f\t%.9f\t%.9f\t", imgLat.sst[0], imgLat.inst[0], ruEuler.UTC);

				//相机和星敏B
				mBase.QuatInterpolation(sb, camQuat.UTC, qb);
				mDeter.CalOptAngle(qb, camQuat, ruEuler);
				fprintf(fp, "%.9f\t%.9f\t%.9f\t", imgLat.sst[1], imgLat.inst[1], ruEuler.UTC);

				//相机和星敏C
				mBase.QuatInterpolation(sc, camQuat.UTC, qc);
				mDeter.CalOptAngle(qc, camQuat, ruEuler);
				fprintf(fp, "%.9f\t%.9f\t%.9f\t", imgLat.sst[2], imgLat.inst[2], ruEuler.UTC);

				//星敏A和星敏B
				mDeter.CalOptAngle(qa, qb, ruEuler);
				fprintf(fp, "%.9f\t", ruEuler.UTC);
				//星敏B和星敏C
				mDeter.CalOptAngle(qb, qc, ruEuler);
				fprintf(fp, "%.9f\t", ruEuler.UTC);
				//星敏C和星敏A
				mDeter.CalOptAngle(qc, qa, ruEuler);
				fprintf(fp, "%.9f\n", ruEuler.UTC);
			}
		}
		fclose(fp);

		for (int k = 0; k < csvPath.size(); k++)
		{
			vector<img> imgJL107; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
			mStarMap.ReadJL107csv(csvPath[k], imgJL107, att, sa, sb, sc);

			StarGCP camOpt;
			string optPath = csvPath[k].substr(0, csvPath[k].rfind('\\') + 1) + "指向信息.txt";
			mStarid.GetCamOptforJL107(optPath, camOpt, imgJL107);
			fprintf(fp, "第%d组数据结果\n", k + 1);

			//相机和J2000姿态
			double theta; Quat q_inter;
			mBase.QuatInterpolation(att, camOpt.UTC, q_inter);
			mDeter.CalOptAngleforJL107(camOpt, q_inter, theta);
			fprintf(fp, "%.9f\t%.9f\t%.9f\t", camOpt.UTC, imgJL107[0].lat, theta);

			//相机和星敏A
			mBase.QuatInterpolation(sa, camOpt.UTC, q_inter);
			mDeter.CalOptAngleforJL107(camOpt, q_inter, theta);
			fprintf(fp, "%.9f\t", theta);

			//相机和星敏B
			mBase.QuatInterpolation(sb, camOpt.UTC, q_inter);
			mDeter.CalOptAngleforJL107(camOpt, q_inter, theta);
			fprintf(fp, "%.9f\t", theta);

			//相机和星敏C
			mBase.QuatInterpolation(sc, camOpt.UTC, q_inter);
			mDeter.CalOptAngleforJL107(camOpt, q_inter, theta);
			fprintf(fp, "%.9f\n", theta);

		}
		fclose(fp);
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：吉林一号视频07星星图仿真
	//参数：F:\1.全天区观测\1113拍摄
	//日期：2020.11.21
	//////////////////////////////////////////////////////////////////////////	
	if (argc == 3 && atoi(argv[2]) == 71)
	{
		BaseFunc mBase;
		ParseSTG mStarMap;
		StarIdentify mStarid;
		AttDetermination mDeter;

		vector<string>csvPath; vector<string>tifPath;
		map<int, string>csvMap;	
		string tmp = string(argv[1]) + "\\";
		mBase.search_directory(tmp, "csv", csvPath);
		mBase.search_directory(tmp, "tif", tifPath);
		for (size_t i = 0; i < csvPath.size(); i++)
		{
			string cvsStr = csvPath[i];
			cvsStr = cvsStr.substr(cvsStr.rfind('.')-3,3);
			int cvsNum = atoi(cvsStr.c_str());
			csvMap.insert(pair<int, string>(cvsNum, csvPath[i]));
		}
		vector<string>tifTmp;
		for (size_t i = 0; i < tifPath.size(); i++)
		{
			string mss = tifPath[i].substr(tifPath[i].rfind('\\')+1);
			mss = mss.substr(mss.find('_')+1);
			mss = mss.substr(0, mss.find('_'));
			if (mss=="MSS1")
			{
				tifTmp.push_back(tifPath[i]);
			}
		}
		tifPath.empty();
		tifPath.assign(tifTmp.begin(),tifTmp.end());
		StarCaliParam param; Quat camQuat; SateEuler ruEuler;
		param.f = 6500 / tan(0.635364 / 180 * PI);//焦距长度以像素单位表示，这是107星MSS1星图识别后的估值
		//param.f = 6500 / tan(0.632246 / 180 * PI);//焦距长度以像素单位表示，这是107星MSS2星图识别后的估值
		//param.f = 3.2 / 5.5 * 1000000;
		param.x0 = 6000, param.y0 = 2500;
		vector<Quat>jlcam(tifPath.size());

		for (int j = 0; j < tifPath.size(); j++)
		{
			string tifStr = tifPath[j];
			tifStr = tifStr.substr(tifStr.rfind('\\') + 1);
			int tifNum = atoi(tifStr.substr(37, 3).c_str());
			int tifNum2 = atoi(tifStr.substr(41, 4).c_str());
			string strcsv = csvMap.find(tifNum)->second;
			vector<img> imgJL107; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
			vector<Orbit_Ep>imgOrb;
			mStarMap.ReadJL107csvOrb(strcsv, imgJL107, att, sa, sb, sc, imgOrb);
			mDeter.AberrationForJLYHStarSensor(sa, imgOrb);
			mDeter.AberrationForJLYHStarSensor(sb, imgOrb);
			mDeter.AberrationForJLYHStarSensor(sc, imgOrb);
			double utc;
			for (size_t i = 0; i < imgJL107.size(); i++)			{
				if (imgJL107[i].id==tifNum2)				{
					utc = imgJL107[i].time;
					break;
				}
			}
			Quat attInter;
			mBase.QuatInterpolation(sc, utc, attInter);
			double R, P, Y;
			//相机MSS1
			//R = 0.0067936220757141546; P = 0.0086088500886015318; Y = 1.5801625048077292;//滤波
			//R=2.2626568731425847; P=0.97050907772505157; Y=2.5420802327784640;//星敏A
			//R=-2.5441346561139864; P=-0.95872822902100208; Y=-0.88847597589993255;//星敏B
			R=2.0465047999674870; P=-0.28502464332487132; Y=-3.0061863664092079;//星敏C
			//相机MSS2
			//R = 0.0059250330655504415; P = -0.0085995651608813360; Y = 1.5704067439676139;//滤波参数
			//R = 1.0033330535061187; P = -0.78312230795773907; Y = 1.7717620910145522;//星敏A
			//R = -2.5482561856221637; P = -0.93958580037967587; Y = -0.89207549048663903;//星敏B
			//R = 2.0512975188863671; P = -0.28513967534725970; Y = -2.9859023048872020;//星敏C
			double r1[9], r2[9], r3[9];
			mBase.quat2matrix(attInter.Q1, attInter.Q2, attInter.Q3, attInter.Q0, r1);//Crj
			mBase.rot123(R, P, Y, r2);//Crc
			mBase.Multi(r2, r1, r3, 3, 3, 3);//相机的Ccj
			mBase.matrix2quat(r3, jlcam[j].Q1, jlcam[j].Q2, jlcam[j].Q3, jlcam[j].Q0);
		}
		mStarMap.StarMapForJL01(tmp, jlcam,param);
	}
	if (argc == 3 && atoi(argv[2]) == 72)//针对吉林一号07星MSS2
	{
		BaseFunc mBase;
		ParseSTG mStarMap;
		StarIdentify mStarid;
		AttDetermination mDeter;

		vector<string>csvPath; vector<string>ptsPath;
		map<int, string>csvMap;
		string tmp = string(argv[1]) + "\\";
		mBase.search_directory(tmp, "csv", csvPath);
		mBase.search_directory(tmp, "pts", ptsPath);
		for (size_t i = 0; i < csvPath.size(); i++)
		{
			string cvsStr = csvPath[i];
			cvsStr = cvsStr.substr(cvsStr.rfind('.') - 3, 3);
			int cvsNum = atoi(cvsStr.c_str());
			csvMap.insert(pair<int, string>(cvsNum, csvPath[i]));
		}

		StarCaliParam param; Quat camQuat; SateEuler ruEuler;
		param.f = 6500 / tan(0.635364 / 180 * PI);//焦距长度以像素单位表示，这是107星MSS1星图识别后的估值
		//param.f = 6500 / tan(0.632246 / 180 * PI);//焦距长度以像素单位表示，这是107星MSS2星图识别后的估值
		//param.f = 3.2 / 5.5 * 1000000;
		param.x0 = 6000, param.y0 = 2500;

		vector<string>ptsTmp;
		for (size_t i = 0; i < ptsPath.size(); i++)
		{
			string mss = ptsPath[i].substr(ptsPath[i].rfind('\\') + 1);
			mss = mss.substr(mss.find('_') + 1);
			mss = mss.substr(0, mss.find('_'));
			if (mss == "MSS2")
			{
				ptsTmp.push_back(ptsPath[i]);
			}
		}
		ptsPath.empty();
		ptsPath.assign(ptsTmp.begin(), ptsTmp.end());

		string fresult = (string)argv[1] + "\\result.txt";
		FILE* fp = fopen(fresult.c_str(), "w");
		for (int j = 0; j < ptsPath.size(); j++)
		{
			string mss = ptsPath[j].substr(ptsPath[j].rfind('\\') + 1);
			mss = mss.substr(mss.find('_') + 1);
			mss = mss.substr(0, mss.find('_'));
			if (mss == "MSS1")
			{
				param.f = 3.225391 / 5.5 * 1000000;
				param.x0 = 34.8780 / 5.5 * 1000, param.y0 = -8.0310 / 5.5 * 1000;
			}
			if (mss == "MSS2")
			{
				param.f = 3.225391 / 5.5 * 1000000;
				param.x0 = 32.25898 / 5.5 * 1000, param.y0 = -3.900990 / 5.5 * 1000;
			}

			string tifStr = ptsPath[j];
			tifStr = tifStr.substr(tifStr.rfind('\\') + 1);
			int tifNum = atoi(tifStr.substr(37, 3).c_str());
			int tifNum2 = atoi(tifStr.substr(41, 4).c_str());
			string strcsv = csvMap.find(tifNum)->second;
			vector<img> imgJL107; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
			img imgJL107one; vector<Orbit_Ep>imgOrb;
			mStarMap.ReadJL107csvOrb(strcsv, imgJL107, att, sa, sb, sc,imgOrb);

			//mDeter.AberrationForJLYHStarSensor(sa,imgOrb);
			//mDeter.AberrationForJLYHStarSensor(sb, imgOrb);
			//mDeter.AberrationForJLYHStarSensor(sc, imgOrb);
			double utc;
			for (size_t i = 0; i < imgJL107.size(); i++)			{
				if (imgJL107[i].id == tifNum2)				{
					utc = imgJL107[i].time;
					imgJL107one = imgJL107[i];
					break;
				}
			}
			vector<StarGCP> camGCP; img imgLat;
			mStarid.GetStarGCPforJL107(ptsPath[j], camGCP, imgJL107one, imgLat);//建立相机与恒星控制点
			mDeter.q_MethodforJL107(camGCP, param, camQuat,imgOrb);//根据qMethod来建立相机在J2000下姿态

			//相机和J2000姿态
			Quat q_inter, qa, qb, qc;
			mBase.QuatInterpolation(att, camQuat.UTC, q_inter);
			mDeter.CalOptAngle(q_inter, camQuat, ruEuler);
			fprintf(fp, "%3d\t%.9f\t%.9f\t%.9f\t%.9f\t", tifNum, camQuat.UTC, imgLat.lat, imgLat.lon, ruEuler.UTC);

			//相机和星敏A
			mBase.QuatInterpolation(sa, camQuat.UTC, qa);
			mDeter.CalOptAngle(qa, camQuat, ruEuler);
			fprintf(fp, "%.9f\t%.9f\t%.9f\t", imgLat.sst[0], imgLat.inst[0], ruEuler.UTC);

			//相机和星敏B
			mBase.QuatInterpolation(sb, camQuat.UTC, qb);
			mDeter.CalOptAngle(qb, camQuat, ruEuler);
			fprintf(fp, "%.9f\t%.9f\t%.9f\t", imgLat.sst[1], imgLat.inst[1], ruEuler.UTC);

			//相机和星敏C
			mBase.QuatInterpolation(sc, camQuat.UTC, qc);
			mDeter.CalOptAngle(qc, camQuat, ruEuler);
			fprintf(fp, "%.9f\t%.9f\t%.9f\t", imgLat.sst[2], imgLat.inst[2], ruEuler.UTC);

			//星敏A和星敏B
			mDeter.CalOptAngle(qa, qb, ruEuler);
			fprintf(fp, "%.9f\t", ruEuler.UTC);
			//星敏B和星敏C
			mDeter.CalOptAngle(qb, qc, ruEuler);
			fprintf(fp, "%.9f\t", ruEuler.UTC);
			//星敏C和星敏A
			mDeter.CalOptAngle(qc, qa, ruEuler);
			fprintf(fp, "%.9f\n", ruEuler.UTC);
		}
		fclose(fp);
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：针对吉林一号07星，根据星敏和相机夹角建模，补偿下一次拍摄
	//参数：E:\1.全天区观测\1113拍摄 73
	//日期：2020.12.19；2021.01.12
	//////////////////////////////////////////////////////////////////////////	
	if (argc == 3 && atoi(argv[2]) == 73)
	{
		BaseFunc mBase;
		ParseSTG mStarMap;
		StarIdentify mStarid;
		AttDetermination mDeter;

		vector<string>csvPath; vector<string>ptsPath;
		map<int, string>csvMap;
		string tmp = string(argv[1]) + "\\";
		mBase.search_directory(tmp, "csv", csvPath);
		mBase.search_directory(tmp, "pts", ptsPath);
		for (size_t i = 0; i < csvPath.size(); i++)
		{
			string cvsStr = csvPath[i];
			cvsStr = cvsStr.substr(cvsStr.rfind('.') - 3, 3);
			int cvsNum = atoi(cvsStr.c_str());
			csvMap.insert(pair<int, string>(cvsNum, csvPath[i]));
		}

		double R, P, Y;
		double Crb1[9], Crb2[9], Crb3[9];
		//相机MSS1
		//R=2.2626568731425847; P=0.97050907772505157; Y=2.5420802327784640;//星敏A
		//R=-2.5441346561139864; P=-0.95872822902100208; Y=-0.88847597589993255;//星敏B
		//R = 2.0465047999674870; P = -0.28502464332487132; Y = -3.0061863664092079;//星敏C

		//相机MSS2
		R = 1.0033330535061187; P = -0.78312230795773907; Y = 1.7717620910145522;//星敏A
		mBase.rot123(R, P, Y, Crb1);//Crc
		R = -2.5482561856221637; P = -0.93958580037967587; Y = -0.89207549048663903;//星敏B
		mBase.rot123(R, P, Y, Crb2);//Crc
		R = 2.0512975188863671; P = -0.28513967534725970; Y = -2.9859023048872020;//星敏C
		mBase.rot123(R, P, Y, Crb3);//Crc

		StarCaliParam param; Quat camQuat; SateEuler ruEuler;
		vector<string>ptsTmp;
		for (size_t i = 0; i < ptsPath.size(); i++)
		{
			string mss = ptsPath[i].substr(ptsPath[i].rfind('\\') + 1);
			mss = mss.substr(mss.find('_') + 1);
			mss = mss.substr(0, mss.find('_'));
			if (mss == "MSS2")
			{
				ptsTmp.push_back(ptsPath[i]);
			}
		}
		ptsPath.empty();
		ptsPath.assign(ptsTmp.begin(), ptsTmp.end());

		string fresult = (string)argv[1] + "\\StarBC_Cam.txt";
		FILE* fp = fopen(fresult.c_str(), "w");
		for (int j = 0; j < ptsPath.size(); j++)
		{
			string mss = ptsPath[j].substr(ptsPath[j].rfind('\\') + 1);
			mss = mss.substr(mss.find('_') + 1);
			mss = mss.substr(0, mss.find('_'));
			if (mss == "MSS1")
			{
				param.f = 3.225391 / 5.5 * 1000000;
				param.x0 = 34.8780 / 5.5 * 1000, param.y0 = -8.0310 / 5.5 * 1000;
			}
			if (mss == "MSS2")
			{
				param.f = 3.225391 / 5.5 * 1000000;
				param.x0 = 32.25898 / 5.5 * 1000, param.y0 = -3.900990 / 5.5 * 1000;
			}

			string tifStr = ptsPath[j];
			tifStr = tifStr.substr(tifStr.rfind('\\') + 1);
			int tifNum = atoi(tifStr.substr(37, 3).c_str());
			int tifNum2 = atoi(tifStr.substr(41, 4).c_str());
			string strcsv = csvMap.find(tifNum)->second;
			vector<img> imgJL107; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
			img imgJL107one; vector<Orbit_Ep>imgOrb;
			mStarMap.ReadJL107csvOrb(strcsv, imgJL107, att, sa, sb, sc, imgOrb);

			mDeter.AberrationForJLYHStarSensor(sa, imgOrb);
			mDeter.AberrationForJLYHStarSensor(sb, imgOrb);
			mDeter.AberrationForJLYHStarSensor(sc, imgOrb);
			double utc;
			for (size_t i = 0; i < imgJL107.size(); i++) {
				if (imgJL107[i].id == tifNum2) {
					utc = imgJL107[i].time;
					imgJL107one = imgJL107[i];
					break;
				}
			}
			vector<StarGCP> camGCP; img imgLat;
			mStarid.GetStarGCPforJL107(ptsPath[j], camGCP, imgJL107one, imgLat);//建立相机与恒星控制点
			mDeter.q_MethodforJL107(camGCP, param, camQuat, imgOrb);//根据qMethod来建立相机在J2000下姿态

			//相机和星敏BC
			Quat camCal;
			Quat q_inter, qa, qb, qc;
			double theta1, theta2, optical[6];
			mBase.QuatInterpolation(sb, camQuat.UTC, qb);
			mBase.QuatInterpolation(sc, camQuat.UTC, qc);

			//1128次作为参数拟合
			theta1 = 0.4652 * imgLat.lat + 429863;//星敏B拟合参数
			theta2 = 0.0119 * imgLat.lat + 417739;//星敏C拟合参数
			//theta1 = 429863;//星敏B固定参数
			//theta2 = 417739;//星敏C固定参数

			//1026次作为参数拟合
			theta1 = 0.2667 * imgLat.lat + 429874;//星敏B拟合参数
			theta2 = -0.2125 * imgLat.lat + 417757;//星敏C拟合参数
			theta1 = 429874;//星敏B固定参数
			theta2 = 417757;//星敏C固定参数

			double opt[3] = { 0,0,1 }, rCam[9], vCam[3];
			mDeter.camOpticalCal(qb, qc, theta1, theta2, optical);
			mBase.quat2matrix(camQuat.Q1, camQuat.Q2, camQuat.Q3, camQuat.Q0, rCam);//恒星观测的Ccj
			mBase.invers_matrix(rCam, 3);
			mBase.Multi(rCam, opt, vCam, 3, 3, 1);
			double ab = vCam[0] * optical[0] + vCam[1] * optical[1] + vCam[2] * optical[2];
			double cd = vCam[0] * optical[3] + vCam[1] * optical[4] + vCam[2] * optical[5];
			ab = ab > cd ? ab : cd;
			double theta = acos(ab) / PI * 180 * 3600;//用utc替代一下
			fprintf(fp,"%.9f\t%.9f\n", imgLat.lat, theta);
		}
		fclose(fp);
	}
	//////////////////////////////////////////////////////////////////////////
//功能：针对吉林一号07星，计算所有星敏夹角
//参数：E:\1.全天区观测\1113拍摄 74
//日期：2021.01.12
//////////////////////////////////////////////////////////////////////////	
	if (argc == 3 && atoi(argv[2]) == 74)
	{
		BaseFunc mBase;
		ParseSTG mStarMap;
		StarIdentify mStarid;
		AttDetermination mDeter;

		vector<string>csvPath; 
		map<int, string>csvMap;
		string tmp = string(argv[1]) + "\\";
		mBase.search_directory(tmp, "csv", csvPath);

		StarCaliParam param; Quat camQuat; SateEuler ruEuler;

		string fresult = (string)argv[1] + "\\StarABC_angle.txt";
		FILE* fp = fopen(fresult.c_str(), "w");
		for (int j = 0; j < csvPath.size(); j++)
		{
			string cvsStr = csvPath[j];
			cvsStr = cvsStr.substr(cvsStr.rfind('.') - 3, 3);
			int cvsNum = atoi(cvsStr.c_str());
			fprintf(fp, "%d\n", cvsNum);

			vector<img> imgJL107; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
			img imgJL107one; vector<Orbit_Ep>imgOrb;
			mStarMap.ReadJL107csvOrb(csvPath[j], imgJL107, att, sa, sb, sc, imgOrb);
			
			mDeter.AberrationForJLYHStarSensor(sa, imgOrb);
			mDeter.AberrationForJLYHStarSensor(sb, imgOrb);
			mDeter.AberrationForJLYHStarSensor(sc, imgOrb);

			for (size_t i = 0; i < sa.size(); i++)
			{
				//相机和星敏BC
				Quat camCal;
				Quat q_inter, qa, qb, qc;
				double theta1, theta2, optical[6];
				mBase.QuatInterpolation(sb, sa[i].UTC, qb);
				mBase.QuatInterpolation(sc, sa[i].UTC, qc);

				//星敏A和星敏B
				mDeter.CalOptAngle(sa[i], qb, ruEuler);
				fprintf(fp, "%.9f\t%.9f\t", sa[i].UTC, ruEuler.UTC);
				//星敏B和星敏C
				mDeter.CalOptAngle(qb, qc, ruEuler);
				fprintf(fp, "%.9f\t", ruEuler.UTC);
				//星敏C和星敏A
				mDeter.CalOptAngle(qc, sa[i], ruEuler);
				fprintf(fp, "%.9f\n", ruEuler.UTC);
			}
			fprintf(fp, "\n");
		}
		fclose(fp);
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：吉林一号视频06星恒星拍摄处理
	//参数：F:\珞珈一号\视频06北极星数据 6
	//日期：2020.05.06
	//////////////////////////////////////////////////////////////////////////	
	if (argc == 3 && atoi(argv[2]) == 6)
	{
		BaseFunc mBase;
		StarExtract mExtract;
		ParseSTG mStarMap;
		StarIdentify mStarid;
		AttDetermination mDeter;
		vector<string>rawRaw;
		//mStarMap.StarMapForJL01("");

		vector<string>csvPath;
		string outPath = string(argv[1]) + "\\";
		mBase.search_directory(outPath, "csv", csvPath);

		//求出相机指向四元数和生成星图
		if (false)
		{
			csvPath.erase(csvPath.begin(), csvPath.begin() + 5);
			vector< Quat>jlcam(csvPath.size());
			FILE* fp = fopen((string(argv[1]) + "\\cameraQuat.txt").c_str(), "w");
			for (int i = 0; i < csvPath.size(); i++)
			{
				vector<img> imgJL106; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
				mStarMap.ReadJL107csv(csvPath[i], imgJL106, att, sa, sb, sc);
				Quat q_inter;
				mBase.QuatInterpolation(sb, imgJL106[0].time, q_inter);
				double R, P, Y;
				//R = -2.0504583150212916; P = -0.37975153041836707; Y = 1.7558305015221418;//星敏A
				R = -2.4679323340640642;  P = 1.0396789012622079;  Y = 0.73042492356819078; //星敏B
				//R = 0.0080833780663713863; P = 0.0016727918023371185; Y = 1.5318656051912103;//双矢量
				double ra, dec;
				mDeter.jl106CamQuat(R, P, Y, q_inter, jlcam[i], ra, dec);//ra，dec表示光轴指向赤经赤纬
				fprintf(fp, "%s\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\t%.9lf\n", csvPath[i].c_str(), jlcam[i].Q1, jlcam[i].Q2, jlcam[i].Q3, jlcam[i].Q0, ra, dec);
			}
			fclose(fp);
			StarCaliParam param;
			param.f = 3.2 / 5.5 * 1000000;
			param.x0 = 6000, param.y0 = 2500;
			mStarMap.StarMapForJL01(outPath, jlcam, param);
		}

		//mBase.search_directory(path[0].c_str(), "raw", rawPath);
		//计算吉林一号背景值
		//mExtract.StarCameraBackgroundForJiLin(rawPath);

		if (true)
		{
			vector<string>rawPath;
			string fresult = (string)argv[1] + "\\result.txt";
			FILE* fp = fopen(fresult.c_str(), "w");

			StarCaliParam param; Quat camQuat; SateEuler ruEuler;
			//param.f = 6500 / tan(0.6360615424140929 / 180 * PI);//焦距长度以像素单位表示，这是星图识别后的估值
			param.f = 3.2 / 5.5 * 1000000;
			param.x0 = 6000, param.y0 = 2500;

			for (int k = 0; k < csvPath.size(); k++)
			{
				string strPts = csvPath[k].substr(0, csvPath[k].rfind('\\') + 1);
				string ptsName = csvPath[k].substr(csvPath[k].rfind('\\'));
				vector<string>ptsPath;
				mBase.search_directory(strPts, "pts", ptsPath);

				vector<img> imgJL106; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
				mStarMap.ReadJL107csv(csvPath[k], imgJL106, att, sa, sb, sc);
				SateEuler attEuler, saEuler, sbEuler, scEuler;

				fprintf(fp, "第%s组数据结果\n", ptsName.c_str());

				for (int i = 0; i < ptsPath.size(); i++)
				{
					vector<StarGCP> camGCP; img imgLat;
					mStarid.GetStarGCPforJL106(ptsPath[i], camGCP, imgJL106, imgLat);//建立相机与恒星控制点
					mDeter.q_MethodforJL106(camGCP, param, camQuat);//根据qMethod来建立相机在J2000下姿态

					//相机和J2000姿态
					Quat q_inter, qa, qb, qc;
					mBase.QuatInterpolation(att, camQuat.UTC, q_inter);
					mDeter.CalOptAngle(q_inter, camQuat, ruEuler);
					fprintf(fp, "%.9f\t%.9f\t%.9f\t", camQuat.UTC, imgLat.lat, ruEuler.UTC);

					//相机和星敏A
					mBase.QuatInterpolation(sa, camQuat.UTC, qa);
					mDeter.CalOptAngle(qa, camQuat, ruEuler);
					fprintf(fp, "%.9f\t", ruEuler.UTC);
					//double R = 0; double P = 0; double Y = 0;
					//mDeter.jl106AlinFix(R, P, Y, qa, camQuat, ruEuler);求取偏置

					//相机和星敏B
					mBase.QuatInterpolation(sb, camQuat.UTC, qb);
					mDeter.CalOptAngle(qb, camQuat, ruEuler);
					fprintf(fp, "%.9f\t", ruEuler.UTC);

					//相机和星敏C
					mBase.QuatInterpolation(sc, camQuat.UTC, qc);
					mDeter.CalOptAngle(qc, camQuat, ruEuler);
					fprintf(fp, "%.9f\t", ruEuler.UTC);


					//星敏A和星敏B
					mDeter.CalOptAngle(qa, qb, ruEuler);
					fprintf(fp, "%.9f\t", ruEuler.UTC);
					//星敏B和星敏C
					mDeter.CalOptAngle(qb, qc, ruEuler);
					fprintf(fp, "%.9f\t", ruEuler.UTC);
					//星敏C和星敏A
					mDeter.CalOptAngle(qc, qa, ruEuler);
					fprintf(fp, "%.9f\n", ruEuler.UTC);
				}

			}
			fclose(fp);
		}
		if (false)
		{
			string fresult = (string)argv[1] + "\\result.txt";
			FILE* fp = fopen(fresult.c_str(), "w");
			for (int k = 0; k < 5; k++)
			{
				string path[5];
				//mBase.search_directory(path, "raw", rawRaw);
				vector<string>csvPath;
				vector<string>ptsPath;
				mBase.search_directory(path[k], "csv", csvPath);
				mBase.search_directory(path[k], "pts", ptsPath);

				vector<img> imgJL106; vector<Quat> att; vector<Quat> sa; vector<Quat> sb; vector<Quat> sc;
				StarCaliParam param; Quat camQuat; SateEuler ruEuler;
				param.f = 6500 / tan(0.6360615424140929 / 180 * PI);//焦距长度以像素单位表示，这是星图识别后的估值
				//param.f = 3.2/5.5*1000000;
				param.x0 = 6000, param.y0 = 2500;
				mStarMap.ReadJL106csv(csvPath[0], imgJL106, att, sa, sb, sc);
				SateEuler attEuler, saEuler, sbEuler, scEuler;

				fprintf(fp, "第%d组数据结果\n", k + 1);

				for (int i = 0; i < ptsPath.size(); i++)
				{
					vector<StarGCP> camGCP;
					mStarid.GetStarGCPforJL106(ptsPath[i], camGCP, imgJL106);//建立相机与恒星控制点
					mDeter.q_MethodforJL106(camGCP, param, camQuat);//根据qMethod来建立相机在J2000下姿态

					//相机和J2000姿态
					Quat q_inter;
					mBase.QuatInterpolation(att, camQuat.UTC, q_inter);
					//double R = 0.0080626; double P = 0.0016332; double Y = 1.5318972;//第一组数据
					//double R = 0.0080833780663713863; double P = 0.0016727918023371185; double Y = 1.5318656051912103;//第二组数据
					double R = 0; double P = 0; double Y = 0;
					mDeter.jl106AlinFix(R, P, Y, q_inter, camQuat, ruEuler);
					attEuler.R += ruEuler.R; attEuler.P += ruEuler.P; attEuler.Y += ruEuler.Y;
					mDeter.CalOptAngle(q_inter, camQuat, ruEuler);

					//相机和星敏A
					mBase.QuatInterpolation(sa, camQuat.UTC, q_inter);
					//R = -2.0519748102509858; P = -0.37776605409000874; Y = 1.7599317995056509;//第一组数据
					//R = -2.0519494937534994, P = -0.37781030587897191, Y = 1.7602067474217258;
					mDeter.jl106AlinFix(R, P, Y, q_inter, camQuat, ruEuler);

					//相机和星敏B
					mBase.QuatInterpolation(sb, camQuat.UTC, q_inter);
					//R =-2.4633377318613845;  P = 1.0430306902972233;  Y = 0.73574831167596222;//第一组数据
					//R = -2.4632432222205840, P = 1.0429908761382047, Y = 0.73584449129197604;
					mDeter.jl106AlinFix(R, P, Y, q_inter, camQuat, ruEuler);

					//相机和星敏C
					mBase.QuatInterpolation(sc, camQuat.UTC, q_inter);
					//R = 2.2216920686545469;  P = 0.21132185766620937;  Y = -1.4169616320250062;//第一组数据
				   // R = 2.2216750200474640, P = 0.21135179320464545, Y = -1.4170581238980589;
					mDeter.jl106AlinFix(R, P, Y, q_inter, camQuat, ruEuler);

				}
				attEuler.R = attEuler.R / ptsPath.size();
				attEuler.P = attEuler.P / ptsPath.size();
				attEuler.Y = attEuler.Y / ptsPath.size();

				for (int i = 0; i < ptsPath.size(); i++)
				{
					vector<StarGCP> camGCP;
					mStarid.GetStarGCPforJL106(ptsPath[i], camGCP, imgJL106);//建立相机与恒星控制点
					mDeter.q_MethodforJL106(camGCP, param, camQuat);//根据qMethod来建立相机在J2000下姿态

					//相机和J2000姿态
					Quat q_inter;
					mBase.QuatInterpolation(att, camQuat.UTC, q_inter);
					//attEuler.R = 0.0080471673058139168; attEuler.P = 0.0016345353160141744; attEuler.Y = 1.5319002547936083;//第一组数据
					//double R = 0.0080833780663713863; double P = 0.0016727918023371185; double Y = 1.5318656051912103;//第二组数据
					//double R = 0; double P = 0; double Y = 0;
					mDeter.jl106AlinFix(attEuler.R, attEuler.P, attEuler.Y, q_inter, camQuat, ruEuler);
					fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ruEuler.R, ruEuler.P, ruEuler.Y, ruEuler.R / PI * 180 * 3600, ruEuler.P / PI * 180 * 3600, ruEuler.Y / PI * 180 * 3600, ruEuler.UTC);

					//相机和星敏A
					//mBase.QuatInterpolation(sa, camQuat.UTC, q_inter);
					//R = -2.0519748102509858; P = -0.37776605409000874; Y = 1.7599317995056509;//第一组数据
					//R = -2.0519494937534994, P = -0.37781030587897191, Y = 1.7602067474217258;
					//mDeter.jl106AlinFix(R, P, Y, q_inter, camQuat, ruEuler);
					//fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ruEuler.R, ruEuler.P, ruEuler.Y, ruEuler.R / PI * 180 * 3600, ruEuler.P / PI * 180 * 3600, ruEuler.Y / PI * 180 * 3600, ruEuler.UTC);

					//相机和星敏B
					//mBase.QuatInterpolation(sb, camQuat.UTC, q_inter);
					//R =-2.4633377318613845;  P = 1.0430306902972233;  Y = 0.73574831167596222;//第一组数据
					//R = -2.4632432222205840, P = 1.0429908761382047, Y = 0.73584449129197604;
					//mDeter.jl106AlinFix(R, P, Y, q_inter, camQuat, ruEuler);
					//fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ruEuler.R, ruEuler.P, ruEuler.Y, ruEuler.R / PI * 180 * 3600, ruEuler.P / PI * 180 * 3600, ruEuler.Y / PI * 180 * 3600, ruEuler.UTC);

					//相机和星敏C
					//mBase.QuatInterpolation(sc, camQuat.UTC, q_inter);
					//R = 2.2216920686545469;  P = 0.21132185766620937;  Y = -1.4169616320250062;//第一组数据
					//R = 2.2216750200474640, P = 0.21135179320464545, Y = -1.4170581238980589;
					//mDeter.jl106AlinFix(R, P, Y, q_inter, camQuat, ruEuler);
					//fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ruEuler.R, ruEuler.P, ruEuler.Y, ruEuler.R / PI * 180 * 3600, ruEuler.P / PI * 180 * 3600, ruEuler.Y / PI * 180 * 3600, ruEuler.UTC);

					//fprintf(fp, "\n");
				}
			}
			fclose(fp);
		}
		//;

		//给raw图像加上hdr，方便在ENVI上查看
		//mExtract.addHDRForJiLin(vecRaw);
		//计算吉林一号背景值
		//mExtract.StarCameraBackgroundForJiLin(vecRaw);


	}
	//////////////////////////////////////////////////////////////////////////
	//功能：珞珈一号恒星拍摄处理
	//日期：2019.04.17
	//////////////////////////////////////////////////////////////////////////	
	if (argc == 2 && atoi(argv[1]) == 1)
	{
		ParseSTG LJAtt; StarIdentify LJstar; AttDetermination LJdeter;
		vector<Quat>LJCamera; vector<StarGCP> starGCPLuojia;
		StarCaliParam Param;
		double f = 0.055086;
		double pixel = 11 / 1.e6;
		Param.f = f / pixel; Param.x0 = 1024, Param.y0 = 1024;
		Quat quater; SateEuler ruEuler; YMD imgTime;  Quat imgAtt; string orbPath; Orbit_Ep imgOrb;

		int cali = 1;
		if (cali == 0)
		{
			vector<Orbit_Ep>vecOrb;
			LJAtt.workpath = "C:\\Users\\wcsgz\\Downloads\\珞珈0级产品\\328\\";
			LJdeter.workpath = LJAtt.workpath;
			orbPath = LJAtt.workpath + "LuoJia1-01_LR201904012992.orb";
			LJAtt.CalcLuojiaCamOpt(LJCamera);
			LJstar.GetStarGCPForLuojia(LJCamera, starGCPLuojia);
			LJAtt.ReadLuojiaAllOrb(orbPath, vecOrb);
			LJdeter.q_MethodForLuojia(starGCPLuojia, Param, quater, vecOrb);
			LJdeter.CalcXYaccuracy(starGCPLuojia, quater, vecOrb);
			LJdeter.CalcStarExtractAccuracy(starGCPLuojia);
			LJdeter.luojiaAlinFix(LJCamera, quater, ruEuler);
		}


		for (int aa = 0; aa < 5; aa++)
		{
			aa = 5;
			if (aa == 0)
			{ //53
				LJAtt.workpath = "C:\\Users\\wcsgz\\Downloads\\珞珈0级产品\\53\\";
				orbPath = LJAtt.workpath + "LuoJia1-01_LR201806284987.orb";
				imgTime.year = 2018; imgTime.mon = 06; imgTime.day = 27;
				imgTime.hour = 23; imgTime.min = 43; imgTime.sec = 24.152792;
			}
			else if (aa == 1)
			{//237
				LJAtt.workpath = "C:\\Users\\wcsgz\\Downloads\\珞珈0级产品\\237\\";
				orbPath = LJAtt.workpath + "LuoJia1-01_LR201811259738.orb";
				imgTime.year = 2018; imgTime.mon = 11; imgTime.day = 23;
				imgTime.hour = 22; imgTime.min = 45; imgTime.sec = 5.287655;
			}
			else if (aa == 2)
			{//315
				LJAtt.workpath = "C:\\Users\\wcsgz\\Downloads\\珞珈0级产品\\315\\";
				orbPath = LJAtt.workpath + "LuoJia1-01_LR201903232988.orb";
				imgTime.year = 2019; imgTime.mon = 03; imgTime.day = 22;
				imgTime.hour = 17; imgTime.min = 40; imgTime.sec = 19.187868;
			}
			else if (aa == 3)
			{//21
				LJAtt.workpath = "C:\\Users\\wcsgz\\Downloads\\珞珈0级产品\\21\\";
				orbPath = LJAtt.workpath + "LuoJia1-01_LR201806109671.orb";
				imgTime.year = 2018; imgTime.mon = 06; imgTime.day = 10;
				imgTime.hour = 8; imgTime.min = 4; imgTime.sec = 10.652282;
			}
			else if (aa == 4)
			{//21
				LJAtt.workpath = "C:\\Users\\wcsgz\\Downloads\\珞珈0级产品\\328\\";
				orbPath = LJAtt.workpath + "LuoJia1-01_LR201904012992.orb";
				imgTime.year = 2019; imgTime.mon = 04; imgTime.day = 1;
				imgTime.hour = 3; imgTime.min = 33; imgTime.sec = 15.696408;
			}
			else if (aa == 5)
			{//521
				LJAtt.workpath = "C:\\Users\\wcsgz\\Downloads\\珞珈0级产品\\521\\";
				orbPath = LJAtt.workpath + "LuoJia1-01_LR000000000000.orb";
				imgTime.year = 2019; imgTime.mon = 05; imgTime.day = 21;
				imgTime.hour = 15; imgTime.min = 3; imgTime.sec = 0.148440;
			}
			LJAtt.CalcLuojiaCamOpt(LJCamera);
			LJAtt.ReadLuojiaAtt(LJCamera, imgTime, imgAtt);
			LJAtt.ReadLuojiaOrb(orbPath, imgTime, imgOrb);
			LJAtt.GetEuler(imgTime, imgOrb);
			LJAtt.MoonDirectionForLuojia(imgOrb, imgAtt, imgTime);
		}





		LJAtt.StarMapForLuojia(LJCamera);
		StarExtract Luojia01;
		Luojia01.StarCameraBackground(0, 9);
		Luojia01.StarCameraBackground(20, 29);
		Luojia01.StarCameraBackground(50, 59);
		Luojia01.StarCameraBackground(100, 109);
		return 0;
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：对星图数据中的APS进行姿态确定
	//日期：2017.11.29
	//////////////////////////////////////////////////////////////////////////	
	if (argc == 4 && atoi(argv[1]) == 1)
	{
		APScalibration ZY3_calibrate;
		StarCaliParam ZY302CaliParam;
		//ZY302CaliParam.f = 2885.019;
		//ZY302CaliParam.x0 = 526.878;
		//ZY302CaliParam.y0 = 534.552;
		//ZY302CaliParam.k1 = -1.066e-08;
		//ZY302CaliParam.k2 = 8e-15;
		ZY302CaliParam.f = 43.3 / 0.015;
		ZY302CaliParam.x0 = 512 - 10;
		ZY302CaliParam.y0 = 512 - 17;
		//ZY302CaliParam.k1 = 0;
		//ZY302CaliParam.k2 = 0;
		//ZY302CaliParam.f = 43.3 / 0.015*2-2885.019;
		//ZY302CaliParam.x0 = 1024 - 526.878;
		//ZY302CaliParam.y0 = 1024 - 534.552;
		ZY302CaliParam.k1 = -1.066e-08;
		ZY302CaliParam.k2 = 8e-15;
		ZY3_calibrate.workpath = argv[2];
		vector<vector<StarGCP>>getGCPall;
		ZY3_calibrate.ReadAllGCP(getGCPall);

		ParseSTG ZY3_STG;
		vector<STGData>stg;
		string stgPath = argv[3];
		ZY3_STG.ParseZY302_STG(stgPath, stg);
		string sOrb = stgPath.substr(0, stgPath.rfind('.') + 1) + "EPH";
		vector<Orbit_Ep>arr_Orb;
		ZY3_STG.ReadZY302OrbTXT2(sOrb, arr_Orb);

		AttDetermination ZY3_AD;
		ZY3_AD.workpath = stgPath.substr(0, stgPath.rfind('\\') + 1);
		vector<vector<BmImStar>>BmIm;
		ZY3_AD.GetImBm(getGCPall, ZY302CaliParam, BmIm);
		ZY3_AD.Aberration(BmIm, arr_Orb);
		ZY3_AD.EKF6StateForStarMap(BmIm, stg);//根据星图进行定姿		
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：添加各种对比
	//日期：2017.12.05
	//////////////////////////////////////////////////////////////////////////	
	else if (argc == 3 && atoi(argv[2]) == 11)
	{
		AttDetermination ZY3_AD;
		ZY3_AD.workpath = argv[1];
		ZY3_AD.compareAPSandStarMap();//对比星图和STG解析数据定姿
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：对STG数据中的APS进行姿态确定
	//日期：2017.11.29
	//////////////////////////////////////////////////////////////////////////	
	else if (atoi(argv[2]) == 10)
	{
		ParseSTG ZY3_STGSTI;
		AttDetermination ZY3_AD;
		vector<STGData> ZY3_02STGdata;
		ZY3_STGSTI.ParseZY302_STG(argv[1], ZY3_02STGdata);
		ZY3_AD.workpath = argv[1];
		ZY3_AD.AttDeter4(ZY3_02STGdata);
	}
	//////////////////////////////////////////////////////////////////////////
	//功能：对星敏定标
	//日期：2017.11.29
	//////////////////////////////////////////////////////////////////////////	
	else if (atoi(argv[2]) == 11)
	{
		ParseSTG ZY3_StarPoint;
		vector<vector<StarGCP>>getGCPall;
		ZY3_StarPoint.ParseZY302_SoftStarData(string(argv[1]) + "StarData.txt", getGCPall);
		APScalibration ZY3_calibrate;
		ZY3_calibrate.workpath = argv[1];
		//ZY3_calibrate.Calibrate5ParamKalman(getGCPall);
		ZY3_calibrate.Calibrate5ParamMultiImg(getGCPall, 1);
	}
	else if (argc == 4 && atoi(argv[1]) == 2)
	{
		//////////////////////////////////////////////////////////////////////////
		//功能：得到所有星点坐标
		//日期：2017.12.04
		//////////////////////////////////////////////////////////////////////////
		ParseSTG ZY3_STGSTI;
		StarExtract ZY3_STMap;
		StarIdentify ZY3_ST;
		APScalibration ZY3_calibrate;
		//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\星图\\";
		string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
		//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\星图\\";
		//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\星图\\";
		ZY3_STMap.workpath = workpath_ST;
		ZY3_ST.workpath = ZY3_STMap.workpath;
		ZY3_calibrate.workpath = ZY3_STMap.workpath;
		//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\0702.STG";
		string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\0707.STG";
		//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\0712.STG";
		//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\0830.STG";
		vector<STGData> ZY3_02STGdata;
		ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata);
		vector<vector<StarGCP>>getGCPall;
		vector<StarGCP > getGCPalltmp(0);
		int index;
		const int nImg = 1090;
		for (index = 5; index < nImg; )
		{
			printf("\r开始生成控制点，累加%d景...", index);
			ZY3_STMap.StarPointExtraction(index);
			//ZY3_ST.GetStarGCP0702(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			ZY3_ST.GetStarGCP0707(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			//ZY3_ST.GetStarGCP0712(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			//ZY3_ST.GetStarGCP(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 100, index);//精化控制点
			getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
			getGCPall.push_back(getGCPalltmp);//累加控制点
			ZY3_STMap.StarPointExtract.clear();//记得清除vector里的内容，不然内容会叠加
			ZY3_ST.getGCP.clear();
			index = index++;//输出间隔
		}
		ZY3_calibrate.OutputAllGCP(getGCPall);
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
		double sigu = sqrt(10) * 1e-10;
		double sigv = sqrt(10) * 1e-7;
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
		double qInitial[] = { 1. / sqrt(3), 0, 1. / sqrt(3), 1. / sqrt(3) };
		//double qInitial[] = { 1. / sqrt(9), 1. / sqrt(9./4), 1. / sqrt(9. / 2), 1. / sqrt(9./2) };
		double alinAB[] = { 50. / 180 * PI,75. / 180 * PI,60. / 180 * PI };
		double alinN = 0.1 / 180 * PI;
		double FOV = 5. / 180 * PI;
		while (true)
		{
			//attSim1.alinSimulation(dt, m, alinN);
			attSim1.wahbaSimulation(dt, m, FOV, alinN);
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
		double alinN = 0.003 / 180 * PI;
		double wBiasA[] = { 0.5,0.1,-0.1 };
		double alin[] = { 0.03 / 180 * PI,0.02 / 180 * PI,-0.04 / 180 * PI };
		double sigu = sqrt(10) * 1e-10;
		double sigv = sqrt(10) * 1e-7;
		Quat* starB = new Quat[m];
		Quat* starC = new Quat[m];
		Quat* starTrue = new Quat[m];
		Gyro* wMeas = new Gyro[m];
		attSim1.twoStarSimulation(dt, m, alinN, alin, wBiasA, sigu, sigv, starB, starC, starTrue, wMeas);
		attDeter1.EKF6StateForStarAB3(m, starB, starC, starTrue, wMeas);
		delete[] starB, starC, wMeas; starB = starC = NULL;
	}
	else if (atoi(argv[2]) == 100)
	{
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
		ParseSTG ZY3_STGSTI;
		StarExtract ZY3_STMap;
		StarIdentify ZY3_ST;
		APScalibration ZY3_calibrate;
		ZY3_calibrate.ZY302CaliParam.f = 2879;
		ZY3_calibrate.ZY302CaliParam.x0 = 512 + 20;
		ZY3_calibrate.ZY302CaliParam.y0 = 512 - 10;
		ZY3_calibrate.ZY302CaliParam.k1 = -1e-8;
		ZY3_calibrate.ZY302CaliParam.k2 = 8e-15;
		//ZY3_calibrate.ZY302CaliParam.k1 = 0;
		//ZY3_calibrate.ZY302CaliParam.k2 = 0;
		//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\星图\\";
		string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
		//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\星图\\";
		//string workpath_ST = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\星图\\";
		ZY3_STMap.workpath = workpath_ST;
		ZY3_ST.workpath = ZY3_STMap.workpath;
		ZY3_calibrate.workpath = ZY3_STMap.workpath;
		//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0702\\0702.STG";
		string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\0707.STG";
		//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0712\\0712.STG";
		//string argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\0830.STG";
		vector<STGData> ZY3_02STGdata;
		ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata);
		vector<vector<StarGCP>>getGCPall;
		vector<StarGCP > getGCPalltmp(0);
		int index;
		const int nImg = 1000;
		for (index = 5; index < nImg; )
		{
			printf("\r开始生成控制点，累加%d景...", index);
			ZY3_STMap.StarPointExtraction(index);
			//ZY3_ST.GetStarGCP0702(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			ZY3_ST.GetStarGCP0707(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			//ZY3_ST.GetStarGCP0712(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			//ZY3_ST.GetStarGCP(ZY3_02STGdata, ZY3_STMap.StarPointExtract, index);
			ZY3_calibrate.OptimizeGCP(ZY3_ST.getGCP, 100, index);//精化控制点
			getGCPalltmp.assign(ZY3_ST.getGCP.begin(), ZY3_ST.getGCP.end());
			getGCPall.push_back(getGCPalltmp);//累加控制点
			ZY3_STMap.StarPointExtract.clear();//记得清除vector里的内容，不然内容会叠加
			ZY3_ST.getGCP.clear();
			index = index++;//输出间隔
		}
		//星点按实际情况分布的控制点仿真
		//ZY3_calibrate.SimulateGCP_PreRand5Param(getGCPall, ZY3_02STGdata, nImg); 
		//星点随机分布的控制点仿真
		//ZY3_calibrate.SimulateGCP_RandomXY5Param(nImg, getGCPall);
		printf("\n开始定标\n");
		//Kalman Filter定标
		//ZY3_calibrate.Calibrate3ParamKalman(getGCPall);
		ZY3_calibrate.Calibrate5ParamKalman(getGCPall);
		ZY3_calibrate.OutputAllGCP(getGCPall);
		getGCPall.clear();
		ZY3_calibrate.ReadAllGCP(getGCPall);

		////////////////////////////////////////////////////////////////////////////
		////功能：接上一步，为姿态确定过程
		////日期：2017.04.27
		////////////////////////////////////////////////////////////////////////////	
		AttDetermination ZY3_AD;
		ZY3_AD.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\姿态确定\\";
		ZY3_AD.AttDeter2(ZY3_02STGdata, getGCPall);

		vector<vector<StarGCP>>getGCPaccu;
		for (index = 0; index < getGCPall.size(); )
		{
			//ZY3_calibrate.Calibrate3Param(getGCPall[index], index);
			//ZY3_calibrate.Calibrate5Param(getGCPall[index], index);
			//getGCPaccu.assign(getGCPall[index].begin(), getGCPall[index].end());
			getGCPaccu.assign(getGCPall.begin(), getGCPall.begin() + index + 1);
			printf("\r定标中，累加%d景...", index + 1);
			ZY3_calibrate.Calibrate5ParamMultiImg(getGCPaccu, index + 1);
			//ZY3_calibrate.Calibrate3ParamMultiImg(getGCPaccu, index + 1);
			//ZY3_calibrate.CalibrateOpticAxisMultiImg(getGCPaccu, index + 1);
			//
			index++;//输出间隔
		}

		//////////////////////////////////////////////////////////////////////////
		//功能：不定标，直接用原始姿态定位
		//日期：2017.04.27
		//////////////////////////////////////////////////////////////////////////	


		argv1 = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\0707.STG";
		//vector<STGData> ZY3_02STGdata;
		ZY3_STGSTI.ParseZY302_STG(argv1, ZY3_02STGdata);
		ZY3_calibrate.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\星图\\";
		//vector<vector<StarGCP>>getGCPall;
		ZY3_calibrate.ReadAllGCP(getGCPall);
		ZY3_AD.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\姿态确定\\";
		ZY3_AD.AttDeter2(ZY3_02STGdata, getGCPall);
	}





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
	//PlaySound(TEXT("C:\\WINDOWS\\Media\\Alarm01.wav"), NULL, SND_FILENAME);
	return 0;
}