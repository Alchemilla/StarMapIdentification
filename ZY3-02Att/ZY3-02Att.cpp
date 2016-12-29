
#include "stdafx.h"

#include "ParseOpticalAux.h"
#include "AttDeter.h"
//#include "Simulation.h"
#include "ParseSTG.h"

int main(int argc, char* argv[])
{
	//////////////
	//解析辅助数据
	//////////////
	vector<Orbit_Ep> allEp;
	vector<Attitude> allAtt;
	vector<LineScanTime> allTime;
	int startLine,endLine;
	//getZY302Aux(auxpath,allEp,allAtt,allTime,sDir);
	//ReadZY3_01OrbTXT(argv[1],a+"\\";
	//测FAN的数据
	//WMData(argv[1]);

	//////////////
	//解析星敏原始STG数据
	//////////////
	ParseSTG ZY3_Star;
	AttDeter ZY3_Deter;
	string auxpath = argv[2];
	ZY3_Star.workpath =  auxpath.substr(0,auxpath.rfind('\\'))+"\\";
	vector<STGData> ZY3_02STGdata;
	//ZY3_Star.ParseZY302_STG(argv[2],ZY3_02STGdata);
	ZY3_Star.ReadSTAtxt(argv[1],ZY3_02STGdata);

	//////////////
	//计算星敏之间的夹角
	//////////////
	ZY3_Deter.StarAngle(ZY3_02STGdata,argv[1],23);//计算STG中星敏的夹角,23表示星敏2和3
	ZY3_Deter.StarAngle(ZY3_02STGdata,argv[1],12);
	ZY3_Deter.StarAngle(ZY3_02STGdata,argv[1],13);
	//ZY3_Deter.Aberration(ZY3_02STGdata,allEp,argv[2],12);
	//ZY3_Deter.Aberration(ZY3_02STGdata,allEp,argv[2],13);

	//////////////
	//解析STG并做滤波处理
	//////////////
	//AttDeter ZY302;
	//ParseSTG ZY3STG;
	//vector<STGData>	AttData;
	//vector<Quat>AttDet;
	//ZY3STG.ParseZY302_STG(argv[1],AttData);
	////ZY302.EKF6StateV2(AttData,AttDet,argv[1],23);
	//ZY302.SingleStar(AttData,1,AttDet);//对APS星直接定姿
	//ZY302.QuatForZY3JJ(AttDet,argv[1]);

	//////////////
	//对星图识别四元数进行处理
	//////////////
	/*ParseSTG ZY3;
	AttDeter ZY3deter;
	vector<STGData>	AttData;
	vector<Quat> AttDet;
	ZY3.ReadStarID(argv[1],AttData);
	ZY3deter.SingleStar(AttData,1,AttDet);
	ZY3deter.QuatForZY3JJ(AttDet,argv[1]);*/

	//////////////
	//星图处理,对比星上星下处理结果
	//////////////
	//ZY3_Star.StarMap(ZY3_02STGdata);
	//定姿软件星图识别结果与STG数据比较
	//ZY3_Star.StarIDComp(ZY3_02STGdata,argv[2]);
	//定姿软件星图识别结果和星敏B星敏C相互之间夹角关系
	//ZY3_Star.StarAngleAPS_B_C(ZY3_02STGdata,argv[1],12);
	//ZY3_Star.StarAngleAPS_B_C(ZY3_02STGdata,argv[1],13);

	//////////////
	//姿态仿真与画图
	//////////////
	/*vector<STGData> ZY3_02STGdata;
	AttDeter ZY3_02Att;
	ZY3_02Att.AttSolution(ZY3_02STGdata,argv[2]);
	Simulation ZY3_02Sim;*/
	//ZY3_02Sim.SimStar();
	//MatlabExample();


	return 0;
}

