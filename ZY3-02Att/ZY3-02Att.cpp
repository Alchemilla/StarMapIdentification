
#include "stdafx.h"

#include "ParseOpticalAux.h"
#include "AttDeter.h"
//#include "Simulation.h"
#include "ParseSTG.h"

int main(int argc, char* argv[])
{
	//////////////
	//������������
	//////////////
	vector<Orbit_Ep> allEp;
	vector<Attitude> allAtt;
	vector<LineScanTime> allTime;
	int startLine,endLine;
	//getZY302Aux(auxpath,allEp,allAtt,allTime,sDir);
	//ReadZY3_01OrbTXT(argv[1],a+"\\";
	//��FAN������
	//WMData(argv[1]);

	//////////////
	//��������ԭʼSTG����
	//////////////
	ParseSTG ZY3_Star;
	AttDeter ZY3_Deter;
	string auxpath = argv[2];
	ZY3_Star.workpath =  auxpath.substr(0,auxpath.rfind('\\'))+"\\";
	vector<STGData> ZY3_02STGdata;
	//ZY3_Star.ParseZY302_STG(argv[2],ZY3_02STGdata);
	ZY3_Star.ReadSTAtxt(argv[1],ZY3_02STGdata);

	//////////////
	//��������֮��ļн�
	//////////////
	ZY3_Deter.StarAngle(ZY3_02STGdata,argv[1],23);//����STG�������ļн�,23��ʾ����2��3
	ZY3_Deter.StarAngle(ZY3_02STGdata,argv[1],12);
	ZY3_Deter.StarAngle(ZY3_02STGdata,argv[1],13);
	//ZY3_Deter.Aberration(ZY3_02STGdata,allEp,argv[2],12);
	//ZY3_Deter.Aberration(ZY3_02STGdata,allEp,argv[2],13);

	//////////////
	//����STG�����˲�����
	//////////////
	//AttDeter ZY302;
	//ParseSTG ZY3STG;
	//vector<STGData>	AttData;
	//vector<Quat>AttDet;
	//ZY3STG.ParseZY302_STG(argv[1],AttData);
	////ZY302.EKF6StateV2(AttData,AttDet,argv[1],23);
	//ZY302.SingleStar(AttData,1,AttDet);//��APS��ֱ�Ӷ���
	//ZY302.QuatForZY3JJ(AttDet,argv[1]);

	//////////////
	//����ͼʶ����Ԫ�����д���
	//////////////
	/*ParseSTG ZY3;
	AttDeter ZY3deter;
	vector<STGData>	AttData;
	vector<Quat> AttDet;
	ZY3.ReadStarID(argv[1],AttData);
	ZY3deter.SingleStar(AttData,1,AttDet);
	ZY3deter.QuatForZY3JJ(AttDet,argv[1]);*/

	//////////////
	//��ͼ����,�Ա��������´�����
	//////////////
	//ZY3_Star.StarMap(ZY3_02STGdata);
	//���������ͼʶ������STG���ݱȽ�
	//ZY3_Star.StarIDComp(ZY3_02STGdata,argv[2]);
	//���������ͼʶ����������B����C�໥֮��нǹ�ϵ
	//ZY3_Star.StarAngleAPS_B_C(ZY3_02STGdata,argv[1],12);
	//ZY3_Star.StarAngleAPS_B_C(ZY3_02STGdata,argv[1],13);

	//////////////
	//��̬�����뻭ͼ
	//////////////
	/*vector<STGData> ZY3_02STGdata;
	AttDeter ZY3_02Att;
	ZY3_02Att.AttSolution(ZY3_02STGdata,argv[2]);
	Simulation ZY3_02Sim;*/
	//ZY3_02Sim.SimStar();
	//MatlabExample();


	return 0;
}

