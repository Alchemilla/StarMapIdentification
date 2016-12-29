#ifndef ATTDETER_H
#define ATTDETER_H
#include "BaseFunc.h"
#include "Markup.h"
#include "EOP_files.h"
#include "DateTime.h"
#include "ParseSTG.h"
#include "Simulation.h"
class AttDeter
{
public:
	AttDeter(void	);
	~AttDeter(void);
	void StarAngle(vector<STGData> StarDat,string Res,int StarTag);//����STG�������ļн�
	void AttSolution(vector<STGData> AttData,string Res);
	//����Ԫ����ʽ��ΪJYH��У�����Ҫ����̬��ʽ
	void QuatForZY3JJ(vector<Quat> AttDet,string Res);
	bool WMData(string AttData);	
	void EKF6StateV2(vector<STGData> AttData,vector<Quat> &AttDet,string Res,int StarTag);
	bool SingleStar(vector<STGData> AttData,int StarTag,vector<Quat> &AttDet);
	bool Aberration(vector<STGData> StarDat,vector<Orbit_Ep> EpDat, string Res,int StarTag);
protected:
	void DoubleStar(vector<STGData> StarDat,vector<Quat> &AttDet,string Res,int StarTag);
	void GyroAtt(vector<STGData> AttData,vector<Quat> &AttDet,string Res,int StarTag);
	void AttdeterCompareAOCC(vector<Quat> StarDat,string AOCC);
	void AttdeterCompare502(vector<Quat> StarDat,string PAD);
	void AttdeterCompareAOCCwith502(string AOCC,string PAD);
private:
	Simulation Sim;
	static double Ainstall[9], Binstall[9], Cinstall[9], GyroIns[9];
};
#endif