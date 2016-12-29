#ifndef PARSESTG_H
#define PARSESTG_H
#include <string>
#include <fstream>
#include <iostream>
#include<iomanip>
#include <direct.h>//�����ļ�����
#include "SateBase.h"
#include "BaseFunc.h"
#include "GeoReadImage.h"
#include "Simulation.h"
struct Star
{
	int ID,DN;//���Ǳ�ź����Ȼ���DNֵ
	double phiX,phiY,mag;//�ྭ��γ������
	double V[3];//J2000ϵ�µ�XYZ����
};
class ParseSTG
{
public:
	void ParseZY302_STG(string STGpath,vector<STGData> &ZY3_02STGdata);
	void ParseZY302_STI();
	void StarMap(vector<STGData> ZY3_02STGdata);
	void FromLL2XY(Star starCatlog, double *R, double &x, double &y);
	void StarIDComp(vector<STGData> ZY3_02STGdata,string IDpath);
	void StarAngleAPS_B_C(vector<STGData> ZY3_02STGdata,string IDpath,int StarTag);
	void ReadStarID(string IDpath,vector<STGData>&APSQ);
	//��ȡ����STG�������STA
	void ReadSTAtxt(string STApath,vector<STGData>&STAdat);
	int Mag2DN(double Mag);
	string workpath;
private:
	Simulation Sim;
};
#endif