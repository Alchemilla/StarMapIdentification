#ifndef PARSESTG_H
#define PARSESTG_H
#include<stdio.h>
#include <string>
#include <fstream>
#include<iomanip>
#include <direct.h>//�����ļ�����
#include "BaseFunc.h"
#include "GeoReadImage.h"
#include "SateBase.h"
#include <bitset>
#include "DateTime.h"

class ParseSTG
{
public:
	BaseFunc mBase;
	//�������ֽ���
	void ParseZY302_STG(string STGpath,vector<STGData> &ZY3_02STGdata);
	bool ParseZY302_STI(string STIpath);
	bool ParseZY302_STI_10B(string STIpath);
	bool ParseZY302_STItime(string STIpath);
	//��ȡ��ʱ�ļ�
	void ReadZY302_STItime(double *UTC);
	//����STI�õ��ģ���5�ֽ�40bitתΪ4��10bit��arr
	void Get4_10Bit(const unsigned char *p, unsigned short(&arr)[4]);
	//������ͼ
	void StarMap(vector<STGData> ZY3_02STGdata);
	//
	void FromLL2XY(Star starCatlog, double *R, double &x, double &y);
	void FromLL2XY(double *W, StarCaliParam Param, double *x, double *y);
	void FromXY2LL(double x, double y, StarCaliParam Param, double *V);
	//void StarIDComp(vector<STGData> ZY3_02STGdata,string IDpath);
	void StarAngleAPS_B_C(vector<STGData> ZY3_02STGdata,int StarTag);
	//�����нǶԱ�
	void StarAngle(vector<STGData> StarDat, int StarTag);
	void ReadStarID(string IDpath,vector<STGData>&APSQ);
	//��ȡ����STG�������STA
	void ReadSTAtxt(string STApath,vector<STGData>&STAdat);
	int Mag2DN(double Mag);
	string workpath;
private:
};
#endif