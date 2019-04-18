#ifndef PARSESTG_H
#define PARSESTG_H
#include<stdio.h>
#include <string>
#include <fstream>
#include<iomanip>
#include <direct.h>//创建文件夹用
#include "BaseFunc.h"
#include "GeoReadImage.h"
#include "SateBase.h"
#include <bitset>
#include "DateTime.h"

class ParseSTG
{
public:
	BaseFunc mBase;
	//星敏各种解析
	void ParseZY302_STG(string STGpath,vector<STGData> &ZY3_02STGdata);
	bool ParseZY302_STI(string STIpath);
	bool ParseZY302_STI_10B(string STIpath);
	bool ParseZY302_STItime(string STIpath);
	bool ParseZY302_SoftStarData(string StarDataPath,vector<vector<StarGCP>>&StarData);
	//轨道数据解析
	bool ReadZY302OrbTXT2(string sOrb, vector<Orbit_Ep> &arr_Orb);
	//读取星时文件
	void ReadZY302_STItime(double *UTC);
	//解析STI用到的，将5字节40bit转为4个10bit的arr
	void Get4_10Bit(const unsigned char *p, unsigned short(&arr)[4]);
	//仿真星图
	void StarMap(vector<STGData> ZY3_02STGdata);
	void StarMapForLuojia(vector<Quat>LuojiaCam);//仿真珞珈一号恒星拍摄
	void CalcLuojiaCamOpt(vector<Quat>LuojiaCam);//计算珞珈光轴指向
	//
	void FromLL2XY(Star starCatlog, double *R, double &x, double &y);
	void FromLL2XY(double *W, StarCaliParam Param, double *x, double *y);
	void FromXY2LL(double x, double y, StarCaliParam Param, double *V);
	//void StarIDComp(vector<STGData> ZY3_02STGdata,string IDpath);
	void StarAngleAPS_B_C(vector<STGData> ZY3_02STGdata,int StarTag);
	//星敏夹角对比
	void StarAngle(vector<STGData> StarDat, int StarTag);
	void ReadStarID(string IDpath,vector<STGData>&APSQ);
	//读取解析STG后的数据STA
	void ReadSTAtxt(string STApath,vector<STGData>&STAdat);
	int Mag2DN(double Mag);
	string workpath;
private:
};
#endif