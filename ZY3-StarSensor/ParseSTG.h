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
#include "EOP_files.h"
#include "tinyxml.h"
#include <math.h>
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
	bool ReadLuojiaAtt(vector<Quat>LuojiaCam, YMD imgTime, Quat &imgAtt);
	//轨道数据解析
	bool ReadZY302OrbTXT2(string sOrb, vector<Orbit_Ep> &arr_Orb);
	bool ReadLuojiaOrb(string orbPath, YMD imgTime, Orbit_Ep &imgOrb);
	bool ReadLuojiaAllOrb(string orbPath, vector<Orbit_Ep> &imgOrb);
	//求取欧拉角
	bool GetEuler(YMD imgTime, Orbit_Ep imgOrb);
	//读取星时文件
	void ReadZY302_STItime(double *UTC);
	//读取吉林一号csv数据
	void ReadJL106csv(string csv, vector<img>& imgJL106, vector<Quat>& att, vector<Quat>& sa, vector<Quat>& sb, vector<Quat>& sc);
	void ReadJL107csv(string csv, vector<img>& imgJL106, vector<Quat>& att, vector<Quat>& sa, vector<Quat>& sb, vector<Quat>& sc);
	void ParseSTG::ReadJL107csvOrb(string csv, vector<img>& imgJL107,
		vector<Quat>& att, vector<Quat>& sa, vector<Quat>& sb, vector<Quat>& sc, vector<Orbit_Ep>&imgOrb);
	//解析STI用到的，将5字节40bit转为4个10bit的arr
	void Get4_10Bit(const unsigned char *p, unsigned short(&arr)[4]);
	//仿真星图
	void StarMap(vector<STGData> ZY3_02STGdata);
	void StarMapForLuojia(vector<Quat>LuojiaCam);//仿真珞珈一号恒星拍摄
	void StarMapForJL01(string txtPath);//仿真吉林一号恒星拍摄
	void StarMapForJL01(string outPath, vector<Quat>jlCam);
	void MoonDirectionForLuojia(Orbit_Ep imgOrb, Quat imgAtt, YMD imgTime);//计算月亮指向
	void CalcLuojiaCamOpt(vector<Quat>&LuojiaCam);//计算珞珈光轴指向
	void FromLL2XYForLuojia(Star starCatlog, double *R, double &x, double &y);
	void FromLL2XYForLuojia(double *starCatlog, double *R, double &x, double &y);
	void FromLL2XYForJL01(Star starCatlog, double* R, double& x, double& y);//计算吉林一号恒星位置

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