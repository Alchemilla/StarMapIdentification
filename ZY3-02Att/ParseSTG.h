#ifndef PARSESTG_H
#define PARSESTG_H
#include <string>
#include <fstream>
#include <iostream>
#include<iomanip>
#include <direct.h>//创建文件夹用
#include "SateBase.h"
#include "BaseFunc.h"
#include "GeoReadImage.h"
#include "Simulation.h"
struct Star
{
	int ID,DN;//恒星编号和亮度换算DN值
	double phiX,phiY,mag;//赤经赤纬和亮度
	double V[3];//J2000系下的XYZ坐标
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
	//读取解析STG后的数据STA
	void ReadSTAtxt(string STApath,vector<STGData>&STAdat);
	int Mag2DN(double Mag);
	string workpath;
private:
	Simulation Sim;
};
#endif