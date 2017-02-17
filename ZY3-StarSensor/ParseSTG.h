#ifndef PARSESTG_H
#define PARSESTG_H
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
	bool ParseZY302_STItime(string STIpath);
	//仿真星图
	void StarMap(vector<STGData> ZY3_02STGdata);
	//
	void FromLL2XY(Star starCatlog, double *R, double &x, double &y);
	void StarIDComp(vector<STGData> ZY3_02STGdata,string IDpath);
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