#pragma once
#include "GeoDefine.h"
#include "GeoBase.h"
#include "tinyxml.h"
#include <windows.h>
#include<stdlib.h>
class ParseZY3Aux
{
public:
	ParseZY3Aux(void);
	~ParseZY3Aux(void);
	GeoBase mBase;
	//����01�ǽ���
	bool ReadZY301AttTXT(string sAtt,vector<Attitude> &arr_att);
	bool ReadZY301OrbTXT(string sOrb,vector<Orbit_Ep> &arr_Orb);
	bool ReadZY301TimeTXT(string sTime,string sInput,vector<LineScanTime> &arr_Time);
	void ZY301PATH(string auxpath, string &sAtt, string &sOrb, string &sTime, string &sInput);
	//����02�ǽ���
	bool getZY302Aux(string sAux, vector<Orbit_Ep>& allEp, vector<Attitude>& allAtt, vector<LineScanTime>& allTime);
	bool parseZY302FrameData(unsigned char*pData, FrameData_ZY3& perFrame);
	bool ReadZY302AttTXT(string sAtt, vector<Attitude> &arr_att);
	bool ReadZY302OrbTXT(string sOrb, vector<Orbit_Ep> &arr_Orb);
	bool ReadZY302OrbTXT2(string sOrb, vector<Orbit_Ep> &arr_Orb);
	bool ReadZY302TimeTXT(string sTime, vector<LineScanTime> &arr_Time);
	void ZY302PATH(string auxpath, string &sAtt, string &sOrb, string &sTime);
	//�����������ݽ���
	bool ReadZY3SimTimeTXT(string sTime, vector<LineScanTime> &arr_Time);
	bool ReadLittleCameraTimeTXT(string sTime, vector<LineScanTime>& arr_Time);
	void ZY3SimPATH(string auxpath, string &sAtt, string &sOrb, string &sTime,string &sCam);
	void ZY3RealPATH(string auxpath, string &sAtt, string &sOrb, string &sTime, string &sCam);
	//Ѱ��Ŀ¼��ĳһ��׺�����ļ�
	void search_directory(const string& caseFileName, char *File_ext, vector<string>& ResPath);
};

