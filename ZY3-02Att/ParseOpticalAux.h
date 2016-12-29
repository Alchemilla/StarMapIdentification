#pragma once
#include "SateBase.h"


bool getZY302Aux(string sAux,vector<Orbit_Ep>& allEp,vector<Attitude>& allAtt,vector<LineScanTime>& allTime,string sDir);
bool parseZY302FrameData(unsigned char*pData,FrameData_ZY302& perFrame);
bool ReadZY3_01AttTXT(string sAtt,vector<Attitude> &arr_att);
bool ReadZY3_01OrbTXT(string sOrb,vector<Orbit_Ep> &arr_Orb);