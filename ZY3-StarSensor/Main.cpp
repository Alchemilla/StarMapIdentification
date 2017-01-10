#include "StarIdentify.h"
#include "StarExtract.h"
#include "ParseSTG.h"

int main(int argc, char* argv[])
{
	//////////////////////////////////////////////////////////////////////////
	//功能：以下为读取STG数据流程
	//日期：2017.01.07
	//////////////////////////////////////////////////////////////////////////
	ParseSTG ZY3_STGSTI;
	vector<STGData> ZY3_02STGdata;
	ZY3_STGSTI.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\";
	ZY3_STGSTI.ParseZY302_STG(argv[1], ZY3_02STGdata);
	//ZY3_STGSTI.ParseZY302_STI(argv[2]);
	//ZY3_STGSTI.StarMap(ZY3_02STGdata);
	ZY3_STGSTI.StarAngle(ZY3_02STGdata, 12);
	ZY3_STGSTI.StarAngle(ZY3_02STGdata, 13);
	ZY3_STGSTI.StarAngle(ZY3_02STGdata, 23);

	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星点提取流程
	//日期：2017.01.04
	//////////////////////////////////////////////////////////////////////////
	StarExtract ZY3_STMap;
	ZY3_STMap.workpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\0830\\星图\\";
	//ZY3_STMap.StarPointExtraction();
	
	//////////////////////////////////////////////////////////////////////////
	//功能：以下为星图识别流程
	//日期：2016.12.25
	//////////////////////////////////////////////////////////////////////////
	StarIdentify ZY3_ST;
	/*double array[] = { -0.00414330123660008, -0.0811396492167516, 0.996694130804353, 3.70000000000000, 10366, -0.531404221740299, -0.804297837296644, 0.265922060078752,	3.62000000000000,
		0.166822334948948, -0.160419053405453,	0.972849441520464,	3.90000000000000,	10330, -0.505400115551280, -0.740484871360861,	0.443004377502573,	3.83000000000000,
		-0.0213161783700601, -0.120677074320824,	0.992462928412473,	3.90000000000000,	10479, -0.494281082201171, -0.826336449536374,	0.269915327363341,	3.56000000000000,
		-0.0539625060505756,	0.00345913500324202,	0.998536970935864,	4.20000000000000,	10231, -0.582019990091462, -0.792337919825955,	0.182902574995030,	3.65000000000000 };
	for (size_t i = 0; i < 4; i++)
	{
		vector<double>tmp(array + 9 * i, array + 9 + 9 * i);
		ZY3_ST.id_list.push_back(tmp);
	}
	ZY3_ST.Q_Method();*/

	/*ZY3_ST.Load_Star_Data(argv[1]);
	double Optical_axistmp[] = {-0.59551143828591213, -0.76970238188778783, 0.23005297256699414};
	memcpy(ZY3_ST.optical_axis,Optical_axistmp,sizeof(double)*3);
	double star_tracker_output[6][3] = { -397,	11,		2.8,
										-12,	-235,	3.7,
										495,	-476,	3.9,
										334,	494,	3.9,
										-62,	-351,	3.9,
										-156,	10,		4.2	};
	double tmpob[3];
	for (int c1=0; c1<6; c1++)
	{
		tmpob[0] = star_tracker_output[c1][0]*0.015;
		tmpob[1] = star_tracker_output[c1][1]*0.015;
		tmpob[2] = 43.3;
		double normob = sqrt(pow(tmpob[0],2) + pow(tmpob[1],2) + pow(tmpob[2],2));
		strStar obstmp;
		obstmp.x = tmpob[0]/normob;
		obstmp.y = tmpob[1]/normob;
		obstmp.z = tmpob[2]/normob;
		obstmp.Mag = star_tracker_output[c1][2];
		ZY3_ST.obs.push_back(obstmp);
	}
	ZY3_ST.fov_radius = 8*PI/180;
	ZY3_ST.Create_Spherical_Polygon_Candidate_Set();
	ZY3_ST.candidate_radius = 25*5e-6;
	ZY3_ST.match_tolerance = 500*5e-6;
	ZY3_ST.Identify_Basis_Pair();
	ZY3_ST.match_tolerance = 500 * 5e-6;
	ZY3_ST.Match_Stars_Relative_To_Basis_Pair();
	ZY3_ST.Q_Method();*/
	return 0;
}