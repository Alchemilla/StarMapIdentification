#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <set>
#include <vector>
#include"ParseSTG.h"
#include"AttDetermination.h"
using namespace Eigen;
using namespace std;

//

class StarIdentify
{
public:
	struct strStar
	{
		double x, y, z;
		double Mag;
	};
	struct catelog
	{
		int id;
		double Pos;
		double Mag;
	};
	struct k_vector
	{
		int xaxis, yaxis, zaxis;
	};

	StarIdentify(void);
	~StarIdentify(void);
	//星表加载函数
	bool Load_Star_Data(string StarCatelog);
	//下面三个为星图识别流程
	double Create_Spherical_Polygon_Candidate_Set();
	double Identify_Basis_Pair();
	double Match_Stars_Relative_To_Basis_Pair();
	//星敏定姿方法
	bool Q_Method();
	double candidate_radius, match_tolerance, fov_radius;
	double optical_axis[3];
	vector<strStar> obs;
	vector<vector<double>> id_list;

	//恒星控制点提取
	ParseSTG mSTGfunc;
	BaseFunc mBase;
	AttDetermination mAtt;
	void GetStarGCP(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	void GetStarGCP0702(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	void GetStarGCP0707(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	void GetStarGCP0712(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	void GetStarGCPForLuojia(vector<Quat> LuojiaCam, vector<StarGCP> &starGCPLuojia);
	void OutputGCPForLuojia(vector<StarGCP> &getGCP);
	vector<StarGCP> getGCP;
	string workpath;
	//吉林一号恒星控制点获取
	void GetStarGCPforJL106(string gcpPath, vector<StarGCP>& JLcam, vector<img> imgJL106);
	void GetStarGCPforJL106(string gcpPath, vector<StarGCP>& JLcam, vector<img> imgJL106, img& imgLat);
	void GetStarGCPforJL107(string gcpPath, vector<StarGCP>& JLcam, img imgJL107, img& imgLat);
	void GetCamOptforJL107(string optPath, StarGCP& JLcam, vector<img> imgJL107);
private: 
		 //升序降序函数
	bool static LessSort (vector<double> a,vector<double> b);
	bool static GreaterSort(vector<double> a,vector<double> b);
	bool static GreaterSort5(vector<double> a,vector<double> b);//根据第5个元素排序
	vector<catelog> catalog_xaxis, catalog_yaxis, catalog_zaxis;
	vector<k_vector> k_vec;
	vector<vector<double>> candidate_set;
	vector<vector<double>> basis_pair;
public:
	//一些基本的算法
	bool norm(vector<double>&a);
	bool cross(vector<double>&a,vector<double>&b);
	double dot(vector<double>&a,vector<double>&b);
};

