#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <set>
#include <vector>
#include"ParseSTG.h"
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
	//�Ǳ���غ���
	bool Load_Star_Data(string StarCatelog);
	//��������Ϊ��ͼʶ������
	double Create_Spherical_Polygon_Candidate_Set();
	double Identify_Basis_Pair();
	double Match_Stars_Relative_To_Basis_Pair();
	//�������˷���
	bool Q_Method();
	double candidate_radius, match_tolerance, fov_radius;
	double optical_axis[3];
	vector<strStar> obs;
	vector<vector<double>> id_list;

	//���ǿ��Ƶ���ȡ
	ParseSTG mSTGfunc;
	BaseFunc mBase;
	void GetStarGCP(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	void GetStarGCP0702(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	void GetStarGCP0707(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	void GetStarGCP0712(vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract, int index);
	vector<StarGCP> getGCP;
	string workpath;
private: 
		 //��������
	bool static LessSort (vector<double> a,vector<double> b);
	bool static GreaterSort(vector<double> a,vector<double> b);
	bool static GreaterSort5(vector<double> a,vector<double> b);//���ݵ�5��Ԫ������
	vector<catelog> catalog_xaxis, catalog_yaxis, catalog_zaxis;
	vector<k_vector> k_vec;
	vector<vector<double>> candidate_set;
	vector<vector<double>> basis_pair;
public:
	//һЩ�������㷨
	bool norm(vector<double>&a);
	bool cross(vector<double>&a,vector<double>&b);
	double dot(vector<double>&a,vector<double>&b);
};

