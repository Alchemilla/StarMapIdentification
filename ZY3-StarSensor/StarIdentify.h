#pragma once
#define PI 3.1415926535897932384626433832795
#include <vector>
using namespace std;

//
struct strStar
{
	double x,y,z;
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
	int xaxis, yaxis,zaxis;
};

class StarIdentify
{
public:
	StarIdentify(void);
	~StarIdentify(void);
	bool Load_Star_Data(string StarCatelog);
	double Create_Spherical_Polygon_Candidate_Set(double optical_axis[3], double fov_radius);
	double Identify_Basis_Pair(vector<strStar> obs);
	double Match_Stars_Relative_To_Basis_Pair();
	double candidate_radius, match_tolerance;	
private: ;
	//升序降序函数
	bool static LessSort (vector<double> a,vector<double> b);
	bool static GreaterSort(vector<double> a,vector<double> b);
	bool static GreaterSort5(vector<double> a,vector<double> b);//根据第5个元素排序
	vector<catelog> catalog_xaxis, catalog_yaxis, catalog_zaxis;
	vector<k_vector> k_vec;
	vector<vector<double>> candidate_set;
public:
	//一些基本的算法
	bool norm(vector<double>&a);
	bool cross(vector<double>&a,vector<double>&b);
	double dot(vector<double>&a,vector<double>&b);
};

