#pragma once
#include "GeoReadImage.h"
#include <iostream>
#include <vector>
#include<algorithm>
#include "BaseFunc.h"
using namespace std;

class StarExtract
{
public:
	StarExtract();
	~StarExtract();
	void StarPointExtraction(int index);
	BaseFunc mbase;
	//void StarPointExtraction();
	//连通区域标记函数
	void bwlabel(GeoReadImage &ImgBW, GeoReadImage &ImgStarMap);
	void fillRunVectors(GeoReadImage &ImgBW, int& NumberOfRuns, vector<int>& stRun, vector<int>& enRun, vector<int>& rowRun);
	void firstPass(vector<int>& stRun, vector<int>& enRun, vector<int>& rowRun, int NumberOfRuns,
		vector<int>& runLabels, vector<pair<int, int>>& equivalences, int offset);
	void replaceSameLabel(vector<int>& runLabels, vector<pair<int, int>>&equivalence, int &equaListsize);
	//最小二乘求星点质心
	bool GetPreciseXYbyFitting(double m_sample, double m_line, double &fm_sample,
		double &fm_line, int num, GeoReadImage &IMAGEFILE);
	string workpath;
	vector<StarPoint> StarPointExtract;
private:
	long width, height;
};

