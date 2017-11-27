#pragma once
#include "GeoDefine.h"
#include "Eigen/Dense"
#include <iostream>
using namespace Eigen;
using namespace std;

class AttDeter
{
public:
	AttDeter();
	~AttDeter();
	void ExtendedKalmanFilter(vector<Attitude>&qMeas, vector<Gyro> wMeas);
};

