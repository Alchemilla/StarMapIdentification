#ifndef SIMULATION_H
#define SIMULATION_H
#include "engine.h"
#include "SateBase.h"
#include "stdlib.h"
#include "time.h"
#include <iostream>
using namespace std;

class Simulation
{
public:
	Simulation(void);
	~Simulation(void);
	void MatlabExample();
	void MatPlotDetQ(SateEuler *EulerArray,long N);
	void MatGaussDist(double *x, long N);
	void SimStar();
	void SimGyro();
protected:
private:
	Engine *ep;
};

#endif