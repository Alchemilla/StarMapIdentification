#pragma once
#include "geotime.h"
class GeoTime_ZY3 :
	public GeoTime
{
public:
	GeoTime_ZY3(void);
	~GeoTime_ZY3(void);
	void ReadZY3TimeFile(vector<LineScanTime> allTime);
};

