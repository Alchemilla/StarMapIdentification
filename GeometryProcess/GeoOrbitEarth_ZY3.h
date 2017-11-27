#pragma once
#include "geoorbitearth.h"
class GeoOrbitEarth_ZY3 :
	public GeoOrbitEarth
{
public:
	GeoOrbitEarth_ZY3(void);
	virtual ~GeoOrbitEarth_ZY3(void);
	void ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input);
};