#pragma once
#include "GeoAttitude.h"
class GeoAttitude_ZY3 :
	public GeoAttitude
{
public:
	GeoAttitude_ZY3(void);
	~GeoAttitude_ZY3(void);
	void ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit,string workpath);
	void ReadZY3RealAttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath);
	void upDateAtt(vector<Attitude> allAtt, StrAttParamInput input);
	void TransZY3AttFile(vector<Attitude> &allAtt, string EopPath);
};

