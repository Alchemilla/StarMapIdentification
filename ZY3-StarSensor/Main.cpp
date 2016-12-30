#include "StarIdentify.h"

int main(int argc, char* argv[])
{
	StarIdentify ZY3_ST;
	ZY3_ST.Load_Star_Data(argv[1]);
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
	return 0;
}