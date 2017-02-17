#include "StarIdentify.h"

StarIdentify::StarIdentify(void)
{
}


StarIdentify::~StarIdentify(void)
{
}

/////////////////////////////////////////////////////////////
// Function
//		获取星图和k-vector数据
// Input
//		文件夹路径
// Output
//		得到类私有成员数据
// Attention
//		Null
// Author
//		GZC
// History
//		2016.12.27
/////////////////////////////////////////////////////////////
bool StarIdentify::Load_Star_Data(string StarCatelog)
{
	string StarCatelotPath = StarCatelog + "\\星表数据.txt";
	string K_vectorPath = StarCatelog + "\\K矢量数据.txt";
	FILE *fpS = fopen(StarCatelotPath.c_str(),"r");
	FILE *fpK = fopen(K_vectorPath.c_str(),"r");
	
	catelog xtmp, ytmp, ztmp;
	while (!feof(fpS))
	{
		fscanf(fpS,"%d\t%lf\t%lf\t%d\t%lf\t%d\t%lf\n",&xtmp.id,&xtmp.Pos,&xtmp.Mag,&ytmp.id,&ytmp.Pos,&ztmp.id,&ztmp.Pos);
		catalog_xaxis.push_back(xtmp);
		catalog_yaxis.push_back(ytmp);
		catalog_zaxis.push_back(ztmp);
	}
	k_vector k_vec_tmp ;
	while(!feof(fpK))
	{
		fscanf(fpK,"%d\t%d\t%d\n",&k_vec_tmp.xaxis,&k_vec_tmp.yaxis,&k_vec_tmp.zaxis);	
		k_vec.push_back(k_vec_tmp);
	}
	return 0;
}

/////////////////////////////////////////////////////////////
// Function
// Create_Polygon_Candidate_Set 
//
// Input
// 1) optical_axis
// 2) fov_radius
// 3) catalog_xaxis, catalog_yaxis, catalog_zaxis
// 4) k_vector_xaxis, k_vector_yaxis, k_vector_zaxis
//
// Output
// 1) candidate_set
//
// Attention
// There are two major parts in this function.
// 1) Spherical Polygon Search Candidate Stars About Optical Axis
// 2) Transform Candidate Set
//
// Author
// Noah Smith, Center for Space Research, University of Texas at Austin
//
// History
// 2006.10.12 Created as part of StarID Release 0.99
// 2016.12.27 Modified by GuanZhichao
/////////////////////////////////////////////////////////////
double StarIdentify::Create_Spherical_Polygon_Candidate_Set()
{
	double x = optical_axis[0];
	double y = optical_axis[1];
	double z = optical_axis[2];
	double h_beta = 2*fov_radius;
	int n = catalog_xaxis.size();
	double a0 = -n*(n+1)/(n-1.)/(n-1.);
	double a1 = 2*n/(n-1.)/(n-1.);

	double xmin, xmax;
	if (x >= cos(h_beta))
	{
		xmin = x*cos(h_beta) - sqrt(1-x*x)*sin(h_beta);
		xmax = 1;
	} 
	else if(x <= -cos(h_beta))
	{
		xmin = -1;
		xmax = x*cos(h_beta) + sqrt(1-x*x)*sin(h_beta);
	}
	else
	{ 
		xmin = x*cos(h_beta) - sqrt(1-x*x)*sin(h_beta);
		xmax = x*cos(h_beta) + sqrt(1-x*x)*sin(h_beta);
	}	
	double ymin, ymax;
	if (y >= cos(h_beta))
	{
		ymin = y*cos(h_beta) - sqrt(1-y*y)*sin(h_beta);
		ymax = 1;
	} 
	else if(y <= -cos(h_beta))
	{
		ymin = -1;
		ymax = y*cos(h_beta) + sqrt(1-y*y)*sin(h_beta);
	}
	else
	{ 
		ymin = y*cos(h_beta) - sqrt(1-y*y)*sin(h_beta);
		ymax = y*cos(h_beta) + sqrt(1-y*y)*sin(h_beta);
	}
	double zmin, zmax;
	if (z >= cos(h_beta))
	{
		zmin = z*cos(h_beta) - sqrt(1-z*z)*sin(h_beta);
		zmax = 1;
	} 
	else if(z <= -cos(h_beta))
	{
		zmin = -1;
		zmax = z*cos(h_beta) + sqrt(1-z*z)*sin(h_beta);
	}
	else
	{ 
		zmin = z*cos(h_beta) - sqrt(1-z*z)*sin(h_beta);
		zmax = z*cos(h_beta) + sqrt(1-z*z)*sin(h_beta);
	}

	double kx1 = k_vec.at(floor((xmin - a0)/a1 + 1)-1).xaxis;
	double kx2 = k_vec.at(ceil((xmax-a0)/a1)-1).xaxis;
	vector<catelog>::iterator i1=catalog_xaxis.begin()+kx1, i2=catalog_xaxis.begin()+kx2;
	vector<catelog> xcone(i1-1, i2);
	double ky1 = k_vec.at(floor((ymin - a0)/a1 + 1)-1).yaxis;
	double ky2 = k_vec.at(ceil((ymax-a0)/a1)-1).yaxis;
	i1 = catalog_yaxis.begin()+ky1, i2 = catalog_yaxis.begin()+ky2;
	vector<catelog> ycone(i1-1, i2);
	double kz1 = k_vec.at(floor((zmin - a0)/a1 + 1)-1).zaxis;
	double kz2 = k_vec.at(ceil((zmax-a0)/a1)-1).zaxis;
	i1 = catalog_zaxis.begin()+kz1, i2 = catalog_zaxis.begin()+kz2;
	vector<catelog> zcone(i1-1, i2);

	int i, j;
	vector<vector<double>> xy;
	double *xytmp = new double[4];
	for (i = 0; i<xcone.size(); i++)
	{
		for (j = 0; j<ycone.size(); j++)
		{
			if (xcone.at(i).id == ycone.at(j).id)
			{
				xytmp[0] = xcone.at(i).id;
				xytmp[1] = xcone.at(i).Pos;
				xytmp[2] = ycone.at(j).Pos;
				xytmp[3] = xcone.at(i).Mag;
				xy.push_back(vector<double>(xytmp, xytmp + 4));//将数组传入vector中
			}
		}
	}
	//使用迭代器，似乎更慢
	//for (vector<catelog>::iterator iter=xcone.begin(); iter !=xcone.end(); ++iter)
	//{
	//	for (vector<catelog>::iterator jter = ycone.begin(); jter != ycone.end(); ++jter)
	//	{
	//		if ((*iter).id == (*jter).id)
	//		{
	//			//xy.emplace_back((*iter).id, (*iter).Pos, (*jter).Pos, (*iter).Mag);
	//			xytmp[0] = (*iter).id;
	//			xytmp[1] = (*iter).Pos;
	//			xytmp[2] = (*jter).Pos;
	//			xytmp[3] = (*iter).Mag;
	//			xy.push_back(vector<double>(xytmp,xytmp+4));//将数组传入vector中
	//		}
	//	}
	//}
	int k, tmp;
	vector<vector<double>> xyz;
	double *xyztmp = new double[5];
	for (tmp=0; tmp<xy.size(); tmp++)
	{
		for (k=0; k<zcone.size(); k++)
		{
			if (xy.at(tmp).at(0) == zcone.at(k).id)
			{
				xyztmp[0] = xy.at(tmp).at(0);
				xyztmp[1] = xy.at(tmp).at(1);
				xyztmp[2] = xy.at(tmp).at(2);
				xyztmp[3] = zcone.at(k).Pos;
				xyztmp[4] = xy.at(tmp).at(3);
				xyz.push_back(vector<double>(xyztmp,xyztmp+5));
			}
		}
	}
	
	int c1, c2;
	multiset<vector<double>>candidate_set_2;
	for (c1=0; c1<xyz.size()-1; c1++)
	{
		for (c2=c1+1; c2<xyz.size(); c2++)
		{
			double angdot = xyz.at(c1).at(1)*xyz.at(c2).at(1) + xyz.at(c1).at(2)*xyz.at(c2).at(2)
				+ xyz.at(c1).at(3)*xyz.at(c2).at(3);
			vector<double> vcandidate_set_tmp(1,angdot);
			vcandidate_set_tmp.insert(vcandidate_set_tmp.end(),xyz.at(c1).begin(),xyz.at(c1).end());
			vcandidate_set_tmp.insert(vcandidate_set_tmp.end(),xyz.at(c2).begin(),xyz.at(c2).end());
			candidate_set_2.insert(vcandidate_set_tmp);//插入值的同时，进行了排序
		}
	}
	candidate_set.assign(candidate_set_2.begin(), candidate_set_2.end());
	//candidate_set.insert(candidate_set.end(), candidate_set_2.begin(), candidate_set_2.end());
	return 0;
}

/////////////////////////////////////////////////////////////
// Input
// 1) obs
// 2) candidate_set
// 3) candidate_radius
// 4) match_tolerance
//
// Output
// 1) basis_pair
//
// There are four major parts in this function.
// 1) Initialize
// 2) Make Trial Identifications
// 3) Test Trial Identifications And Output Residuals
// 4) Select Basis Pair From Trial Identifications Based On Residuals
//
// Author
// Noah Smith, Center for Space Research, University of Texas at Austin
//
// History
// 2006.10.12 Created as part of StarID Release 0.99
// 2016.12.27 Start to transform it to C++ by Guanzhichao
/////////////////////////////////////////////////////////////
double StarIdentify::Identify_Basis_Pair()
{
	int c1,c2;
	vector<vector<double>> candidate_set_stars;
	for (c1=0; c1<candidate_set.size(); c1++)
	{
		int star1alreadyinlist = 0;
		int star2alreadyinlist = 0;
		for (c2=0; c2<candidate_set_stars.size(); c2++)
		{
			if (candidate_set.at(c1).at(1)==candidate_set_stars.at(c2).at(0))
				star1alreadyinlist = 1;
			if (candidate_set.at(c1).at(6)==candidate_set_stars.at(c2).at(0))
				star2alreadyinlist = 1;
		}
		if (star1alreadyinlist==0)
		{
			vector<double>candidate_set_stars_tmp(candidate_set.at(c1).begin()+1,candidate_set.at(c1).begin()+6);
			candidate_set_stars.push_back(candidate_set_stars_tmp);
		}
		if (star2alreadyinlist==0)
		{
			vector<double>candidate_set_stars_tmp(candidate_set.at(c1).begin()+6,candidate_set.at(c1).begin()+11);
			candidate_set_stars.push_back(candidate_set_stars_tmp);
		}
	}
	vector<vector<double>>obs_dotproducts_matrix(obs.size(), vector<double>(obs.size()));
	for (c1=0; c1<obs.size(); c1++)
	{
		for (c2=0; c2<obs.size(); c2++)
		{
			obs_dotproducts_matrix[c1][c2] = obs[c1].x*obs[c2].x + obs[c1].y*obs[c2].y + obs[c1].z*obs[c2].z;
		}
	}

	//sort(candidate_set.begin(),candidate_set.end(),StarIdentify::LessSort);//这个算法太慢了,21.784s
	int m = candidate_set.size();
	double candidate_set_1 = candidate_set[0][0];
	double candidate_set_m = candidate_set[m-1][0];
	double d = (candidate_set_m - candidate_set_1)/(m -1);
	double aa1 = m*d/(m-1);
	double a0 = candidate_set_1 - aa1 - d/2;
	vector<int> k_vec_tmp(m);
	k_vec_tmp[0] = 0;
	c1 = 1;
	for (c2=1; c2<m; c2++)
	{
		double fitline = aa1*(c2 + 1) + a0;
		int go = 1;
		c1 = c1 - 1;
		while(c1<m-1&&go==1)
		{
			if (candidate_set[c1][0]<=fitline&&candidate_set[c1+1][0]>fitline)
			{
				k_vec_tmp[c2] = c1 + 1;
				go = 0;
			}
			c1 = c1 + 1;
		}
	}
	k_vec_tmp[m-1] = m;

	vector<vector<double>> trial_ids;
	for (int primaryobs=0; primaryobs<obs.size(); primaryobs++)
	{
		vector<vector<double>> trial_id_occurences;
		for (int secondaryobs=0; secondaryobs<obs.size(); secondaryobs++)
		{
			if (primaryobs == secondaryobs)
				continue;
			double theta = acos(obs_dotproducts_matrix[primaryobs][secondaryobs]);
			double lbot = floor((cos(theta + candidate_radius) - a0)/aa1);
			double ltop = ceil((cos(theta - candidate_radius) - a0)/aa1);
			if (lbot<1 || lbot>m || ltop>m)
				continue;
			int kstart = k_vec_tmp[lbot - 1] + 1;
			int kend = k_vec_tmp[ltop - 1];
			for (int ii=kstart-1; ii<kend; ii++)
			{
				vector<double>trial_id_occurences_tmp(candidate_set.at(ii).begin()+1,candidate_set.at(ii).begin()+6);
				trial_id_occurences.push_back(trial_id_occurences_tmp);
			}
			for (int ii=kstart-1; ii<kend; ii++)
			{
				vector<double>trial_id_occurences_tmp(candidate_set.at(ii).begin()+6,candidate_set.at(ii).begin()+11);
				trial_id_occurences.push_back(trial_id_occurences_tmp);
			}			
		}
		sort(trial_id_occurences.begin(),trial_id_occurences.end(),StarIdentify::LessSort);//这个算法太慢了

		vector<vector<double>> trial_id_occurences_count;
		int count = 1;
		if (trial_id_occurences.size() == 0)
			continue;
		vector<double>previous_trial_id_occurences_line(trial_id_occurences.at(0).begin(),trial_id_occurences.at(0).end());
		for (c1=1; c1<trial_id_occurences.size(); c1++)
		{
			//if (trial_id_occurences.at(c1) != previous_trial_id_occurences_line)//这个是根据整个数组判断是否相等，原matlab写的是这样，但源程序判断出错了
			if (trial_id_occurences.at(c1).at(4) != previous_trial_id_occurences_line.at(4))//这个是根据最后一个元素，星等来判断，可以和源程序数据对上
			{
				vector<double>trial_id_occurences_count_tmp(1,count);
				trial_id_occurences_count_tmp.insert(trial_id_occurences_count_tmp.end(),
					previous_trial_id_occurences_line.begin(),previous_trial_id_occurences_line.end());
				trial_id_occurences_count.push_back(trial_id_occurences_count_tmp);
				count = 1;
				previous_trial_id_occurences_line = trial_id_occurences.at(c1);
			}
			else
			{
				count++;
			}
		}
		sort(trial_id_occurences_count.begin(),trial_id_occurences_count.end(),StarIdentify::GreaterSort);//再根据第一个排序

		double temp1[4] ={obs.at(primaryobs).x,obs.at(primaryobs).y,obs.at(primaryobs).z,obs.at(primaryobs).Mag};
		vector<double>trial_ids_tmp(temp1,temp1+4);
		trial_ids_tmp.insert(trial_ids_tmp.end(),trial_id_occurences_count.at(0).begin(),trial_id_occurences_count.at(0).end());
		trial_ids.push_back(trial_ids_tmp);
		vector<double>trial_ids_tmp2(temp1,temp1+4);
		trial_ids_tmp2.insert(trial_ids_tmp2.end(),trial_id_occurences_count.at(1).begin(),trial_id_occurences_count.at(1).end());
		trial_ids.push_back(trial_ids_tmp2);
	}
	sort(trial_ids.begin(),trial_ids.end(),StarIdentify::GreaterSort5);//再根据第5个元素排序

	int trial_id1, trial_id2;
	vector<vector<double>> residuals;
	for (trial_id1=0; trial_id1<trial_ids.size()-1; trial_id1++)
	{
		for (trial_id2=trial_id1; trial_id2<trial_ids.size(); trial_id2++)
		{
			if (trial_ids[trial_id1][5] == trial_ids[trial_id2][5])
				continue;
			vector<double>a1(3);
			a1[0] = trial_ids[trial_id2][0] - trial_ids[trial_id1][0];
			a1[1] = trial_ids[trial_id2][1] - trial_ids[trial_id1][1];
			a1[2] = trial_ids[trial_id2][2] - trial_ids[trial_id1][2];
			if(a1[0]==0&&a1[1]==0&&a1[2]==0)
				continue;
			norm(a1);
			vector<double>a3(trial_ids.at(trial_id1).begin(),trial_ids.at(trial_id1).begin()+3);
			vector<double>a2(a1);
			cross(a3,a2); 
			norm(a2);//得到归一化a2
			vector<double>a3tmp(a3);
			cross(a2,a3tmp);//不改变a3
			a1 = a3tmp;//得到归一化a1

			vector<double>b1(3);
			b1[0] = trial_ids[trial_id2][6] - trial_ids[trial_id1][6];
			b1[1] = trial_ids[trial_id2][7] - trial_ids[trial_id1][7];
			b1[2] = trial_ids[trial_id2][8] - trial_ids[trial_id1][8];
			norm(b1);
			vector<double>b3(trial_ids.at(trial_id1).begin()+6,trial_ids.at(trial_id1).begin()+9);
			vector<double>b2(b1);
			cross(b3,b2);
			norm(b2);
			vector<double>b3tmp(b3);
			cross(b2,b3tmp);
			b1 = b3tmp;

			vector<vector<double>> new_obs;
			for (c1=0; c1<obs.size(); c1++)
			{
				double dotobs[3];
				dotobs[0] = obs.at(c1).x; dotobs[1] = obs.at(c1).y; dotobs[2] = obs.at(c1).z;
				vector<double>obsc1 (dotobs,dotobs+3);
				dotobs[0] = dot(obsc1,a1); dotobs[1] = dot(obsc1,a2); dotobs[2] = dot(obsc1,a3);
				vector<double>vec(dotobs,dotobs+3);
				norm(vec);
				vector<double>new_obs_tmp(vec);
				new_obs_tmp.insert(new_obs_tmp.end(),obs.at(c1).Mag);//只插入一个值
				new_obs.push_back(new_obs_tmp);
			}

			vector<vector<double>> new_candidate_set_stars;			
			for (c1=0; c1<candidate_set_stars.size(); c1++)
			{				
				double set_stars[3];
				set_stars[0] = candidate_set_stars.at(c1).at(1);
				set_stars[1] = candidate_set_stars.at(c1).at(2);
				set_stars[2] = candidate_set_stars.at(c1).at(3);
				vector<double>set_stars_c1(set_stars,set_stars+3);
				set_stars[0] = dot(set_stars_c1,b1);
				set_stars[1] = dot(set_stars_c1,b2);
				set_stars[2] = dot(set_stars_c1,b3);
				vector<double>vec(set_stars,set_stars+3);
				norm(vec);
				vector<double>new_candidate_set_stars_tmp(vec);
				new_candidate_set_stars_tmp.insert(new_candidate_set_stars_tmp.begin(),candidate_set_stars.at(c1).at(0));
				new_candidate_set_stars_tmp.insert(new_candidate_set_stars_tmp.end(),candidate_set_stars.at(c1).at(4));
				new_candidate_set_stars.push_back(new_candidate_set_stars_tmp);
			}

			for (int new_obs_number=0; new_obs_number<new_obs.size(); new_obs_number++)
			{
				for (c2=0; c2<new_candidate_set_stars.size(); c2++)
				{
					vector<double>dot_new_obs(3),dot_new_set_stars(3);
					dot_new_obs.assign(new_obs[new_obs_number].begin(),new_obs[new_obs_number].begin()+3);
					dot_new_set_stars.assign(new_candidate_set_stars[c2].begin()+1,new_candidate_set_stars[c2].begin()+4);
					double residual = acos(dot(dot_new_obs,dot_new_set_stars));
					if (residual<match_tolerance&&residual>0)
					{
						vector<double>residuals_tmp(4);
						residuals_tmp[0] = trial_id1 + 1;
						residuals_tmp[1] = trial_id2 + 1;
						residuals_tmp[2] = new_obs_number + 1;
						residuals_tmp[3] = residual*648e3/PI;
						residuals.push_back(residuals_tmp);
					}
				}
			}
		}
	}

	int count = 0;
	vector<vector<double>> residuals_count;
	vector<double>previous_pair(residuals.at(0).begin(),residuals.at(0).begin()+2);
	vector<double>previous_line(previous_pair);
	previous_line.insert(previous_line.begin(),1);
	for (c1 = 1; c1 < residuals.size(); c1++)
	{
		if (residuals[c1][0] != previous_pair[0] || residuals[c1][1] != previous_pair[1])
		{
			vector<double>residuals_count_tmp(previous_line);
			residuals_count.push_back(residuals_count_tmp);
			count = 1;
			previous_pair.assign(residuals.at(c1).begin(),residuals.at(c1).begin()+2);
		}
		else
		{
			count++;
		}
		previous_line[0] = count;
		previous_line[1] = residuals[c1][0];
		previous_line[2] = residuals[c1][1];
	}
	sort(residuals_count.begin(),residuals_count.end(),StarIdentify::GreaterSort);
	vector<double>basis_pair_tmp;
	basis_pair_tmp.assign(trial_ids.at(residuals_count[0][1]-1).begin(), trial_ids.at(residuals_count[0][1]-1).end());
	basis_pair.push_back(basis_pair_tmp);
	basis_pair_tmp.assign(trial_ids.at(residuals_count[0][2]-1).begin(), trial_ids.at(residuals_count[0][2]-1).end());
	basis_pair.push_back(basis_pair_tmp);

	return 0;
}
/////////////////////////////////////////////////////////////
//Match_Stars_Relative_To_Basis_Pair 
//
// Input
// 1) obs
// 2) basis_pair
// 3) match_tolerance
// 4) fov_radius
// 5) catalog_xaxis, catalog_yaxis, catalog_zaxis
// 6) k_vector_xaxis, k_vector_yaxis, k_vector_zaxis
//
// Output
// 1) id_list
//
// There are two major parts in this function.
// 1) Spherical Polygon Search Candidate Stars About Basis Pair
// 2) Match Candidate Set To Obs In Basis Pair Coordinate Frame
//
// Author
// Noah Smith, Center for Space Research, University of Texas at Austin
//
// History
// 2006.10.12 Created as part of StarID Release 0.99
// 2016.12.28 Start to transform it to C++ by Guanzhichao
/////////////////////////////////////////////////////////////
double StarIdentify::Match_Stars_Relative_To_Basis_Pair()
{
	double x = basis_pair[0][6];
	double y = basis_pair[0][7];
	double z = basis_pair[0][8];
	double h_beta = 2 * fov_radius;
	int n = catalog_xaxis.size();
	double a0 = -n*(n + 1) / (n - 1.) / (n - 1.);
	double a1 = 2 * n / (n - 1.) / (n - 1.);

	double xmin, xmax;
	if (x >= cos(h_beta))
	{
		xmin = x*cos(h_beta) - sqrt(1 - x*x)*sin(h_beta);
		xmax = 1;
	}
	else if (x <= -cos(h_beta))
	{
		xmin = -1;
		xmax = x*cos(h_beta) + sqrt(1 - x*x)*sin(h_beta);
	}
	else
	{
		xmin = x*cos(h_beta) - sqrt(1 - x*x)*sin(h_beta);
		xmax = x*cos(h_beta) + sqrt(1 - x*x)*sin(h_beta);
	}
	double ymin, ymax;
	if (y >= cos(h_beta))
	{
		ymin = y*cos(h_beta) - sqrt(1 - y*y)*sin(h_beta);
		ymax = 1;
	}
	else if (y <= -cos(h_beta))
	{
		ymin = -1;
		ymax = y*cos(h_beta) + sqrt(1 - y*y)*sin(h_beta);
	}
	else
	{
		ymin = y*cos(h_beta) - sqrt(1 - y*y)*sin(h_beta);
		ymax = y*cos(h_beta) + sqrt(1 - y*y)*sin(h_beta);
	}
	double zmin, zmax;
	if (z >= cos(h_beta))
	{
		zmin = z*cos(h_beta) - sqrt(1 - z*z)*sin(h_beta);
		zmax = 1;
	}
	else if (z <= -cos(h_beta))
	{
		zmin = -1;
		zmax = z*cos(h_beta) + sqrt(1 - z*z)*sin(h_beta);
	}
	else
	{
		zmin = z*cos(h_beta) - sqrt(1 - z*z)*sin(h_beta);
		zmax = z*cos(h_beta) + sqrt(1 - z*z)*sin(h_beta);
	}

	double kx1 = k_vec.at(floor((xmin - a0) / a1 + 1) - 1).xaxis;
	double kx2 = k_vec.at(ceil((xmax - a0) / a1) - 1).xaxis;
	vector<catelog>::iterator i1 = catalog_xaxis.begin() + kx1, i2 = catalog_xaxis.begin() + kx2;
	vector<catelog> xcone(i1 - 1, i2);
	double ky1 = k_vec.at(floor((ymin - a0) / a1 + 1) - 1).yaxis;
	double ky2 = k_vec.at(ceil((ymax - a0) / a1) - 1).yaxis;
	i1 = catalog_yaxis.begin() + ky1, i2 = catalog_yaxis.begin() + ky2;
	vector<catelog> ycone(i1 - 1, i2);
	double kz1 = k_vec.at(floor((zmin - a0) / a1 + 1) - 1).zaxis;
	double kz2 = k_vec.at(ceil((zmax - a0) / a1) - 1).zaxis;
	i1 = catalog_zaxis.begin() + kz1, i2 = catalog_zaxis.begin() + kz2;
	vector<catelog> zcone(i1 - 1, i2);

	int i, j;
	vector<vector<double>> xy;
	double *xytmp = new double[4];
	for (i = 0; i<xcone.size(); i++)
	{
		for (j = 0; j<ycone.size(); j++)
		{
			if (xcone.at(i).id == ycone.at(j).id)
			{
				xytmp[0] = xcone.at(i).id;
				xytmp[1] = xcone.at(i).Pos;
				xytmp[2] = ycone.at(j).Pos;
				xytmp[3] = xcone.at(i).Mag;
				xy.push_back(vector<double>(xytmp, xytmp + 4));//将数组传入vector中
			}
		}
	}
	int k, tmp;
	vector<vector<double>> xyz;
	double *xyztmp = new double[5];
	for (tmp = 0; tmp<xy.size(); tmp++)
	{
		for (k = 0; k<zcone.size(); k++)
		{
			if (xy.at(tmp).at(0) == zcone.at(k).id)
			{
				xyztmp[0] = xy.at(tmp).at(0);
				xyztmp[1] = xy.at(tmp).at(1);
				xyztmp[2] = xy.at(tmp).at(2);
				xyztmp[3] = zcone.at(k).Pos;
				xyztmp[4] = xy.at(tmp).at(3);
				xyz.push_back(vector<double>(xyztmp, xyztmp + 5));
			}
		}
	}
	vector<vector<double>> candidate_stars(xyz);

	vector<double>x1(3);
	x1[0] = basis_pair[1][0] - basis_pair[0][0];
	x1[1] = basis_pair[1][1] - basis_pair[0][1];
	x1[2] = basis_pair[1][2] - basis_pair[0][2];
	norm(x1);
	vector<double>x3(basis_pair.at(0).begin(), basis_pair.at(0).begin() + 3);
	vector<double>x2(x1);
	cross(x3, x2);
	norm(x2);
	vector<double>x3tmp(x3);
	cross(x2, x3tmp);
	x1 = x3tmp;
	
	vector<double>y1(3);
	y1[0] = basis_pair[1][6] - basis_pair[0][6];
	y1[1] = basis_pair[1][7] - basis_pair[0][7];
	y1[2] = basis_pair[1][8] - basis_pair[0][8];
	norm(y1);
	vector<double>y3(basis_pair.at(0).begin()+6, basis_pair.at(0).begin() + 9);
	vector<double>y2(y1);
	cross(y3, y2);
	norm(y2);
	vector<double>y3tmp(y3);
	cross(y2, y3tmp);
	y1 = y3tmp;

	vector<vector<double>> new_obs;
	int c1, c2;
	for (c1 = 0; c1<obs.size(); c1++)
	{
		double dotobs[3];
		dotobs[0] = obs.at(c1).x; dotobs[1] = obs.at(c1).y; dotobs[2] = obs.at(c1).z;
		vector<double>obsc1(dotobs, dotobs + 3);
		dotobs[0] = dot(obsc1, x1); dotobs[1] = dot(obsc1, x2); dotobs[2] = dot(obsc1, x3);
		vector<double>vec(dotobs, dotobs + 3);
		norm(vec);
		vector<double>new_obs_tmp(vec);
		new_obs_tmp.insert(new_obs_tmp.end(), obs.at(c1).Mag);//只插入一个值
		new_obs.push_back(new_obs_tmp);
	}

	vector<vector<double>> new_candidate_stars;
	for (c1 = 0; c1<candidate_stars.size(); c1++)
	{
		double set_stars[3];
		set_stars[0] = candidate_stars.at(c1).at(1);
		set_stars[1] = candidate_stars.at(c1).at(2);
		set_stars[2] = candidate_stars.at(c1).at(3);
		vector<double>set_stars_c1(set_stars, set_stars + 3);
		set_stars[0] = dot(set_stars_c1, y1);
		set_stars[1] = dot(set_stars_c1, y2);
		set_stars[2] = dot(set_stars_c1, y3);
		vector<double>vec(set_stars, set_stars + 3);
		norm(vec);
		vector<double>new_candidate_stars_tmp(vec);
		new_candidate_stars_tmp.insert(new_candidate_stars_tmp.begin(), candidate_stars.at(c1).at(0));
		new_candidate_stars_tmp.insert(new_candidate_stars_tmp.end(), candidate_stars.at(c1).at(4));
		new_candidate_stars.push_back(new_candidate_stars_tmp);
	}

	for (c1 = 0; c1 < new_obs.size(); c1++)
	{
		for (c2 = 0; c2 < new_candidate_stars.size(); c2++)
		{
			vector<double> dot_new_obs(3), dot_candidate_stars(3);
			dot_new_obs.assign(new_obs.at(c1).begin(), new_obs.at(c1).begin() + 3);
			dot_candidate_stars.assign(new_candidate_stars.at(c2).begin() + 1, new_candidate_stars.at(c2).begin() + 4);
			double residual = acos(dot(dot_new_obs, dot_candidate_stars));
			if (residual<match_tolerance)
			{
				vector<double> id_list_tmp(9);
				id_list_tmp[0] = obs[c1].x; 
				id_list_tmp[1] = obs[c1].y; 
				id_list_tmp[2] = obs[c1].z;
				id_list_tmp[3] = obs[c1].Mag;
				copy(candidate_stars.at(c2).begin(), candidate_stars.at(c2).end(),id_list_tmp.begin()+4);
				id_list.push_back(id_list_tmp);
			}
		}
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//功能：q_Method定姿方式
//输入：vector<vector<double>> id_list;
//输出：四元数Q
//注意：采用Eigen库进行编写
//作者：GZC
//日期：2016.12.30
//////////////////////////////////////////////////////////////////////////
bool StarIdentify::Q_Method()
{
	MatrixXd obs_in_startrackerframe(id_list.size(),3), stars_in_celestialframeV(id_list.size(), 3);
	for (int i = 0; i < id_list.size(); i++)
	{
		obs_in_startrackerframe.row(i) << id_list[i][0], id_list[i][1], id_list[i][2];
		stars_in_celestialframeV.row(i) << id_list[i][5], id_list[i][6], id_list[i][7];
	}
	MatrixXd W(3,id_list.size()), V(3, id_list.size());
	W = obs_in_startrackerframe.transpose();
	V = stars_in_celestialframeV.transpose();
	Matrix3d B,S;
	B = W*V.transpose();
	S = B + B.transpose();
	Vector3d Z;
	Z << B(1, 2) - B(2, 1), B(2, 0) - B(0, 2), B(0, 1) - B(1, 0);
	double SIGMA = B.trace();
	Matrix<double, 4, 4>K;
	K << S - MatrixXd::Identity(3, 3)*SIGMA, Z, Z.transpose(), SIGMA;
	EigenSolver<Matrix<double, 4, 4>> es(K);
	MatrixXcd evecs = es.eigenvectors();
	MatrixXcd evals = es.eigenvalues();
	MatrixXd evalsReal;
	evalsReal=evals.real();
	//cout << evalsReal << endl;
	MatrixXf::Index evalsMax;
	evalsReal.rowwise().sum().maxCoeff(&evalsMax);
	Vector4d q;
	q << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax), evecs.real()(3, evalsMax);
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//功能：获取星点控制
//输入：ZY3_02STGdata，解析后存储的星敏STG数据
//			 StarPointExtract，星点提取结果
//输出：getCCP：控制点vector
//注意：本方法还未依赖星图识别
//作者：GZC
//日期：2017.01.11
//////////////////////////////////////////////////////////////////////////
void StarIdentify::GetStarGCP(	vector<STGData> ZY3_02STGdata, vector<StarPoint> StarPointExtract,int index)
{
	//打开星表文件
	string starCatlogpath = "D:\\2_ImageData\\ZY3-02\\星图处理\\星表\\导航星表矢量.txt";
	FILE *fp;
	fp = fopen(starCatlogpath.c_str(), "r");
	if (fp == NULL)
	{
		printf("打开%s文件失败,请确认文件路径是否正确!\n", starCatlogpath.c_str());
		return;
	}
	int i, n;
	fscanf(fp, "%d", &n);//读取星表数据
	Star *starCatlog = new Star[n];
	for (i = 0; i < n; i++)
	{
		fscanf(fp, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", &starCatlog[i].ID, &starCatlog[i].phiX,
			&starCatlog[i].phiY, &starCatlog[i].mag, &starCatlog[i].V[0], &starCatlog[i].V[1], &starCatlog[i].V[2]);
	}
	fclose(fp);
	int j, m = StarPointExtract.size();
	double R[9];
	double x, y;
	char GCPpathtmp[100];
	sprintf(GCPpathtmp, "控制点\\GCP(%d).pts", index);
	string GCPpath = workpath + GCPpathtmp;
	FILE *fp2 = fopen(GCPpath.c_str(), "w");

	string str1 = "; ENVI Image to Image GCP File";
	string str2 = "; base file : D:\\2_ImageData\\ZY3 - 02\\星图处理\\0830\\星图\\星图(1).tiff";
	string str3 = "; warp file : D:\\2_ImageData\\ZY3 - 02\\星图处理\\0830\\星图\\星图(1).tiff";
	string str4 = "; Base Image(x, y), Warp Image(x, y)";
	fprintf(fp2, "%s\n%s\n%s\n%s\n", str1.c_str(), str2.c_str(), str3.c_str(), str4.c_str());

	//第一景对应第三个姿态
	index = 2 * index;
	mBase.quat2matrix(ZY3_02STGdata[index].StarA.Q1, ZY3_02STGdata[index].StarA.Q2,
		ZY3_02STGdata[index].StarA.Q3, ZY3_02STGdata[index].StarA.Q0, R);//Crj
	for (j = 0; j < n; j++)
	{
		mSTGfunc.FromLL2XY(starCatlog[j], R, x, y);//对星表每颗星遍历，计算像面坐标
		if (x > 0 && x < 1024 && y>0 && y < 1024)
		{			
			for (int k = 0; k < m; k++)
			{
				int xPixel = int(y + 0.5);
				int yPixel = 1024 - int(x + 0.5);
				/*int xPixel = int(x+0.5);
				int yPixel = int(y+0.5);*/
				StarGCP getGCPtmp;
				if ((StarPointExtract[k].x - xPixel > 5) && (StarPointExtract[k].x - xPixel < 15)
					&& (StarPointExtract[k].y - yPixel > -25) && (StarPointExtract[k].y - yPixel < -15))
				{
					getGCPtmp.x = StarPointExtract[k].x;
					getGCPtmp.y = StarPointExtract[k].y;
					getGCPtmp.V[0] = starCatlog[j].V[0];
					getGCPtmp.V[1] = starCatlog[j].V[1];
					getGCPtmp.V[2] = starCatlog[j].V[2];
					getGCP.push_back(getGCPtmp);
				}
				if ((StarPointExtract[k].x - xPixel >  5 ) && (StarPointExtract[k].x - xPixel < 15)
					&& (StarPointExtract[k].y - yPixel > -25) && (StarPointExtract[k].y - yPixel < -15))
				{
					fprintf(fp2, "%.9f\t%.9f\t%d\t%d\n", StarPointExtract[k].x, StarPointExtract[k].y, xPixel, yPixel);
				}
			}
		}
	}
	fclose(fp2);
}

////////////////////////////////////////
//基本算法部分：
////////////////////////////////////////

	/////////////////////////////////////////
//定义升序降序排列的函数，从小到大
////////////////////////////////////////
inline bool StarIdentify::LessSort (vector<double> a,vector<double> b) { return a[0]<b[0]; }
/////////////////////////////////////////
//定义升序降序排列的函数，从大到小
////////////////////////////////////////
inline bool StarIdentify::GreaterSort (vector<double> a,vector<double> b)
{ 
	if(a[0] != b[0])
		return a[0] > b[0];//首先判断第一列，大的在前
	else if(a[1]!=b[1])
	{
		return a[1] < b[1];//然后判断第二列，matlab 中是小的在前
	}
	else
	{
		return a[2] < b[2];//然后判断第三列，matlab 中是小的在前
	}
}
/////////////////////////////////////////
//定义升序降序排列的函数，从大到小
////////////////////////////////////////
inline bool StarIdentify::GreaterSort5 (vector<double> a,vector<double> b) { return a[4]>b[4]; }


/////////////////////////////////////////
//向量的单位化，输入返回a
////////////////////////////////////////
inline bool StarIdentify::norm(vector<double>&a)
{
	int m =a.size(),i;
	double x = 0;
	for (i=0; i<m; i++)
	{
		x += a.at(i)*a.at(i);
	}
	x = sqrt(x);
	for (i=0; i<m; i++)
	{
		a.at(i) = a.at(i)/x;
	}
	return true;
}
/////////////////////////////////////////
//叉积公式，输入b，返回也是b
////////////////////////////////////////
inline bool StarIdentify::cross(vector<double>&a,vector<double>&b)
{
	if (a.size()!=3||b.size()!=3)
	{
		printf("The vector should be 3*1 dim");
		return false;
	}
	vector<double>c(3);
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];
	b = c;
	return true;
}

inline double StarIdentify::dot(vector<double>&a,vector<double>&b)
{
	double c = 0, i;
	for (i=0; i<3; i++)
	{
		c +=a[i]*b[i];
	}
	return c;
}