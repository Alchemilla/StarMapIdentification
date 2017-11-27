#include "GeoAttitude_ZY3.h"


GeoAttitude_ZY3::GeoAttitude_ZY3(void)
{
}


GeoAttitude_ZY3::~GeoAttitude_ZY3(void)
{
}

void GeoAttitude_ZY3::ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit,string workpath)
{
	// 清空数据
	ClearData();

	//int num;
	//FILE *fp = fopen((workpath + "\\WGS84ToJ2000_Error.txt").c_str(), "r");
	//fscanf(fp, "%d\n", &num);

	// 获得参数
	m_Input = input;
	m_Orbit = orbit;
	// 累计秒从2009年1月1日0时开始计数
	double jd0;
	Cal2JD(2009, 1, 1, 0, &jd0, &m_Input.refMJD);
	long m = allAtt.size();
	int i;	
	vector<StrAttPoint> m_Body2WGS84tmp;
	vector<StrAttPoint> m_Body2Orbittmp;
	// 读取姿态文件
	for (i = 0;i < m;i++)
	{
		// 开始读取
		StrAttPoint tmp;
		double R[9], R1[9];
		//tmp.UT = allAtt.at(i).UTC;//对应仿真姿轨数据，以下四元数是Cbj
		tmp.UT = allAtt.at(i).UTC -28800;//对应真实姿轨数据
		tmp.q[0] = allAtt.at(i).Q0,tmp.q[1] = allAtt.at(i).Q1,tmp.q[2] = allAtt.at(i).Q2,tmp.q[3] = allAtt.at(i).Q3;

		m_Base.FromSecondtoYMD(m_Input.refMJD,tmp.UT,tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second);
		m_Base.Quat2Matrix(tmp.q[1], tmp.q[2], tmp.q[3], tmp.q[0], tmp.R);	// J20002body									
		m_Base.Transpose(tmp.R, 3, 3);										// body2J2000
		memcpy(R1, tmp.R, 9*sizeof(double));								// 保存
		//fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]);// WGS84ToJ2000
		m_Orbit->GetJ20002WGS84Rotation(tmp.UT, R);							// J20002WGS84
		//m_Base.invers_matrix(R, 3);
		m_Base.Multi(R, R1, tmp.R, 3, 3, 3);								// body2WGS84
		m_Base.Matrix2Quat(tmp.R, tmp.q[0], tmp.q[1], tmp.q[2], tmp.q[3]);
		m_Base.Matrix2Eulor(tmp.R, 213, tmp.Eulor[0], tmp.Eulor[1], tmp.Eulor[2]);
		for(int i=0; i<3; i++)
			tmp.Eulor[i] = tmp.Eulor[i]*180/PI;
		m_Body2WGS84tmp.push_back(tmp);
		// 开始考虑body2orbit
		m_Orbit->GetJ20002OrbitRotation(tmp.UT, R);							// J20002orbit
		m_Base.Multi(R, R1, tmp.R, 3, 3, 3);								// body2orbit
		m_Base.Matrix2Quat(tmp.R, tmp.q[0], tmp.q[1], tmp.q[2], tmp.q[3]);
		m_Base.Matrix2Eulor(tmp.R, 213, tmp.Eulor[0], tmp.Eulor[1], tmp.Eulor[2]);
		for(int i=0; i<3; i++)
			tmp.Eulor[i] = tmp.Eulor[i]*180/PI;
		m_Body2Orbittmp.push_back(tmp);
	}
	//fclose(fp);
	m_num = m_Body2WGS84tmp.size();
	// 存储
	m_Body2WGS84 = new StrAttPoint[m_num];
	m_Body2Orbit = new StrAttPoint[m_num];
	for(int i=0; i<m_num; i++)
	{
		m_Body2WGS84[i] = m_Body2WGS84tmp[i];
		m_Body2Orbit[i] = m_Body2Orbittmp[i];
	}
	m_Body2WGS84tmp.clear();
	m_Body2Orbittmp.clear();
	// 排序
	OrderAtt(m_Body2WGS84, m_num);
	OrderAtt(m_Body2Orbit, m_num);
}

void GeoAttitude_ZY3::ReadZY3RealAttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath)
{
	// 清空数据
	ClearData();

	int num;
	FILE *fp = fopen((workpath + "\\WGS84ToJ2000.txt").c_str(), "r");
	fscanf(fp, "%d\n", &num);

	// 获得参数
	m_Input = input;
	m_Orbit = orbit;
	// 累计秒从2009年1月1日0时开始计数
	double jd0;
	Cal2JD(2009, 1, 1, 0, &jd0, &m_Input.refMJD);
	long m = allAtt.size();
	int i;
	vector<StrAttPoint> m_Body2WGS84tmp;
	vector<StrAttPoint> m_Body2Orbittmp;
	// 读取姿态文件
	for (i = 0; i < m; i++)
	{
		// 开始读取
		StrAttPoint tmp;
		double R[9], R1[9];
		tmp.UT = allAtt.at(i).UTC;//对应仿真姿轨数据，以下四元数是Cbj
								  //tmp.UT = allAtt.at(i).UTC -28800;//对应真实姿轨数据
		tmp.q[0] = allAtt.at(i).Q0, tmp.q[1] = allAtt.at(i).Q1, tmp.q[2] = allAtt.at(i).Q2, tmp.q[3] = allAtt.at(i).Q3;

		m_Base.FromSecondtoYMD(m_Input.refMJD, tmp.UT, tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second);
		m_Base.Quat2Matrix(tmp.q[1], tmp.q[2], tmp.q[3], tmp.q[0], tmp.R);	// J20002body									
		m_Base.Transpose(tmp.R, 3, 3);										// body2J2000
		memcpy(R1, tmp.R, 9 * sizeof(double));								// 保存
		fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]);// WGS84ToJ2000
		//m_Orbit->GetJ20002WGS84Rotation(tmp.UT, R);							// J20002WGS84
		m_Base.invers_matrix(R, 3);
		m_Base.Multi(R, R1, tmp.R, 3, 3, 3);								// body2WGS84
		m_Base.Matrix2Quat(tmp.R, tmp.q[0], tmp.q[1], tmp.q[2], tmp.q[3]);
		m_Base.Matrix2Eulor(tmp.R, 213, tmp.Eulor[0], tmp.Eulor[1], tmp.Eulor[2]);
		for (int i = 0; i < 3; i++)
			tmp.Eulor[i] = tmp.Eulor[i] * 180 / PI;
		m_Body2WGS84tmp.push_back(tmp);
		// 开始考虑body2orbit
		m_Orbit->GetJ20002OrbitRotation(tmp.UT, R);							// J20002orbit
		m_Base.Multi(R, R1, tmp.R, 3, 3, 3);								// body2orbit
		m_Base.Matrix2Quat(tmp.R, tmp.q[0], tmp.q[1], tmp.q[2], tmp.q[3]);
		m_Base.Matrix2Eulor(tmp.R, 213, tmp.Eulor[0], tmp.Eulor[1], tmp.Eulor[2]);
		for (int i = 0; i < 3; i++)
			tmp.Eulor[i] = tmp.Eulor[i] * 180 / PI;
		m_Body2Orbittmp.push_back(tmp);
	}
	fclose(fp);
	m_num = m_Body2WGS84tmp.size();
	// 存储
	m_Body2WGS84 = new StrAttPoint[m_num];
	m_Body2Orbit = new StrAttPoint[m_num];
	for (int i = 0; i < m_num; i++)
	{
		m_Body2WGS84[i] = m_Body2WGS84tmp[i];
		m_Body2Orbit[i] = m_Body2Orbittmp[i];
	}
	m_Body2WGS84tmp.clear();
	m_Body2Orbittmp.clear();
	// 排序
	OrderAtt(m_Body2WGS84, m_num);
	OrderAtt(m_Body2Orbit, m_num);
}

//滤波后，更新姿态
void GeoAttitude_ZY3::upDateAtt(vector<Attitude> allAtt, StrAttParamInput input)
{
	// 清空数据
	ClearData();

	// 获得参数
	m_Input = input;

	vector<StrAttPoint> m_Body2WGS84tmp;
	// 读取姿态文件
	for (int i = 0; i < allAtt.size(); i++)
	{
		// 开始读取
		StrAttPoint tmp;
		double R[9], R1[9];
		tmp.UT = allAtt.at(i).UTC;//对应仿真姿轨数据，以下四元数是Ctb
		//tmp.UT = allAtt.at(i).UTC -28800;//对应真实姿轨数据
		tmp.q[0] = allAtt.at(i).Q1, tmp.q[1] = allAtt.at(i).Q2, tmp.q[2] = allAtt.at(i).Q3, tmp.q[3] = allAtt.at(i).Q0;
		m_Base.Quat2Matrix(tmp.q[1], tmp.q[2], tmp.q[3], tmp.q[0], tmp.R);
		m_Base.Matrix2Eulor(tmp.R, 213, tmp.Eulor[0], tmp.Eulor[1], tmp.Eulor[2]);
		for (int i = 0; i < 3; i++)
			tmp.Eulor[i] = tmp.Eulor[i] * 180 / PI;
		m_Body2WGS84tmp.push_back(tmp);
	
	}
	
	m_num = m_Body2WGS84tmp.size();
	// 存储
	m_Body2WGS84 = new StrAttPoint[m_num];
	m_Body2Orbit = new StrAttPoint[m_num];
	for (int i = 0; i < m_num; i++)
	{
		m_Body2WGS84[i] = m_Body2WGS84tmp[i];
	}
	m_Body2WGS84tmp.clear();
	// 排序
	OrderAtt(m_Body2WGS84, m_num);
}


void GeoAttitude_ZY3::TransZY3AttFile(vector<Attitude> &allAtt,string EopPath)
{
	// 累计秒从2009年1月1日0时开始计数
	double jd0;
	Cal2JD(2009, 1, 1, 0, &jd0, &m_Input.refMJD);
	long m = allAtt.size();
	int i;
	vector<StrAttPoint> m_Body2WGS84tmp;
	// 读取姿态文件
	for (i = 0; i < m; i++)
	{
		// 开始读取
		StrAttPoint tmp;
		double R[9], R1[9];
		tmp.UT = allAtt.at(i).UTC;//对应仿真姿轨数据，以下四元数是Cbj
		tmp.q[0] = allAtt.at(i).Q0, tmp.q[1] = allAtt.at(i).Q1, tmp.q[2] = allAtt.at(i).Q2, tmp.q[3] = allAtt.at(i).Q3;

		m_Base.FromSecondtoYMD(m_Input.refMJD, tmp.UT, tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second);
		m_Base.Quat2Matrix(tmp.q[1], tmp.q[2], tmp.q[3], tmp.q[0], tmp.R);	// J20002body									
		m_Base.Transpose(tmp.R, 3, 3);										// body2J2000
		memcpy(R1, tmp.R, 9 * sizeof(double));								// 保存
		IAU2000ABaseCIOCelToTer(tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second, (char*)EopPath.c_str(), 2, R);// J20002WGS84
		m_Base.Multi(R, R1, tmp.R, 3, 3, 3);								// body2WGS84
		m_Base.Matrix2Quat(tmp.R, tmp.q[0], tmp.q[1], tmp.q[2], tmp.q[3]);
		m_Body2WGS84tmp.push_back(tmp);	
	}
	allAtt.clear();
	for (int i=0;i<m_Body2WGS84tmp.size();i++)
	{
		Attitude tmp;
		tmp.UTC = m_Body2WGS84tmp[i].UT;
		tmp.Q1 = m_Body2WGS84tmp[i].q[0]; tmp.Q2 = m_Body2WGS84tmp[i].q[1];
		tmp.Q3 = m_Body2WGS84tmp[i].q[2]; tmp.Q0 = m_Body2WGS84tmp[i].q[3];
		allAtt.push_back(tmp);
	}
}