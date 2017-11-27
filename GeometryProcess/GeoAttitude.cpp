#include "GeoAttitude.h"

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoAttitude::GeoAttitude(void)
{
	m_Body2WGS84 = NULL;
	m_Body2Orbit = NULL;
}


GeoAttitude::~GeoAttitude(void)
{
	ClearData();
}

// 清空数据
void GeoAttitude::ClearData()
{
	m_num  = 0;
	if(m_Body2WGS84!=NULL) { delete []m_Body2WGS84; m_Body2WGS84 = NULL; }
	if(m_Body2Orbit!=NULL) { delete []m_Body2Orbit; m_Body2Orbit = NULL; }
	memset(&m_modelWGS84, 0, sizeof(StrAttPolyModel));
}


// 获取轨道点个数
long GeoAttitude::get_num()
{
	return m_num;
}

// 获取从测姿坐标系到本体坐标系的旋转矩阵
void GeoAttitude::get_ROff(double *R)
{
	memcpy(R, m_Input.ROff, sizeof(double)*9);
}

//设置从测姿坐标系到本体坐标系的旋转矩阵
void GeoAttitude::set_ROff(double *R)
{
	memcpy(m_Input.ROff, R, sizeof(double) * 9);
}

//////////////////////////////////////
// 功能：获取姿态拟合模型
// 返回值：
//		StrAttPolyModel：		轨道拟合模型
//////////////////////////////////////
StrAttPolyModel *GeoAttitude::GetPolyModel()
{
	return &m_modelWGS84;
}


//////////////////////////////////////
// 功能：设置姿态拟合模型
// 输入：
//		StrAttPolyModel attModel：	姿态拟合模型
// 返回值：
//		void：	
//////////////////////////////////////
void GeoAttitude::SetPolyModel(StrAttPolyModel attModel)
{
	m_modelWGS84 = attModel;
}


//////////////////////////////////////
// 功能：修正姿态拟合模型系数
// 输入：
//		StrOrbitPolyModel attModel：	轨道拟合模型
// 返回值：
//		void：	
//////////////////////////////////////
void GeoAttitude::ModifyPolyModelPara(double *para)
{
	long index = 0;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<m_modelWGS84.m_adjustOrder; j++)
		{
			m_modelWGS84.T[i][j] = para[index];
			index++;
		}
	}
}

//////////////////////////////////////
// 返回索引处姿态点信息
//////////////////////////////////////
StrAttPoint GeoAttitude::get_m_Body2WGS84(long index)
{
	if( index>-1 && index<m_num )
		return m_Body2WGS84[index];
	else
	{
		StrAttPoint temp;
		return temp;
	}
}

//////////////////////////////////////
// 返回索引处姿态信息
//////////////////////////////////////
StrAttPoint GeoAttitude::get_m_Body2Orbit(long index)
{
	if( index>-1 && index<m_num )
		return m_Body2Orbit[index];
	else
	{
		StrAttPoint temp;
		return temp;
	}
}


////////////////////////////////////////////////////////
// 内插姿态
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：获取指定时间的从本体坐标系到WGS84坐标系的旋转矩阵信息
// 输入:
//		double UT:		从起始历元开始的累计秒
// 返回值：
//		StrAttPoint：	内插出来的从本体坐标系到WGS84坐标系的旋转矩阵信息
//////////////////////////////////////
StrAttPoint GeoAttitude::GetAttBody2WGS84(double UT)
{
	StrAttPoint pVal;
	m_Base.QuatInterpolation(m_Body2WGS84, m_num, UT, pVal);
	m_Base.Quat2Matrix(pVal.q[0], pVal.q[1], pVal.q[2], pVal.q[3], pVal.R);
	return pVal;
}


//////////////////////////////////////
// 功能：获取指定时间的从本体坐标系到轨道坐标系(J2000系下定义)的旋转矩阵信息
// 输入:
//		double UT:		从起始历元开始的累计秒
// 返回值：
//		StrAttPoint：	内插出来的从本体坐标系到轨道坐标系(J2000系下定义)的旋转矩阵信息
////////////////////////////////////// 
StrAttPoint GeoAttitude::GetAttBody2Orbit(double UT)
{
	StrAttPoint pVal;
	m_Base.QuatInterpolation(m_Body2Orbit, m_num, UT, pVal);
	m_Base.Quat2Matrix(pVal.q[0], pVal.q[1], pVal.q[2], pVal.q[3], pVal.R);
	return pVal;
}



////////////////////////////////////////////////////////
// 姿态多项式拟合
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：根据时间段获得对应姿态的多项式模型，仅仅对Body2WGS84进行
// 输入:
//		startUT：	开始时间
//		endUT：		结束时间
// 返回值：
//		void
////////////////////////////////////// 
void GeoAttitude::GenPolyModel(double startUT, double endUT)
{
	long order = m_Input.m_PolyOrder;
	StrAttPolyModel *model = &m_modelWGS84;
	order += 1;		// n阶多项式至少需要用到n+1个观测值
	// 首先清空多项式结构体
	long i;
	memset(model, 0, sizeof(StrAttPolyModel));
	model->m_adjustOrder = m_Input.m_adjustOrder;
	model->Order = 213;
	long index_left, index_right, num;
	double *T[4], *err[4];
	for(i=0; i<4; i++)	
	{	
		err[i] = NULL;	T[i] = NULL;	
	}
	// 寻找数据点
	num = m_num;
	index_left = 0;
	for(i=0; i<num-1; i++)	
		if((startUT>=m_Body2WGS84[i].UT) && (startUT<=m_Body2WGS84[i+1].UT))	
		{	index_left = i;	break;	}
	index_right = num-1;
	for(i=num-1; i>=1; i--)	
		if((endUT>=m_Body2WGS84[i-1].UT) && (endUT<=m_Body2WGS84[i].UT))
		{	index_right = i;	break;	}
	// 开始算多少点参与多项式拟合
	index_left -= order/2;		if(index_left<0) index_left=0;
	index_right += order/2;		if(index_right>num-1)	index_right = num-1;
	if(index_right-index_left < order)	model->PolyOrder = index_right-index_left-1;	// 观测值数少于阶数,则阶数设为和观测值数一样
	else								model->PolyOrder = order;						// 否则和用户指定的一样
	num = index_right-index_left+1;
	for(i=0; i<4; i++)	
	{	
		err[i] = new double[num];	T[i] = new double[num];
	}
	long num1 = 0;
	for(i=index_left; i<=index_right; i++)
	{
		T[3][num1] = m_Body2WGS84[i].UT;
		m_Base.Matrix2Eulor(m_Body2WGS84[i].R, model->Order, T[0][num1], T[1][num1], T[2][num1]);
		num1++;
	}
	// t
	m_Base.Compute_avAnddx(T[3], num, model->Toff[3], model->Tscale[3]);
	m_Base.Normaliza_avAnddx(T[3], num, model->Toff[3], model->Tscale[3]);
	// Eulor1,Eulor2,Eulor3
	for(int i=0; i<3; i++)
	{
		m_Base.Compute_avAnddx(T[i], num, model->Toff[i], model->Tscale[i]);
		m_Base.Normaliza_avAnddx(T[i], num, model->Toff[i], model->Tscale[i]);
		m_Base.PolynominalFittingError(T[3], T[i], num, model->PolyOrder, err[i], model->T[i]);
		m_Base.DNormaliza_avAnddx(err[i], num, 0, model->Tscale[i]);
	}
	// 输出拟合残差信息
//	printf("%ld个点姿态拟合残差(°):\n%16.6lf\t%16.6lf\t%16.6lf\n", num, m_Base.FabsAndAve(err[0], num)*180/PI, 
//								  m_Base.FabsAndAve(err[1], num)*180/PI, m_Base.FabsAndAve(err[2], num)*180/PI);
	// 释放内存
	for(int i=0; i<4; i++)
	{
		delete []err[i];	err[i] = NULL;
		delete []T[i];		T[i] = NULL;
	}
	model = NULL;
}


//////////////////////////////////////
// 功能：根据姿态多项式获得姿态，仅仅对Body2WGS84进行
// 输入:
//		double startUT：	开始时间
// 输出：
//		double *R:			拟合得到的姿态旋转矩阵
// 返回值：
//		void
////////////////////////////////////// 
void GeoAttitude::PolyValue(double UT, double* R)
{
	StrAttPolyModel *model = &m_modelWGS84;
	double t = (UT - model->Toff[3])/model->Tscale[3];
	double temp[3];
	for(int i=0; i<3; i++)
	{
		m_Base.PolyValue(model->T[i], model->Tdelta[i], t, &(temp[i]));	
		temp[i] = temp[i]*model->Tscale[i] + model->Toff[i];
	}
	m_Base.Eulor2Matrix(temp[0], temp[1], temp[2], model->Order, R);
	model = NULL;
}




//////////////////////////////////////
// 功能：姿态根据时间排序
// 输入:
//		
// 返回值：
//		
////////////////////////////////////// 
void GeoAttitude::OrderAtt(StrAttPoint *m_point, int num)
{
	StrAttPoint temp;
	for(int i=0; i<num-1; i++)
	{
		for(int j=i+1; j<num; j++)
		{
			if(m_point[i].UT>m_point[j].UT)
			{
				temp = m_point[i];
				m_point[i] = m_point[j];
				m_point[j] = temp;
			}
		}
	}
}


////////////////////////////////////////////////////////
// 以下几个函数跟星球有关系,地球和外行星是不同的,需要分别实现,但是注意不要用纯虚函数,否则无法实例化
////////////////////////////////////////////////////////
// 读取姿态文件
void GeoAttitude::ReadAttFile(string filepath, StrAttParamInput input, GeoOrbit *orbit){}
// 读取姿态
void GeoAttitude::ReadAttNoFile(int num, StrAttPoint *point, StrAttParamInput input, GeoOrbit *orbit){}
// 读取ZY3姿态文件
void GeoAttitude::ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit) {}
void GeoAttitude::ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath){}
void GeoAttitude::ReadZY3RealAttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath) {}
void GeoAttitude::upDateAtt(vector<Attitude> allAtt, StrAttParamInput input)
{
}
// 写出姿态文件
void GeoAttitude::WriteAttFile(string filepath, bool iserror){}
// 对姿态进行高频修正
void GeoAttitude::ModifyAtt(string filepath){}






