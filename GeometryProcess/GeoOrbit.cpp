#include "GeoOrbit.h"

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoOrbit::GeoOrbit(void)
{
	m_EpWGS84 = NULL;       // 存储WGS84系下真实轨道点信息
	m_EpJ2000 = NULL;       // 存储J2000系下真实轨道点信息
	ClearData();
}


GeoOrbit::~GeoOrbit(void)
{
	ClearData();
}


// 清空数据
void GeoOrbit::ClearData()
{
	m_num  = 0;
	if(m_EpWGS84!=NULL) { delete []m_EpWGS84; m_EpWGS84 = NULL; }
	if(m_EpJ2000!=NULL) { delete []m_EpJ2000; m_EpJ2000 = NULL; }
	memset(&m_modelWGS84, 0, sizeof(StrOrbitPolyModel));
	memset(&m_modelJ2000, 0, sizeof(StrOrbitPolyModel));
}

// 获取所用的参考椭球参数
StrDATUM GeoOrbit::get_datum()	
{	
	StrDATUM datum;
	m_Base.GetRefEllipsoid(m_Input.DatumName, datum);
	return datum;	
};

// 获得参考历元
double GeoOrbit::get_RefUT()	{	return m_Input.refMJD;	}					

// 获取轨道点个数
long GeoOrbit::get_num()	{	return m_num;	}

// 轨道相位中心相对本体中心的偏心距X(定轨-本体)
double GeoOrbit::get_OffX()	{	return m_Input.m_Off[0];	}

// 轨道相位中心相对本体中心的偏心距Y(定轨-本体)
double GeoOrbit::get_OffY()	{	return m_Input.m_Off[1];	}

// 轨道相位中心相对本体中心的偏心距Z(定轨-本体)
double GeoOrbit::get_OffZ()	{	return m_Input.m_Off[2];	}


//////////////////////////////////////
// 功能：获取轨道拟合模型
// 返回值：
//		StrOrbitPolyModel：		轨道拟合模型
//////////////////////////////////////
StrOrbitPolyModel *GeoOrbit::GetPolyModel()
{
	return &m_modelWGS84;
}


//////////////////////////////////////
// 功能：设置轨道拟合模型
// 输入：
//		StrOrbitPolyModel attModel：	轨道拟合模型
// 返回值：
//		void：	
//////////////////////////////////////
void GeoOrbit::SetPolyModel(StrOrbitPolyModel orbModel)
{
	m_modelWGS84 = orbModel;
}


//////////////////////////////////////
// 功能：修正轨道拟合模型系数
// 输入：
//		StrOrbitPolyModel attModel：	轨道拟合模型
// 返回值：
//		void：	
//////////////////////////////////////
void GeoOrbit::ModifyPolyModelPara(double *para)
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


////////////////////////////////////////////////////////
// 内插轨道
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：获取指定时间的WGS84的轨道信息
// 输入:
//		double UT:		从起始历元开始的累计秒
// 返回值：
//		StrOrbitPoint：	内插出来的WGS84轨道点信息
//////////////////////////////////////
StrOrbitPoint GeoOrbit::GetEpWGS84(double UT)
{
	StrOrbitPoint pVal;
	m_Base.LagrangianInterpolation(m_EpWGS84, m_num, UT, pVal, m_Input.m_InterOrder);
	return pVal;
}


//////////////////////////////////////
// 功能：获取指定时间的J2000的轨道信息
// 输入:
//		double UT:		从起始历元开始的累计秒
// 返回值：
//		StrOrbitPoint：	内插出来的J2000轨道点信息
////////////////////////////////////// 
StrOrbitPoint GeoOrbit::GetEpJ2000(double UT)
{
	StrOrbitPoint pVal;
	m_Base.LagrangianInterpolation(m_EpJ2000, m_num, UT, pVal, m_Input.m_InterOrder);
	return pVal;
}


//////////////////////////////////////
// 返回索引处WGS84系下轨道点信息
//////////////////////////////////////
StrOrbitPoint GeoOrbit::get_m_EpWGS84(long index)
{
	if( index>-1 && index<m_num )
		return m_EpWGS84[index];
	else
	{
		StrOrbitPoint temp;
		return temp;
	}
}


//////////////////////////////////////
// 返回索引处J2000系下轨道点信息
//////////////////////////////////////
StrOrbitPoint GeoOrbit::get_m_EpJ2000(long index)
{
	if( index>-1 && index<m_num )
		return m_EpJ2000[index];
	else
	{
		StrOrbitPoint temp;
		return temp;
	}
}

//////////////////////////////////////
// 获得轨道的仿真的起始时间
//////////////////////////////////////
double GeoOrbit::get_m_startTime()
{
	if(m_num>0)
		return m_EpWGS84[0].UT;
	else
		return 0.0;
}


//////////////////////////////////////
// 获得轨道的仿真的结束时间
//////////////////////////////////////
double GeoOrbit::get_m_endTime()
{
	if(m_num>0)
		return m_EpWGS84[m_num-1].UT;
	else
		return 0.0;
}


////////////////////////////////////////////////////////
// 轨道坐标系变换
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：获取从J2000到轨道坐标系的旋转矩阵(轨道坐标系定义在J2000系下)
// 输入:
//		double UT:		从起始历元开始的累计秒
//		bool isJ2000:	是否是相对J2000坐标系,如果为false,则所有定义都换成WGS84
// 输出：
//		double *R:		旋转矩阵3*3，行优先	
// 返回值：
//		void
////////////////////////////////////// 
void GeoOrbit::GetJ20002OrbitRotation(double UT, double *R, bool isJ2000)
{
	// 获得UT时刻轨道点在J2000系下的信息
	StrOrbitPoint m_Point;
	if(isJ2000)	m_Point = GetEpJ2000(UT);
	else		m_Point = GetEpWGS84(UT);
	// 获得轨道坐标系三个轴在J2000系下的指向
	// 轨道坐标系：
    //	 0点：当前时刻卫星质心所处的轨道位置；
    //	 X轴：位于轨道面内，垂直于卫星与地心连线，指向卫星前进方向为正向；
    //	 Z轴：位于轨道面内，指向地心方向；
    //	 Y轴：垂直于轨道面内，指向由右手法则确定。
	double X[3], Y[3], Z[3];
	X[0] = m_Point.X[3];    X[1] = m_Point.X[4];   X[2] = m_Point.X[5]; 
	Z[0] = -m_Point.X[0];   Z[1] = -m_Point.X[1];  Z[2] = -m_Point.X[2];
	m_Base.CrossMult(Z, X, Y);
	m_Base.CrossMult(Y, Z, X);
	// 归一化
	m_Base.NormVector(X,3);
	m_Base.NormVector(Y,3);
	m_Base.NormVector(Z,3);
	// 构建旋转矩阵
	R[0] = X[0];     R[1] = X[1];     R[2] = X[2];
	R[3] = Y[0];     R[4] = Y[1];     R[5] = Y[2];
	R[6] = Z[0];     R[7] = Z[1];     R[8] = Z[2];
}


//////////////////////////////////////
// 功能：获取从轨道坐标系到J2000的旋转矩阵(轨道坐标系定义在J2000系下)
// 输入:
//		double UT:		从起始历元开始的累计秒
//		bool isJ2000:	是否是相对J2000坐标系,如果为false,则所有定义都换成WGS84
// 输出：
//		double *R:		旋转矩阵3*3，行优先	
// 返回值：
//		void
////////////////////////////////////// 
void GeoOrbit::GetOrbit2J2000Rotation(double UT, double *R, bool isJ2000)
{
	// 获得UT时刻轨道点在J2000系下的信息
	StrOrbitPoint m_Point;
	if(isJ2000)	m_Point = GetEpJ2000(UT);
	else		m_Point = GetEpWGS84(UT);
	// 获得轨道坐标系三个轴在J2000系下的指向
	// 轨道坐标系：
    //	 0点：当前时刻卫星质心所处的轨道位置；
    //	 X轴：位于轨道面内，垂直于卫星与地心连线，指向卫星前进方向为正向；
    //	 Z轴：位于轨道面内，指向地心方向；
    //	 Y轴：垂直于轨道面内，指向由右手法则确定。
	double X[3], Y[3], Z[3];
	X[0] = m_Point.X[3];    X[1] = m_Point.X[4];   X[2] = m_Point.X[5];
	Z[0] = -m_Point.X[0];    Z[1] = -m_Point.X[1];   Z[2] = -m_Point.X[2];
	m_Base.CrossMult(Z, X, Y);
	m_Base.CrossMult(Y, Z, X);
	// 归一化
	m_Base.NormVector(X,3);
	m_Base.NormVector(Y,3);
	m_Base.NormVector(Z,3);
	// 构建旋转矩阵
	R[0] = X[0];     R[1] = Y[0];     R[2] = Z[0];
	R[3] = X[1];     R[4] = Y[1];     R[5] = Z[1];
	R[6] = X[2];     R[7] = Y[2];     R[8] = Z[2];
}


////////////////////////////////////////////////////////
// 轨道多项式拟合
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：根据时间段获得对应轨道的多项式模型
// 输入:
//		startUT：	开始时间
//		endUT：		结束时间
//		isJ2000:	如果为ture,对J2000轨道拟合
//					如果为false,对WGS84轨道拟合
//					默认为false
// 返回值：
//		void
////////////////////////////////////// 
void GeoOrbit::GenPolyModel(double startUT, double endUT, bool isJ2000)
{
	long order = m_Input.m_PolyOrder;
	StrOrbitPolyModel *model = NULL;
	if(isJ2000) model = &m_modelJ2000;
	else		model = &m_modelWGS84;
	order += 1;		// n阶多项式至少需要用到n+1个观测值
	// 首先清空多项式结构体
	long i;
	memset(model, 0, sizeof(StrOrbitPolyModel));
	model->m_adjustOrder = m_Input.m_adjustOrder;
	long index_left, index_right, num;
	double *err[7], *T[7];
	for(int i=0; i<7; i++)	
	{	
		err[i] = NULL;	T[i] = NULL;	
	}
	// 寻找数据点
	num = m_num;
	index_left = 0;
	for(i=0; i<num-1; i++)	
		if((startUT>=m_EpWGS84[i].UT) && (startUT<=m_EpWGS84[i+1].UT))	
		{	index_left = i;	break;	}
	index_right = num-1;
	for(i=num-1; i>=1; i--)	
		if((endUT>=m_EpWGS84[i-1].UT) && (endUT<=m_EpWGS84[i].UT))
		{	index_right = i;	break;	}
	// 开始算多少点参与多项式拟合
	index_left -= order/2;		if(index_left<0) index_left=0;
	index_right += order/2;		if(index_right>num-1)	index_right = num-1;
	if(index_right-index_left < order)	model->PolyOrder = index_right-index_left-1;	// 观测值数少于阶数,则阶数设为和观测值数一样
	else								model->PolyOrder = order;						// 否则和用户指定的一样
	num = index_right-index_left+1;
	for(i=0; i<7; i++)	
	{	
		err[i] = new double[num];	T[i] = new double[num];
	}
	// 赋值
	long num1 = 0;
	for(i=index_left; i<=index_right; i++)
	{
		T[6][num1] = m_EpWGS84[i].UT;
		T[0][num1] = m_EpWGS84[i].X[0];	T[1][num1] = m_EpWGS84[i].X[1];	T[2][num1] = m_EpWGS84[i].X[2];
		T[3][num1] = m_EpWGS84[i].X[3];	T[4][num1] = m_EpWGS84[i].X[4];	T[5][num1] = m_EpWGS84[i].X[5];
		num1++;
	}
	// t
	m_Base.Compute_avAnddx(T[6], num, model->Toff[6], model->Tscale[6]);
	m_Base.Normaliza_avAnddx(T[6], num, model->Toff[6], model->Tscale[6]);
	// X,Y,Z,Vx,Vy,Vz
	for(int i=0; i<6; i++)
	{
		m_Base.Compute_avAnddx(T[i], num, model->Toff[i], model->Tscale[i]);
		m_Base.Normaliza_avAnddx(T[i], num, model->Toff[i], model->Tscale[i]);
		m_Base.PolynominalFittingError(T[6], T[i], num, model->PolyOrder, err[i], model->T[i]);
		m_Base.DNormaliza_avAnddx(err[i], num, 0, model->Tscale[i]);
	}
	// 输出拟合残差信息
//	printf("%ld个点轨道拟合残差(m,m/s):\n%16.6lf\t%16.6lf\t%16.6lf\n%16.6lf\t%16.6lf\t%16.6lf\n", num,
//		m_Base.FabsAndAve(err[0], num), m_Base.FabsAndAve(err[1], num), m_Base.FabsAndAve(err[2], num), 
//		m_Base.FabsAndAve(err[3], num), m_Base.FabsAndAve(err[4], num), m_Base.FabsAndAve(err[5], num));
	// 释放内存
	for(int i=0; i<7; i++)
	{
		delete []err[i];	err[i] = NULL;
		delete []T[i];		T[i] = NULL;
	}
	model = NULL;
}


//////////////////////////////////////
// 功能：提供时间根据多项式模型获得对应的轨道点信息
// 输入:
//		startUT：	开始时间
//		isJ2000:	如果为ture,对J2000轨道进行
//					如果为false,对WGS84轨道进行
//					默认为false
// 返回值：
//		StrOrbitPoint：返回的轨道点
////////////////////////////////////// 
StrOrbitPoint GeoOrbit::PolyValue(double UT, bool isJ2000)
{
	StrOrbitPolyModel *model = NULL;
	if(isJ2000) model = &m_modelJ2000;
	else		model = &m_modelWGS84;
	StrOrbitPoint pVal;
	memset(&pVal, 0, sizeof(struct StrOrbitPoint));
	pVal.UT = UT;
	double t = (UT - model->Toff[6])/model->Tscale[6];
	double temp[6];
	for(int i=0; i<6; i++)
	{
		m_Base.PolyValue(model->T[i], model->Tdelta[i], t, &(temp[i]));	
		temp[i] = temp[i]*model->Tscale[i] + model->Toff[i];
	}
	memcpy(pVal.X, temp, sizeof(double)*6);
	model = NULL;
	return pVal;
}


//////////////////////////////////////
// 功能：轨道根据时间排序
// 输入:
//		
// 返回值：
//		
////////////////////////////////////// 
void GeoOrbit::OrderOrbit(StrOrbitPoint *m_point, int num)
{
	StrOrbitPoint temp;
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
// 读取轨道文件
void GeoOrbit::ReadEphFile(string filepath, StrOrbitParamInput input){}
// 读取轨道
void GeoOrbit::ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input){}
// 读取ZY3轨道
void GeoOrbit::ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input){}		
// 写出轨道文件
void GeoOrbit::WriteEphFile(string filepath, bool iserror){}
// 根据指定时间获取从J2000到WGS84的旋转矩阵
void GeoOrbit::GetJ20002WGS84Rotation(double UT, double *R){}
// 根据指定时间获取从WGS84到J2000的旋转矩阵
void GeoOrbit::GetWGS842J2000Rotation(double UT, double *R){}
// 根据指定时间将位置速度从J2000转化到WGS84
void GeoOrbit::State_J20002WGS84(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz){}
// 根据指定时间将位置速度从WGS84转化到J2000
void GeoOrbit::State_WGS842J2000(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz){}

