#include "GeoModel.h"
#include "GeoModelRFM.h"


//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoModel::GeoModel(void)
{
	m_orbit = NULL;		// 指向轨道对象
	m_att = NULL;		// 指向姿态对象
	m_time = NULL;		// 指向时间对象
	m_cam = NULL;		// 指向相机对象
	//m_range = NULL;		// 指向测距对象
	delDis = 0;			// 距离修正
	// 偏置修正
	Ru[0] = 1;	Ru[1] = 0;	Ru[2] = 0;
	Ru[3] = 0;	Ru[4] = 1;	Ru[5] = 0;
	Ru[6] = 0;	Ru[7] = 0;	Ru[8] = 1;
	// 仿射修正
	// x = a[0] + a[1]*x + a[2]*y;
	// y = a[3] + a[4]*x + a[5]*y;
	m_affine[0] = 0.0;	m_affine[1] = 1.0;	m_affine[2] = 0.0;
	m_affine[3] = 0.0;	m_affine[4] = 0.0;	m_affine[5] = 1.0;
}

GeoModel::~GeoModel(void)
{
	//if(m_orbit!=NULL)	delete []m_orbit;	m_orbit = NULL;	// 指向轨道对象
	//if(m_att!=NULL)		delete []m_att;		m_att = NULL;	// 指向姿态对象
	//if(m_time!=NULL)	delete []m_time;	m_time = NULL;	// 指向时间对象
	//if(m_cam!=NULL)		delete []m_cam;		m_cam = NULL;	// 指向相机对象
	//if(m_range!=NULL)	delete []m_range;	m_range = NULL;	// 指向测距对象
}


//////////////////////////////////////////////////////////////////////////
// 以下函数由各个子类实现
//////////////////////////////////////////////////////////////////////////
// 计算姿轨多项式模型以及整数位的姿轨以及常见的安装
void GeoModel::ReCalPosAndAtt(){}
// 被动遥感从影像到地面
bool GeoModel::FromXY2LatLon(double x, double y, double &H, double &lat, double &lon) { return true; }
// 被动遥感从地面到影像
void GeoModel::FromLatLon2XY(double lat, double lon, double H, double &x, double &y){}
// 获取缩放比例
double GeoModel::GetScale(){ return 0; }
// 根据xy像素坐标得到轨道姿态和内方位元素
void GeoModel::GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner){}
// 根据物点和像点坐标得到对应的偏导数
void GeoModel::GetPartialDerivative(double lat, double lon, double H, double scale, double &x, double &y, 
				double &x_lat, double &y_lat, double &x_lon, double &y_lon, double &x_h, double &y_h){}
// 获取RFM模型的模型参数
struct StrRFMData GeoModel::GetRFMModel(){ struct StrRFMData temp; return temp; }
// 获取全局变换参数以及经纬度/像素
void GeoModel::ComputerGlobalParam(){};

// 获得影像的高度
long GeoModel::get_m_height()
{
	return m_xnum;
}

// 获得影像的宽度
long GeoModel::get_m_width()
{
	return m_ynum;
}

// 获得轨道对象
GeoOrbit* GeoModel::GetOrbitObject()
{
	return m_orbit;
}

// 获得姿态对象
GeoAttitude* GeoModel::GetAttitudeObject()
{
	return m_att;
}

// 获得时间对象
GeoTime* GeoModel::GetTimeObject()
{
	return m_time;
}

// 获得相机对象
GeoCamera* GeoModel::GetCameraObject()
{
	return m_cam;
}

// 获得测距对象
//GeoRange* GeoModel::GetRangeObject()
//{
//	return m_range;
//}


// 获取轨道拟合模型
StrOrbitPolyModel* GeoModel::GetOrbPolyModel()
{
	return m_orbit->GetPolyModel();
}


// 设置轨道拟合模型
void GeoModel::SetOrbPolyModel(StrOrbitPolyModel orbModel)
{
	m_orbit->SetPolyModel(orbModel);
}

//////////////////////////////////////
// 功能：修正轨道拟合模型系数
// 输入：
//		StrOrbitPolyModel attModel：	轨道拟合模型
// 返回值：
//		void：	
//////////////////////////////////////
void GeoModel::ModifyOrbitPolyModelPara(double *para)
{
	m_orbit->ModifyPolyModelPara(para);
}


// 获取姿态拟合模型
StrAttPolyModel* GeoModel::GetAttPolyModel()
{
	return m_att->GetPolyModel();
}


// 设置姿态拟合模型
void GeoModel::SetAttPolyModel(StrAttPolyModel attModel)
{
	m_att->SetPolyModel(attModel);
}


//////////////////////////////////////
// 功能：修正姿态拟合模型系数
// 输入：
//		StrOrbitPolyModel attModel：	轨道拟合模型
// 返回值：
//		void：	
//////////////////////////////////////
void GeoModel::ModifyAttPolyModelPara(double *para)
{
	m_att->ModifyPolyModelPara(para);
}


// 获取仿射模型参数
double *GeoModel::GetModelAffinePara()
{
	return m_affine;
}


// 设置仿射模型参数
void GeoModel::SetModelAffinePara(double *para)
{
	memcpy(m_affine, para, sizeof(double)*6);
}

//////////////////////////////////////////////////////////////////
//一像素大小在地面的投影,是一个比例系数:经纬度/像素
//////////////////////////////////////////////////////////////////
double GeoModel::DistanceOnePixel(double x, double y, double H, double dx, double dy)
{
	double lat1, lat2, lon1, lon2;
	FromXY2LatLon(x, y, H, lat1, lon1);
	FromXY2LatLon(x + dx, y + dy, H, lat2, lon2);
	double dis = sqrt(pow((lat1 - lat2), 2) + pow((lon1 - lon2), 2) / (dx*dx + dy*dy));
	return dis;
}
//////////////////////////////////////
// 功能：严密模型的前方交会
// 输入:
//		long num:			用于前方交会的像片的数目
//		GeoModel **model：	用于前方交会的像片的成像模型,与num对应
//		double *x:			用于前方交会的同名点x坐标(沿轨)
//		double *y:			用于前方交会的同名点y坐标(垂轨)
// 输出：
//		double &X：			定位点的X坐标
//		double &Y：			定位点的Y坐标
//		double &Z:			定位点的Z坐标
// 返回值：
//		void
//////////////////////////////////////
void GeoModel::RigorousForwardIntersection(long num, GeoModel **model, double *x, double *y, double &X, double &Y, double &Z)
{
	double eph[3], R[9], inner[3];
	// 循环获取
	double a[3], aa[9], al[3], L;
	memset(aa,0, sizeof(double)*9);		memset(al,0, sizeof(double)*3);
	for(long i=0; i<num; i++)
	{
		// 根据像素坐标得到姿轨内方位元素信息
		model[i]->GetOrbitAttitudeInnerBaseXY(x[i], y[i], eph, R, inner);
		// 第一个方程
		a[0] = R[0] - R[2]*inner[0];
		a[1] = R[3] - R[5]*inner[0];
		a[2] = R[6] - R[8]*inner[0];
		L = a[0]*eph[0] + a[1]*eph[1] + a[2]*eph[2];
		m_base.pNormal(a, 3, L, aa, al, 1.0);
		// 第二个方程
		a[0] = R[1] - R[2]*inner[1];
		a[1] = R[4] - R[5]*inner[1];
		a[2] = R[7] - R[8]*inner[1];
		L = a[0]*eph[0] + a[1]*eph[1] + a[2]*eph[2];
		m_base.pNormal(a, 3, L, aa, al, 1.0);
	}
	m_base.solve33(aa, al);
	X = al[0];	Y = al[1];	Z = al[2];
}



//////////////////////////////////////////////////////////////////////////
// 各类检校
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：地形匹配进行激光器检校
// 输入:
//		long num:			参与检校的点数目
//		GeoModel *model：	距离成像模型
//		long *index:		激光点对应的索引号
//		double *X：			对应地面点X
//		double *Y:			对应地面点Y
//		double *Z：			对应地面点Z
// 输出：
//		double &deltaT：	检校得到的时间修正
//		double &deltaPhi：	检校得到的俯仰角修正
//		double &deltaOmega：检校得到的滚动角修正
// 返回值：
//		void
//////////////////////////////////////
void GeoModel::TopoMatchForRangeCalibration3(long num, GeoModel *model, long *index, double *X, 
						double *Y, double *Z, double &deltaT, double &deltaPhi, double &deltaOmega)
{
	double eph[3], R[9], inner[3], XYZ[3], a3, b3, c3;
	double a[3], aa[9], al[3], L;
	// 迭代求解
	do
	{
		a3 = sin(deltaPhi);
		b3 = -sin(deltaOmega)*cos(deltaPhi);
		c3 = cos(deltaOmega)*cos(deltaPhi);
		// 循环获取
		memset(aa,0, sizeof(double)*9);		memset(al,0, sizeof(double)*3);
		for(long i=0; i<num; i++)
		{
			// 根据像素坐标得到姿轨内方位元素信息
			model->GetOrbitAttitudeInnerBaseXY(index[i], 0, eph, R, inner);
			// 求取XYZ[3]
			m_base.Transpose(R, 3, 3);
			eph[0] = X[i] - eph[0];
			eph[1] = Y[i] - eph[1];
			eph[2] = Z[i] - eph[2];
			m_base.Multi(R, eph, XYZ, 3, 3, 1);
			// 第一个方程
			a[0] = a3;
			a[1] = (inner[0]+deltaT)*cos(deltaPhi);
			a[2] = 0;
			L = XYZ[0]-(inner[0]+deltaT)*a3;
			m_base.pNormal(a, 3, L, aa, al, 1.0);
			// 第二个方程
			a[0] = b3;
			a[1] = (inner[0]+deltaT)*sin(deltaOmega)*sin(deltaPhi);
			a[2] =-(inner[0]+deltaT)*cos(deltaOmega)*cos(deltaPhi);
			L = XYZ[1]-(inner[0]+deltaT)*b3;
			m_base.pNormal(a, 3, L, aa, al, 1.0);
			// 第三个方程
			a[0] = c3;
			a[1] =-(inner[0]+deltaT)*cos(deltaOmega)*sin(deltaPhi);
			a[2] =-(inner[0]+deltaT)*sin(deltaOmega)*cos(deltaPhi);
			L = XYZ[2]-(inner[0]+deltaT)*c3;
			m_base.pNormal(a, 3, L, aa, al, 1.0);
		}
		m_base.solve33(aa, al);
		deltaT += al[0];	
		deltaPhi += al[1];	
		deltaOmega += al[2];
	}while((fabs(al[0])>1)||(fabs(al[1])>10e-6)||(fabs(al[2])>10e-6));
}



//////////////////////////////////////////////////////////////////////////
// 各类检校参数设置
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：测距检校参数设置
// 输入:
//		double dis:		距离改正值
//		double phi：	俯仰改正值(弧度)
//		double omega:	滚动改正值(弧度)
// 返回值：
//		void
//////////////////////////////////////
void GeoModel::SetRangeCalPara(double dis, double phi, double omega)
{
	delDis = dis;
	double cosphi, cosomg, sinphi, sinomg;
	cosphi = cos(phi);		sinphi = sin(phi);
	cosomg = cos(omega);	sinomg = sin(omega);
	Ru[0] = cosphi;			Ru[1] = 0;			Ru[2] = sinphi;
	Ru[3] = sinomg*sinphi;	Ru[4] = cosomg;		Ru[5] = -sinomg*cosphi;
	Ru[6] = -cosomg*sinphi;	Ru[7] = sinomg;		Ru[8] = cosomg*cosphi;
}

