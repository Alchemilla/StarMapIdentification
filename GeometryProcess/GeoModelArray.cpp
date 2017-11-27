// GeoModelBaseArray.cpp : Implementation of CGeoModelBaseArray
#include "GeoModelArray.h"


//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoModelArray::GeoModelArray(void)
{

}


GeoModelArray::~GeoModelArray(void)
{

}

// TDI成像模型初始化
void GeoModelArray::InitModelArray(GeoOrbit *orbit, GeoAttitude *att, GeoCamera *inner,
	StrModelParamInput param,double UT)
{
	// 获得各个对象
	m_input = param;
	m_orbit = orbit;
	m_att = att;
	m_cam = inner;
	// 获得影像大小
	m_xnum = m_cam->get_xnum();
	m_ynum = m_cam->get_ynum();
	// 计算姿态和轨道
	ReCalPosAndAtt(UT);
	// 计算全局仿射系数
	ComputerGlobalParam();
}


//////////////////////////////////////////////////////////////////
// 重新计算姿态和轨道
//////////////////////////////////////////////////////////////////
void GeoModelArray::ReCalPosAndAtt(double UT)
{	
	// 进行多项式拟合(轨道,本体姿态,相机姿态)
	m_orbit->GenPolyModel(UT-10, UT+10);
	m_att->GenPolyModel(UT-10, UT+10);
	//m_cam->GenPolyModel(UT-1, UT+1, m_param.attPoly);
	// 获得此时间对应的姿轨信息
	struct StrOrbitPoint m_ephpoint;
	if(m_input.isOrbitPoly == false)
		m_ephpoint = m_orbit->GetEpWGS84(UT);
	else
		m_ephpoint = m_orbit->PolyValue(UT);
	if(m_input.isAttPoly == false)
	{		
		m_attbody2wgs84 = m_att->GetAttBody2WGS84(UT);	// 本体到wgs84
	}
	else
	{
		m_att->PolyValue(UT, m_attbody2wgs84.R);	// 本体到wgs84
	}
	// 分别存储相机在WGS84系下的位置、GPS相位中心在WGS84系下的位置
	// GPS在本体坐标系下的位置、相机在本体坐标系下的位置
	double REphPos[3], RGPSSet[3], RCamSet[3];
	double Rtemp1[3], Rtemp2[3];
	REphPos[0] = m_ephpoint.X[0];
	REphPos[1] = m_ephpoint.X[1];
	REphPos[2] = m_ephpoint.X[2];
	// GPS在本体坐标系下的位置
	RGPSSet[0] = m_orbit->get_OffX();
	RGPSSet[1] = m_orbit->get_OffY();
	RGPSSet[2] = m_orbit->get_OffZ();
	// 相机在本体坐标系下的位置
	RCamSet[0] = m_cam->get_OffX(); 
	RCamSet[1] = m_cam->get_OffY();
	RCamSet[2] = m_cam->get_OffZ();
	// 相机在WGS84系下的位置
	Rtemp1[0] = -RGPSSet[0] + RCamSet[0];
	Rtemp1[1] = -RGPSSet[1] + RCamSet[1];
	Rtemp1[2] = -RGPSSet[2] + RCamSet[2];
	m_base.Multi(m_attbody2wgs84.R, Rtemp1, Rtemp2, 3, 3, 1);
	RCamPos[0] = REphPos[0] + Rtemp2[0];
	RCamPos[1] = REphPos[1] + Rtemp2[1];
	RCamPos[2] = REphPos[2] + Rtemp2[2];
	// 从测姿坐标系到本体坐标系的旋转矩阵
	m_att->get_ROff(m_body2att.R);
	//m_base.Transpose(m_body2att.R, 3, 3);
	// 从相机到本体的旋转矩阵
	m_cam->get_ROff(m_cam2body.R);
	//m_base.Transpose(m_cam2body.R,3,3);
	//相机到WGS84的旋转矩阵
	m_base.Multi(m_attbody2wgs84.R, m_cam2body.R, m_attcam2wgs84.R, 3, 3, 3);
	m_base.Matrix2Quat(m_attcam2wgs84.R, qCam2wgs84.Q1, qCam2wgs84.Q2, qCam2wgs84.Q3, qCam2wgs84.Q0);
	qCam2wgs84.UTC = m_attcam2wgs84.UT=UT;
}



////////////////////////////////////////////////////////
// 成像模型正算
////////////////////////////////////////////////////////
// 成像模型真实值正算
bool GeoModelArray::FromXY2LatLon(double x, double y, double &H, double &lat, double &lon)
{
	// 开始求和地球的交点
	double A, B, C, scale, B2_4AC;
	double a = 6378137 + H;
	double b = 6356752.314 + H;
	double a2 = a*a;
	double b2 = b*b;
	double RPos[3],  phi[3];
	// 获取探元指向角
	m_cam->GetInner(x, y, phi[0], phi[1]);
	phi[2] = 1;
	// 获得探元指向角在wgs84系下的指向,为了减少精度损失,放大a倍
	m_base.Multi(m_attcam2wgs84.R, phi, RPos, 3, 3, 1);
	RPos[0] *= a;   RPos[1] *= a;   RPos[2] *=a;
	A = (pow(RPos[0],2)+pow(RPos[1],2))/a2+pow(RPos[2],2)/b2;
	B = 2*((RPos[0]*RCamPos[0]+RPos[1]*RCamPos[1])/a2+RPos[2]*RCamPos[2]/b2);
	C = (pow(RCamPos[0],2) + pow(RCamPos[1],2))/a2 + pow(RCamPos[2],2)/b2-1;
	B2_4AC = B*B - 4*A*C;
	// 如果与地球不想交,则输出提示信息
	if(B2_4AC<0 || A==0)
	{
		printf("不与地球相交");
		return false;
	}
	double scaletemp1 = -B/(2*A);
	double scaletemp2 = sqrt(B2_4AC)/(2*A);
	double scale1 = scaletemp1 - scaletemp2;
	double scale2 = scaletemp1 + scaletemp2;
	double scale12 = scale1*scale2;
	if(scale12>0)
		scale = (fabs(scale1)<fabs(scale2))?scale1:scale2;
	else
	{
		printf("位于地球内部");
		return false;
	}
	// 求出地面点
	struct StrOrbitPoint m_Surface;
	m_Surface.X[0] = RCamPos[0] + scale*RPos[0];
	m_Surface.X[1] = RCamPos[1] + scale*RPos[1];
	m_Surface.X[2] = RCamPos[2] + scale*RPos[2];
	double tempH;
	m_base.Rect2Geograph(m_datum, m_Surface.X[0], m_Surface.X[1], m_Surface.X[2], lat, lon, tempH);
	return true;
}
bool GeoModelArray::Fromxyh2XYZ(double x, double y, double H, double &X, double &Y, double &Z)
{
	double lat, lon;
	FromXY2LatLon(x, y, H, lat, lon);
	StrDATUM WGS84;
	m_base.Geograph2Rect(WGS84,lat,lon,H,X,Y,Z);
	return true;
}

////////////////////////////////////////////////////////
// 成像模型反算
////////////////////////////////////////////////////////
void GeoModelArray::FromLatLon2XY(double lat, double lon, double H, double &x, double &y)
{
	// 基于仿射约束
	FromLatLon2XYBaseAffine(lat, lon, H, x, y);
	// 基于像方约束
	//	FromLatLon2XYBaseImageConst(lat, lon, H, x, y);
	// 基于物方约束
	//	FromLatLon2XYBaseObjectConst(lat, lon, H, x, y);
}

//////////////////////////////////////
// 功能：从地面到影像(光学反算,基于仿射)
// 输入:
//		double lat：	定位点的纬度(单位:弧度)
//		double lon：	定位点的经度(单位:弧度)	
//		double H:		定位点的高程(单位:m)
// 输出：
//		double &x:		沿轨向的像素号
//		double &y:		垂轨向的像素号
// 返回值：
//		void
//////////////////////////////////////
void GeoModelArray::FromLatLon2XYBaseAffine(double lat, double lon, double H, double &x, double &y)
{
	double latp, lonp;
	m_trans.GetValueBaseAffine(lat, lon, x, y);
	FromXY2LatLon(x, y, H, latp, lonp);
	double e = 3 * sqrt(((latp - lat)*(latp - lat) + (lonp - lon)*(lonp - lon)) / m_pixelsize2);
	int num = 0;
	do
	{
		PreciseBasedAffine(x, y, lat, lon, H, e, e);
		FromXY2LatLon(x, y, H, latp, lonp);
		e = 3 * ((latp - lat)*(latp - lat) + (lonp - lon)*(lonp - lon)) / m_pixelsize2;
		num++;
		if (e < 1e-8)  break;
		if (num > 20)    break;
	} while (1);
	//	printf("\t%d\t", num);
}
//////////////////////////////////////
// 功能：局部仿射变换预测
// 输入输出:
//		double &x：	输入及输出的像点x坐标
//		double &y：	输入及输出的像点y坐标
// 输入：
//		double lat:	输入的物方点纬度(单位:弧度)
//		double lon：输入的物方点经度(单位:弧度)
//		double H：	输入的物方点高程(单位:米)
//		double dx：	输入的像方点x偏移值
//		double dy：	输入的像方点y偏移值
// 返回值：
//		void
//////////////////////////////////////
void GeoModelArray::PreciseBasedAffine(double &x, double &y, double lat, double lon, double H, double dx, double dy)
{
	// 获取四个角点以及中心点
	double xtemp[4], ytemp[4], lattemp[4], lontemp[4];
	xtemp[0] = x + dx;   ytemp[0] = y + dy;
	xtemp[1] = x + dx;   ytemp[1] = y - dy;
	xtemp[2] = x - dx;   ytemp[2] = y + dy;
	xtemp[3] = x - dx;   ytemp[3] = y - dy;
	for (int i = 0; i < 4; i++)
	{
		FromXY2LatLon(xtemp[i], ytemp[i], H, lattemp[i], lontemp[i]);
	}
	// 进行仿射变换
	GeoTranslation m_transtemp;
	m_transtemp.CalAffineParam(lattemp, lontemp, xtemp, ytemp, 4);
	// 开始预测
	m_transtemp.GetValueBaseAffine(lat, lon, x, y);
}

//////////////////////////////////////////////////////////////////
//获取无误差全局仿射变换参数
//////////////////////////////////////////////////////////////////
void GeoModelArray::ComputerGlobalParam()
{
	double H = 0;
	// 获取四个角点以及中心点
	double x[5], y[5], lat[5], lon[5];
	//double latTmp[5], lonTmp[5];
	x[0] = m_xnum / 2;    y[0] = m_ynum / 2;
	x[1] = 0;           y[1] = 0;
	x[2] = m_xnum - 1;    y[2] = 0;
	x[3] = m_xnum - 1;    y[3] = m_ynum - 1;
	x[4] = 0;           y[4] = m_ynum - 1;
	double h = 0;
	for (int i = 0; i < 5; i++)
	{
		FromXY2LatLon(x[i], y[i], h, lat[i], lon[i]);
		//latTmp[i] = lat[i]; lonTmp[i] = lon[i];
	}
	m_trans.CalAffineParam(lat, lon, x, y, 5);
	//m_trans.GetValueBaseAffine(0.60037593087133145, 1.9722106651691769, x[0], y[0]);
	//m_trans.GetValueBaseAffine(latTmp[1], lonTmp[1], x[1], y[1]);

	// 获取一像素大小在地面的投影,是一个比例系数:经纬度/像素
	double lat1, lat2, lon1, lon2;
	double dx, dy;
	dx = dy = 10;
	FromXY2LatLon(m_xnum / 2.0, m_ynum / 2.0, H, lat1, lon1);
	FromXY2LatLon(m_xnum / 2.0 + dx, m_ynum / 2.0 + dy, H, lat2, lon2);
	m_pixelsize = sqrt(pow((lat1 - lat2), 2) + pow((lon1 - lon2), 2) / (dx*dx + dy*dy));
	m_pixelsize2 = pow(m_pixelsize, 2);
}

//根据Ru更新相机和wgs84坐标系关系
void GeoModelArray::updateOffsetmatrix(OffsetAngle & angle)
{
	double Rupdate[9], Ru[9];
	m_base.rot(angle.RuPhi, angle.RuOmega, angle.RuKappa, Ru);
	//m_base.invers_matrix(Ru, 3);
	m_base.Multi(m_attcam2wgs84.R, Ru, Rupdate,3,3,3);
	memcpy(m_attcam2wgs84.R, Rupdate, sizeof(double) * 9);
	m_base.Matrix2Quat(m_attcam2wgs84.R, qCam2wgs84.Q1, qCam2wgs84.Q2, qCam2wgs84.Q3, qCam2wgs84.Q0);
	qCam2wgs84.UTC = m_attcam2wgs84.UT;
	memcpy(m_attbody2wgs84.R, m_attcam2wgs84.R, sizeof(StrAttPoint));
	ComputerGlobalParam();
}

double GeoModelArray::CalcHeihtFromDEM(double lx,double ly,double &Lat,double &Lon)
{
	//读取DEM
	GeoReadImage DEM;
	DEM.Open(sDEM, GA_ReadOnly);
	DEM.ReadBlock(0, 0, DEM.m_xRasterSize, DEM.m_yRasterSize, 0, DEM.pBuffer[0]);
	double H = 0;
	FromXY2LatLon(lx, ly, H, Lat, Lon);
	int j = 0;
	while (1)
	{
		double HH = DEM.GetDataValue(Lon / PI * 180, Lat / PI * 180, -99999, 0, 0);
		if (abs(HH - H) > 1e-4&&j++ < 30)
		{
			H = HH;
			FromXY2LatLon(lx, ly, H, Lat, Lon);
		}
		else
		{
			break;
		}
	}
	DEM.Destroy();
	return H;
}

//////////////////////////////////////////////////////////////////////////
//功能：前方交会代码
//输入：左右影像模型，匹配控制点
//输出：交会点空间坐标XYZ
//作者：GZC
//时间：2017.09.07
//////////////////////////////////////////////////////////////////////////
void GeoModelArray::Intersection(GeoModelArray *model,MatchPoint pts, double *XYZ)
{
	double inner1[3], inner2[3], Rcam2wgs84L[9], Rcam2wgs84R[9],XYZ1[3],XYZ2[3], S1[3], S2[3];
	model[0].GetInner(pts.lx, pts.ly, inner1[0], inner1[1]);
	model[1].GetInner(pts.rx, pts.ry, inner2[0], inner2[1]);	
	inner1[2] = inner2[2] = 1;
	model[0].GetCam2WGS84(Rcam2wgs84L);
	model[1].GetCam2WGS84(Rcam2wgs84R);
	m_base.Multi(Rcam2wgs84L, inner1, S1, 3, 3, 1);
	m_base.Multi(Rcam2wgs84R, inner2, S2, 3, 3, 1);
	
	double Bu, Bv, Bw;
	model[0].GetCamPos(XYZ1);
	model[1].GetCamPos(XYZ2);
	Bu = XYZ1[0] - XYZ2[0]; 
	Bv = XYZ1[1] - XYZ2[1]; 
	Bw = XYZ1[2] - XYZ2[2];

	double N1 = (Bu * S2[2] - Bw * S2[0]) / (S1[0] * S2[2] - S2[0] * S1[2]);
	double N2 = (Bu * S1[2] - Bw * S1[0]) / (S1[0] * S2[2] - S2[0] * S1[2]);

	XYZ[0] = XYZ1[0] + N1 * S1[0];
	XYZ[1] = 0.5 * ((XYZ1[1] + N1 * S1[1]) + (XYZ2[1] + N2 * S2[1]));
	XYZ[2] = XYZ1[2] + N1 * S1[2];

	double lat, lon, H;
	m_base.Rect2Geograph(m_datum, XYZ[0], XYZ[1], XYZ[2], lat, lon, H);
	
}


// 根据行列号获得位置、姿态、内方位元素-真实姿态内方位元素
void GeoModelArray::GetPosAndInner(double x, double y, struct StrOrbitPoint *pos,
									struct StrAttPoint *att, double *innerx, double *innery)
{
	// 获取探元指向角
	long level;
	double phi[3];
	m_cam->GetInner(x, y, phi[0], phi[1]);
	phi[2] = -1;
	// 输出结果
	pos->X[0] = RCamPos[0];   pos->X[1] = RCamPos[1];		pos->X[2] = RCamPos[2];
	memcpy(att->R, m_attcam2wgs84.R, sizeof(double)*9);
	*innerx = phi[0];
	*innery = phi[1];
	return;
}

void GeoModelArray::GetCam2WGS84(double *Rcam2WGS84)
{
	memcpy(Rcam2WGS84, m_attcam2wgs84.R, sizeof(double) * 9);
}

void GeoModelArray::GetCamPos(double * PosCam)
{
	memcpy(PosCam, RCamPos, sizeof(double) * 3);
}

// 根据行列号获得内方位元素
void GeoModelArray::GetInner(double x, double y,double &innerx, double &innery)
{
	// 获取探元指向角
	long level;
	double phi[3];
	m_cam->GetInner(x, y, phi[0], phi[1]);
	phi[2] = -1;
	innerx = phi[0];
	innery = phi[1];
	return;
}

void GeoModelArray::GetCamInstall(double * Rcam)
{
	memcpy(Rcam, m_cam2body.R, sizeof(double) * 9);
}

void GeoModelArray::SetDEMpath(string strDEM)
{
	sDEM = strDEM;
}

Attitude GeoModelArray::GetQuatCam2wgs84()
{
	return qCam2wgs84;
}



