#include "GeoModelRFM.h"


#include "GeoCameraLine.h"
#include "GeoModelLine.h"
#include <float.h>


GeoModelRFM::GeoModelRFM(void)
{
	m_RFMgcp.m_X = NULL;
	m_RFMgcp.m_Y = NULL;
	m_RFMgcp.m_P = NULL;
	m_RFMgcp.m_L = NULL;
	m_RFMgcp.m_H = NULL;
	rigorousmodel = NULL;
}


GeoModelRFM::~GeoModelRFM(void)
{
	Destroy();
}


////////////////////////////////////////////////////////
// 释放内存空间
////////////////////////////////////////////////////////
void GeoModelRFM::Destroy()
{
	if(m_RFMgcp.m_X!=NULL)		delete []m_RFMgcp.m_X;		m_RFMgcp.m_X = NULL;
	if(m_RFMgcp.m_Y!=NULL)		delete []m_RFMgcp.m_Y;		m_RFMgcp.m_Y = NULL;
	if(m_RFMgcp.m_P!=NULL)		delete []m_RFMgcp.m_P;		m_RFMgcp.m_P = NULL;
	if(m_RFMgcp.m_L!=NULL)		delete []m_RFMgcp.m_L;		m_RFMgcp.m_L = NULL;
	if(m_RFMgcp.m_H!=NULL)		delete []m_RFMgcp.m_H;		m_RFMgcp.m_H = NULL;
	if(rigorousmodel!=NULL)		delete []rigorousmodel;		rigorousmodel = NULL;
}


////////////////////////////////////////////////////////
// 写出RPC文件
//	filepath：输出文件路径
////////////////////////////////////////////////////////
bool GeoModelRFM::WriteRFMFile(string filepath)
{
	int i;
	FILE *fp = fopen(filepath.c_str(), "w");
	if (fp == NULL)
	{
		printf("RFM文件打开失败!\n");
		return false;
	}
	fprintf(fp, "LINE_OFF: %+010.2lf pixels\n", m_rfm.LINE_OFF);
	fprintf(fp, "SAMP_OFF: %+010.2lf pixels\n", m_rfm.SAMP_OFF);
	fprintf(fp, "LAT_OFF: %+011.8lf degrees\n", m_rfm.LAT_OFF);
	fprintf(fp, "LONG_OFF: %+012.8lf degrees\n", m_rfm.LONG_OFF);
	fprintf(fp, "HEIGHT_OFF: %+08.3lf meters\n", m_rfm.HEIGHT_OFF);
	
	fprintf(fp, "LINE_SCALE: %+010.2lf pixels\n", m_rfm.LINE_SCALE);
	fprintf(fp, "SAMP_SCALE: %+010.2lf pixels\n", m_rfm.SAMP_SCALE);
	fprintf(fp, "LAT_SCALE: %0+12.8lf degrees\n", m_rfm.LAT_SCALE);
	fprintf(fp, "LONG_SCALE: %+012.8lf degrees\n", m_rfm.LONG_SCALE);
	fprintf(fp, "HEIGHT_SCALE: %+08.3lf meters\n", m_rfm.HEIGHT_SCALE);
	
	for( i=0; i<20; i++)
	{
		fprintf(fp, "LINE_NUM_COEFF_%d:  %+50.40le\n", i+1, m_rfm.LNUM[i]);
	}
	
	for( i=0; i<20; i++)
	{
		fprintf(fp, "LINE_DEN_COEFF_%d:  %+50.40le\n", i+1, m_rfm.LDEN[i]);
	}
	
	for( i=0; i<20; i++)
	{
		fprintf(fp, "SAMP_NUM_COEFF_%d:  %+50.40le\n", i+1, m_rfm.SNUM[i]);
	}
	for( i=0; i<20; i++)
	{
		fprintf(fp, "SAMP_DEN_COEFF_%d:  %+50.40le\n", i+1, m_rfm.SDEN[i]);
	}
	fclose(fp);
	fp = NULL;
	return true;
}


////////////////////////////////////////////////////////
// 读取RFM文件
////////////////////////////////////////////////////////
bool GeoModelRFM::ReadRFMFile(string filepath, bool isrpc)
{
	if(isrpc==true)
		ReadRPCFile(filepath);
	else
		ReadRPBFile(filepath);
	// 生成严密模型
//	GenRSM(50, 0, 10);
	m_xnum = m_rfm.LINE_SCALE*2;
	m_ynum = m_rfm.SAMP_SCALE*2;
	return true;
}


////////////////////////////////////////////////////////
// 读取RPC文件
////////////////////////////////////////////////////////
bool GeoModelRFM::ReadRPCFile(string filepath)
{
	FILE *fp = fopen(filepath.c_str(), "r");
	if (fp == NULL)
	{
		printf("RFM文件打开失败!\n");
		return false;
	}
	fscanf(fp, "%*s%lf%*s", &m_rfm.LINE_OFF);
	fscanf(fp, "%*s%lf%*s", &m_rfm.SAMP_OFF);
	fscanf(fp, "%*s%lf%*s", &m_rfm.LAT_OFF);
	fscanf(fp, "%*s%lf%*s", &m_rfm.LONG_OFF);
	fscanf(fp, "%*s%lf%*s", &m_rfm.HEIGHT_OFF);
	fscanf(fp, "%*s%lf%*s", &m_rfm.LINE_SCALE);
	fscanf(fp, "%*s%lf%*s", &m_rfm.SAMP_SCALE);
	fscanf(fp, "%*s%lf%*s", &m_rfm.LAT_SCALE);
	fscanf(fp, "%*s%lf%*s", &m_rfm.LONG_SCALE);
	fscanf(fp, "%*s%lf%*s", &m_rfm.HEIGHT_SCALE);

	m_AveHeight = m_rfm.HEIGHT_OFF;//平均高程

	int i;
	for(i=0; i<20; i++)		fscanf(fp, "%*s%lf", &m_rfm.LNUM[i]);
	for(i=0; i<20; i++)		fscanf(fp, "%*s%lf", &m_rfm.LDEN[i]);
	for(i=0; i<20; i++)		fscanf(fp, "%*s%lf", &m_rfm.SNUM[i]);
	for(i=0; i<20; i++)		fscanf(fp, "%*s%lf", &m_rfm.SDEN[i]);
	fclose(fp);

	// 反算RPC用的
	// 像素大小估算，影像一像素大小在地面大小以及其平方项，4.5e-4°换算成地面距离约等于50m
	m_pixelsize = DistanceOnePixel(m_rfm.LAT_OFF*PI/180.0, m_rfm.LONG_OFF*PI/180.0, m_rfm.HEIGHT_OFF, 4.5e-4, 4.5e-4);
	m_pixelsize2 = pow(m_pixelsize, 2);
	// 计算全局仿射系数
	ComputerGlobalAffine();
	return true;
}


////////////////////////////////////////////////////////
// 读取RPB文件
////////////////////////////////////////////////////////
bool GeoModelRFM::ReadRPBFile(string filepath)
{
	FILE *fp = fopen(filepath.c_str(), "r");
	if (fp == NULL)
	{
		printf("RFM文件打开失败!\n");
		return false;
	}
	char strtemp[1024], c, temp[1024];
	int i;
	for(i=0;i<6;i++)
	{
		fscanf(fp, "%[^\n]s", strtemp);
		fscanf(fp, "%c", &c);
	}

	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.LINE_OFF);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.SAMP_OFF);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.LAT_OFF);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.LONG_OFF);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.HEIGHT_OFF);

	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.LINE_SCALE);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.SAMP_SCALE);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.LAT_SCALE);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.LONG_SCALE);
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	sscanf(strtemp,"%s%s%lf", temp, temp, &m_rfm.HEIGHT_SCALE);

	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	for(i=0; i<20; i++)	
	{
		fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
		sscanf(strtemp, "%lf", &m_rfm.LNUM[i]);
	}
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	for(i=0; i<20; i++)		
	{
		fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
		sscanf(strtemp, "%lf", &m_rfm.LDEN[i]);
	}
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	for(i=0; i<20; i++)		
	{
		fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
		sscanf(strtemp, "%lf", &m_rfm.SNUM[i]);
	}
	fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
	for(i=0; i<20; i++)	
	{
		fscanf(fp, "%[^\n]s", strtemp);		fscanf(fp, "%c", &c);
		sscanf(strtemp, "%lf", &m_rfm.SDEN[i]);
	}
	fclose(fp);
	
	// 反算RPC用的
	// 像素大小估算，影像一像素大小在地面大小以及其平方项，4.5e-4°换算成地面距离约等于50m
	m_pixelsize = DistanceOnePixel(m_rfm.LAT_OFF*PI/180.0, m_rfm.LONG_OFF*PI/180.0, m_rfm.HEIGHT_OFF, 4.5e-4, 4.5e-4);
	m_pixelsize2 = pow(m_pixelsize, 2);
	// 计算全局仿射系数
	ComputerGlobalAffine();
	return true;
}


//////////////////////////////////////////////////////////////////
// 一像素大小在地面的投影,是一个比例系数:经纬度/像素
//////////////////////////////////////////////////////////////////
double GeoModelRFM::DistanceOnePixel(double x,double y,double H,double dx,double dy)
{
	double lat1,lat2,lon1,lon2;
	FromLatLon2XY(x, y, H, lat1, lon1);
	FromLatLon2XY(x+dx, y+dy, H, lat2, lon2);	
	return sqrt((dx*dx+dy*dy)/(pow((lat1-lat2),2) + pow((lon1-lon2),2)));
}


//////////////////////////////////////////////////////////////////
// 全局仿射变换参数
//////////////////////////////////////////////////////////////////
void GeoModelRFM::ComputerGlobalAffine()
{
	// 获取四个角点以及中心点
	double x[5], y[5] ,lat[5], lon[5], h;
	lat[0] = m_rfm.LAT_OFF;							lon[0] = m_rfm.LONG_OFF;
	lat[1] = -1.1*m_rfm.LAT_SCALE + m_rfm.LAT_OFF;	lon[1] = 1.1*m_rfm.LONG_SCALE + m_rfm.LONG_OFF;
	lat[2] = 1.1*m_rfm.LAT_SCALE + m_rfm.LAT_OFF;	lon[2] = 1.1*m_rfm.LONG_SCALE + m_rfm.LONG_OFF;
	lat[3] = 1.1*m_rfm.LAT_SCALE + m_rfm.LAT_OFF;	lon[3] = -1.1*m_rfm.LONG_SCALE + m_rfm.LONG_OFF;
	lat[4] = -1.1*m_rfm.LAT_SCALE + m_rfm.LAT_OFF;	lon[4] = -1.1*m_rfm.LONG_SCALE + m_rfm.LONG_OFF;
	h = m_rfm.HEIGHT_OFF;
	for(int i=0;i<5;i++)
	{
		lat[i] = lat[i]*PI/180.0;
		lon[i] = lon[i]*PI/180.0;
		FromLatLon2XY(lat[i], lon[i], h, x[i], y[i]);
	}
	m_trans.CalAffineParam(x, y, lat, lon, 5);
}


//////////////////////////////////////////////////////////////////
// 局部仿射变换预测
//////////////////////////////////////////////////////////////////
void GeoModelRFM::PreciseBasedAffine(double &x,double &y,double lat,double lon,double H,double dx,double dy)
{
	// 获取四个角点以及中心点
	double xtemp[4], ytemp[4], lattemp[4], lontemp[4], xptemp[4], yptemp[4];
	xtemp[0] = x+dx;   ytemp[0] = y+dy;
	xtemp[1] = x+dx;   ytemp[1] = y-dy;
	xtemp[2] = x-dx;   ytemp[2] = y+dy;
	xtemp[3] = x-dx;   ytemp[3] = y-dy;
	memcpy(xptemp, xtemp, sizeof(double)*4);
	memcpy(yptemp, ytemp, sizeof(double)*4);
	for(int i=0;i<4;i++)
	{
		FromLatLon2XY(xtemp[i], ytemp[i], H, lattemp[i], lontemp[i]);
	}
	// 进行仿射变换
	GeoTranslation m_transtemp;
	m_transtemp.CalAffineParam(lattemp, lontemp, xptemp, yptemp, 4);
	// 开始预测
	m_transtemp.GetValueBaseAffine(lat, lon, x, y);
}


//////////////////////////////////////
// 功能：从影像到地面(光学正算)
// 输入:
//		double x:		沿轨向的像素号
//		double y:		垂轨向的像素号
//		double &H:		定位点的高程(单位:m),此也作为输出
// 输出：
//		double &lat：	定位点的纬度(单位:弧度)
//		double &lon：	定位点的经度(单位:弧度)	
// 返回值：
//		void
//////////////////////////////////////
bool GeoModelRFM::FromXY2LatLon(double x, double y, double &H, double &lat, double &lon)
{
	double xp, yp;
	m_trans.GetValueBaseAffine(x, y, lat, lon);
	FromLatLon2XY(lat, lon, H, xp, yp);
	double e = sqrt(((xp-x)*(xp-x)+(yp-y)*(yp-y))*m_pixelsize2);
    int num=0;
	do 
	{
		PreciseBasedAffine(lat, lon, x, y, H, e, e);
		FromLatLon2XY(lat, lon, H, xp, yp);
		e = sqrt(((xp-x)*(xp-x)+(yp-y)*(yp-y))*m_pixelsize2);
		num++;
		if(e < 1e-10)  break;
		if(num > 100)    break;
	} while(1);
	return true;
}


//////////////////////////////////////
// 功能：从地面到影像(光学反算)
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
void GeoModelRFM::FromLatLon2XY(double lat, double lon, double H, double &x, double &y)
{
	double P = lat*180.0/PI;
	double L = lon*180.0/PI;
	P = (P - m_rfm.LAT_OFF)/m_rfm.LAT_SCALE;
	L = (L - m_rfm.LONG_OFF)/m_rfm.LONG_SCALE;
	H = (H - m_rfm.HEIGHT_OFF)/m_rfm.HEIGHT_SCALE;
	double Num[20];
	Num[0] = 1;		Num[1] = L;		Num[2] = P;		Num[3] = H;		Num[4] = L*P;
	Num[5] = L*H;	Num[6] = P*H;	Num[7] = L*L;	Num[8] = P*P;	Num[9] = H*H;
	Num[10]= P*L*H;	Num[11]= L*L*L;	Num[12]= L*P*P;	Num[13]= L*H*H;	Num[14]= L*L*P;
	Num[15]= P*P*P;	Num[16]= P*H*H;	Num[17]= L*L*H;	Num[18]= P*P*H;	Num[19]= H*H*H;
	double NumL = 0, DenL = 0, NumS = 0, DenS = 0;
	for(long i=0; i<20; i++)
	{
		NumS += m_rfm.SNUM[i]*Num[i];
		DenS += m_rfm.SDEN[i]*Num[i];
		NumL += m_rfm.LNUM[i]*Num[i];
		DenL += m_rfm.LDEN[i]*Num[i];
	}		
	x = NumL/DenL;
	y = NumS/DenS;
	x = x*m_rfm.LINE_SCALE + m_rfm.LINE_OFF;
	y = y*m_rfm.SAMP_SCALE + m_rfm.SAMP_OFF;
	double x_, y_;
	x_ = m_affine[0] + m_affine[1]*x + m_affine[2]*y;
	y_ = m_affine[3] + m_affine[4]*x + m_affine[5]*y;
	x = x_;
	y = y_;
}


////////////////////////////////////////////////////////
// 获取缩放比例
////////////////////////////////////////////////////////
double GeoModelRFM::GetScale()
{
	return m_rfm.SAMP_SCALE;
}

////////////////////////////////////////////////////////
//得到平均高程
////////////////////////////////////////////////////////
double GeoModelRFM::getAveHeight()
{
	return m_AveHeight;
}


////////////////////////////////////////////////////////
// 功能：生成RPC模型
// 输入：
//		GeoModel *model：	严密成像模型
//		double minH：		区域最小高程
//		double maxH：		区域最大高程
//		double dx:			影像高度方向间距(像素数)
//		double dy:			影像宽度方向间距(像素数)
//		int nz：			空间高度方向点个数
//		string errFile:		精度报告文件输出路径,默认为""
//		int order:			RPC采用的阶数,默认是采用3阶
////////////////////////////////////////////////////////
bool GeoModelRFM::GenRPCFile(GeoModel *model, double minH, double maxH, int nx, int ny, int nz, string errFile, int order)
{
	// 释放内存
	Destroy();
	long i, j, k;
	double BottomPencnt = 0.0;
	m_nx = nx;
	m_ny = ny;
	// 获取影像宽度,高度
	m_xnum = model->get_m_height();
	m_ynum = model->get_m_width();
	double RealHeight = (1.0 + BottomPencnt)*m_xnum;
	double RealWidth  = (1.0 + BottomPencnt)*m_ynum;
	//得到间隔像素
	ControlBlockHeight = long(RealHeight/m_nx)+1;
	//得到间隔像素
	ControlBlockWidth = long(RealWidth/m_ny)+1;
	//得到点数目
	m_nx -= 1;		m_ny -= 1;		m_nz = nz-1;
	// 获得最大,最小高程
	m_MaxHeight = maxH;
	m_MinHeight = minH;
	// 得到间隔的高度	
	ControlHeight = (m_MaxHeight - m_MinHeight)/(m_nz - 1);
	//////////////////////////////
	// 开始构网
	//////////////////////////////
	m_RFMgcp.m_nGCPNum = m_nx*m_ny*m_nz;
	m_RFMgcp.m_X = new double[m_RFMgcp.m_nGCPNum];
	m_RFMgcp.m_Y = new double[m_RFMgcp.m_nGCPNum];
	m_RFMgcp.m_P = new double[m_RFMgcp.m_nGCPNum];
	m_RFMgcp.m_L = new double[m_RFMgcp.m_nGCPNum];
	m_RFMgcp.m_H = new double[m_RFMgcp.m_nGCPNum];
	// 循环求取
	double DemH = m_MinHeight;
	m_Minx = -BottomPencnt*m_ynum/2;
	m_Miny = -BottomPencnt*m_xnum/2;
	double x, y, lat, lon;
	m_RFMgcp.m_nGCPNum = 0;
	for(k=0; k<m_nz; k++)
	{
		for(i=0; i<m_nx; i++)
		{	
			x = m_Minx + i*ControlBlockHeight;
			for(j=0; j<m_ny; j++)
			{
				y = m_Miny + j*ControlBlockWidth;
				model->FromXY2LatLon(x, y, DemH, lat, lon);
				double aa = lat / PI*180.0;
				double bb = lon / PI*180.0;
				m_RFMgcp.m_X[m_RFMgcp.m_nGCPNum] = x;
				m_RFMgcp.m_Y[m_RFMgcp.m_nGCPNum] = y;
				//m_RFMgcp.m_X[m_RFMgcp.m_nGCPNum] = m_affine[0] + m_affine[1] * x + m_affine[2] * y;
				//m_RFMgcp.m_Y[m_RFMgcp.m_nGCPNum] = m_affine[3] + m_affine[4] * x + m_affine[5] * y;
				m_RFMgcp.m_P[m_RFMgcp.m_nGCPNum] = lat/PI*180.0;
				m_RFMgcp.m_L[m_RFMgcp.m_nGCPNum] = lon/PI*180.0;
				m_RFMgcp.m_H[m_RFMgcp.m_nGCPNum] = DemH;
				m_RFMgcp.m_nGCPNum++;	 				
			}
		}
		DemH += ControlHeight; 
	}
	// 计算归一化参数
	m_base.Compute_avAnddx(m_RFMgcp.m_X, m_RFMgcp.m_nGCPNum, m_rfm.LINE_OFF, m_rfm.LINE_SCALE);
	m_base.Compute_avAnddx(m_RFMgcp.m_Y, m_RFMgcp.m_nGCPNum, m_rfm.SAMP_OFF, m_rfm.SAMP_SCALE);
	m_base.Compute_avAnddx(m_RFMgcp.m_P, m_RFMgcp.m_nGCPNum, m_rfm.LAT_OFF, m_rfm.LAT_SCALE);
	m_base.Compute_avAnddx(m_RFMgcp.m_L, m_RFMgcp.m_nGCPNum, m_rfm.LONG_OFF, m_rfm.LONG_SCALE);
	m_base.Compute_avAnddx(m_RFMgcp.m_H, m_RFMgcp.m_nGCPNum, m_rfm.HEIGHT_OFF, m_rfm.HEIGHT_SCALE);
	// 进行归一化
	m_base.Normaliza_avAnddx(m_RFMgcp.m_X, m_RFMgcp.m_nGCPNum, m_rfm.LINE_OFF, m_rfm.LINE_SCALE);
	m_base.Normaliza_avAnddx(m_RFMgcp.m_Y, m_RFMgcp.m_nGCPNum, m_rfm.SAMP_OFF, m_rfm.SAMP_SCALE);
	m_base.Normaliza_avAnddx(m_RFMgcp.m_P, m_RFMgcp.m_nGCPNum, m_rfm.LAT_OFF, m_rfm.LAT_SCALE);
	m_base.Normaliza_avAnddx(m_RFMgcp.m_L, m_RFMgcp.m_nGCPNum, m_rfm.LONG_OFF, m_rfm.LONG_SCALE);
	m_base.Normaliza_avAnddx(m_RFMgcp.m_H, m_RFMgcp.m_nGCPNum, m_rfm.HEIGHT_OFF, m_rfm.HEIGHT_SCALE);

	// 开始进行RPC参数计算
	//Cal_DirectLM();
	//Cal_DirectLME();
	// Cal_DirectLCurve();
	// Cal_DirectOMP();

	//Cal_IndirectLM();
	//Cal_IndirectLME();
	Cal_IndirectLCurve();
	//Cal_IndirectOMP();

	// 输出精度报告
	if(errFile.compare("")!=0)
		WriteAccuracyReport(errFile, model);

	// 反算RPC用的
	// 像素大小估算，影像一像素大小在地面大小以及其平方项，4.5e-4°换算成地面距离约等于50m
	m_pixelsize = DistanceOnePixel(m_rfm.LAT_OFF*PI/180.0, m_rfm.LONG_OFF*PI/180.0, 0, 4.5e-4, 4.5e-4);
	m_pixelsize2 = m_pixelsize*m_pixelsize;
	// 计算全局仿射系数
	ComputerGlobalAffine();

	// 释放内存
	Destroy();
	return true;
}


////////////////////////////////////////////////////////
// 输出精度报告文件
////////////////////////////////////////////////////////
void GeoModelRFM::WriteAccuracyReport(string filepath, GeoModel *model)
{
	FILE *fp = fopen(filepath.c_str(),"w");
	if(fp==NULL) 
	{
		printf("精度报告文件打开失败!\n");
		return;
	}
	long i, j, k;
	double x, y, lat, lon, xp, yp;
	double tempx, tempy;
	double DemH = m_MinHeight;
	long index = 0;
	double maxErrx = DBL_MIN;
	double maxErry = DBL_MIN;
	double minErrx = DBL_MAX;
	double minErry = DBL_MAX;
	double meanErrx = 0,meanErry = 0,devErrx = 0,devErry  = 0;
	long m_nGCPNum = m_nx*m_ny*m_nz;
	fprintf(fp,"总点数:%ld\n\tSample大小\t\tLine大小\t\tDemH大小\t\tSample残差\t\tLine残差\n", m_nGCPNum);
	for(k=0; k<m_nz-1; k++)
	{
		for(i=0; i<m_nx-1; i++)
		{			
			for(j=0; j<m_ny-1; j++)
			{
				x = m_Minx + (i+0.5)*ControlBlockHeight;		// 检查格网点不和控制各网点在一起
				y = m_Miny + (j+0.5)*ControlBlockWidth;			// 检查格网点不和控制各网点在一起
				DemH = m_MinHeight + (k+0.5)*ControlHeight;		// 检查格网点不和控制各网点在一起
				model->FromXY2LatLon(x, y, DemH, lat, lon);
				
				tempx = m_affine[0] + m_affine[1]*x + m_affine[2]*y;
				tempy = m_affine[3] + m_affine[4]*x + m_affine[5]*y;
				x = tempx;
				y = tempy;

				FromLatLon2XY(lat, lon, DemH, xp, yp);			// RPC正算
				double errx = fabs(x - xp);
				double erry = fabs(y - yp);
				maxErrx = max(errx, maxErrx);
				minErrx = min(errx, minErrx);
				maxErry = max(erry, maxErry);
				minErry = min(erry, minErry);
				meanErrx += errx;
				meanErry += erry;
				devErrx += errx*errx;
				devErry += erry*erry;
				fprintf(fp,"%20.10lf\t%20.10lf\t%20.10lf\t%20.10lf\t%20.10lf\n", y, x, DemH, (yp-y), (xp-x));
				index++;
			}
		}
	}
	meanErrx /= index;
	meanErry /= index;
	devErrx = sqrt(devErrx/index);
	devErry = sqrt(devErry/index);
	fprintf(fp, "\n");
	fprintf(fp, "最小偏差:sample\t %.10lf\t\tline\t %.10lf\n", minErry, minErrx);
	fprintf(fp, "最大偏差:sample\t %.10lf\t\tline\t %.10lf\n", maxErry, maxErrx);
	fprintf(fp, "平均偏差:sample\t %.10lf\t\tline\t %.10lf\n", meanErry, meanErrx);
	fprintf(fp, "替代精度:sample\t %.10lf\t\tline\t %.10lf\n", devErry, devErrx);
	fclose(fp);
	fp = NULL;
}


////////////////////////////////////////////////////////
// 求解RPC的各种方法
////////////////////////////////////////////////////////
///////////////////////////////////////////
// 阶数为1的求解
///////////////////////////////////////////
void GeoModelRFM::Cal_Order1()
{
	double a[7], aa[49], al[7];
	double bb[49], bl[7];
	memset(aa, 0, sizeof(double)*49);
	memset(al, 0, sizeof(double)*7);
	memset(bb, 0, sizeof(double)*49);
	memset(bl, 0, sizeof(double)*7);
	// 构建法方程
	double P, L, H, X, Y;
	for(int i=0; i<m_RFMgcp.m_nGCPNum; i++)
	{	
		P = m_RFMgcp.m_P[i];	L = m_RFMgcp.m_L[i];	H = m_RFMgcp.m_H[i];
		X = m_RFMgcp.m_X[i];	Y = m_RFMgcp.m_Y[i];
		a[0] = 1.0;		a[1] = L;		a[2] = P;		a[3] = H;
		a[4] = -X*L;	a[5] = -X*P;	a[6] = -X*H;
		m_base.pNormal(a, 7, X, aa, al, 1.0);		
		a[0] = 1.0;		a[1] = L;		a[2] = P;		a[3] = H;
		a[4] = -Y*L;	a[5] = -Y*P;	a[6] = -Y*H;
		m_base.pNormal(a, 7, Y, bb, bl, 1.0);	
	}
	// 求解
	m_base.Gauss(aa, al, 7);
	m_base.Gauss(bb, bl, 7);
	// RPC参数初始化
	memset(m_rfm.LNUM, 0, sizeof(double)*20);
	memset(m_rfm.LDEN, 0, sizeof(double)*20);
	memset(m_rfm.SNUM, 0, sizeof(double)*20);
	memset(m_rfm.SDEN, 0, sizeof(double)*20);
	// RPC参数赋值
	memcpy(m_rfm.LNUM,	 al,   sizeof(double)*4);
	memcpy(m_rfm.LDEN+1, al+4, sizeof(double)*3);
	memcpy(m_rfm.SNUM,	 bl,   sizeof(double)*4);
	memcpy(m_rfm.SDEN+1, bl+4, sizeof(double)*3);
	m_rfm.LDEN[0]=1.0; 	
	m_rfm.SDEN[0]=1.0; 
}

///////////////////////////////////////////
// 阶数为2的求解
///////////////////////////////////////////
void GeoModelRFM::Cal_Order2()
{
	// 在算出一阶量的基础上再算二阶量
	Cal_Order1();
	// 开始计算二阶量
	double a[19], aa[361], al[19], bb[361], bl[19];
	memset(aa,0, sizeof(double)*361);	memset(bb,0, sizeof(double)*361);
	memset(al,0, sizeof(double)*19);	memset(bl,0, sizeof(double)*19);
	// 构建法方程
	double P, L, H, X, Y;
	for(int i=0; i<m_RFMgcp.m_nGCPNum; i++)
	{
		P = m_RFMgcp.m_P[i];	L = m_RFMgcp.m_L[i];	H = m_RFMgcp.m_H[i];
		X = m_RFMgcp.m_X[i];	Y = m_RFMgcp.m_Y[i];
		a[0] = 1.0;		a[1] = L;		a[2] = P;		a[3] = H;		a[4] = L*P;		
		a[5] = L*H;		a[6] = P*H;		a[7] = L*L;		a[8] = P*P;		a[9] = H*H;	
		for(int j=10;j<19;j++)
		{
			a[j] = -a[j-9]*X;
		}
		m_base.pNormal(a, 19, X, aa, al, 1.0);	
		for(int j=10;j<19;j++)
		{
			a[j] = -a[j-9]*Y;
		}
		m_base.pNormal(a, 19, Y, bb, bl, 1.0);
	}
	// 求解
    double x[19];
	memcpy(x, m_rfm.LNUM, sizeof(double)*10);
	memcpy(x+10, m_rfm.LDEN+1, sizeof(double)*9);
	m_base.GaussExt(aa, al, x, 19);
	memcpy(al, x, sizeof(double)*19);

	memcpy(x, m_rfm.SNUM,sizeof(double)*10);
	memcpy(x+10, m_rfm.SDEN+1,sizeof(double)*9);
	m_base.GaussExt(bb, bl, x, 19);
	memcpy(bl, x, sizeof(double)*19);
	// RPC参数初始化
	memset(m_rfm.LNUM, 0, sizeof(double)*20);
	memset(m_rfm.LDEN, 0, sizeof(double)*20);
	memset(m_rfm.SNUM, 0, sizeof(double)*20);
	memset(m_rfm.SDEN, 0, sizeof(double)*20);
	// RPC参数赋值
	memcpy(m_rfm.LNUM,	 al,	sizeof(double)*10);
	memcpy(m_rfm.LDEN+1, al+10, sizeof(double)*9);
	memcpy(m_rfm.SNUM,	 bl,	sizeof(double)*10);
	memcpy(m_rfm.SDEN+1, bl+10, sizeof(double)*9);
	m_rfm.LDEN[0]=1.0; 	
	m_rfm.SDEN[0]=1.0; 
}

///////////////////////////////////////////
// 求解RPC的直接最小二乘方法
///////////////////////////////////////////
void GeoModelRFM::Cal_DirectLM()
{
	printf("直接最小二乘方法\n");
	// 开始计算二阶量
	double a[39], aa[1521], al[39], bb[1521], bl[39];
	memset(aa,0, sizeof(double)*1521);	memset(bb,0, sizeof(double)*1521);
	memset(al,0, sizeof(double)*39);	memset(bl,0, sizeof(double)*39);
	// 构建法方程
	double P, L, H, X, Y;
	for(int i=0; i<m_RFMgcp.m_nGCPNum; i++)
	{
		P = m_RFMgcp.m_P[i];	L = m_RFMgcp.m_L[i];	H = m_RFMgcp.m_H[i];
		X = m_RFMgcp.m_X[i];	Y = m_RFMgcp.m_Y[i];
		a[0] = 1;		a[1] = L;		a[2] = P;		a[3] = H;		a[4] = L*P;
		a[5] = L*H;		a[6] = P*H;		a[7] = L*L;		a[8] = P*P;		a[9] = H*H;
		a[10]= P*L*H;	a[11]= L*L*L;	a[12]= L*P*P;	a[13]= L*H*H;	a[14]= L*L*P;
		a[15]= P*P*P;	a[16]= P*H*H;	a[17]= L*L*H;	a[18]= P*P*H;	a[19]= H*H*H;
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*X;
		}
		m_base.pNormal(a, 39, X, aa, al, 1.0);	
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*Y;
		}
		m_base.pNormal(a, 39, Y, bb, bl, 1.0);
	}
	// 求解
	m_base.Gauss(aa, al, 39);
	printf("\t沿轨向条件数：%lf\n", m_base.Mat_Cov(aa, 39, 2));
	m_base.Gauss(bb, bl, 39);
	printf("\t垂轨向条件数：%lf\n", m_base.Mat_Cov(bb, 39, 2));
	// RPC参数初始化
	memset(m_rfm.LNUM, 0, sizeof(double)*20);
	memset(m_rfm.LDEN, 0, sizeof(double)*20);
	memset(m_rfm.SNUM, 0, sizeof(double)*20);
	memset(m_rfm.SDEN, 0, sizeof(double)*20);
	// RPC参数赋值
	memcpy(m_rfm.LNUM,	 al,	sizeof(double)*20);
	memcpy(m_rfm.LDEN+1, al+20, sizeof(double)*19);
	memcpy(m_rfm.SNUM,	 bl,	sizeof(double)*20);
	memcpy(m_rfm.SDEN+1, bl+20, sizeof(double)*19);
	m_rfm.LDEN[0]=1.0; 	
	m_rfm.SDEN[0]=1.0; 
}


///////////////////////////////////////////
// 求解RPC的直接谱修正方法
///////////////////////////////////////////
void GeoModelRFM::Cal_DirectLME()
{
	printf("直接谱修正方法\n");
	// 开始计算二阶量
	double a[39], aa[1521], al[39], bb[1521], bl[39], altmp[39], bltmp[39];
	memset(aa, 0, sizeof(double)*1521);		memset(bb, 0, sizeof(double)*1521);
	memset(al, 0, sizeof(double)*39);		memset(bl, 0, sizeof(double)*39);
	memset(altmp, 0, sizeof(double)*39);	memset(bltmp, 0, sizeof(double)*39);
	// 构建法方程
	double P, L, H, X, Y;
	for(int i=0; i<m_RFMgcp.m_nGCPNum; i++)
	{
		P = m_RFMgcp.m_P[i];	L = m_RFMgcp.m_L[i];	H = m_RFMgcp.m_H[i];
		X = m_RFMgcp.m_X[i];	Y = m_RFMgcp.m_Y[i];
		a[0] = 1;		a[1] = L;		a[2] = P;		a[3] = H;		a[4] = L*P;
		a[5] = L*H;		a[6] = P*H;		a[7] = L*L;		a[8] = P*P;		a[9] = H*H;
		a[10]= P*L*H;	a[11]= L*L*L;	a[12]= L*P*P;	a[13]= L*H*H;	a[14]= L*L*P;
		a[15]= P*P*P;	a[16]= P*H*H;	a[17]= L*L*H;	a[18]= P*P*H;	a[19]= H*H*H;
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*X;
		}
		m_base.pNormal(a, 39, X, aa, al, 1.0);	
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*Y;
		}
		m_base.pNormal(a, 39, Y, bb, bl, 1.0);
	}
	// 求解
	altmp[0] = altmp[1] = altmp[2] = altmp[3] = altmp[20] = altmp[21] = altmp[22] = 1.0;
	m_base.GaussExt(aa, al, altmp, 39);
	printf("\t沿轨向条件数：%lf\n", m_base.Mat_Cov(aa, 39, 2));
	bltmp[0] = bltmp[1] = bltmp[2] = bltmp[3] = bltmp[20] = bltmp[21] = bltmp[22] = 1.0;
	m_base.GaussExt(bb, bl, altmp, 39);
	printf("\t垂轨向条件数：%lf\n", m_base.Mat_Cov(bb, 39, 2));
	// RPC参数初始化
	memset(m_rfm.LNUM, 0, sizeof(double)*20);
	memset(m_rfm.LDEN, 0, sizeof(double)*20);
	memset(m_rfm.SNUM, 0, sizeof(double)*20);
	memset(m_rfm.SDEN, 0, sizeof(double)*20);
	// RPC参数赋值
	memcpy(m_rfm.LNUM,	 altmp,	sizeof(double)*20);
	memcpy(m_rfm.LDEN+1, altmp+20, sizeof(double)*19);
	memcpy(m_rfm.SNUM,	 bltmp,	sizeof(double)*20);
	memcpy(m_rfm.SDEN+1, bltmp+20, sizeof(double)*19);
	m_rfm.LDEN[0]=1.0; 	
	m_rfm.SDEN[0]=1.0; 
}

///////////////////////////////////////////
// 求解RPC的直接岭估计方法(L曲线法)
///////////////////////////////////////////
void GeoModelRFM::Cal_DirectLCurve()
{
	printf("直接岭估计方法\n");
	// 开始计算二阶量
	double a[39], aa[1521], al[39], bb[1521], bl[39], altmp[39], bltmp[39];
	memset(aa, 0, sizeof(double)*1521);		memset(bb, 0, sizeof(double)*1521);
	memset(al, 0, sizeof(double)*39);		memset(bl, 0, sizeof(double)*39);
	memset(altmp, 0, sizeof(double)*39);	memset(bltmp, 0, sizeof(double)*39);
	// 构建法方程
	double P, L, H, X, Y;
	double Lx, Ly;
	Lx = Ly = 0.0;
	for(int i=0; i<m_RFMgcp.m_nGCPNum; i++)
	{
		P = m_RFMgcp.m_P[i];	L = m_RFMgcp.m_L[i];	H = m_RFMgcp.m_H[i];
		X = m_RFMgcp.m_X[i];	Y = m_RFMgcp.m_Y[i];
		a[0] = 1;		a[1] = L;		a[2] = P;		a[3] = H;		a[4] = L*P;
		a[5] = L*H;		a[6] = P*H;		a[7] = L*L;		a[8] = P*P;		a[9] = H*H;
		a[10]= P*L*H;	a[11]= L*L*L;	a[12]= L*P*P;	a[13]= L*H*H;	a[14]= L*L*P;
		a[15]= P*P*P;	a[16]= P*H*H;	a[17]= L*L*H;	a[18]= P*P*H;	a[19]= H*H*H;
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*X;
		}
		m_base.pNormal(a, 39, X, aa, al, 1.0);
		Lx += X*X;
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*Y;
		}
		m_base.pNormal(a, 39, Y, bb, bl, 1.0);
		Ly += Y*Y;
	}
	// 求解
	m_base.GaussL(aa, al, 39, Lx);
	printf("\t沿轨向条件数：%lf\n", m_base.Mat_Cov(aa, 39, 2));
	m_base.GaussL(bb, bl, 39, Ly);
	printf("\t垂轨向条件数：%lf\n", m_base.Mat_Cov(bb, 39, 2));
	// RPC参数初始化
	memset(m_rfm.LNUM, 0, sizeof(double)*20);
	memset(m_rfm.LDEN, 0, sizeof(double)*20);
	memset(m_rfm.SNUM, 0, sizeof(double)*20);
	memset(m_rfm.SDEN, 0, sizeof(double)*20);
	// RPC参数赋值
	memcpy(m_rfm.LNUM,	 altmp,	sizeof(double)*20);
	memcpy(m_rfm.LDEN+1, altmp+20, sizeof(double)*19);
	memcpy(m_rfm.SNUM,	 bltmp,	sizeof(double)*20);
	memcpy(m_rfm.SDEN+1, bltmp+20, sizeof(double)*19);
	m_rfm.LDEN[0]=1.0; 	
	m_rfm.SDEN[0]=1.0; 
}


///////////////////////////////////////////
// 求解RPC的直接正交匹配方法
///////////////////////////////////////////
void GeoModelRFM::Cal_DirectOMP()
{
	printf("直接正交匹配方法\n");
	// 开始计算二阶量
	double a[39], aa[1521], al[39], bb[1521], bl[39], altmp[39], bltmp[39], x[39], y[39];
	memset(aa, 0, sizeof(double)*1521);		memset(bb, 0, sizeof(double)*1521);
	memset(al, 0, sizeof(double)*39);		memset(bl, 0, sizeof(double)*39);
	memset(altmp, 0, sizeof(double)*39);	memset(bltmp, 0, sizeof(double)*39);
	// 构建法方程
	double P, L, H, X, Y;
	for(int i=0; i<m_RFMgcp.m_nGCPNum; i++)
	{
		P = m_RFMgcp.m_P[i];	L = m_RFMgcp.m_L[i];	H = m_RFMgcp.m_H[i];
		X = m_RFMgcp.m_X[i];	Y = m_RFMgcp.m_Y[i];
		a[0] = 1;		a[1] = L;		a[2] = P;		a[3] = H;		a[4] = L*P;
		a[5] = L*H;		a[6] = P*H;		a[7] = L*L;		a[8] = P*P;		a[9] = H*H;
		a[10]= P*L*H;	a[11]= L*L*L;	a[12]= L*P*P;	a[13]= L*H*H;	a[14]= L*L*P;
		a[15]= P*P*P;	a[16]= P*H*H;	a[17]= L*L*H;	a[18]= P*P*H;	a[19]= H*H*H;
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*X;
		}
		m_base.pNormal(a, 39, X, aa, al, 1.0);
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*Y;
		}
		m_base.pNormal(a, 39, Y, bb, bl, 1.0);
	}
	// 求解
	int num;
	num = m_base.OMP_Solve(aa, al, 39, 39, x, 10e-8, 10e-10);
	printf("\t沿轨向参数个数：%d\n\t沿轨向条件数：%lf\n", num, m_base.Mat_Cov(aa, num, 2));
	num = m_base.OMP_Solve(bb, bl, 39, 39, y, 10e-6, 10e-8);
	printf("\t垂轨向参数个数：%d\n\t垂轨向条件数：%lf\n", num, m_base.Mat_Cov(bb, num, 2));
	// RPC参数初始化
	memset(m_rfm.LNUM, 0, sizeof(double)*20);
	memset(m_rfm.LDEN, 0, sizeof(double)*20);
	memset(m_rfm.SNUM, 0, sizeof(double)*20);
	memset(m_rfm.SDEN, 0, sizeof(double)*20);
	// RPC参数赋值
	memcpy(m_rfm.LNUM,	 x,	sizeof(double)*20);
	memcpy(m_rfm.LDEN+1, x+20, sizeof(double)*19);
	memcpy(m_rfm.SNUM,	 y,	sizeof(double)*20);
	memcpy(m_rfm.SDEN+1, y+20, sizeof(double)*19);
	m_rfm.LDEN[0]=1.0; 	
	m_rfm.SDEN[0]=1.0; 
}


///////////////////////////////////////////
// 求解RPC的间接最小二乘方法
///////////////////////////////////////////
void GeoModelRFM::Cal_IndirectLM()
{
	printf("间接最小二乘方法\n");
	// 在算出二阶量的基础上再算三阶量
	Cal_Order2();
	// 开始计算三阶量
	double X[39], Y[39], XTLx[39], YTLy[39];
	double XTX[1521], XPre[39], YTY[1521], YPre[39];

	double sigma0, sigma1 = FLT_MAX;
	double NumS, DenS, NumL, DenL, Lx, Ly;
	int iter = 0;
	double Num[20], P, L, H;
	do 
	{
		sigma0 = 0.;
		memset(XTLx, 0, sizeof(double)*39);		memset(YTLy, 0, sizeof(double)*39);
		memset(XTX, 0, sizeof(double)*1521);	memset(YTY, 0, sizeof(double)*1521);
		// 法方程的构建
		for(long m=0; m<m_RFMgcp.m_nGCPNum; m++)		// 控制点循环
		{
			// 求取20个的值
			P = m_RFMgcp.m_P[m];	L = m_RFMgcp.m_L[m];	H = m_RFMgcp.m_H[m];
			Num[0] = 1;		Num[1] = L;		Num[2] = P;		Num[3] = H;		Num[4] = L*P;
			Num[5] = L*H;	Num[6] = P*H;	Num[7] = L*L;	Num[8] = P*P;	Num[9] = H*H;
			Num[10]= P*L*H;	Num[11]= L*L*L;	Num[12]= L*P*P;	Num[13]= L*H*H;	Num[14]= L*L*P;
			Num[15]= P*P*P;	Num[16]= P*H*H;	Num[17]= L*L*H;	Num[18]= P*P*H;	Num[19]= H*H*H;
			NumS = DenS = NumL = DenL = 0;
			memcpy(X, Num, sizeof(double)*20);		memcpy(Y, Num, sizeof(double)*20);
			for(int i=20; i<39; i++)
			{
				X[i] = -m_RFMgcp.m_X[m]*X[i-20+1];
				Y[i] = -m_RFMgcp.m_Y[m]*Y[i-20+1];
			}
			for(int i=0; i<20; i++)
			{
				NumS += m_rfm.SNUM[i]*Y[i];
				NumL += m_rfm.LNUM[i]*X[i];
				DenS += m_rfm.SDEN[i]*Y[i];
				DenL += m_rfm.LDEN[i]*X[i];
			}
			// 计算残差
			Lx = -(NumL - DenL*m_RFMgcp.m_X[m]);
			Ly = -(NumS - DenS*m_RFMgcp.m_Y[m]);
			sigma0 += Lx*Lx;
			sigma0 += Ly*Ly;
			
			// 开始逐点构建法方程
			m_base.pNormal(X, 39, Lx, XTX, XTLx, 1.0);
			m_base.pNormal(Y, 39, Ly, YTY, YTLy, 1.0);
		}
		// 计算标准差
		sigma0 = sqrt(sigma0/m_RFMgcp.m_nGCPNum);
		if(sigma0>sigma1)
		{
			for(int i = 0; i<20; i++)
			{
				m_rfm.LNUM[i] -= XPre[i];
				m_rfm.SNUM[i] -= YPre[i];
			}
			for(int i = 20+1; i<=39; i++)
			{
				m_rfm.LDEN[i-20] -= XPre[i-1];
				m_rfm.SDEN[i-20] -= YPre[i-1];
			}
			break;
		}
		sigma1 = sigma0;
		// 王新洲方法求解
		m_base.Gauss(XTX, XTLx, 39);
		memcpy(XPre, XTLx, sizeof(double)*39);
		m_base.Gauss(YTY, YTLy, 39);
		memcpy(YPre, YTLy, sizeof(double)*39);
		printf("\t沿轨向条件数：%lf\n", m_base.Mat_Cov(XTX, 39, 2));
		printf("\t垂轨向条件数：%lf\n", m_base.Mat_Cov(YTY, 39, 2));
		for(int i=0; i<20; i++)
		{
			m_rfm.LNUM[i] += X[i];	
			m_rfm.SNUM[i] += Y[i];
		}
		for(int i=20+1; i<=39; i++)
		{
			m_rfm.LDEN[i-20] += X[i-1];
			m_rfm.SDEN[i-20] += Y[i-1];
		}
		iter++;
		if(iter>100)
			break;
	}while(1);
}

///////////////////////////////////////////
// 求解RPC的间接谱修正方法
///////////////////////////////////////////
void GeoModelRFM::Cal_IndirectLME()
{
	//printf("间接谱修正方法\n");
	// 在算出二阶量的基础上再算三阶量
	Cal_Order2();
	// 开始计算三阶量
	double X[39], Y[39], XTLx[39], YTLy[39];
	double XTX[1521], XPre[39], YTY[1521], YPre[39];

	double sigma0, sigma1 = FLT_MAX;
	double NumS, DenS, NumL, DenL, Lx, Ly;
	int iter = 0;
	double Num[20], P, L, H;
	do 
	{
		sigma0 = 0.;
		memset(XTLx, 0, sizeof(double)*39);		memset(YTLy, 0, sizeof(double)*39);
		memset(XTX, 0, sizeof(double)*1521);	memset(YTY, 0, sizeof(double)*1521);
		// 法方程的构建
		for(long m=0; m<m_RFMgcp.m_nGCPNum; m++)		// 控制点循环
		{
			// 求取20个的值
			P = m_RFMgcp.m_P[m];	L = m_RFMgcp.m_L[m];	H = m_RFMgcp.m_H[m];
			Num[0] = 1;		Num[1] = L;		Num[2] = P;		Num[3] = H;		Num[4] = L*P;
			Num[5] = L*H;	Num[6] = P*H;	Num[7] = L*L;	Num[8] = P*P;	Num[9] = H*H;
			Num[10]= P*L*H;	Num[11]= L*L*L;	Num[12]= L*P*P;	Num[13]= L*H*H;	Num[14]= L*L*P;
			Num[15]= P*P*P;	Num[16]= P*H*H;	Num[17]= L*L*H;	Num[18]= P*P*H;	Num[19]= H*H*H;
			NumS = DenS = NumL = DenL = 0;
			memcpy(X, Num, sizeof(double)*20);		memcpy(Y, Num, sizeof(double)*20);
			for(int i=20; i<39; i++)
			{
				X[i] = -m_RFMgcp.m_X[m]*X[i-20+1];
				Y[i] = -m_RFMgcp.m_Y[m]*Y[i-20+1];
			}
			for(int i=0; i<20; i++)
			{
				NumS += m_rfm.SNUM[i]*Y[i];
				NumL += m_rfm.LNUM[i]*X[i];
				DenS += m_rfm.SDEN[i]*Y[i];
				DenL += m_rfm.LDEN[i]*X[i];
			}
			// 计算残差
			Lx = -(NumL - DenL*m_RFMgcp.m_X[m]);
			Ly = -(NumS - DenS*m_RFMgcp.m_Y[m]);
			sigma0 += Lx*Lx;
			sigma0 += Ly*Ly;
			
			// 开始逐点构建法方程
			m_base.pNormal(X, 39, Lx, XTX, XTLx, 1.0);
			m_base.pNormal(Y, 39, Ly, YTY, YTLy, 1.0);
		}
		// 计算标准差
		sigma0 = sqrt(sigma0/m_RFMgcp.m_nGCPNum);
		if(sigma0>sigma1)
		{
			for(int i = 0; i<20; i++)
			{
				m_rfm.LNUM[i] -= XPre[i];
				m_rfm.SNUM[i] -= YPre[i];
			}
			for(int i = 20+1; i<=39; i++)
			{
				m_rfm.LDEN[i-20] -= XPre[i-1];
				m_rfm.SDEN[i-20] -= YPre[i-1];
			}
			break;
		}
		sigma1 = sigma0;
		// 王新洲方法求解
		m_base.GaussExt(XTX, XTLx, X, 39);
		memcpy(XPre, X, sizeof(double)*39);
		m_base.GaussExt(YTY, YTLy, Y, 39);
		memcpy(YPre, Y, sizeof(double)*39);
		//printf("\t沿轨向条件数：%lf\n", m_base.Mat_Cov(XTX, 39, 2));
		//printf("\t垂轨向条件数：%lf\n", m_base.Mat_Cov(YTY, 39, 2));
		for(int i=0; i<20; i++)
		{
			m_rfm.LNUM[i] += X[i];	
			m_rfm.SNUM[i] += Y[i];
		}
		for(int i=20+1; i<=39; i++)
		{
			m_rfm.LDEN[i-20] += X[i-1];
			m_rfm.SDEN[i-20] += Y[i-1];
		}
		iter++;
		if(iter>10)
			break;
	}while(1);
}

///////////////////////////////////////////
// 求解RPC的间接岭估计方法
///////////////////////////////////////////
void GeoModelRFM::Cal_IndirectLCurve()
{
	printf("间接岭估计方法\n");
	// 在算出二阶量的基础上再算三阶量
	Cal_Order2();
	// 开始计算三阶量
	double X[39], Y[39], XTLx[39], YTLy[39];
	double XTX[1521], XPre[39], YTY[1521], YPre[39];

	double sigma0, sigma1 = FLT_MAX;
	double NumS, DenS, NumL, DenL, Lx, Ly;
	int iter = 0;
	double Num[20], P, L, H;
	double Lx2, Ly2;
	do 
	{
		Lx2 = Ly2 = sigma0 = 0.;
		memset(XTLx, 0, sizeof(double)*39);		memset(YTLy, 0, sizeof(double)*39);
		memset(XTX, 0, sizeof(double)*1521);	memset(YTY, 0, sizeof(double)*1521);
		// 法方程的构建
		for(long m=0; m<m_RFMgcp.m_nGCPNum; m++)		// 控制点循环
		{
			// 求取20个的值
			P = m_RFMgcp.m_P[m];	L = m_RFMgcp.m_L[m];	H = m_RFMgcp.m_H[m];
			Num[0] = 1;		Num[1] = L;		Num[2] = P;		Num[3] = H;		Num[4] = L*P;
			Num[5] = L*H;	Num[6] = P*H;	Num[7] = L*L;	Num[8] = P*P;	Num[9] = H*H;
			Num[10]= P*L*H;	Num[11]= L*L*L;	Num[12]= L*P*P;	Num[13]= L*H*H;	Num[14]= L*L*P;
			Num[15]= P*P*P;	Num[16]= P*H*H;	Num[17]= L*L*H;	Num[18]= P*P*H;	Num[19]= H*H*H;
			NumS = DenS = NumL = DenL = 0;
			memcpy(X, Num, sizeof(double)*20);		memcpy(Y, Num, sizeof(double)*20);
			for(int i=20; i<39; i++)
			{
				X[i] = -m_RFMgcp.m_X[m]*X[i-20+1];
				Y[i] = -m_RFMgcp.m_Y[m]*Y[i-20+1];
			}
			for(int i=0; i<20; i++)
			{
				NumS += m_rfm.SNUM[i]*Y[i];
				NumL += m_rfm.LNUM[i]*X[i];
				DenS += m_rfm.SDEN[i]*Y[i];
				DenL += m_rfm.LDEN[i]*X[i];
			}
			// 计算残差
			Lx = -(NumL - DenL*m_RFMgcp.m_X[m]);
			Ly = -(NumS - DenS*m_RFMgcp.m_Y[m]);
			sigma0 += Lx*Lx;
			sigma0 += Ly*Ly;
			Lx2 += Lx*Lx;
			Ly2 += Ly*Ly;
			
			// 开始逐点构建法方程
			m_base.pNormal(X, 39, Lx, XTX, XTLx, 1.0);
			m_base.pNormal(Y, 39, Ly, YTY, YTLy, 1.0);
		}
		// 计算标准差
		sigma0 = sqrt(sigma0/m_RFMgcp.m_nGCPNum);
		if(sigma0>sigma1)
		{
			for(int i = 0; i<20; i++)
			{
				m_rfm.LNUM[i] -= XPre[i];
				m_rfm.SNUM[i] -= YPre[i];
			}
			for(int i = 20+1; i<=39; i++)
			{
				m_rfm.LDEN[i-20] -= XPre[i-1];
				m_rfm.SDEN[i-20] -= YPre[i-1];
			}
			break;
		}
		sigma1 = sigma0;
		// 王新洲方法求解
		m_base.GaussL(XTX, XTLx, 39, Lx2);
		memcpy(XPre, X, sizeof(double)*39);
		m_base.GaussL(YTY, YTLy, 39, Ly2);
		memcpy(YPre, Y, sizeof(double)*39);
		printf("\t沿轨向条件数：%lf\n", m_base.Mat_Cov(XTX, 39, 2));
		printf("\t垂轨向条件数：%lf\n", m_base.Mat_Cov(YTY, 39, 2));
		for(int i=0; i<20; i++)
		{
			m_rfm.LNUM[i] += X[i];	
			m_rfm.SNUM[i] += Y[i];
		}
		for(int i=20+1; i<=39; i++)
		{
			m_rfm.LDEN[i-20] += X[i-1];
			m_rfm.SDEN[i-20] += Y[i-1];
		}
		iter++;
		if(iter>100)
			break;
	}while(1);
}


///////////////////////////////////////////
// 求解RPC的间接正交匹配方法
///////////////////////////////////////////
void GeoModelRFM::Cal_IndirectOMP()
{
	printf("直接正交匹配方法\n");
	// 开始计算二阶量
	double a[39], aa[1521], al[39], bb[1521], bl[39], altmp[39], bltmp[39], x[39], y[39];
	memset(aa, 0, sizeof(double)*1521);		memset(bb, 0, sizeof(double)*1521);
	memset(al, 0, sizeof(double)*39);		memset(bl, 0, sizeof(double)*39);
	memset(altmp, 0, sizeof(double)*39);	memset(bltmp, 0, sizeof(double)*39);
	// 构建法方程
	double P, L, H, X, Y;
	for(int i=0; i<m_RFMgcp.m_nGCPNum; i++)
	{
		P = m_RFMgcp.m_P[i];	L = m_RFMgcp.m_L[i];	H = m_RFMgcp.m_H[i];
		X = m_RFMgcp.m_X[i];	Y = m_RFMgcp.m_Y[i];
		a[0] = 1;		a[1] = L;		a[2] = P;		a[3] = H;		a[4] = L*P;
		a[5] = L*H;		a[6] = P*H;		a[7] = L*L;		a[8] = P*P;		a[9] = H*H;
		a[10]= P*L*H;	a[11]= L*L*L;	a[12]= L*P*P;	a[13]= L*H*H;	a[14]= L*L*P;
		a[15]= P*P*P;	a[16]= P*H*H;	a[17]= L*L*H;	a[18]= P*P*H;	a[19]= H*H*H;
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*X;
		}
		m_base.pNormal(a, 39, X, aa, al, 1.0);
		for(int j=20;j<39;j++)
		{
			a[j] = -a[j-19]*Y;
		}
		m_base.pNormal(a, 39, Y, bb, bl, 1.0);
	}
	// 求解
	int num;
	num = m_base.OMP_Solve2(aa, al, 39, 39, x, 10e-8, 10e-8);
	printf("\t沿轨向参数个数：%d\n\t沿轨向条件数：%lf\n", num, m_base.Mat_Cov(aa, num, 2));
	num = m_base.OMP_Solve2(bb, bl, 39, 39, y, 10e-10, 10e-8);
	printf("\t垂轨向参数个数：%d\n\t垂轨向条件数：%lf\n", num, m_base.Mat_Cov(bb, num, 2));
	// RPC参数初始化
	memset(m_rfm.LNUM, 0, sizeof(double)*20);
	memset(m_rfm.LDEN, 0, sizeof(double)*20);
	memset(m_rfm.SNUM, 0, sizeof(double)*20);
	memset(m_rfm.SDEN, 0, sizeof(double)*20);
	// RPC参数赋值
	memcpy(m_rfm.LNUM,	 x,	sizeof(double)*20);
	memcpy(m_rfm.LDEN+1, x+20, sizeof(double)*19);
	memcpy(m_rfm.SNUM,	 y,	sizeof(double)*20);
	memcpy(m_rfm.SDEN+1, y+20, sizeof(double)*19);
	m_rfm.LDEN[0]=1.0; 	
	m_rfm.SDEN[0]=1.0; 
}