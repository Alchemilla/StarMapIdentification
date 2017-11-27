#include "GeoModelLine.h"


//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoModelLine::GeoModelLine(void)
{
	m_timeUT = NULL;			// 存储UT时间,按照扫描行来存储
	m_wgs84 = NULL;				// GPS相位中心在wgs84下位置,按照扫描行来存储
	m_body2wgs84 = NULL;		// 本体到wgs84的姿态,按照扫描行来存储
	m_dppoint = NULL;			// DouglasPeucker存储变量
	m_Plane = NULL;				// 面方程的变量
	m_CCDnum = 0;				// 垂轨向通过压缩后的分片CCD个数
}


GeoModelLine::~GeoModelLine(void)
{
	//Destroy();
}


//////////////////////////////////////
// 初始化参数
//////////////////////////////////////
void GeoModelLine::Destroy()
{
	if(m_timeUT!=NULL)		delete []m_timeUT;		m_timeUT = NULL;
	if(m_wgs84!=NULL)		delete []m_wgs84;		m_wgs84 = NULL;
	if(m_body2wgs84!=NULL)	delete []m_body2wgs84;	m_body2wgs84 = NULL;
	if(m_dppoint!=NULL)		delete []m_dppoint;		m_dppoint = NULL;
	if(m_Plane!=NULL)
	{
		for(int i=0; i<m_CCDnum; i++)
		{
			delete [](m_Plane[i]);
			m_Plane[i] = NULL;
		}
		m_Plane = NULL;
		m_CCDnum = 0;
	}
}


//////////////////////////////////////
// 计算姿轨多项式模型以及整数位的姿轨以及常见的安装
//////////////////////////////////////
void GeoModelLine::ReCalPosAndAtt()
{
	// 清空之前的内存
	Destroy();
	// 分配空间
	m_timeUT = new double[m_xnum];
	m_wgs84 = new struct StrOrbitPoint[m_xnum];
	m_body2wgs84 = new struct StrAttPoint[m_xnum];
	// 获得时间
	for(int i=0;i<m_xnum;i++)
	{
		m_timeUT[i] = m_time->get_time(i);
	}
	// 进行多项式拟合(轨道,姿态)
	m_orbit->GenPolyModel(m_timeUT[0]-m_input.timeExtend, m_timeUT[m_xnum-1]+m_input.timeExtend);
	m_att->GenPolyModel(m_timeUT[0]-m_input.timeExtend, m_timeUT[m_xnum-1]+m_input.timeExtend);
	// GPS在本体坐标系下的偏心距(定轨-本体)
	m_GPSSet.X[0] = m_orbit->get_OffX();
	m_GPSSet.X[1] = m_orbit->get_OffY();
	m_GPSSet.X[2] = m_orbit->get_OffZ();
	// 相机在本体坐标系下的偏心距
	m_camSet.X[0] = m_cam->get_OffX();
	m_camSet.X[1] = m_cam->get_OffY();
	m_camSet.X[2] = m_cam->get_OffZ();
	// 将相机从相机系移到GPS系的偏心距(本体坐标系下)
	m_cam2GPSSet.X[0] = -m_GPSSet.X[0] + m_camSet.X[0];
	m_cam2GPSSet.X[1] = -m_GPSSet.X[1] + m_camSet.X[1];
	m_cam2GPSSet.X[2] = -m_GPSSet.X[2] + m_camSet.X[2];
	// 从测姿坐标系到本体坐标系的旋转矩阵
	//m_att->get_ROff(m_body2att.R);
	//m_base.Transpose(m_body2att.R, 3, 3);
	// 从相机到本体的旋转矩阵
	m_cam->get_ROff(m_cam2body.R);
	//m_base.Transpose(m_cam2body.R,3,3);
	// 真实姿态和轨道
	for(int i=0;i<m_xnum;i++)
	{
		// WGS4的位置
		if(m_input.isOrbitPoly == false)	m_wgs84[i] = m_orbit->GetEpWGS84(m_timeUT[i]);
		else								m_wgs84[i] = m_orbit->PolyValue(m_timeUT[i]);
		// 本体到wgs84旋转矩阵
		if(m_input.isAttPoly == false)		m_body2wgs84[i] = m_att->GetAttBody2WGS84(m_timeUT[i]);		
		else								m_att->PolyValue(m_timeUT[i], m_body2wgs84[i].R);
	}
}


//////////////////////////////////////
// 功能：获取全局变换参数以及经纬度/像素
// 输入:
//		无
// 返回值：
//		void
//////////////////////////////////////
void GeoModelLine::ComputerGlobalParam()
{
	double H = 0;
	// 获取四个角点以及中心点
	double x[5], y[5] ,lat[5], lon[5];
	x[0] = m_xnum/2;    y[0] = m_ynum/2;
	x[1] = 0;           y[1] = 0;
	x[2] = m_xnum-1;    y[2] = 0;
	x[3] = m_xnum-1;    y[3] = m_ynum-1;
	x[4] = 0;           y[4] = m_ynum-1;
	for(int i=0;i<5;i++)
	{
		FromXY2LatLon(x[i], y[i], H, lat[i], lon[i]);
	}
	m_trans.CalAffineParam(lat, lon, x, y, 5);
	// 获取一像素大小在地面的投影,是一个比例系数:经纬度/像素
	double lat1, lat2, lon1, lon2;
	double dx, dy;
	dx = dy = 10;
	FromXY2LatLon(m_xnum/2.0, m_ynum/2.0, H, lat1, lon1);
	FromXY2LatLon(m_xnum/2.0+dx, m_ynum/2.0+dy, H, lat2, lon2);	
	m_pixelsize = sqrt(pow((lat1-lat2),2)+pow((lon1-lon2),2)/(dx*dx+dy*dy));
	m_pixelsize2 = pow(m_pixelsize, 2);
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
void GeoModelLine::PreciseBasedAffine(double &x,double &y,double lat,double lon,double H,double dx,double dy)
{
	// 获取四个角点以及中心点
	double xtemp[4], ytemp[4], lattemp[4], lontemp[4];
	xtemp[0] = x+dx;   ytemp[0] = y+dy;
	xtemp[1] = x+dx;   ytemp[1] = y-dy;
	xtemp[2] = x-dx;   ytemp[2] = y+dy;
	xtemp[3] = x-dx;   ytemp[3] = y-dy;
	for(int i=0;i<4;i++)
	{
		FromXY2LatLon(xtemp[i], ytemp[i], H, lattemp[i], lontemp[i]);
	}
	// 进行仿射变换
	GeoTranslation m_transtemp;
	m_transtemp.CalAffineParam(lattemp, lontemp, xtemp, ytemp, 4);
	// 开始预测
	m_transtemp.GetValueBaseAffine(lat, lon, x, y);
}

//////////////////////////////////////////////////////////////////////////
//功能：前方交会代码
//输入：左右影像模型，匹配控制点
//输出：交会点空间坐标XYZ
//作者：GZC
//时间：2017.09.08
//////////////////////////////////////////////////////////////////////////
void GeoModelLine::Intersection(GeoModelLine *model, MatchPoint pts, double *LatLonH)
{
	double inner1[3], inner2[3], Rcam2wgs84L[9], Rcam2wgs84R[9], XYZ1[3], XYZ2[3], S1[3], S2[3];
	model[0].GetOrbitAttitudeInnerBaseXY(pts.lx, pts.ly, XYZ1, Rcam2wgs84L, inner1);
	model[1].GetOrbitAttitudeInnerBaseXY(pts.rx, pts.ry, XYZ2, Rcam2wgs84R, inner2);

	m_base.Multi(Rcam2wgs84L, inner1, S1, 3, 3, 1);
	m_base.Multi(Rcam2wgs84R, inner2, S2, 3, 3, 1);

	double Bu, Bv, Bw;
	Bu = XYZ2[0] - XYZ1[0];
	Bv = XYZ2[1] - XYZ1[1];
	Bw = XYZ2[2] - XYZ1[2];

	double N1 = (Bu * S2[2] - Bw * S2[0]) / (S1[0] * S2[2] - S2[0] * S1[2]);
	double N2 = (Bu * S1[2] - Bw * S1[0]) / (S1[0] * S2[2] - S2[0] * S1[2]);

	LatLonH[0] = XYZ1[0] + N1 * S1[0];
	LatLonH[1] = 0.5 * ((XYZ1[1] + N1 * S1[1]) + (XYZ2[1] + N2 * S2[1]));
	LatLonH[2] = XYZ1[2] + N1 * S1[2];

	double lat, lon, H;
	m_base.Rect2Geograph(m_datum, LatLonH[0], LatLonH[1], LatLonH[2], lat, lon, H);
	LatLonH[0] = lat;
	LatLonH[1] = lon;
	LatLonH[2] = H;
}



//////////////////////////////////////
// 功能：线阵模型初始化函数
// 输入:
//		GeoOrbit *orbit:	轨道对象
//		GeoAttitude *att:	姿态对象
//		GeoTime *time:		时间对象
//		GeoCamera *cam：	相机对象
//		StrModelParamInput input：	线阵模型输入参数
// 返回值：
//		void
//////////////////////////////////////
void GeoModelLine::InitModelLine(GeoOrbit *orbit, GeoAttitude *att, GeoTime *time, GeoCamera *cam, StrModelParamInput input)
{
	m_orbit = orbit;
	m_att = att;
	m_time = time;
	m_cam = cam;
	m_input = input;
	// 获取模型影像长度和宽度
	m_xnum = m_time->get_num();
	m_ynum = m_cam->get_ynum();
	// 获取模型所定位的基准椭球参数
	m_datum = m_orbit->get_datum();
	// 计算姿轨多项式模型以及整数位的姿轨以及常见的安装
	ReCalPosAndAtt();
	// 获取全局变换参数以及经纬度/像素
	ComputerGlobalParam();
	// DouglasPeucker对线阵CCD进行分片
//	CreateProjectPlane();
}

/////////////////////////////////////////////
//功能：读取外检校参数
//输入：辅参数路径fpath
//输出：m_cam2body.R
//注意：如果fpath没有填写，则默认不改变m_cam2body.R
//日期：2016.12.22
////////////////////////////////////////////
bool GeoModelLine::InitExtFile(string fpath)
{
	if (fpath.empty()==1)
	{
		printf("The Ru file is empty\n");
		return 1;
	} 
	else
	{
		FILE *fp = fopen(fpath.c_str(),"r");
		if (!fp)
		{
			printf("Fail to read Ru file\n");
			return false;
		}
		fscanf(fp, "%*s\t%*s\t%*s\n");
		double phi, omg, kap;
		fscanf(fp, "%lf\n",&phi);
		fscanf(fp, "%lf\n",&omg);
		fscanf(fp, "%lf\n",&kap);
		m_base.Eulor2Matrix(phi,omg,kap,213,m_body2att.R);
		return true;
	}
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
bool GeoModelLine::FromXY2LatLon(double x, double y, double &H, double &lat, double &lon)
{
	//存储对应时刻的姿轨信息
	double UT;
	struct StrOrbitPoint m_ephpoint;
	struct StrAttPoint m_attbody2wgs84;
	
	// 如果在已经存储的点上面,直接使用以提高速度
	if((fabs(x-(long)(x+0.5))<10e-10)&&(x>=0)&&(x<m_xnum))
	{
		UT = m_timeUT[(long)(x+0.5)];
		m_ephpoint = m_wgs84[(long)(x+0.5)];
		m_attbody2wgs84 = m_body2wgs84[(long)(x+0.5)];
	}
	else
	{
		// 获得对应的时间
		UT = m_time->get_time(x);
		// 获得此时间对应的轨道信息
		if(m_input.isOrbitPoly == false)	m_ephpoint = m_orbit->GetEpWGS84(UT);
		else								m_ephpoint = m_orbit->PolyValue(UT);
		// 获得此时间对应的姿态信息,本体到WGS84
		if(m_input.isAttPoly == false)		m_attbody2wgs84 = m_att->GetAttBody2WGS84(UT);
		else								m_att->PolyValue(UT, m_attbody2wgs84.R);
	}
	//////////////////////////////////////
	// 开始求和地球的交点
	//////////////////////////////////////
	double A, B, C, scale, B2_4AC, a, b, a2, b2;
	a = m_datum.a+H;		a2 = a*a;
	b = m_datum.b+H;		b2 = b*b;
	// GPS相位中心在WGS84系下的位置
	double EphPos[3];
	memcpy(EphPos, m_ephpoint.X, sizeof(double)*3);
	// 获得从本体到WGS84的旋转矩阵
	double m_body2WGS84[9];
	m_base.Multi(m_attbody2wgs84.R, m_body2att.R, m_body2WGS84, 3, 3, 3);
	// 相机在WGS84系下的位置
	double CamPos[3], temp1[3];
	m_base.Multi(m_body2WGS84, m_cam2GPSSet.X, temp1, 3, 3, 1);
	CamPos[0] = EphPos[0] + temp1[0];
	CamPos[1] = EphPos[1] + temp1[1];
	CamPos[2] = EphPos[2] + temp1[2];
	// 获取探元指向角
	double phi[3];
	m_cam->GetInner(x, y, phi[0], phi[1]);
	phi[2] = 1;
	// 获得探元指向角在wgs84系下的指向,为了减少精度损失,放大a倍
	double temp3[9], Pos[3], m_cam2WGS84[9];
	m_base.Multi(m_body2WGS84, m_cam2body.R, m_cam2WGS84, 3, 3, 3);
	m_base.Multi(m_cam2WGS84, phi, Pos, 3, 3, 1);
	Pos[0] *= a;   Pos[1] *= a;   Pos[2] *= a;
	A = (pow(Pos[0],2)+pow(Pos[1],2))/a2+pow(Pos[2],2)/b2;
	B = 2*((Pos[0]*CamPos[0]+Pos[1]*CamPos[1])/a2+Pos[2]*CamPos[2]/b2);
	C = (pow(CamPos[0],2) + pow(CamPos[1],2))/a2 + pow(CamPos[2],2)/b2-1;
	B2_4AC = B*B - 4*A*C;
	// 如果与地球不相交,则输出提示信息
	if(B2_4AC<0 || A==0)
	{
		printf("%lf-%lf	不与椭球相交:B2_4AC<0 || A==0\n", x, y);
		return false;
	}
	scale = (-B-sqrt(B2_4AC))/(2*A);
	if(scale<0)
	{
		printf("%lf-%lf	不与椭球相交:scale<0\n", x, y);
		return false;
	}
	// 求出地面点
	struct StrOrbitPoint m_Surface;
	m_Surface.X[0] = CamPos[0] + scale*Pos[0];
	m_Surface.X[1] = CamPos[1] + scale*Pos[1];
	m_Surface.X[2] = CamPos[2] + scale*Pos[2];
	double Htemp;
	m_base.Rect2Geograph(m_datum, m_Surface.X[0], m_Surface.X[1], m_Surface.X[2], lat, lon, Htemp);
	return true;
}


//////////////////////////////////////
// 功能：根据xy像素坐标得到轨道姿态和内方位元素
// 输入：
//		double x:		沿轨向的像素号
//		double y:		垂轨向的像素号
// 输出：
//		double *eph：	得到的轨道
//		double *R：		得到的姿态
//		double *inner:	得到的内方位元素
//////////////////////////////////////
void GeoModelLine::GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner)
{
	//存储对应时刻的姿轨信息
	double UT;
	struct StrOrbitPoint m_ephpoint;
	struct StrAttPoint m_attbody2wgs84;	
	// 如果在已经存储的点上面,直接使用以提高速度
	if((fabs(x-(long)(x+0.5))<10e-10)&&(x>=0)&&(x<m_xnum))
	{
		UT = m_timeUT[(long)(x+0.5)];
		m_ephpoint = m_wgs84[(long)(x+0.5)];
		m_attbody2wgs84 = m_body2wgs84[(long)(x+0.5)];
	}
	else
	{
		// 获得对应的时间
		UT = m_time->get_time(x);
		// 获得此时间对应的轨道信息
		if(m_input.isOrbitPoly == false)	m_ephpoint = m_orbit->GetEpWGS84(UT);
		else								m_ephpoint = m_orbit->PolyValue(UT);
		// 获得此时间对应的姿态信息,本体到WGS84
		if(m_input.isAttPoly == false)		m_attbody2wgs84 = m_att->GetAttBody2WGS84(UT);
		else								m_att->PolyValue(UT, m_attbody2wgs84.R);
	}
	// 获得从本体到WGS84的旋转矩阵，乘了一个偏置
	m_base.Multi(m_attbody2wgs84.R, m_body2att.R, R, 3, 3, 3);
	// 相机在WGS84系下的位置
	double temp1[3];
	m_base.Multi(R, m_cam2GPSSet.X, temp1, 3, 3, 1);
	eph[0] = m_ephpoint.X[0] + temp1[0];
	eph[1] = m_ephpoint.X[1] + temp1[1];
	eph[2] = m_ephpoint.X[2] + temp1[2];
	// 获取探元指向角
	m_cam->GetInner(x, y, inner[0], inner[1]);
	inner[2] = 1;
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
void GeoModelLine::FromLatLon2XY(double lat, double lon, double H, double &x, double &y)
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
void GeoModelLine::FromLatLon2XYBaseAffine(double lat, double lon, double H, double &x, double &y)
{
	double latp,lonp;
	m_trans.GetValueBaseAffine(lat, lon, x, y);
	FromXY2LatLon(x, y, H, latp, lonp);
	double e = 3*sqrt(((latp-lat)*(latp-lat)+(lonp-lon)*(lonp-lon))/m_pixelsize2);
    int num=0;
	do 
	{
		PreciseBasedAffine(x, y, lat, lon, H, e, e);
		FromXY2LatLon(x, y, H, latp, lonp);
		e = 3*((latp-lat)*(latp-lat)+(lonp-lon)*(lonp-lon))/m_pixelsize2;
		num++;
		if(e<1e-8)  break;
		if(num>20)    break;
	} while(1);
//	printf("\t%d\t", num);
}


//////////////////////////////////////
// 功能：从地面到影像(光学反算,基于像方约束)
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
void GeoModelLine::FromLatLon2XYBaseImageConst(double lat, double lon, double H, double &x, double &y)
{

}


//////////////////////////////////////
// 功能：从地面到影像(光学反算,基于物方约束)
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
void GeoModelLine::FromLatLon2XYBaseObjectConst(double lat, double lon, double H, double &x, double &y)
{
	double UT, temp1[3], eph[3], eph1[3], R[9];
	struct StrOrbitPoint m_ephpoint;
	struct StrAttPoint m_attbody2wgs84;	
	struct StrOrbitPoint m_Surface;
	m_base.Geograph2Rect(m_datum, lat, lon, H, m_Surface.X[0], m_Surface.X[1], m_Surface.X[2]);
	// 采用仿射预测大概位置,预测的x即预测行号,预测的y可以得到对应的面片号
	m_trans.GetValueBaseAffine(lat, lon, x, y);
	int xindex, yindex, yindextemp, count = 0;
	double dx, dxtemp[2], xtemp, ytemp;
	long indexx[2];
	double innerleft[2], innerright[2];
	// 越界判断
	if(int(y+0.5)<0)				yindextemp = 0;
	else if(int(y+0.5)>m_ynum-1)	yindextemp = m_CCDnum-1;
	else							yindextemp = m_dppoint[int(y+0.5)].index;
	if(int(x+0.5)<0)				xindex = 0;
	if(int(x+0.5)>m_xnum-1)			xindex = m_xnum-1;
	else							xindex = int(x+0.5);
	/////////////////////////////////////////////////////////
	// 当迭代次数太多或者反投影所在面片号和初始的面片号相同时
	// 退出循环
	/////////////////////////////////////////////////////////
	do
	{
		yindex = yindextemp;
		// 求出偏移像素,并迭代偏移
		do
		{
			// 越界判断,防止越界
			if(yindex>=0 && yindex<=m_CCDnum-1 && xindex>=0 && xindex<=m_xnum-1)	
			{
				dx = m_Plane[yindex][xindex].isPos*GetPoint2PlaneDis(m_Plane[yindex][xindex], m_Surface)/m_ccdDis;
			}
			else
				break;
			x = xindex + dx;
			if(dx<1)
				break;
			if(int(x)<0)				{	xindex = 0;	break; }
			if(int(x)>m_xnum-1)			{	xindex = m_xnum-1;	break; }
			else						xindex = int(x);
		}while(1);
		// 内插出准确行
		indexx[0] = long(x);
		indexx[1] = long(x+1);
		if(indexx[0]<0)
		{
			indexx[0] = 0;			indexx[1] = 1;
		}
		if(indexx[1]>m_xnum-1)
		{
			indexx[0] = m_xnum-2;	indexx[1] = m_xnum-1;
		}
		dxtemp[0] = m_Plane[yindex][indexx[0]].isPos*GetPoint2PlaneDis(m_Plane[yindex][indexx[0]], m_Surface);
		dxtemp[1] = -m_Plane[yindex][indexx[1]].isPos*GetPoint2PlaneDis(m_Plane[yindex][indexx[1]], m_Surface);
		x = indexx[0] + dxtemp[0]/(dxtemp[0]+dxtemp[1]);	// 求出的内插的沿轨向坐标
		//////////////////////////////////////////
		// 进行反投影计算
		//////////////////////////////////////////
		if((fabs(x-(long)(x+0.5))<10e-10)&&(x>=0)&&(x<m_xnum))
		{
			UT = m_timeUT[(long)(x+0.5)];
			m_ephpoint = m_wgs84[(long)(x+0.5)];
			m_attbody2wgs84 = m_body2wgs84[(long)(x+0.5)];
		}
		else
		{
			// 获得对应的时间
			UT = m_time->get_time(x);
			// 获得此时间对应的轨道信息
			if(m_input.isOrbitPoly == false)	m_ephpoint = m_orbit->GetEpWGS84(UT);
			else								m_ephpoint = m_orbit->PolyValue(UT);
			// 获得此时间对应的姿态信息,本体到WGS84
			if(m_input.isAttPoly == false)		m_attbody2wgs84 = m_att->GetAttBody2WGS84(UT);
			else								m_att->PolyValue(UT, m_attbody2wgs84.R);
		}
		// 获得从本体到WGS84的旋转矩阵
		m_base.Multi(m_attbody2wgs84.R, m_body2att.R, R, 3, 3, 3);
		// 相机在WGS84系下的位置
		m_base.Multi(R, m_cam2GPSSet.X, temp1, 3, 3, 1);
		eph[0] = m_ephpoint.X[0] + temp1[0];
		eph[1] = m_ephpoint.X[1] + temp1[1];
		eph[2] = m_ephpoint.X[2] + temp1[2];
		// 求取矢量
		eph[0] = m_Surface.X[0] - eph[0];
		eph[1] = m_Surface.X[1] - eph[1];
		eph[2] = m_Surface.X[2] - eph[2];
		// 乘以旋转矢量
		m_base.Transpose(R, 3, 3);
		m_base.Multi(R, eph, eph1, 3, 3, 1);
		xtemp = eph1[0]/eph1[2];
		ytemp = eph1[1]/eph1[2];
		m_cam->GetInner(x, m_StartEnd[yindex].Start, innerleft[0], innerleft[1]);
		m_cam->GetInner(x, m_StartEnd[yindex].End, innerright[0], innerright[1]);
//		y = m_StartEnd[yindex].Start + (ytemp-innerleft[1])*(m_StartEnd[yindex].End-
//			m_StartEnd[yindex].Start)/(innerright[1]-innerleft[1]);
		m_cam->GetIndexBaseInner(ytemp, 0.0, y, x);

		// 判断越界情况
		if(int(y+0.5)<0)				yindextemp = 0;
		else if(int(y+0.5)>m_ynum-1)	yindextemp = m_CCDnum-1;
		else							yindextemp = m_dppoint[int(y+0.5)].index;
		if(int(x+0.5)<0)				xindex = 0;
		if(int(x+0.5)>m_xnum-1)			xindex = m_xnum-1;
		else							xindex = int(x+0.5);
		count++;
		if(count>10||((yindextemp==yindex)))//&&(fabs(xtemp)<10e-6)
			break;
	}while(1);
}


//////////////////////////////////////
// 功能：创建每个扫描行的投影面(DouglasPeucker算法)
// 输入：
//		double tolerance：	容差(按照像素的百分比)
// 返回值：
//		void:
//////////////////////////////////////
void GeoModelLine::CreateProjectPlane(double tolerance)
{
	// 初始化存储变量
	if(m_Plane!=NULL)
	{
		for(int i=0; i<m_CCDnum; i++)
		{
			delete [](m_Plane[i]);
			m_Plane[i] = NULL;
		}
		m_Plane = NULL;
		m_CCDnum = 0;
	}
	if(m_dppoint!=NULL)	delete []m_dppoint;	m_dppoint = NULL;
	m_dppoint = new StrDP[m_ynum];
	double phi[3];
	for(long i=0; i<m_ynum; i++)
	{
		m_dppoint[i].x = i+1;
		m_cam->GetInner(0, i, phi[0], phi[1]);
		m_dppoint[i].y = phi[0];
		m_dppoint[i].flag = false;
	}
	m_cam->GetInner(0, m_xnum/2, phi[0], phi[1]);
	m_cam->GetInner(0, m_xnum/2+1, phi[0], phi[2]);
	tolerance *= fabs(phi[1]-phi[2]);
	// 初始化算法变量
	m_CCDnum = 1;
	double distance = 0;
	int nStart = 0;							//起点
	int nEnd = m_ynum-1;					//终点
	m_dppoint[nStart].flag = true;			//首尾两个点原理上需要保留
	m_dppoint[nEnd].flag = true;			//首尾两个点原理上需要保留
	int PtIndex = 0;						//高度最大点的索引
	bool flag = true;						//循环是否继续的标志
	StrStartEnd m_temp;						//中间变量
	vector<StrStartEnd> Stack;				//为了避免递归调用而设置的栈,保存起点和终点的状态
	////////////////////////////////
	// 化递归调用为循环,进行压缩
	////////////////////////////////
	do 
	{
		//获取最大距离
		distance = GetMaxHeight(m_dppoint, nStart, nEnd, PtIndex);
		//如果最大距离那个点高度大于容差,就将这个点保留
		if(distance>tolerance)
		{
			m_dppoint[PtIndex].flag = true;  //此点保留
			m_CCDnum++;
			//以此点作为分界线,进行类似二叉树的循环
			//为了防止递归计算,使用一个栈将中间结果保存
			m_temp.Start = PtIndex;
			m_temp.End = nEnd;
			Stack.push_back(m_temp);
			nEnd = PtIndex;
		}	
		else
		{
			if(Stack.empty())	flag = false;
			else
			{
				m_temp = Stack[Stack.size()-1];
				nStart = m_temp.Start;
				nEnd = m_temp.End;
				Stack.erase(Stack.end()-1);
			}
		}
	}while(flag);
	Stack.clear();
	printf("\nCCD分段数为：%d\n", m_CCDnum);
	////////////////////////////////
	// 开始进行CCD分片
	////////////////////////////////
	m_Plane = new StrPlane*[m_CCDnum];
	for(int i=0; i<m_CCDnum; i++)
	{
		m_Plane[i] = new StrPlane[m_xnum];
	}
	int m_CCDnumTemp = 0;
	m_StartEnd.clear();
	m_temp.Start = 0;
	m_dppoint[0].index = 0;
	for(int i=1; i<m_ynum; i++)
	{
		m_dppoint[i].index = m_CCDnumTemp;
		if(m_dppoint[i].flag==true)
		{
			m_CCDnumTemp++;
			m_temp.End = i;
			m_StartEnd.push_back(m_temp);
			m_temp.Start = i;
		}
	}
	////////////////////////////////
	// 开始求取各片CCD平面参数
	////////////////////////////////
	StrOrbitPoint point[3], pointtemp;
	double lat, lon, h, UT, temp1;
	h = 0;
	// 沿轨向循环
	for(int i=0; i<m_xnum; i++)
	{
		// 第一个点
		UT = m_time->get_time(i);
		if(m_input.isOrbitPoly == false)	point[0] = m_orbit->GetEpWGS84(UT);
		else								point[0] = m_orbit->PolyValue(UT);
		// 第二个点
		FromXY2LatLon(i, m_StartEnd[0].Start, h, lat, lon);
		m_base.Geograph2Rect(m_datum, lat, lon, h, point[1].X[0], point[1].X[1], point[1].X[2]);
		// 用于判断平面正负性的点
		UT = m_time->get_time(i+10);	// +10？
		if(m_input.isOrbitPoly == false)	pointtemp = m_orbit->GetEpWGS84(UT);
		else								pointtemp = m_orbit->PolyValue(UT);
		// CCD片循环
		for(int j=0; j<m_CCDnum; j++)
		{
			// 第三个点
			FromXY2LatLon(i, m_StartEnd[j].End, h, lat, lon);
			m_base.Geograph2Rect(m_datum, lat, lon, h, point[2].X[0], point[2].X[1], point[2].X[2]);
			GetPlanePara(point, m_Plane[j][i]);
			// 判断正负性
			temp1 = m_Plane[j][i].para[0]*pointtemp.X[0] + m_Plane[j][i].para[1]*pointtemp.X[1] + 
					m_Plane[j][i].para[2]*pointtemp.X[2] - 1;
			if(temp1>0)
			   m_Plane[j][i].isPos = 1;
			else m_Plane[j][i].isPos = -1;
			// 将第三个点移到第二个点
			memcpy(point[1].X, point[2].X, sizeof(double)*3);
		}
	}
	////////////////////////////////
	// 开始求取相邻两行间的面距离
	////////////////////////////////
	m_ccdDis = fabs(GetPoint2PlaneDis(m_Plane[m_CCDnum/2][0], point[0]))/(m_xnum-1);	// 记得减一以及加上绝对值
}


//////////////////////////////////////
// 功能：根据空间三个点求取空间平面参数
//		 假设空间平面方程为Ax+By+Cz-1=0
// 输入：
//		StrOrbitPoint *point:	三个空间点坐标
//		StrPlane &para：		存储的平面描述参数
// 返回值：
//		void
//////////////////////////////////////  
void GeoModelLine::GetPlanePara(StrOrbitPoint *point, StrPlane &para)
{
	double ATA[9], ATL[3], A[3], L;
	memset(ATA, 0, sizeof(double)*9);	memset(ATL, 0, sizeof(double)*3);
	for(long i=0; i<3; i++)
	{
		A[0] = point[i].X[0];	A[1] = point[i].X[1];	
		A[2] = point[i].X[2];	L = 1;
		m_base.pNormal(A, 3, L, ATA, ATL, 1.0);
	}
	m_base.solve33(ATA, ATL);
	memcpy(para.para, ATL, sizeof(double)*3);
}


//////////////////////////////////////
// 功能：求取空间点到空间面的距离
//		 假设空间平面方程为Ax+By+Cz-1=0
//		 空间点为(x0,y0,z0),则点到此平面的距离为
//		 |Ax0+By0+Cz0-1|/sqrt(A*A+B*B+C*C)
// 输入：
//		StrPlane para：			平面描述参数
//		StrOrbitPoint point:	空间点坐标
// 返回值：
//		double:					返回高度
////////////////////////////////////// 
double GeoModelLine::GetPoint2PlaneDis(StrPlane para, StrOrbitPoint point)
{
	return (para.para[0]*point.X[0] + para.para[1]*point.X[1] + para.para[2]*point.X[2] - 1)/
		sqrt(pow(para.para[0], 2) + pow(para.para[1], 2) + pow(para.para[2], 2));
}


//////////////////////////////////////
// 功能：获得DouglasPeucker最大高度
// 输入：
//		StrDP *pt：		用于压缩的指针
//		int nStart:		压缩的起始位置
//		int nEnd:		压缩的结束位置
// 输入输出：
//		int &PtIndex：	保留点的索引号
// 返回值：
//		double:			返回的最大高度
//////////////////////////////////////
double GeoModelLine::GetMaxHeight(StrDP *pt, int nStart, int nEnd, int &PtIndex)
{
	double maxLength = 0;
	double distance = 0;
	for(int i=nStart+1;i<nEnd;i++)
	{
		distance = GetDistance(pt[i], pt[nStart], pt[nEnd]);
	    if(distance>maxLength)
		{
			maxLength=distance;
			PtIndex=i;
		}
	}
	return maxLength;
}


//////////////////////////////////////
// 功能：根据三点坐标求取高度
// 输入：
//		StrDP pt：		高的顶点
//		StrDP pt1：		底边的第一个点
//		StrDP pt2：		底边的第二个点
// 返回值：
//		double：		
//////////////////////////////////////
double GeoModelLine::GetDistance(StrDP pt, StrDP pt1, StrDP pt2)
{
	//求取三角形的面积*2
	double area=fabs((pt1.x*pt2.y + pt2.x*pt.y + pt.x*pt1.y 
					- pt2.x*pt1.y - pt.x*pt2.y - pt1.x*pt.y));
	//求取三角形底边的长度
	double bottom=sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
	//求取高度
	return (area/bottom);
}


