#include "GeoModelLine.h"


//////////////////////////////////////////////////////////////////////////
// ����/��������
//////////////////////////////////////////////////////////////////////////
GeoModelLine::GeoModelLine(void)
{
	m_timeUT = NULL;			// �洢UTʱ��,����ɨ�������洢
	m_wgs84 = NULL;				// GPS��λ������wgs84��λ��,����ɨ�������洢
	m_body2wgs84 = NULL;		// ���嵽wgs84����̬,����ɨ�������洢
	m_dppoint = NULL;			// DouglasPeucker�洢����
	m_Plane = NULL;				// �淽�̵ı���
	m_CCDnum = 0;				// ������ͨ��ѹ����ķ�ƬCCD����
}


GeoModelLine::~GeoModelLine(void)
{
	//Destroy();
}


//////////////////////////////////////
// ��ʼ������
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
// �����˹����ʽģ���Լ�����λ���˹��Լ������İ�װ
//////////////////////////////////////
void GeoModelLine::ReCalPosAndAtt()
{
	// ���֮ǰ���ڴ�
	Destroy();
	// ����ռ�
	m_timeUT = new double[m_xnum];
	m_wgs84 = new struct StrOrbitPoint[m_xnum];
	m_body2wgs84 = new struct StrAttPoint[m_xnum];
	// ���ʱ��
	for(int i=0;i<m_xnum;i++)
	{
		m_timeUT[i] = m_time->get_time(i);
	}
	// ���ж���ʽ���(���,��̬)
	m_orbit->GenPolyModel(m_timeUT[0]-m_input.timeExtend, m_timeUT[m_xnum-1]+m_input.timeExtend);
	m_att->GenPolyModel(m_timeUT[0]-m_input.timeExtend, m_timeUT[m_xnum-1]+m_input.timeExtend);
	// GPS�ڱ�������ϵ�µ�ƫ�ľ�(����-����)
	m_GPSSet.X[0] = m_orbit->get_OffX();
	m_GPSSet.X[1] = m_orbit->get_OffY();
	m_GPSSet.X[2] = m_orbit->get_OffZ();
	// ����ڱ�������ϵ�µ�ƫ�ľ�
	m_camSet.X[0] = m_cam->get_OffX();
	m_camSet.X[1] = m_cam->get_OffY();
	m_camSet.X[2] = m_cam->get_OffZ();
	// ����������ϵ�Ƶ�GPSϵ��ƫ�ľ�(��������ϵ��)
	m_cam2GPSSet.X[0] = -m_GPSSet.X[0] + m_camSet.X[0];
	m_cam2GPSSet.X[1] = -m_GPSSet.X[1] + m_camSet.X[1];
	m_cam2GPSSet.X[2] = -m_GPSSet.X[2] + m_camSet.X[2];
	// �Ӳ�������ϵ����������ϵ����ת����
	//m_att->get_ROff(m_body2att.R);
	//m_base.Transpose(m_body2att.R, 3, 3);
	// ��������������ת����
	m_cam->get_ROff(m_cam2body.R);
	//m_base.Transpose(m_cam2body.R,3,3);
	// ��ʵ��̬�͹��
	for(int i=0;i<m_xnum;i++)
	{
		// WGS4��λ��
		if(m_input.isOrbitPoly == false)	m_wgs84[i] = m_orbit->GetEpWGS84(m_timeUT[i]);
		else								m_wgs84[i] = m_orbit->PolyValue(m_timeUT[i]);
		// ���嵽wgs84��ת����
		if(m_input.isAttPoly == false)		m_body2wgs84[i] = m_att->GetAttBody2WGS84(m_timeUT[i]);		
		else								m_att->PolyValue(m_timeUT[i], m_body2wgs84[i].R);
	}
}


//////////////////////////////////////
// ���ܣ���ȡȫ�ֱ任�����Լ���γ��/����
// ����:
//		��
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModelLine::ComputerGlobalParam()
{
	double H = 0;
	// ��ȡ�ĸ��ǵ��Լ����ĵ�
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
	// ��ȡһ���ش�С�ڵ����ͶӰ,��һ������ϵ��:��γ��/����
	double lat1, lat2, lon1, lon2;
	double dx, dy;
	dx = dy = 10;
	FromXY2LatLon(m_xnum/2.0, m_ynum/2.0, H, lat1, lon1);
	FromXY2LatLon(m_xnum/2.0+dx, m_ynum/2.0+dy, H, lat2, lon2);	
	m_pixelsize = sqrt(pow((lat1-lat2),2)+pow((lon1-lon2),2)/(dx*dx+dy*dy));
	m_pixelsize2 = pow(m_pixelsize, 2);
}


//////////////////////////////////////
// ���ܣ��ֲ�����任Ԥ��
// �������:
//		double &x��	���뼰��������x����
//		double &y��	���뼰��������y����
// ���룺
//		double lat:	������﷽��γ��(��λ:����)
//		double lon��������﷽�㾭��(��λ:����)
//		double H��	������﷽��߳�(��λ:��)
//		double dx��	������񷽵�xƫ��ֵ
//		double dy��	������񷽵�yƫ��ֵ
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModelLine::PreciseBasedAffine(double &x,double &y,double lat,double lon,double H,double dx,double dy)
{
	// ��ȡ�ĸ��ǵ��Լ����ĵ�
	double xtemp[4], ytemp[4], lattemp[4], lontemp[4];
	xtemp[0] = x+dx;   ytemp[0] = y+dy;
	xtemp[1] = x+dx;   ytemp[1] = y-dy;
	xtemp[2] = x-dx;   ytemp[2] = y+dy;
	xtemp[3] = x-dx;   ytemp[3] = y-dy;
	for(int i=0;i<4;i++)
	{
		FromXY2LatLon(xtemp[i], ytemp[i], H, lattemp[i], lontemp[i]);
	}
	// ���з���任
	GeoTranslation m_transtemp;
	m_transtemp.CalAffineParam(lattemp, lontemp, xtemp, ytemp, 4);
	// ��ʼԤ��
	m_transtemp.GetValueBaseAffine(lat, lon, x, y);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�ǰ���������
//���룺����Ӱ��ģ�ͣ�ƥ����Ƶ�
//����������ռ�����XYZ
//���ߣ�GZC
//ʱ�䣺2017.09.08
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
// ���ܣ�����ģ�ͳ�ʼ������
// ����:
//		GeoOrbit *orbit:	�������
//		GeoAttitude *att:	��̬����
//		GeoTime *time:		ʱ�����
//		GeoCamera *cam��	�������
//		StrModelParamInput input��	����ģ���������
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModelLine::InitModelLine(GeoOrbit *orbit, GeoAttitude *att, GeoTime *time, GeoCamera *cam, StrModelParamInput input)
{
	m_orbit = orbit;
	m_att = att;
	m_time = time;
	m_cam = cam;
	m_input = input;
	// ��ȡģ��Ӱ�񳤶ȺͿ��
	m_xnum = m_time->get_num();
	m_ynum = m_cam->get_ynum();
	// ��ȡģ������λ�Ļ�׼�������
	m_datum = m_orbit->get_datum();
	// �����˹����ʽģ���Լ�����λ���˹��Լ������İ�װ
	ReCalPosAndAtt();
	// ��ȡȫ�ֱ任�����Լ���γ��/����
	ComputerGlobalParam();
	// DouglasPeucker������CCD���з�Ƭ
//	CreateProjectPlane();
}

/////////////////////////////////////////////
//���ܣ���ȡ���У����
//���룺������·��fpath
//�����m_cam2body.R
//ע�⣺���fpathû����д����Ĭ�ϲ��ı�m_cam2body.R
//���ڣ�2016.12.22
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
// ���ܣ���Ӱ�񵽵���(��ѧ����)
// ����:
//		double x:		�ع�������غ�
//		double y:		����������غ�
//		double &H:		��λ��ĸ߳�(��λ:m),��Ҳ��Ϊ���
// �����
//		double &lat��	��λ���γ��(��λ:����)
//		double &lon��	��λ��ľ���(��λ:����)	
// ����ֵ��
//		void
//////////////////////////////////////
bool GeoModelLine::FromXY2LatLon(double x, double y, double &H, double &lat, double &lon)
{
	//�洢��Ӧʱ�̵��˹���Ϣ
	double UT;
	struct StrOrbitPoint m_ephpoint;
	struct StrAttPoint m_attbody2wgs84;
	
	// ������Ѿ��洢�ĵ�����,ֱ��ʹ��������ٶ�
	if((fabs(x-(long)(x+0.5))<10e-10)&&(x>=0)&&(x<m_xnum))
	{
		UT = m_timeUT[(long)(x+0.5)];
		m_ephpoint = m_wgs84[(long)(x+0.5)];
		m_attbody2wgs84 = m_body2wgs84[(long)(x+0.5)];
	}
	else
	{
		// ��ö�Ӧ��ʱ��
		UT = m_time->get_time(x);
		// ��ô�ʱ���Ӧ�Ĺ����Ϣ
		if(m_input.isOrbitPoly == false)	m_ephpoint = m_orbit->GetEpWGS84(UT);
		else								m_ephpoint = m_orbit->PolyValue(UT);
		// ��ô�ʱ���Ӧ����̬��Ϣ,���嵽WGS84
		if(m_input.isAttPoly == false)		m_attbody2wgs84 = m_att->GetAttBody2WGS84(UT);
		else								m_att->PolyValue(UT, m_attbody2wgs84.R);
	}
	//////////////////////////////////////
	// ��ʼ��͵���Ľ���
	//////////////////////////////////////
	double A, B, C, scale, B2_4AC, a, b, a2, b2;
	a = m_datum.a+H;		a2 = a*a;
	b = m_datum.b+H;		b2 = b*b;
	// GPS��λ������WGS84ϵ�µ�λ��
	double EphPos[3];
	memcpy(EphPos, m_ephpoint.X, sizeof(double)*3);
	// ��ôӱ��嵽WGS84����ת����
	double m_body2WGS84[9];
	m_base.Multi(m_attbody2wgs84.R, m_body2att.R, m_body2WGS84, 3, 3, 3);
	// �����WGS84ϵ�µ�λ��
	double CamPos[3], temp1[3];
	m_base.Multi(m_body2WGS84, m_cam2GPSSet.X, temp1, 3, 3, 1);
	CamPos[0] = EphPos[0] + temp1[0];
	CamPos[1] = EphPos[1] + temp1[1];
	CamPos[2] = EphPos[2] + temp1[2];
	// ��ȡ̽Ԫָ���
	double phi[3];
	m_cam->GetInner(x, y, phi[0], phi[1]);
	phi[2] = 1;
	// ���̽Ԫָ�����wgs84ϵ�µ�ָ��,Ϊ�˼��پ�����ʧ,�Ŵ�a��
	double temp3[9], Pos[3], m_cam2WGS84[9];
	m_base.Multi(m_body2WGS84, m_cam2body.R, m_cam2WGS84, 3, 3, 3);
	m_base.Multi(m_cam2WGS84, phi, Pos, 3, 3, 1);
	Pos[0] *= a;   Pos[1] *= a;   Pos[2] *= a;
	A = (pow(Pos[0],2)+pow(Pos[1],2))/a2+pow(Pos[2],2)/b2;
	B = 2*((Pos[0]*CamPos[0]+Pos[1]*CamPos[1])/a2+Pos[2]*CamPos[2]/b2);
	C = (pow(CamPos[0],2) + pow(CamPos[1],2))/a2 + pow(CamPos[2],2)/b2-1;
	B2_4AC = B*B - 4*A*C;
	// ���������ཻ,�������ʾ��Ϣ
	if(B2_4AC<0 || A==0)
	{
		printf("%lf-%lf	���������ཻ:B2_4AC<0 || A==0\n", x, y);
		return false;
	}
	scale = (-B-sqrt(B2_4AC))/(2*A);
	if(scale<0)
	{
		printf("%lf-%lf	���������ཻ:scale<0\n", x, y);
		return false;
	}
	// ��������
	struct StrOrbitPoint m_Surface;
	m_Surface.X[0] = CamPos[0] + scale*Pos[0];
	m_Surface.X[1] = CamPos[1] + scale*Pos[1];
	m_Surface.X[2] = CamPos[2] + scale*Pos[2];
	double Htemp;
	m_base.Rect2Geograph(m_datum, m_Surface.X[0], m_Surface.X[1], m_Surface.X[2], lat, lon, Htemp);
	return true;
}


//////////////////////////////////////
// ���ܣ�����xy��������õ������̬���ڷ�λԪ��
// ���룺
//		double x:		�ع�������غ�
//		double y:		����������غ�
// �����
//		double *eph��	�õ��Ĺ��
//		double *R��		�õ�����̬
//		double *inner:	�õ����ڷ�λԪ��
//////////////////////////////////////
void GeoModelLine::GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner)
{
	//�洢��Ӧʱ�̵��˹���Ϣ
	double UT;
	struct StrOrbitPoint m_ephpoint;
	struct StrAttPoint m_attbody2wgs84;	
	// ������Ѿ��洢�ĵ�����,ֱ��ʹ��������ٶ�
	if((fabs(x-(long)(x+0.5))<10e-10)&&(x>=0)&&(x<m_xnum))
	{
		UT = m_timeUT[(long)(x+0.5)];
		m_ephpoint = m_wgs84[(long)(x+0.5)];
		m_attbody2wgs84 = m_body2wgs84[(long)(x+0.5)];
	}
	else
	{
		// ��ö�Ӧ��ʱ��
		UT = m_time->get_time(x);
		// ��ô�ʱ���Ӧ�Ĺ����Ϣ
		if(m_input.isOrbitPoly == false)	m_ephpoint = m_orbit->GetEpWGS84(UT);
		else								m_ephpoint = m_orbit->PolyValue(UT);
		// ��ô�ʱ���Ӧ����̬��Ϣ,���嵽WGS84
		if(m_input.isAttPoly == false)		m_attbody2wgs84 = m_att->GetAttBody2WGS84(UT);
		else								m_att->PolyValue(UT, m_attbody2wgs84.R);
	}
	// ��ôӱ��嵽WGS84����ת���󣬳���һ��ƫ��
	m_base.Multi(m_attbody2wgs84.R, m_body2att.R, R, 3, 3, 3);
	// �����WGS84ϵ�µ�λ��
	double temp1[3];
	m_base.Multi(R, m_cam2GPSSet.X, temp1, 3, 3, 1);
	eph[0] = m_ephpoint.X[0] + temp1[0];
	eph[1] = m_ephpoint.X[1] + temp1[1];
	eph[2] = m_ephpoint.X[2] + temp1[2];
	// ��ȡ̽Ԫָ���
	m_cam->GetInner(x, y, inner[0], inner[1]);
	inner[2] = 1;
}


//////////////////////////////////////
// ���ܣ��ӵ��浽Ӱ��(��ѧ����)
// ����:
//		double lat��	��λ���γ��(��λ:����)
//		double lon��	��λ��ľ���(��λ:����)	
//		double H:		��λ��ĸ߳�(��λ:m)
// �����
//		double &x:		�ع�������غ�
//		double &y:		����������غ�
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModelLine::FromLatLon2XY(double lat, double lon, double H, double &x, double &y)
{
	// ���ڷ���Լ��
	FromLatLon2XYBaseAffine(lat, lon, H, x, y);
	// ������Լ��
//	FromLatLon2XYBaseImageConst(lat, lon, H, x, y);
	// �����﷽Լ��
//	FromLatLon2XYBaseObjectConst(lat, lon, H, x, y);
}


//////////////////////////////////////
// ���ܣ��ӵ��浽Ӱ��(��ѧ����,���ڷ���)
// ����:
//		double lat��	��λ���γ��(��λ:����)
//		double lon��	��λ��ľ���(��λ:����)	
//		double H:		��λ��ĸ߳�(��λ:m)
// �����
//		double &x:		�ع�������غ�
//		double &y:		����������غ�
// ����ֵ��
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
// ���ܣ��ӵ��浽Ӱ��(��ѧ����,������Լ��)
// ����:
//		double lat��	��λ���γ��(��λ:����)
//		double lon��	��λ��ľ���(��λ:����)	
//		double H:		��λ��ĸ߳�(��λ:m)
// �����
//		double &x:		�ع�������غ�
//		double &y:		����������غ�
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModelLine::FromLatLon2XYBaseImageConst(double lat, double lon, double H, double &x, double &y)
{

}


//////////////////////////////////////
// ���ܣ��ӵ��浽Ӱ��(��ѧ����,�����﷽Լ��)
// ����:
//		double lat��	��λ���γ��(��λ:����)
//		double lon��	��λ��ľ���(��λ:����)	
//		double H:		��λ��ĸ߳�(��λ:m)
// �����
//		double &x:		�ع�������غ�
//		double &y:		����������غ�
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModelLine::FromLatLon2XYBaseObjectConst(double lat, double lon, double H, double &x, double &y)
{
	double UT, temp1[3], eph[3], eph1[3], R[9];
	struct StrOrbitPoint m_ephpoint;
	struct StrAttPoint m_attbody2wgs84;	
	struct StrOrbitPoint m_Surface;
	m_base.Geograph2Rect(m_datum, lat, lon, H, m_Surface.X[0], m_Surface.X[1], m_Surface.X[2]);
	// ���÷���Ԥ����λ��,Ԥ���x��Ԥ���к�,Ԥ���y���Եõ���Ӧ����Ƭ��
	m_trans.GetValueBaseAffine(lat, lon, x, y);
	int xindex, yindex, yindextemp, count = 0;
	double dx, dxtemp[2], xtemp, ytemp;
	long indexx[2];
	double innerleft[2], innerright[2];
	// Խ���ж�
	if(int(y+0.5)<0)				yindextemp = 0;
	else if(int(y+0.5)>m_ynum-1)	yindextemp = m_CCDnum-1;
	else							yindextemp = m_dppoint[int(y+0.5)].index;
	if(int(x+0.5)<0)				xindex = 0;
	if(int(x+0.5)>m_xnum-1)			xindex = m_xnum-1;
	else							xindex = int(x+0.5);
	/////////////////////////////////////////////////////////
	// ����������̫����߷�ͶӰ������Ƭ�źͳ�ʼ����Ƭ����ͬʱ
	// �˳�ѭ��
	/////////////////////////////////////////////////////////
	do
	{
		yindex = yindextemp;
		// ���ƫ������,������ƫ��
		do
		{
			// Խ���ж�,��ֹԽ��
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
		// �ڲ��׼ȷ��
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
		x = indexx[0] + dxtemp[0]/(dxtemp[0]+dxtemp[1]);	// ������ڲ���ع�������
		//////////////////////////////////////////
		// ���з�ͶӰ����
		//////////////////////////////////////////
		if((fabs(x-(long)(x+0.5))<10e-10)&&(x>=0)&&(x<m_xnum))
		{
			UT = m_timeUT[(long)(x+0.5)];
			m_ephpoint = m_wgs84[(long)(x+0.5)];
			m_attbody2wgs84 = m_body2wgs84[(long)(x+0.5)];
		}
		else
		{
			// ��ö�Ӧ��ʱ��
			UT = m_time->get_time(x);
			// ��ô�ʱ���Ӧ�Ĺ����Ϣ
			if(m_input.isOrbitPoly == false)	m_ephpoint = m_orbit->GetEpWGS84(UT);
			else								m_ephpoint = m_orbit->PolyValue(UT);
			// ��ô�ʱ���Ӧ����̬��Ϣ,���嵽WGS84
			if(m_input.isAttPoly == false)		m_attbody2wgs84 = m_att->GetAttBody2WGS84(UT);
			else								m_att->PolyValue(UT, m_attbody2wgs84.R);
		}
		// ��ôӱ��嵽WGS84����ת����
		m_base.Multi(m_attbody2wgs84.R, m_body2att.R, R, 3, 3, 3);
		// �����WGS84ϵ�µ�λ��
		m_base.Multi(R, m_cam2GPSSet.X, temp1, 3, 3, 1);
		eph[0] = m_ephpoint.X[0] + temp1[0];
		eph[1] = m_ephpoint.X[1] + temp1[1];
		eph[2] = m_ephpoint.X[2] + temp1[2];
		// ��ȡʸ��
		eph[0] = m_Surface.X[0] - eph[0];
		eph[1] = m_Surface.X[1] - eph[1];
		eph[2] = m_Surface.X[2] - eph[2];
		// ������תʸ��
		m_base.Transpose(R, 3, 3);
		m_base.Multi(R, eph, eph1, 3, 3, 1);
		xtemp = eph1[0]/eph1[2];
		ytemp = eph1[1]/eph1[2];
		m_cam->GetInner(x, m_StartEnd[yindex].Start, innerleft[0], innerleft[1]);
		m_cam->GetInner(x, m_StartEnd[yindex].End, innerright[0], innerright[1]);
//		y = m_StartEnd[yindex].Start + (ytemp-innerleft[1])*(m_StartEnd[yindex].End-
//			m_StartEnd[yindex].Start)/(innerright[1]-innerleft[1]);
		m_cam->GetIndexBaseInner(ytemp, 0.0, y, x);

		// �ж�Խ�����
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
// ���ܣ�����ÿ��ɨ���е�ͶӰ��(DouglasPeucker�㷨)
// ���룺
//		double tolerance��	�ݲ�(�������صİٷֱ�)
// ����ֵ��
//		void:
//////////////////////////////////////
void GeoModelLine::CreateProjectPlane(double tolerance)
{
	// ��ʼ���洢����
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
	// ��ʼ���㷨����
	m_CCDnum = 1;
	double distance = 0;
	int nStart = 0;							//���
	int nEnd = m_ynum-1;					//�յ�
	m_dppoint[nStart].flag = true;			//��β������ԭ������Ҫ����
	m_dppoint[nEnd].flag = true;			//��β������ԭ������Ҫ����
	int PtIndex = 0;						//�߶����������
	bool flag = true;						//ѭ���Ƿ�����ı�־
	StrStartEnd m_temp;						//�м����
	vector<StrStartEnd> Stack;				//Ϊ�˱���ݹ���ö����õ�ջ,���������յ��״̬
	////////////////////////////////
	// ���ݹ����Ϊѭ��,����ѹ��
	////////////////////////////////
	do 
	{
		//��ȡ������
		distance = GetMaxHeight(m_dppoint, nStart, nEnd, PtIndex);
		//����������Ǹ���߶ȴ����ݲ�,�ͽ�����㱣��
		if(distance>tolerance)
		{
			m_dppoint[PtIndex].flag = true;  //�˵㱣��
			m_CCDnum++;
			//�Դ˵���Ϊ�ֽ���,�������ƶ�������ѭ��
			//Ϊ�˷�ֹ�ݹ����,ʹ��һ��ջ���м�������
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
	printf("\nCCD�ֶ���Ϊ��%d\n", m_CCDnum);
	////////////////////////////////
	// ��ʼ����CCD��Ƭ
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
	// ��ʼ��ȡ��ƬCCDƽ�����
	////////////////////////////////
	StrOrbitPoint point[3], pointtemp;
	double lat, lon, h, UT, temp1;
	h = 0;
	// �ع���ѭ��
	for(int i=0; i<m_xnum; i++)
	{
		// ��һ����
		UT = m_time->get_time(i);
		if(m_input.isOrbitPoly == false)	point[0] = m_orbit->GetEpWGS84(UT);
		else								point[0] = m_orbit->PolyValue(UT);
		// �ڶ�����
		FromXY2LatLon(i, m_StartEnd[0].Start, h, lat, lon);
		m_base.Geograph2Rect(m_datum, lat, lon, h, point[1].X[0], point[1].X[1], point[1].X[2]);
		// �����ж�ƽ�������Եĵ�
		UT = m_time->get_time(i+10);	// +10��
		if(m_input.isOrbitPoly == false)	pointtemp = m_orbit->GetEpWGS84(UT);
		else								pointtemp = m_orbit->PolyValue(UT);
		// CCDƬѭ��
		for(int j=0; j<m_CCDnum; j++)
		{
			// ��������
			FromXY2LatLon(i, m_StartEnd[j].End, h, lat, lon);
			m_base.Geograph2Rect(m_datum, lat, lon, h, point[2].X[0], point[2].X[1], point[2].X[2]);
			GetPlanePara(point, m_Plane[j][i]);
			// �ж�������
			temp1 = m_Plane[j][i].para[0]*pointtemp.X[0] + m_Plane[j][i].para[1]*pointtemp.X[1] + 
					m_Plane[j][i].para[2]*pointtemp.X[2] - 1;
			if(temp1>0)
			   m_Plane[j][i].isPos = 1;
			else m_Plane[j][i].isPos = -1;
			// �����������Ƶ��ڶ�����
			memcpy(point[1].X, point[2].X, sizeof(double)*3);
		}
	}
	////////////////////////////////
	// ��ʼ��ȡ�������м�������
	////////////////////////////////
	m_ccdDis = fabs(GetPoint2PlaneDis(m_Plane[m_CCDnum/2][0], point[0]))/(m_xnum-1);	// �ǵü�һ�Լ����Ͼ���ֵ
}


//////////////////////////////////////
// ���ܣ����ݿռ���������ȡ�ռ�ƽ�����
//		 ����ռ�ƽ�淽��ΪAx+By+Cz-1=0
// ���룺
//		StrOrbitPoint *point:	�����ռ������
//		StrPlane &para��		�洢��ƽ����������
// ����ֵ��
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
// ���ܣ���ȡ�ռ�㵽�ռ���ľ���
//		 ����ռ�ƽ�淽��ΪAx+By+Cz-1=0
//		 �ռ��Ϊ(x0,y0,z0),��㵽��ƽ��ľ���Ϊ
//		 |Ax0+By0+Cz0-1|/sqrt(A*A+B*B+C*C)
// ���룺
//		StrPlane para��			ƽ����������
//		StrOrbitPoint point:	�ռ������
// ����ֵ��
//		double:					���ظ߶�
////////////////////////////////////// 
double GeoModelLine::GetPoint2PlaneDis(StrPlane para, StrOrbitPoint point)
{
	return (para.para[0]*point.X[0] + para.para[1]*point.X[1] + para.para[2]*point.X[2] - 1)/
		sqrt(pow(para.para[0], 2) + pow(para.para[1], 2) + pow(para.para[2], 2));
}


//////////////////////////////////////
// ���ܣ����DouglasPeucker���߶�
// ���룺
//		StrDP *pt��		����ѹ����ָ��
//		int nStart:		ѹ������ʼλ��
//		int nEnd:		ѹ���Ľ���λ��
// ���������
//		int &PtIndex��	�������������
// ����ֵ��
//		double:			���ص����߶�
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
// ���ܣ���������������ȡ�߶�
// ���룺
//		StrDP pt��		�ߵĶ���
//		StrDP pt1��		�ױߵĵ�һ����
//		StrDP pt2��		�ױߵĵڶ�����
// ����ֵ��
//		double��		
//////////////////////////////////////
double GeoModelLine::GetDistance(StrDP pt, StrDP pt1, StrDP pt2)
{
	//��ȡ�����ε����*2
	double area=fabs((pt1.x*pt2.y + pt2.x*pt.y + pt.x*pt1.y 
					- pt2.x*pt1.y - pt.x*pt2.y - pt1.x*pt.y));
	//��ȡ�����εױߵĳ���
	double bottom=sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
	//��ȡ�߶�
	return (area/bottom);
}


