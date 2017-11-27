#include "GeoModel.h"
#include "GeoModelRFM.h"


//////////////////////////////////////////////////////////////////////////
// ����/��������
//////////////////////////////////////////////////////////////////////////
GeoModel::GeoModel(void)
{
	m_orbit = NULL;		// ָ��������
	m_att = NULL;		// ָ����̬����
	m_time = NULL;		// ָ��ʱ�����
	m_cam = NULL;		// ָ���������
	//m_range = NULL;		// ָ�������
	delDis = 0;			// ��������
	// ƫ������
	Ru[0] = 1;	Ru[1] = 0;	Ru[2] = 0;
	Ru[3] = 0;	Ru[4] = 1;	Ru[5] = 0;
	Ru[6] = 0;	Ru[7] = 0;	Ru[8] = 1;
	// ��������
	// x = a[0] + a[1]*x + a[2]*y;
	// y = a[3] + a[4]*x + a[5]*y;
	m_affine[0] = 0.0;	m_affine[1] = 1.0;	m_affine[2] = 0.0;
	m_affine[3] = 0.0;	m_affine[4] = 0.0;	m_affine[5] = 1.0;
}

GeoModel::~GeoModel(void)
{
	//if(m_orbit!=NULL)	delete []m_orbit;	m_orbit = NULL;	// ָ��������
	//if(m_att!=NULL)		delete []m_att;		m_att = NULL;	// ָ����̬����
	//if(m_time!=NULL)	delete []m_time;	m_time = NULL;	// ָ��ʱ�����
	//if(m_cam!=NULL)		delete []m_cam;		m_cam = NULL;	// ָ���������
	//if(m_range!=NULL)	delete []m_range;	m_range = NULL;	// ָ�������
}


//////////////////////////////////////////////////////////////////////////
// ���º����ɸ�������ʵ��
//////////////////////////////////////////////////////////////////////////
// �����˹����ʽģ���Լ�����λ���˹��Լ������İ�װ
void GeoModel::ReCalPosAndAtt(){}
// ����ң�д�Ӱ�񵽵���
bool GeoModel::FromXY2LatLon(double x, double y, double &H, double &lat, double &lon) { return true; }
// ����ң�дӵ��浽Ӱ��
void GeoModel::FromLatLon2XY(double lat, double lon, double H, double &x, double &y){}
// ��ȡ���ű���
double GeoModel::GetScale(){ return 0; }
// ����xy��������õ������̬���ڷ�λԪ��
void GeoModel::GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner){}
// ���������������õ���Ӧ��ƫ����
void GeoModel::GetPartialDerivative(double lat, double lon, double H, double scale, double &x, double &y, 
				double &x_lat, double &y_lat, double &x_lon, double &y_lon, double &x_h, double &y_h){}
// ��ȡRFMģ�͵�ģ�Ͳ���
struct StrRFMData GeoModel::GetRFMModel(){ struct StrRFMData temp; return temp; }
// ��ȡȫ�ֱ任�����Լ���γ��/����
void GeoModel::ComputerGlobalParam(){};

// ���Ӱ��ĸ߶�
long GeoModel::get_m_height()
{
	return m_xnum;
}

// ���Ӱ��Ŀ��
long GeoModel::get_m_width()
{
	return m_ynum;
}

// ��ù������
GeoOrbit* GeoModel::GetOrbitObject()
{
	return m_orbit;
}

// �����̬����
GeoAttitude* GeoModel::GetAttitudeObject()
{
	return m_att;
}

// ���ʱ�����
GeoTime* GeoModel::GetTimeObject()
{
	return m_time;
}

// ����������
GeoCamera* GeoModel::GetCameraObject()
{
	return m_cam;
}

// ��ò�����
//GeoRange* GeoModel::GetRangeObject()
//{
//	return m_range;
//}


// ��ȡ������ģ��
StrOrbitPolyModel* GeoModel::GetOrbPolyModel()
{
	return m_orbit->GetPolyModel();
}


// ���ù�����ģ��
void GeoModel::SetOrbPolyModel(StrOrbitPolyModel orbModel)
{
	m_orbit->SetPolyModel(orbModel);
}

//////////////////////////////////////
// ���ܣ�����������ģ��ϵ��
// ���룺
//		StrOrbitPolyModel attModel��	������ģ��
// ����ֵ��
//		void��	
//////////////////////////////////////
void GeoModel::ModifyOrbitPolyModelPara(double *para)
{
	m_orbit->ModifyPolyModelPara(para);
}


// ��ȡ��̬���ģ��
StrAttPolyModel* GeoModel::GetAttPolyModel()
{
	return m_att->GetPolyModel();
}


// ������̬���ģ��
void GeoModel::SetAttPolyModel(StrAttPolyModel attModel)
{
	m_att->SetPolyModel(attModel);
}


//////////////////////////////////////
// ���ܣ�������̬���ģ��ϵ��
// ���룺
//		StrOrbitPolyModel attModel��	������ģ��
// ����ֵ��
//		void��	
//////////////////////////////////////
void GeoModel::ModifyAttPolyModelPara(double *para)
{
	m_att->ModifyPolyModelPara(para);
}


// ��ȡ����ģ�Ͳ���
double *GeoModel::GetModelAffinePara()
{
	return m_affine;
}


// ���÷���ģ�Ͳ���
void GeoModel::SetModelAffinePara(double *para)
{
	memcpy(m_affine, para, sizeof(double)*6);
}

//////////////////////////////////////////////////////////////////
//һ���ش�С�ڵ����ͶӰ,��һ������ϵ��:��γ��/����
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
// ���ܣ�����ģ�͵�ǰ������
// ����:
//		long num:			����ǰ���������Ƭ����Ŀ
//		GeoModel **model��	����ǰ���������Ƭ�ĳ���ģ��,��num��Ӧ
//		double *x:			����ǰ�������ͬ����x����(�ع�)
//		double *y:			����ǰ�������ͬ����y����(����)
// �����
//		double &X��			��λ���X����
//		double &Y��			��λ���Y����
//		double &Z:			��λ���Z����
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModel::RigorousForwardIntersection(long num, GeoModel **model, double *x, double *y, double &X, double &Y, double &Z)
{
	double eph[3], R[9], inner[3];
	// ѭ����ȡ
	double a[3], aa[9], al[3], L;
	memset(aa,0, sizeof(double)*9);		memset(al,0, sizeof(double)*3);
	for(long i=0; i<num; i++)
	{
		// ������������õ��˹��ڷ�λԪ����Ϣ
		model[i]->GetOrbitAttitudeInnerBaseXY(x[i], y[i], eph, R, inner);
		// ��һ������
		a[0] = R[0] - R[2]*inner[0];
		a[1] = R[3] - R[5]*inner[0];
		a[2] = R[6] - R[8]*inner[0];
		L = a[0]*eph[0] + a[1]*eph[1] + a[2]*eph[2];
		m_base.pNormal(a, 3, L, aa, al, 1.0);
		// �ڶ�������
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
// �����У
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ�����ƥ����м�������У
// ����:
//		long num:			�����У�ĵ���Ŀ
//		GeoModel *model��	�������ģ��
//		long *index:		������Ӧ��������
//		double *X��			��Ӧ�����X
//		double *Y:			��Ӧ�����Y
//		double *Z��			��Ӧ�����Z
// �����
//		double &deltaT��	��У�õ���ʱ������
//		double &deltaPhi��	��У�õ��ĸ���������
//		double &deltaOmega����У�õ��Ĺ���������
// ����ֵ��
//		void
//////////////////////////////////////
void GeoModel::TopoMatchForRangeCalibration3(long num, GeoModel *model, long *index, double *X, 
						double *Y, double *Z, double &deltaT, double &deltaPhi, double &deltaOmega)
{
	double eph[3], R[9], inner[3], XYZ[3], a3, b3, c3;
	double a[3], aa[9], al[3], L;
	// �������
	do
	{
		a3 = sin(deltaPhi);
		b3 = -sin(deltaOmega)*cos(deltaPhi);
		c3 = cos(deltaOmega)*cos(deltaPhi);
		// ѭ����ȡ
		memset(aa,0, sizeof(double)*9);		memset(al,0, sizeof(double)*3);
		for(long i=0; i<num; i++)
		{
			// ������������õ��˹��ڷ�λԪ����Ϣ
			model->GetOrbitAttitudeInnerBaseXY(index[i], 0, eph, R, inner);
			// ��ȡXYZ[3]
			m_base.Transpose(R, 3, 3);
			eph[0] = X[i] - eph[0];
			eph[1] = Y[i] - eph[1];
			eph[2] = Z[i] - eph[2];
			m_base.Multi(R, eph, XYZ, 3, 3, 1);
			// ��һ������
			a[0] = a3;
			a[1] = (inner[0]+deltaT)*cos(deltaPhi);
			a[2] = 0;
			L = XYZ[0]-(inner[0]+deltaT)*a3;
			m_base.pNormal(a, 3, L, aa, al, 1.0);
			// �ڶ�������
			a[0] = b3;
			a[1] = (inner[0]+deltaT)*sin(deltaOmega)*sin(deltaPhi);
			a[2] =-(inner[0]+deltaT)*cos(deltaOmega)*cos(deltaPhi);
			L = XYZ[1]-(inner[0]+deltaT)*b3;
			m_base.pNormal(a, 3, L, aa, al, 1.0);
			// ����������
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
// �����У��������
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ�����У��������
// ����:
//		double dis:		�������ֵ
//		double phi��	��������ֵ(����)
//		double omega:	��������ֵ(����)
// ����ֵ��
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

