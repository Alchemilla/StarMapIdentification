#include "GeoOrbit.h"

//////////////////////////////////////////////////////////////////////////
// ����/��������
//////////////////////////////////////////////////////////////////////////
GeoOrbit::GeoOrbit(void)
{
	m_EpWGS84 = NULL;       // �洢WGS84ϵ����ʵ�������Ϣ
	m_EpJ2000 = NULL;       // �洢J2000ϵ����ʵ�������Ϣ
	ClearData();
}


GeoOrbit::~GeoOrbit(void)
{
	ClearData();
}


// �������
void GeoOrbit::ClearData()
{
	m_num  = 0;
	if(m_EpWGS84!=NULL) { delete []m_EpWGS84; m_EpWGS84 = NULL; }
	if(m_EpJ2000!=NULL) { delete []m_EpJ2000; m_EpJ2000 = NULL; }
	memset(&m_modelWGS84, 0, sizeof(StrOrbitPolyModel));
	memset(&m_modelJ2000, 0, sizeof(StrOrbitPolyModel));
}

// ��ȡ���õĲο��������
StrDATUM GeoOrbit::get_datum()	
{	
	StrDATUM datum;
	m_Base.GetRefEllipsoid(m_Input.DatumName, datum);
	return datum;	
};

// ��òο���Ԫ
double GeoOrbit::get_RefUT()	{	return m_Input.refMJD;	}					

// ��ȡ��������
long GeoOrbit::get_num()	{	return m_num;	}

// �����λ������Ա������ĵ�ƫ�ľ�X(����-����)
double GeoOrbit::get_OffX()	{	return m_Input.m_Off[0];	}

// �����λ������Ա������ĵ�ƫ�ľ�Y(����-����)
double GeoOrbit::get_OffY()	{	return m_Input.m_Off[1];	}

// �����λ������Ա������ĵ�ƫ�ľ�Z(����-����)
double GeoOrbit::get_OffZ()	{	return m_Input.m_Off[2];	}


//////////////////////////////////////
// ���ܣ���ȡ������ģ��
// ����ֵ��
//		StrOrbitPolyModel��		������ģ��
//////////////////////////////////////
StrOrbitPolyModel *GeoOrbit::GetPolyModel()
{
	return &m_modelWGS84;
}


//////////////////////////////////////
// ���ܣ����ù�����ģ��
// ���룺
//		StrOrbitPolyModel attModel��	������ģ��
// ����ֵ��
//		void��	
//////////////////////////////////////
void GeoOrbit::SetPolyModel(StrOrbitPolyModel orbModel)
{
	m_modelWGS84 = orbModel;
}


//////////////////////////////////////
// ���ܣ�����������ģ��ϵ��
// ���룺
//		StrOrbitPolyModel attModel��	������ģ��
// ����ֵ��
//		void��	
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
// �ڲ���
////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ���ȡָ��ʱ���WGS84�Ĺ����Ϣ
// ����:
//		double UT:		����ʼ��Ԫ��ʼ���ۼ���
// ����ֵ��
//		StrOrbitPoint��	�ڲ������WGS84�������Ϣ
//////////////////////////////////////
StrOrbitPoint GeoOrbit::GetEpWGS84(double UT)
{
	StrOrbitPoint pVal;
	m_Base.LagrangianInterpolation(m_EpWGS84, m_num, UT, pVal, m_Input.m_InterOrder);
	return pVal;
}


//////////////////////////////////////
// ���ܣ���ȡָ��ʱ���J2000�Ĺ����Ϣ
// ����:
//		double UT:		����ʼ��Ԫ��ʼ���ۼ���
// ����ֵ��
//		StrOrbitPoint��	�ڲ������J2000�������Ϣ
////////////////////////////////////// 
StrOrbitPoint GeoOrbit::GetEpJ2000(double UT)
{
	StrOrbitPoint pVal;
	m_Base.LagrangianInterpolation(m_EpJ2000, m_num, UT, pVal, m_Input.m_InterOrder);
	return pVal;
}


//////////////////////////////////////
// ����������WGS84ϵ�¹������Ϣ
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
// ����������J2000ϵ�¹������Ϣ
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
// ��ù���ķ������ʼʱ��
//////////////////////////////////////
double GeoOrbit::get_m_startTime()
{
	if(m_num>0)
		return m_EpWGS84[0].UT;
	else
		return 0.0;
}


//////////////////////////////////////
// ��ù���ķ���Ľ���ʱ��
//////////////////////////////////////
double GeoOrbit::get_m_endTime()
{
	if(m_num>0)
		return m_EpWGS84[m_num-1].UT;
	else
		return 0.0;
}


////////////////////////////////////////////////////////
// �������ϵ�任
////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ���ȡ��J2000���������ϵ����ת����(�������ϵ������J2000ϵ��)
// ����:
//		double UT:		����ʼ��Ԫ��ʼ���ۼ���
//		bool isJ2000:	�Ƿ������J2000����ϵ,���Ϊfalse,�����ж��嶼����WGS84
// �����
//		double *R:		��ת����3*3��������	
// ����ֵ��
//		void
////////////////////////////////////// 
void GeoOrbit::GetJ20002OrbitRotation(double UT, double *R, bool isJ2000)
{
	// ���UTʱ�̹������J2000ϵ�µ���Ϣ
	StrOrbitPoint m_Point;
	if(isJ2000)	m_Point = GetEpJ2000(UT);
	else		m_Point = GetEpWGS84(UT);
	// ��ù������ϵ��������J2000ϵ�µ�ָ��
	// �������ϵ��
    //	 0�㣺��ǰʱ���������������Ĺ��λ�ã�
    //	 X�᣺λ�ڹ�����ڣ���ֱ��������������ߣ�ָ������ǰ������Ϊ����
    //	 Z�᣺λ�ڹ�����ڣ�ָ����ķ���
    //	 Y�᣺��ֱ�ڹ�����ڣ�ָ�������ַ���ȷ����
	double X[3], Y[3], Z[3];
	X[0] = m_Point.X[3];    X[1] = m_Point.X[4];   X[2] = m_Point.X[5]; 
	Z[0] = -m_Point.X[0];   Z[1] = -m_Point.X[1];  Z[2] = -m_Point.X[2];
	m_Base.CrossMult(Z, X, Y);
	m_Base.CrossMult(Y, Z, X);
	// ��һ��
	m_Base.NormVector(X,3);
	m_Base.NormVector(Y,3);
	m_Base.NormVector(Z,3);
	// ������ת����
	R[0] = X[0];     R[1] = X[1];     R[2] = X[2];
	R[3] = Y[0];     R[4] = Y[1];     R[5] = Y[2];
	R[6] = Z[0];     R[7] = Z[1];     R[8] = Z[2];
}


//////////////////////////////////////
// ���ܣ���ȡ�ӹ������ϵ��J2000����ת����(�������ϵ������J2000ϵ��)
// ����:
//		double UT:		����ʼ��Ԫ��ʼ���ۼ���
//		bool isJ2000:	�Ƿ������J2000����ϵ,���Ϊfalse,�����ж��嶼����WGS84
// �����
//		double *R:		��ת����3*3��������	
// ����ֵ��
//		void
////////////////////////////////////// 
void GeoOrbit::GetOrbit2J2000Rotation(double UT, double *R, bool isJ2000)
{
	// ���UTʱ�̹������J2000ϵ�µ���Ϣ
	StrOrbitPoint m_Point;
	if(isJ2000)	m_Point = GetEpJ2000(UT);
	else		m_Point = GetEpWGS84(UT);
	// ��ù������ϵ��������J2000ϵ�µ�ָ��
	// �������ϵ��
    //	 0�㣺��ǰʱ���������������Ĺ��λ�ã�
    //	 X�᣺λ�ڹ�����ڣ���ֱ��������������ߣ�ָ������ǰ������Ϊ����
    //	 Z�᣺λ�ڹ�����ڣ�ָ����ķ���
    //	 Y�᣺��ֱ�ڹ�����ڣ�ָ�������ַ���ȷ����
	double X[3], Y[3], Z[3];
	X[0] = m_Point.X[3];    X[1] = m_Point.X[4];   X[2] = m_Point.X[5];
	Z[0] = -m_Point.X[0];    Z[1] = -m_Point.X[1];   Z[2] = -m_Point.X[2];
	m_Base.CrossMult(Z, X, Y);
	m_Base.CrossMult(Y, Z, X);
	// ��һ��
	m_Base.NormVector(X,3);
	m_Base.NormVector(Y,3);
	m_Base.NormVector(Z,3);
	// ������ת����
	R[0] = X[0];     R[1] = Y[0];     R[2] = Z[0];
	R[3] = X[1];     R[4] = Y[1];     R[5] = Z[1];
	R[6] = X[2];     R[7] = Y[2];     R[8] = Z[2];
}


////////////////////////////////////////////////////////
// �������ʽ���
////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ�����ʱ��λ�ö�Ӧ����Ķ���ʽģ��
// ����:
//		startUT��	��ʼʱ��
//		endUT��		����ʱ��
//		isJ2000:	���Ϊture,��J2000������
//					���Ϊfalse,��WGS84������
//					Ĭ��Ϊfalse
// ����ֵ��
//		void
////////////////////////////////////// 
void GeoOrbit::GenPolyModel(double startUT, double endUT, bool isJ2000)
{
	long order = m_Input.m_PolyOrder;
	StrOrbitPolyModel *model = NULL;
	if(isJ2000) model = &m_modelJ2000;
	else		model = &m_modelWGS84;
	order += 1;		// n�׶���ʽ������Ҫ�õ�n+1���۲�ֵ
	// ������ն���ʽ�ṹ��
	long i;
	memset(model, 0, sizeof(StrOrbitPolyModel));
	model->m_adjustOrder = m_Input.m_adjustOrder;
	long index_left, index_right, num;
	double *err[7], *T[7];
	for(int i=0; i<7; i++)	
	{	
		err[i] = NULL;	T[i] = NULL;	
	}
	// Ѱ�����ݵ�
	num = m_num;
	index_left = 0;
	for(i=0; i<num-1; i++)	
		if((startUT>=m_EpWGS84[i].UT) && (startUT<=m_EpWGS84[i+1].UT))	
		{	index_left = i;	break;	}
	index_right = num-1;
	for(i=num-1; i>=1; i--)	
		if((endUT>=m_EpWGS84[i-1].UT) && (endUT<=m_EpWGS84[i].UT))
		{	index_right = i;	break;	}
	// ��ʼ����ٵ�������ʽ���
	index_left -= order/2;		if(index_left<0) index_left=0;
	index_right += order/2;		if(index_right>num-1)	index_right = num-1;
	if(index_right-index_left < order)	model->PolyOrder = index_right-index_left-1;	// �۲�ֵ�����ڽ���,�������Ϊ�͹۲�ֵ��һ��
	else								model->PolyOrder = order;						// ������û�ָ����һ��
	num = index_right-index_left+1;
	for(i=0; i<7; i++)	
	{	
		err[i] = new double[num];	T[i] = new double[num];
	}
	// ��ֵ
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
	// �����ϲв���Ϣ
//	printf("%ld��������ϲв�(m,m/s):\n%16.6lf\t%16.6lf\t%16.6lf\n%16.6lf\t%16.6lf\t%16.6lf\n", num,
//		m_Base.FabsAndAve(err[0], num), m_Base.FabsAndAve(err[1], num), m_Base.FabsAndAve(err[2], num), 
//		m_Base.FabsAndAve(err[3], num), m_Base.FabsAndAve(err[4], num), m_Base.FabsAndAve(err[5], num));
	// �ͷ��ڴ�
	for(int i=0; i<7; i++)
	{
		delete []err[i];	err[i] = NULL;
		delete []T[i];		T[i] = NULL;
	}
	model = NULL;
}


//////////////////////////////////////
// ���ܣ��ṩʱ����ݶ���ʽģ�ͻ�ö�Ӧ�Ĺ������Ϣ
// ����:
//		startUT��	��ʼʱ��
//		isJ2000:	���Ϊture,��J2000�������
//					���Ϊfalse,��WGS84�������
//					Ĭ��Ϊfalse
// ����ֵ��
//		StrOrbitPoint�����صĹ����
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
// ���ܣ��������ʱ������
// ����:
//		
// ����ֵ��
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
// ���¼��������������й�ϵ,������������ǲ�ͬ��,��Ҫ�ֱ�ʵ��,����ע�ⲻҪ�ô��麯��,�����޷�ʵ����
////////////////////////////////////////////////////////
// ��ȡ����ļ�
void GeoOrbit::ReadEphFile(string filepath, StrOrbitParamInput input){}
// ��ȡ���
void GeoOrbit::ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input){}
// ��ȡZY3���
void GeoOrbit::ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input){}		
// д������ļ�
void GeoOrbit::WriteEphFile(string filepath, bool iserror){}
// ����ָ��ʱ���ȡ��J2000��WGS84����ת����
void GeoOrbit::GetJ20002WGS84Rotation(double UT, double *R){}
// ����ָ��ʱ���ȡ��WGS84��J2000����ת����
void GeoOrbit::GetWGS842J2000Rotation(double UT, double *R){}
// ����ָ��ʱ�佫λ���ٶȴ�J2000ת����WGS84
void GeoOrbit::State_J20002WGS84(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz){}
// ����ָ��ʱ�佫λ���ٶȴ�WGS84ת����J2000
void GeoOrbit::State_WGS842J2000(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz){}

