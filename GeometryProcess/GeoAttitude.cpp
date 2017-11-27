#include "GeoAttitude.h"

//////////////////////////////////////////////////////////////////////////
// ����/��������
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

// �������
void GeoAttitude::ClearData()
{
	m_num  = 0;
	if(m_Body2WGS84!=NULL) { delete []m_Body2WGS84; m_Body2WGS84 = NULL; }
	if(m_Body2Orbit!=NULL) { delete []m_Body2Orbit; m_Body2Orbit = NULL; }
	memset(&m_modelWGS84, 0, sizeof(StrAttPolyModel));
}


// ��ȡ��������
long GeoAttitude::get_num()
{
	return m_num;
}

// ��ȡ�Ӳ�������ϵ����������ϵ����ת����
void GeoAttitude::get_ROff(double *R)
{
	memcpy(R, m_Input.ROff, sizeof(double)*9);
}

//���ôӲ�������ϵ����������ϵ����ת����
void GeoAttitude::set_ROff(double *R)
{
	memcpy(m_Input.ROff, R, sizeof(double) * 9);
}

//////////////////////////////////////
// ���ܣ���ȡ��̬���ģ��
// ����ֵ��
//		StrAttPolyModel��		������ģ��
//////////////////////////////////////
StrAttPolyModel *GeoAttitude::GetPolyModel()
{
	return &m_modelWGS84;
}


//////////////////////////////////////
// ���ܣ�������̬���ģ��
// ���룺
//		StrAttPolyModel attModel��	��̬���ģ��
// ����ֵ��
//		void��	
//////////////////////////////////////
void GeoAttitude::SetPolyModel(StrAttPolyModel attModel)
{
	m_modelWGS84 = attModel;
}


//////////////////////////////////////
// ���ܣ�������̬���ģ��ϵ��
// ���룺
//		StrOrbitPolyModel attModel��	������ģ��
// ����ֵ��
//		void��	
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
// ������������̬����Ϣ
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
// ������������̬��Ϣ
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
// �ڲ���̬
////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ���ȡָ��ʱ��Ĵӱ�������ϵ��WGS84����ϵ����ת������Ϣ
// ����:
//		double UT:		����ʼ��Ԫ��ʼ���ۼ���
// ����ֵ��
//		StrAttPoint��	�ڲ�����Ĵӱ�������ϵ��WGS84����ϵ����ת������Ϣ
//////////////////////////////////////
StrAttPoint GeoAttitude::GetAttBody2WGS84(double UT)
{
	StrAttPoint pVal;
	m_Base.QuatInterpolation(m_Body2WGS84, m_num, UT, pVal);
	m_Base.Quat2Matrix(pVal.q[0], pVal.q[1], pVal.q[2], pVal.q[3], pVal.R);
	return pVal;
}


//////////////////////////////////////
// ���ܣ���ȡָ��ʱ��Ĵӱ�������ϵ���������ϵ(J2000ϵ�¶���)����ת������Ϣ
// ����:
//		double UT:		����ʼ��Ԫ��ʼ���ۼ���
// ����ֵ��
//		StrAttPoint��	�ڲ�����Ĵӱ�������ϵ���������ϵ(J2000ϵ�¶���)����ת������Ϣ
////////////////////////////////////// 
StrAttPoint GeoAttitude::GetAttBody2Orbit(double UT)
{
	StrAttPoint pVal;
	m_Base.QuatInterpolation(m_Body2Orbit, m_num, UT, pVal);
	m_Base.Quat2Matrix(pVal.q[0], pVal.q[1], pVal.q[2], pVal.q[3], pVal.R);
	return pVal;
}



////////////////////////////////////////////////////////
// ��̬����ʽ���
////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ�����ʱ��λ�ö�Ӧ��̬�Ķ���ʽģ�ͣ�������Body2WGS84����
// ����:
//		startUT��	��ʼʱ��
//		endUT��		����ʱ��
// ����ֵ��
//		void
////////////////////////////////////// 
void GeoAttitude::GenPolyModel(double startUT, double endUT)
{
	long order = m_Input.m_PolyOrder;
	StrAttPolyModel *model = &m_modelWGS84;
	order += 1;		// n�׶���ʽ������Ҫ�õ�n+1���۲�ֵ
	// ������ն���ʽ�ṹ��
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
	// Ѱ�����ݵ�
	num = m_num;
	index_left = 0;
	for(i=0; i<num-1; i++)	
		if((startUT>=m_Body2WGS84[i].UT) && (startUT<=m_Body2WGS84[i+1].UT))	
		{	index_left = i;	break;	}
	index_right = num-1;
	for(i=num-1; i>=1; i--)	
		if((endUT>=m_Body2WGS84[i-1].UT) && (endUT<=m_Body2WGS84[i].UT))
		{	index_right = i;	break;	}
	// ��ʼ����ٵ�������ʽ���
	index_left -= order/2;		if(index_left<0) index_left=0;
	index_right += order/2;		if(index_right>num-1)	index_right = num-1;
	if(index_right-index_left < order)	model->PolyOrder = index_right-index_left-1;	// �۲�ֵ�����ڽ���,�������Ϊ�͹۲�ֵ��һ��
	else								model->PolyOrder = order;						// ������û�ָ����һ��
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
	// �����ϲв���Ϣ
//	printf("%ld������̬��ϲв�(��):\n%16.6lf\t%16.6lf\t%16.6lf\n", num, m_Base.FabsAndAve(err[0], num)*180/PI, 
//								  m_Base.FabsAndAve(err[1], num)*180/PI, m_Base.FabsAndAve(err[2], num)*180/PI);
	// �ͷ��ڴ�
	for(int i=0; i<4; i++)
	{
		delete []err[i];	err[i] = NULL;
		delete []T[i];		T[i] = NULL;
	}
	model = NULL;
}


//////////////////////////////////////
// ���ܣ�������̬����ʽ�����̬��������Body2WGS84����
// ����:
//		double startUT��	��ʼʱ��
// �����
//		double *R:			��ϵõ�����̬��ת����
// ����ֵ��
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
// ���ܣ���̬����ʱ������
// ����:
//		
// ����ֵ��
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
// ���¼��������������й�ϵ,������������ǲ�ͬ��,��Ҫ�ֱ�ʵ��,����ע�ⲻҪ�ô��麯��,�����޷�ʵ����
////////////////////////////////////////////////////////
// ��ȡ��̬�ļ�
void GeoAttitude::ReadAttFile(string filepath, StrAttParamInput input, GeoOrbit *orbit){}
// ��ȡ��̬
void GeoAttitude::ReadAttNoFile(int num, StrAttPoint *point, StrAttParamInput input, GeoOrbit *orbit){}
// ��ȡZY3��̬�ļ�
void GeoAttitude::ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit) {}
void GeoAttitude::ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath){}
void GeoAttitude::ReadZY3RealAttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath) {}
void GeoAttitude::upDateAtt(vector<Attitude> allAtt, StrAttParamInput input)
{
}
// д����̬�ļ�
void GeoAttitude::WriteAttFile(string filepath, bool iserror){}
// ����̬���и�Ƶ����
void GeoAttitude::ModifyAtt(string filepath){}






