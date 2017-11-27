#include "GeoCamera.h"

//////////////////////////////////////////////////////////////////////////
// ����/��������
//////////////////////////////////////////////////////////////////////////
GeoCamera::GeoCamera(void){}
GeoCamera::~GeoCamera(void){}

// ��ȡ�ع������ظ���
long GeoCamera::get_xnum()
{
	return m_Input.Xnum;
}

// ��ȡ���������ظ���
long GeoCamera::get_ynum()
{
	return m_Input.Ynum;
}

// ��������Ա���ƫ�ľ�X
double GeoCamera::get_OffX()
{
	return m_Input.m_Off[0];
}

// ��������Ա���ƫ�ľ�Y
double GeoCamera::get_OffY()
{
	return m_Input.m_Off[1];
}

// ��������Ա���ƫ�ľ�Z
double GeoCamera::get_OffZ()
{
	return m_Input.m_Off[2];
}

// ��ȡ���������ϵ����������ϵ����ת����
void GeoCamera::get_ROff(double *R)
{
	memcpy(R, m_Input.ROff, sizeof(double)*9);
}
// ���ô��������ϵ����������ϵ����ת����
void GeoCamera::set_ROff(double *R)
{
	memcpy( m_Input.ROff, R,sizeof(double) * 9);
}

/////////////////////////////////////////////////////
// ���¼���������Ҫ�ֱ�ʵ��,����ע�ⲻҪ�ô��麯��,�����޷�ʵ����
/////////////////////////////////////////////////////
// ��ȡ�ڷ�λԪ��
void GeoCamera::ReadCamFile(string filepath, StrCamParamInput input){}
// ����ڷ�λԪ���ļ�
void GeoCamera::WriteCamFile(string filepath){}
// ����CCD���л�ȡ��ʼ�����ڷ�λԪ���ļ�
void GeoCamera::InitInnerFile(string outpath, StrCamParamInput input){}
// ��ȡ�ڷ�λԪ��(̽Ԫָ�����ʽ)
void GeoCamera::GetInner(double x, double y, double &phiX, double &phiY){}
// ����̽Ԫָ��ǵ�ֵȡ����������
void GeoCamera::GetIndexBaseInner(double phiY, double phiX, double &y, double &x){	return;	};
