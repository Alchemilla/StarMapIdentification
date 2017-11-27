#include "GeoCameraArray.h"

//////////////////////////////////////////////////////////////////////////
// ����/��������
//////////////////////////////////////////////////////////////////////////
GeoCameraArray::GeoCameraArray(void){}
GeoCameraArray::~GeoCameraArray(void){}


//////////////////////////////////////
// ���ܣ�����CCD���л�ȡ��ʼ�����ڷ�λԪ���ļ�
// ����:
//		string outpath:				�ڷ�λԪ������ļ�,���Ϊ"",�����
//		StrCamParamInput input��	�ڷ�λԪ�������������
// ����ֵ��
//		void
//////////////////////////////////////
void GeoCameraArray::InitInnerFile(string outpath, StrCamParamInput input)
{
	m_Input = input;
}


//////////////////////////////////////
// ���ܣ���ȡ�ڷ�λԪ��(̽Ԫָ�����ʽ)
// ����:
//		string outpath:				�ڷ�λԪ������ļ�
//		StrCamParamInput input��	�ڷ�λԪ�������������
// ����ֵ��
//		void
//////////////////////////////////////
void GeoCameraArray::GetInner(double x, double y, double &phiX, double &phiY)
{
	// �����ڲ�
	//x = m_Input.Ynum - x;//���������y�Ƿ���
	phiX = (x-m_Input.Xnum/2)*m_Input.Xsize/m_Input.f;
	y = m_Input.Ynum - y;//���������y�Ƿ���
	phiY = (y-m_Input.Ynum/2)*m_Input.Ysize/m_Input.f;
}


//////////////////////////////////////
// ���ܣ�����̽Ԫָ��ǵ�ֵȡ����������
// ����:
//		double y:					̽Ԫָ��ǵ�ֵ
// ����ֵ��
//		double:						���ص�������������
//////////////////////////////////////
void GeoCameraArray::GetIndexBaseInner(double phiY, double phiX, double &y, double &x)
{
	x = phiX*m_Input.f/m_Input.Xsize + m_Input.Xnum/2;
	y = phiY*m_Input.f/m_Input.Ysize + m_Input.Ynum/2;
}
