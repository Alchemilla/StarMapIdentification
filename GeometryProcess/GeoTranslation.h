#ifndef _GEOTRANSLATION
#define	_GEOTRANSLATION
#include "GeoBase.h"
#include "triangle.h"
#include <vector>
using namespace std;

////////////////////////////////////////////////////////
// ����任���������
////////////////////////////////////////////////////////
class GeoTranslation
{
public:
	GeoTranslation(void);
	virtual ~GeoTranslation(void);

private:
	GeoBase m_Base;								  // �ײ�ͨ���㷨�����

// ����任���õ��Ĳ���
public:
	double m_aff[6];							// ����任ϵ��
	double xscale, yscale, uscale, vscale;		// ���Ų���
    double xoff, yoff, uoff, voff;				// ƽ�Ʋ���
	double minx, miny, minu, minv;				// ��Сֵ
	double maxx, maxy, maxu, maxv;				// ���ֵ


// ����任���õ��ĺ���
public:
	// �������任ϵ��
	bool CalAffineParam(double *x, double *y, double *u, double *v, long num);
	// ���ݷ���任ϵ�����ֵ
	void GetValueBaseAffine(double x, double y, double &u, double &v);

// ������ʽ
public:	
	vector<double> m_polyx, m_polyy;			// ������ϵ��
	int m_xnum, m_ynum, m_xynum;				// ϵ������
	// ���ݶ���ʽϵ�����ֵ
	void GetValueBasePoly(double x, double y, double &u, double &v);
	// ���㷴��ϵ��
	vector<double> m_polyxinv, m_polyyinv;		// ����������ϵ��
	void CalPolyParam();						// ���㷴��ϵ��
	void GetValueBaseInvPoly(double u, double v, double &x, double &y);

	////////////////////////////////////////////////////////
	// CE5ר��
	////////////////////////////////////////
	// �������任ϵ��_CE5ר��
	void CalAffineParamCE5(double *x, double *y, double *u, double *v, long num);
	// ����ǰһ������ϵ��,��������ϵ��,CE5ר��
	void ModifyAffineCE5(double aff[6]);
	// ��������,�Ӿֲ���ȫ��
	void GetValueCE5(double x, double y, double &u, double &v);
	// ���䷴��,��ȫ�ֵ��ֲ�
	void GetValueInvCE5(double u, double v, double &x, double &y);


// ���������õ��Ĳ���
private:
	triangulateio m_in, m_out;
	int  m_nCurTriangle;
	int m_num;

// ���������õ��ĺ���
public:
	// ����������ģ��
	void CreateTriangleModel(long num, double *x, double *y, double *u, double *v);
	// ���������������ڲ�
	void FromXY2UV(double x, double y, double& u, double& v);
private:
	// �ͷ�������ģ��
	void ReleaseTriangleModel(triangulateio &m_tri, bool isdel=true);
	int area(int a,int b, double x3, double y3);
	int GetLocalTriangle(double x, double y);

// ��һ�����õ��ĺ���
private:
	// ����ƽ��ֵ
	double Average(double *value, long num);
	// ���������Сֵ
	void MinMax(double *value, long num, double &max, double &min);
	// ���ݹ�һ��
	void DataNormalization(long num, double *x, double *y, double *u, double *v);
};


#endif
