#ifndef _GEOORBIT
#define	_GEOORBIT

#include "GeoDefine.h"
#include "GeoBase.h"

////////////////////////////////////////////////////////
// �������
////////////////////////////////////////////////////////
class GeoOrbit
{
public:
	GeoOrbit(void);
	virtual ~GeoOrbit(void);

protected:
	StrOrbitParamInput m_Input;			// �����������ṹ��,������ܱ����ⲿ�ı�,ͨ��get_OffX��get_OffY��get_OffZ��ȡ
	long m_num;							// ��������
	StrOrbitPoint *m_EpWGS84;			// �洢WGS84ϵ�¹������Ϣ(ע�������������ָ�����¹�����ϵ)
	StrOrbitPoint *m_EpJ2000;			// �洢J2000ϵ�¹������Ϣ(ע�������������ָ����������������ϵ)
	StrOrbitPolyModel m_modelWGS84;		// �洢WGS84ϵ�¹�����ģ��
	StrOrbitPolyModel m_modelJ2000;		// �洢J2000ϵ�¹�����ģ��
	GeoBase m_Base;						// �ײ�ͨ���㷨��
	// �������
	void ClearData();

public:
	StrDATUM get_datum();					// ��ȡ���õĲο��������
	long get_num();							// ��ȡ��������
	double get_OffX();						// �����λ������Ա������ĵ�ƫ�ľ�X(����-����)
	double get_OffY();						// �����λ������Ա������ĵ�ƫ�ľ�Y(����-����)
	double get_OffZ();						// �����λ������Ա������ĵ�ƫ�ľ�Z(����-����)
	double get_RefUT();						// ��òο���Ԫ
	StrOrbitPolyModel *GetPolyModel();				// ��ȡ������ģ��
	void SetPolyModel(StrOrbitPolyModel orbModel);	// ���ù�����ģ��
	void ModifyPolyModelPara(double *para);			// ����������ģ��ϵ��

	// �ڲ���
	StrOrbitPoint GetEpWGS84(double UT);	// ��ȡָ��ʱ���WGS84�������Ϣ
	StrOrbitPoint GetEpJ2000(double UT);	// ��ȡָ��ʱ���J2000�������Ϣ
	// �������ϵ�ı任
	void GetJ20002OrbitRotation(double UT, double *R, bool isJ2000=true);		// ��ȡ��J2000���������ϵ����ת����(�������ϵ������J2000ϵ��)
	void GetOrbit2J2000Rotation(double UT, double *R, bool isJ2000=true);		// ��ȡ�ӹ������ϵ��J2000����ת����(�������ϵ������J2000ϵ��)
	// ����������WGS84ϵ�¹������Ϣ
	struct StrOrbitPoint get_m_EpWGS84(long index);
	// ����������J2000ϵ�¹������Ϣ
	struct StrOrbitPoint get_m_EpJ2000(long index);
	// ��ù���ķ������ʼʱ��
	double get_m_startTime();
	// ��ù���ķ���Ľ���ʱ��
	double get_m_endTime();
	// �������ʽ���
	void GenPolyModel(double startUT, double endUT, bool isJ2000=false);		// ����ʱ��λ�ö�Ӧ����Ķ���ʽģ��
	StrOrbitPoint PolyValue(double UT, bool isJ2000 = false);					// �ṩʱ����ݶ���ʽģ�ͻ�ö�Ӧ�Ĺ������Ϣ
	// �������ʱ������
	void OrderOrbit(StrOrbitPoint *m_point, int num);

	// ���¼��������������й�ϵ,������������ǲ�ͬ��,��Ҫ�ֱ�ʵ��,����ע�ⲻҪ�ô��麯��,�����޷�ʵ����
	virtual void ReadEphFile(string filepath, StrOrbitParamInput input);					// ��ȡ����ļ�
	virtual void ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input);	// ��ȡ���
	virtual void ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input);			// ��ȡZY3���
	virtual void WriteEphFile(string filepath, bool iserror=true);							// д������ļ�
	virtual void GetJ20002WGS84Rotation(double UT, double *R);					// ��ȡ��J2000��WGS84����ת����
	virtual void GetWGS842J2000Rotation(double UT, double *R);					// ��ȡ��WGS84��J2000����ת����
	virtual void State_J20002WGS84(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	// ����ָ��ʱ�佫λ���ٶȴ�J2000ת����WGS84
	virtual void State_WGS842J2000(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	// ����ָ��ʱ�佫λ���ٶȴ�WGS84ת����J2000
};

#endif

