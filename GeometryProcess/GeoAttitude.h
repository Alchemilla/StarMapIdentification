#ifndef _GEOATTITUDE
#define	_GEOATTITUDE

#include "GeoOrbit.h"

////////////////////////////////////////////////////////
// ��̬����
////////////////////////////////////////////////////////
class GeoAttitude 
{
public:
	GeoAttitude(void);
	virtual ~GeoAttitude(void);

protected:
	StrAttParamInput m_Input;			// ��̬��������ṹ��,������ܱ����ⲿ�ı�,ͨ��get_ROff��ȡ
	long m_num;							// ��̬�����
	StrAttPoint *m_Body2WGS84;			// �洢�ӱ�������ϵ��WGS84����ϵ����ת������Ϣ
	StrAttPoint *m_Body2Orbit;			// �洢�ӱ�������ϵ���������ϵ(J2000ϵ�¶���)����ת������Ϣ�������������鿴��ڽǵ�
	StrAttPolyModel m_modelWGS84;		// �洢�ӱ�������ϵ��WGS84����ϵ����̬���ģ��
	GeoBase m_Base;						// �ײ�ͨ���㷨��
	GeoOrbit *m_Orbit;					// �ײ�ͨ�ù����
	// �������
	void ClearData();
	
public:
	long get_num();										// ��ȡ��̬�����
	void get_ROff(double *R);							// ��ȡ�Ӳ�������ϵ����������ϵ����ת����
	void set_ROff(double *R);
	StrAttPolyModel *GetPolyModel();					// ��ȡ��̬���ģ��
	void SetPolyModel(StrAttPolyModel attModel);		// ������̬���ģ��
	void ModifyPolyModelPara(double *para);				// ������̬���ģ��ϵ��

	// ������������̬����Ϣ
	struct StrAttPoint get_m_Body2WGS84(long index);
	// ������������̬����Ϣ
	struct StrAttPoint get_m_Body2Orbit(long index);
	// �ڲ���̬
	StrAttPoint GetAttBody2WGS84(double UT);			// ��ȡָ��ʱ��Ĵӱ�������ϵ��WGS84����ϵ����ת������Ϣ
	StrAttPoint GetAttBody2Orbit(double UT);			// ��ȡָ��ʱ��Ĵӱ�������ϵ���������ϵ(J2000ϵ�¶���)����ת������Ϣ
	// ��̬����ʽ��ϣ�������Body2WGS84����
	void GenPolyModel(double startUT, double endUT);	// ����ʱ��λ�ö�Ӧ��̬�Ķ���ʽģ��
	void PolyValue(double UT, double* R);				// ������̬����ʽ�����̬	
	// ��̬����ʱ������
	void OrderAtt(StrAttPoint *m_point, int num);

	// ���º�����Ҫ�ֱ�ʵ��,����ע�ⲻҪ�ô��麯��,�����޷�ʵ����
	virtual void ReadAttFile(string filepath, StrAttParamInput input, GeoOrbit *orbit);					// ��ȡ��̬�ļ�
	virtual void ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit,string workpath);	
	virtual void ReadZY3RealAttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath);
	virtual void upDateAtt(vector<Attitude> allAtt, StrAttParamInput input);
	virtual void ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit);		// ��ȡZY3��̬�ļ�
	virtual void ReadAttNoFile(int num, StrAttPoint *point, StrAttParamInput input, GeoOrbit *orbit);	// ��ȡ��̬
	virtual void WriteAttFile(string filepath, bool iserror = true);									// д����̬�ļ�
	virtual void ModifyAtt(string filepath);															// ����̬���и�Ƶ����
};

#endif

