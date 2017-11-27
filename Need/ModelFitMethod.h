#ifndef MODEL_FIT_METHOD_H
#define MODEL_FIT_METHOD_H


#if defined(_WIN32)
	#ifdef MATCHMETHOD_EXPORTS
		#define DLL_EXPORT __declspec(dllexport)
	#else
		#define DLL_EXPORT
	#endif
#else
	#define DLL_EXPORT
#endif

////////////////////////////////////////////////////////
// ʵ��ƽ�����任����
////////////////////////////////////////////////////////
class DLL_EXPORT ModelFitMethod
{
public:
	ModelFitMethod(void);
	virtual ~ModelFitMethod(void);

private:
	// ����任ϵ��
	double m_aff[6];

private:
	// ���ݹ�һ������
	float xscale, yscale, uscale, vscale;         // ���Ų���
    float xoff, yoff, uoff, voff;				  // ƽ�Ʋ���
	float minx, miny, minu, minv;                 // ��Сֵ
	float maxx, maxy, maxu, maxv;                 // ���ֵ
	// ����ƽ��ֵ
	float Average(float *value, long num);
	// ���������Сֵ
	void MinMax(float *value, long num, float &max, float &min);
	// ���ݹ�һ��
	void DataNormalization(long num, float *x, float *y, float *u, float *v);
	// �޳��ֲ��
	void DeleteErrorPoint(long &mnum, float *x, float *y, float *u, float *v, long obnum);

public:
	// ���ݷ���任ϵ�����ֵ
	void GetValue_Affine(float x, float y, float &u, float &v);
	// ����RANSAC��AFFINE���дֲ��޳�
	void GrossErrorDetection_Affine(long &mnum, float *x, float *y, float *u, float *v, bool isReject=true);
	// ƽ���ع�����
	double m_move[3];
	// ����ƽ�Ʊ任ϵ�����ֵ
	void GetValue_Move(double x, double y, double &u, double &v);
	// ���¼���ƫ��
	void ReCalMovePara(ModelFitMethod m_fit);
};

#endif

