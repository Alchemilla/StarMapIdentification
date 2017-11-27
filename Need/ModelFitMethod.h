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
// 实现平面仿射变换的类
////////////////////////////////////////////////////////
class DLL_EXPORT ModelFitMethod
{
public:
	ModelFitMethod(void);
	virtual ~ModelFitMethod(void);

private:
	// 仿射变换系数
	double m_aff[6];

private:
	// 数据归一化参数
	float xscale, yscale, uscale, vscale;         // 缩放参数
    float xoff, yoff, uoff, voff;				  // 平移参数
	float minx, miny, minu, minv;                 // 最小值
	float maxx, maxy, maxu, maxv;                 // 最大值
	// 计算平均值
	float Average(float *value, long num);
	// 计算最大最小值
	void MinMax(float *value, long num, float &max, float &min);
	// 数据归一化
	void DataNormalization(long num, float *x, float *y, float *u, float *v);
	// 剔除粗差点
	void DeleteErrorPoint(long &mnum, float *x, float *y, float *u, float *v, long obnum);

public:
	// 根据仿射变换系数获得值
	void GetValue_Affine(float x, float y, float &u, float &v);
	// 采用RANSAC和AFFINE进行粗差剔除
	void GrossErrorDetection_Affine(long &mnum, float *x, float *y, float *u, float *v, bool isReject=true);
	// 平移沿轨缩放
	double m_move[3];
	// 根据平移变换系数获得值
	void GetValue_Move(double x, double y, double &u, double &v);
	// 重新计算偏移
	void ReCalMovePara(ModelFitMethod m_fit);
};

#endif

