#ifndef TINMETHOD_H_
#define TINMETHOD_H_

#include "triangle.h"
#include "ModelFitMethod.h"

#if defined(_WIN32)
	#ifdef MATCHMETHOD_EXPORTS
		#define DLL_EXPORT __declspec(dllexport)
	#else
		#define DLL_EXPORT
	#endif
#else
	#define DLL_EXPORT
#endif

////////////////////////////////////////////////////////////////////////////
//	三角网类
////////////////////////////////////////////////////////////////////////////
class DLL_EXPORT TinMethod
{
private: 
	triangulateio m_in, m_out;
	int  m_nCurTriangle;
	ModelFitMethod m_trans;
	int m_num;

public:
	TinMethod();
	virtual ~TinMethod();
	// 构建三角网模型
	void CreateTriangleModel(long num, float *x, float *y, float *u, float *v);
	// 根据三角网进行内插
	void FromXY2UV(float x, float y, float& u, float& v);

private:
	// 释放三角网模型
	void ReleaseTriangleModel(triangulateio &m_tri, bool isdel=true);
	int area(int a,int b, float x3, float y3);
	int GetLocalTriangle(float x, float y);
};


#endif