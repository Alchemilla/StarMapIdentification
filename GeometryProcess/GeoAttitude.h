#ifndef _GEOATTITUDE
#define	_GEOATTITUDE

#include "GeoOrbit.h"

////////////////////////////////////////////////////////
// 姿态基类
////////////////////////////////////////////////////////
class GeoAttitude 
{
public:
	GeoAttitude(void);
	virtual ~GeoAttitude(void);

protected:
	StrAttParamInput m_Input;			// 姿态参数输入结构体,这个不能被类外部改变,通过get_ROff获取
	long m_num;							// 姿态点个数
	StrAttPoint *m_Body2WGS84;			// 存储从本体坐标系到WGS84坐标系的旋转矩阵信息
	StrAttPoint *m_Body2Orbit;			// 存储从本体坐标系到轨道坐标系(J2000系下定义)的旋转矩阵信息，此项是用来查看侧摆角的
	StrAttPolyModel m_modelWGS84;		// 存储从本体坐标系到WGS84坐标系的姿态拟合模型
	GeoBase m_Base;						// 底层通用算法类
	GeoOrbit *m_Orbit;					// 底层通用轨道类
	// 清空数据
	void ClearData();
	
public:
	long get_num();										// 获取姿态点个数
	void get_ROff(double *R);							// 获取从测姿坐标系到本体坐标系的旋转矩阵
	void set_ROff(double *R);
	StrAttPolyModel *GetPolyModel();					// 获取姿态拟合模型
	void SetPolyModel(StrAttPolyModel attModel);		// 设置姿态拟合模型
	void ModifyPolyModelPara(double *para);				// 修正姿态拟合模型系数

	// 返回索引处姿态点信息
	struct StrAttPoint get_m_Body2WGS84(long index);
	// 返回索引处姿态点信息
	struct StrAttPoint get_m_Body2Orbit(long index);
	// 内插姿态
	StrAttPoint GetAttBody2WGS84(double UT);			// 获取指定时间的从本体坐标系到WGS84坐标系的旋转矩阵信息
	StrAttPoint GetAttBody2Orbit(double UT);			// 获取指定时间的从本体坐标系到轨道坐标系(J2000系下定义)的旋转矩阵信息
	// 姿态多项式拟合，仅仅对Body2WGS84进行
	void GenPolyModel(double startUT, double endUT);	// 根据时间段获得对应姿态的多项式模型
	void PolyValue(double UT, double* R);				// 根据姿态多项式获得姿态	
	// 姿态根据时间排序
	void OrderAtt(StrAttPoint *m_point, int num);

	// 以下函数需要分别实现,但是注意不要用纯虚函数,否则无法实例化
	virtual void ReadAttFile(string filepath, StrAttParamInput input, GeoOrbit *orbit);					// 读取姿态文件
	virtual void ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit,string workpath);	
	virtual void ReadZY3RealAttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit, string workpath);
	virtual void upDateAtt(vector<Attitude> allAtt, StrAttParamInput input);
	virtual void ReadZY3AttFile(vector<Attitude> allAtt, StrAttParamInput input, GeoOrbit *orbit);		// 读取ZY3姿态文件
	virtual void ReadAttNoFile(int num, StrAttPoint *point, StrAttParamInput input, GeoOrbit *orbit);	// 读取姿态
	virtual void WriteAttFile(string filepath, bool iserror = true);									// 写出姿态文件
	virtual void ModifyAtt(string filepath);															// 对姿态进行高频修正
};

#endif

