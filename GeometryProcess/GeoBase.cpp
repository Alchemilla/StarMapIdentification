#include "GeoBase.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "GeoDefine.h"
#include <map>
using namespace std;

#ifdef WIN32
#include <io.h>
#else
#include <dirent.h>
#endif

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoBase::GeoBase(void)	{}
GeoBase::~GeoBase(void)	{}

//////////////////////////////////////////////////////////////////////////
// 通用函数
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：从一个文件夹中按照通配符去搜索所有满足条件的文件(单层文件)
// 输入:
//		string dirPath:			搜索的文件夹路径
//		vector<string> &outPath:满足条件的文件列表
//		string dirCode：		通配符,如"*.txt"
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::DiretorySearch(string dirPath, vector<string> &outPath, string dirCode)
{
#ifdef WIN32
	int i;
	outPath.clear();
	// begin search
	struct _finddata_t filefind;
	char temp[2]="\\";
	char curr[1024],curr2[1024];
	char currtemp[1024];
	strcpy(curr,dirPath.c_str());
	strcat(curr,temp);
	strcpy(curr2,curr);
	strcat(curr,dirCode.c_str());
	int done=0,handle;
	if((handle=_findfirst(curr,&filefind))==-1)
		return;
	strcpy(currtemp,curr2);
	strcat(currtemp,filefind.name);
	outPath.push_back((string)currtemp);
	while(!(done=_findnext(handle,&filefind)))
	{
		if(!strcmp(filefind.name,".."))
			continue;
		strcpy(currtemp,curr2);
		strcat(currtemp,filefind.name);
		outPath.push_back((string)currtemp);
	}
	_findclose(handle);
#else

    if(dirPath.size()< 1 || dirPath == "" || dirCode.size() <1 || dirCode == "")
		return;
	DIR* pdir;
	int i;
    struct dirent*	pdir_ent;
    string str_tmp;
    string last_baking;
    int dot_pos;
    if(dirPath[dirPath.size()-1] != '/')
		dirPath += '/';
    int getcount = 0;
	outPath.clear();

    if((pdir = opendir(dirPath.c_str())) == NULL)
    {
        printf("Open Folder %s Failed\n", dirPath.c_str());
        return;
    }
    while((pdir_ent = readdir(pdir)) != NULL)
    {
		str_tmp = pdir_ent->d_name;
		if(pdir_ent->d_type == DT_DIR)
        {
            continue;
        }
        if(strcmp(pdir_ent->d_name, ".") == 0 || strcmp(pdir_ent->d_name, "..")==0 )
        {
            continue;
        }
        if(str_tmp.find(dirCode) == string::npos )
        {
            continue;
        }
        bool add = true;
        for(i=0; i<dirCode.size(); i++)
        {
        	if(str_tmp[str_tmp.size()-dirCode.size()+i]!=dirCode[i])
        		add = false;
        }
        if(add==true)
        	outPath.push_back(dirPath + str_tmp);
    }//while
    closedir(pdir);

#endif
}


//////////////////////////////////////
// 功能：从一个文件夹中按照通配符去搜索所有满足条件的文件(多层文件)
// 输入:
//		string dirPath:			搜索的文件夹路径
//		vector<string> &outPath:满足条件的文件列表
//		string dirCode：		通配符,如"*.txt"
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::DiretorySearchAll(string dirPath, vector<string> &outPath, string dirCode)
{
	int i;
	// 第一层
	struct _finddata_t filefind;
	char temp[2]="\\";
	char curr[1024],curr2[1024];
	char currtemp[1024];
	strcpy(curr, dirPath.c_str());
	strcat(curr, temp);
	strcpy(curr2, curr);
	strcat(curr, dirCode.c_str());
	int done=0,handle;
	if((handle=_findfirst(curr,&filefind))!=-1)
	{
		strcpy(currtemp,curr2);
		strcat(currtemp,filefind.name);
		outPath.push_back((string)currtemp);
		while(!(done=_findnext(handle,&filefind)))
		{
			if(!strcmp(filefind.name,".."))
				continue;
			strcpy(currtemp,curr2);
			strcat(currtemp,filefind.name);
			outPath.push_back((string)currtemp);
		}
		_findclose(handle);
	}
	// 子目录
	strcpy(curr, dirPath.c_str());
	strcat(curr, temp);
	strcpy(curr2, curr);
	strcat(curr, "*.*");
	// 判断是否有子目录
	if((handle=_findfirst(curr, &filefind))!=-1)
	{
		do  
        {  
			if(filefind.attrib&_A_SUBDIR)
			{
				if(strcmp(filefind.name,"..")!=0&&strcmp(filefind.name,".")!=0)
				{
					// 递归调用
					DiretorySearchAll(dirPath+"\\"+filefind.name, outPath, dirCode);
				}
			}
		} while(_findnext(handle, &filefind) == 0);  
		_findclose(handle);
	}
}


//////////////////////////////////////////////////////////
// 从一个文件夹中搜索所有子文件夹(单层)
//////////////////////////////////////////////////////////
void GeoBase::DiretorySearchFolder(string dirPath, vector<string> &folder)
{
	folder.clear();
	//文件句柄 
	long  hFile  =  0; 
	//文件信息 
	struct _finddata_t fileinfo; 
	string p; 
	if((hFile = _findfirst(p.assign(dirPath).append("\\*").c_str(),&fileinfo)) != -1) 
	{ 
		do
		{  
			if((fileinfo.attrib & _A_SUBDIR)) 
			{ 
				if(strcmp(fileinfo.name,".") != 0 && strcmp(fileinfo.name,"..") != 0) 
				{
					folder.push_back(fileinfo.name);
				}           
			}  
		}while(_findnext(hFile, &fileinfo) == 0); 
		_findclose(hFile); 
	} 
}



//////////////////////////////////////////////////////////
// 功能：更改前缀后缀名(全部)
// 输入:
//		string outFolder：		输出文件夹名称,如果为"",则相同文件夹
//		string ext：			更改的后缀名称
//		vector<string> &list1：	待更改的文件列表
//		vector<string> &list2：	已更改的文件列表
// 返回值：
//		void
//////////////////////////////////////////////////////////
void GeoBase::ChangeAndSaveExt(string outFolder, string ext, vector<string> &list1, vector<string> &list2)
{
	long size = list1.size();
	list2.resize(size);
	for(long i=0; i<size; i++)
	{
		string name1 = list1[i].substr(0, list1[i].find_last_of('.')+1);
		string name = name1.substr(list1[i].find_last_of("\\//")+1);
		if(outFolder.compare(""))
			list2[i] = outFolder.append(name).append(ext);
		else
		{
			string folder = list1[i].substr(0, list1[i].find_last_of("\\//")+1);
			list2[i] = folder.append(name).append(ext);
		}
	}
}

//////////////////////////////////////////////////////////
// 功能：截取得到路径名称
// 输入:
//		vector<string> &list1：	待更改的文件列表
//		vector<string> &list2：	已更改的文件列表
// 返回值：
//		void
////////////////////////////////////////////////////////// 
void GeoBase::GetDiretory(vector<string> &list1, vector<string> &list2)
{
	long size = list1.size();
	list2.resize(size);
	for(long i=0; i<size; i++)
	{
		list2[i] = list1[i].substr(0, list1[i].find_last_of("\\//")+1);
	}
}


//////////////////////////////////////////////////////////
// 功能：更改前缀后缀名(单个)
// 输入:
//		string dirPath:			搜索的文件夹路径
//		vector<string> &outPath:满足条件的文件列表
//		string dirCode：		通配符,如"*.txt"
// 返回值：
//		void
//////////////////////////////////////////////////////////
void GeoBase::ChangeAndSaveExt(string outFolder, string ext, string &list1, string &list2)
{
	string name1 = list1.substr(0, list1.find_last_of('.')+1);
	string name = name1.substr(list1.find_last_of("\\//")+1);
	if(outFolder.compare(""))
		list2 = outFolder.append(name).append(ext);
	else
	{
		string folder = list1.substr(0, list1.find_last_of("\\//")+1);
		list2 = folder.append(name).append(ext);
	}
}


//////////////////////////////////////////////////////////////////////////
// 时间转化函数
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////
// 功能：将格里高利历与UT时转化为用户规定的累计秒
// 输入:
//		double refMJD：		参考历元的约化儒略日
//		int year:			待转化历元的年
//		int month：			待转化历元的月
//		int day：			待转化历元的日
//		int hour：			待转化历元的时
//		int minute：		待转化历元的分
//		double second：		待转化历元的秒
// 输出：
//		double &refsecond：	当前历元距离参考历元的累计秒
// 返回值：
//     void:			
////////////////////////////////////////////////////
void GeoBase::FromYMDtoSecond(double refMJD, int year, int month, int day, int hour, int minute, double second, double& refsecond)
{
	double jd0, mjd;
	Cal2JD(year, month, day, 0, &jd0, &mjd);
	refsecond = (mjd-refMJD)*86400 + hour*3600 + minute*60 + second;
}


////////////////////////////////////////////////////
// 功能：将用户规定的累计秒转化为格里高利历和UT时
// 输入:
//		double refMJD：		参考历元的约化儒略日
//		double refsecond：	当前历元距离参考历元的累计秒
// 输出：
//		int &year:			待转化历元的年
//		int &month：		待转化历元的月
//		int &day：			待转化历元的日
//		int &hour：			待转化历元的时
//		int &minute：		待转化历元的分
//		double &second：	待转化历元的秒
// 返回值：
//     void:			
////////////////////////////////////////////////////
void GeoBase::FromSecondtoYMD(double refMJD, double refsecond, int& year, int& month, int& day, int& hour, int& minute, double& second)
{
	double fracday, jd0;
	double mjd = refMJD + refsecond/86400;
	JD2Cal(2400000.5, mjd, &year, &month, &day, &fracday);
	Cal2JD(year, month, day, 0, &jd0, &mjd);
	fracday = refsecond - (mjd-refMJD)*86400;
	hour = (int)(fracday/3600);
	minute = (int)((fracday-hour*3600)/60.0);
	second = fracday - hour*3600.0 - minute*60.0;
}



//////////////////////////////////////////////////////////////////////////
// 坐标转化函数
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
// 功能：获得参考椭球系数
// 输入:
//     string datumname：	椭球基准名称
// 输出：
//	   StrDATUM &datum:		输出的椭球基准
// 返回值：
//     int:	   0			成功
//			   -1			失败
////////////////////////////////////////////////////////////////
int GeoBase::GetRefEllipsoid(string datumname, StrDATUM &datum)
{
	if(!strcmp(datumname.c_str(),"WGS84"))
	{
		datum.DatumName = "WGS84";	datum.a = 6378137.0;	datum.f = 298.257223563;
		datum.e2 = (2*datum.f-1)/pow(datum.f, 2);	// e = (2*f-1)/f
		datum.b = (datum.f-1)/datum.f*datum.a;
		datum.a2 = pow(datum.a, 2);
		datum.b2 = pow(datum.b, 2);
		datum.a2_b2 = pow(datum.a/datum.b, 2);
	}
	if(!strcmp(datumname.c_str(),"IAUMOON"))
	{
		datum.DatumName = "IAUMOON";	datum.a = 1737400.0;	datum.f = 0.0;
		datum.e2 = 0;					datum.b = datum.a;
		datum.a2 = pow(datum.a, 2);		datum.b2 = pow(datum.b, 2);
		datum.a2_b2 = pow(datum.a/datum.b, 2);
	}
	return 0;
}


////////////////////////////////////////////////////////////////
// 功能：大地坐标转直角坐标
// 输入:
//		StrDATUM datum：	参考椭球参数
//		double B：			纬度(单位弧度)
//		double L：			经度(单位弧度)
//		double H：			高程(单位为米)
// 输出：
//		double &X：			直角坐标X(单位为米)
//		double &Y：			直角坐标Y(单位为米)
//		double &Z：			直角坐标Z(单位为米)
// 返回值：
//		void
////////////////////////////////////////////////////////////////
void GeoBase::Geograph2Rect(StrDATUM datum, double B, double L, double H, double &X, double &Y, double &Z)
{
	double W = sqrt(1-datum.e2*pow(sin(B),2));	// 辅助函数
	if(W<=0) return;							// 特殊情况考虑,以让程序正确运行
	double N = datum.a/W;						// 卯酉圈半径
	double r = (N+H)*cos(B);					// 考虑高程后投影到赤道面
	X = r*cos(L);
	Y = r*sin(L);
	Z = (N*(1-datum.e2) + H)*sin(B);
}


////////////////////////////////////////////////////////////////
// 功能：直角坐标转大地坐标(此方法对于地球质心附近情况不适用)
// 输入:
//		StrDATUM datum：	参考椭球参数
//		double &X：			直角坐标X(单位为米)
//		double &Y：			直角坐标Y(单位为米)
//		double &Z：			直角坐标Z(单位为米)
// 输出：
//		double B：			纬度(单位弧度)
//		double L：			经度(单位弧度)
//		double H：			高程(单位为米)
// 返回值：
//		void
////////////////////////////////////////////////////////////////
void GeoBase::Rect2Geograph(StrDATUM datum, double X, double Y, double Z,double &B, double &L, double &H)
{
	// 对于球的特殊处理
	if(datum.f==0)
	{
		// 对于椭球情况的处理
		double XY = sqrt(X*X + Y*Y);				// XY投影面上的长度
		// 先求出经度
		L = atan2(Y, X);
		// 再求出纬度
		B = atan2(Z, XY);
		// 再求出高程
		H = sqrt(XY*XY+Z*Z)-datum.a;
		return;
	}
	// 对于椭球情况的处理
	double XY = sqrt(X*X + Y*Y);				// XY投影面上的长度
	// 先求出经度
	L = atan2(Y, X);
	////////////////////////////////////////
	// 接着求纬度
	////////////////////////////////////////
	double z1, z2, dz, B0, B1, B2, delta;
	B0 = atan2(Z, XY);							// 地心纬度
	delta = 0.0001;	
	int num = 0;
	do 
	{	
		B1 = B0;		B2 = B0-delta;
		z1 = XY*datum.a2_b2*tan(B1)-(datum.a2_b2-1)*sin(B1)*datum.b/sqrt(1-datum.e2*pow(cos(B1), 2));
		z2 = XY*datum.a2_b2*tan(B2)-(datum.a2_b2-1)*sin(B2)*datum.b/sqrt(1-datum.e2*pow(cos(B2), 2));
		dz = z1-z2;		B0 += (Z-z1)/dz*delta;
		if(num++>1000) break;
	}while(fabs(Z-z1)>fabs(dz));
	B = atan(tan(B0)*datum.a2_b2);
	////////////////////////////////////////
	// 最后求高程
	////////////////////////////////////////
	double re = datum.b/sqrt(1.0-datum.e2*pow(cos(B0),2));	
	H = sqrt(pow(Z-re*sin(B0), 2) + pow(XY-re*cos(B0), 2));	
	if((XY*XY+Z*Z)<pow(re,2))
		H *= -1.0;
}



//////////////////////////////////////////////////////////////////////////
// 内插函数
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 轨道拉格朗日内插
// 功能：轨道拉格朗日内插
// 输入:
//		StrOrbitPoint *Eph：存储轨道点的结构体数组指针
//		long EphNum：		轨道点个数
//		double UT：			内插轨道点所对应的的累计秒
//		int order：			拉格朗日内插次数,默认是7次
// 输出：
//		StrOrbitPoint &m_point：内插得到的轨道点
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::LagrangianInterpolation(StrOrbitPoint *Eph, long EphNum, double UT, StrOrbitPoint &m_point, int order)
{
	memset(&m_point, 0, sizeof(StrOrbitPoint));
	m_point.UT = UT;
	// 搜索内插用的起始和结束点(对分查找)
	double up=1, down=1;
	long posstart, posend, pos;
	posstart=0, posend=EphNum-1, pos=0;
	while(posstart<posend)  
    {  
		pos = (posstart+posend)/2;
		if(pos==posstart) break;	// 记得加上这句判断,否则会陷入死循环
		if((Eph[pos].UT<=UT)&&(UT<Eph[pos+1].UT))
			break;  
        if (Eph[pos].UT <= UT)  
            posstart = pos;  
		else
            posend = pos;       
    }  
	if(pos-order/2<0)			posstart = 0;
	else						posstart = pos-order/2;
	if(pos+order/2>=EphNum-1)	posend = EphNum-1;
	else						posend = pos+order/2;
    int i,j;
	// 开始进行内插
	for(j=posstart;j<=posend;j++)
	{
        up=1,down=1;
		for(i=posstart;i<=posend;i++)
			if(i!=j)	{	up *= (UT-Eph[i].UT);	down *= (Eph[j].UT-Eph[i].UT);	}
		for(i=0; i<6; i++)
			m_point.X[i] += Eph[j].X[i]*up/down;
	}
}


//////////////////////////////////////
// 功能：姿态四元数内插
// 输入:
//		StrAttPoint *Att：	存储姿态点的结构体数组指针
//		long AttNum：		姿态点个数
//		double UT：			内插姿态点的所对应的累计秒
// 输出：
//		StrAttPoint &m_att：内插得到的姿态点
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::QuatInterpolation(StrAttPoint *Att, long AttNum, double UT, StrAttPoint &m_att)
{
	if(AttNum<2)	{	printf("QuatInterpolation Error：AttNum is less than 2, can't interpolation!\n");	return;	}
	// 寻找临近的两个点(对分查找)
	StrAttPoint attleft, attright, att;
	long posstart, posend, pos;
	posstart=0, posend=AttNum-1, pos=0;
	while(posstart<posend)  
    {  
		pos = (posstart+posend)/2;
		if(pos==posstart) break;	// 记得加上这句判断,否则会陷入死循环
		if((Att[pos].UT<=UT)&&(Att[pos+1].UT>UT))
			break;
        if (Att[pos].UT <= UT)  
            posstart = pos;  
		else
            posend = pos;       
    }  
	if(pos < 0)	pos = 0;
	if(pos >= AttNum-1)		pos = AttNum-2;
	attleft = Att[pos];		attright = Att[pos+1];
	
	// 进行内插
	double sp,sq;
	double t = (UT - attleft.UT)/(attright.UT - attleft.UT);
	double cosa = attleft.q[0]*attright.q[0] + attleft.q[1]*attright.q[1] + attleft.q[2]*attright.q[2] + attleft.q[3]*attright.q[3];
	// 这个错误需要注意了,防止邻近两个值互为反号的情况,需要确保length>0
	if(cosa<0)			
	{	
		cosa = -cosa;	
		attright.q[0] = -attright.q[0];	attright.q[1] = -attright.q[1];	attright.q[2] = -attright.q[2];	attright.q[3] = -attright.q[3];
	}
	if(cosa>0.9999f)
	{
		sp = 1.0-t;	sq = t;
	}
	else
	{
		double sina = sqrt(1.0-pow(cosa,2));	double a = atan2(sina, cosa);	double invSina = 1.0/sina;
		sp = sin((1.0-t)*a)*invSina;			sq = sin(t*a)*invSina;
	}
	m_att.q[0] = sp*attleft.q[0] + sq*attright.q[0];	m_att.q[1] = sp*attleft.q[1] + sq*attright.q[1];
	m_att.q[2] = sp*attleft.q[2] + sq*attright.q[2];	m_att.q[3] = sp*attleft.q[3] + sq*attright.q[3];
}
void GeoBase::QuatInterpolation(vector<Attitude>Att, double UT, Attitude &m_att)
{
	int AttNum = Att.size();
	StrAttPoint *AttCopy = new StrAttPoint[AttNum];
	StrAttPoint m_attCopy;
	for (int i=0;i<AttNum;i++)
	{
		AttCopy[i].UT = Att[i].UTC;
		AttCopy[i].q[0] = Att[i].Q1; AttCopy[i].q[1] = Att[i].Q2;
		AttCopy[i].q[2] = Att[i].Q3; AttCopy[i].q[3] = Att[i].Q0;
	}
	QuatInterpolation(AttCopy, AttNum, UT, m_attCopy);
	m_att.UTC = m_attCopy.UT;
	m_att.Q1 = m_attCopy.q[0]; m_att.Q2 = m_attCopy.q[1];
	m_att.Q3 = m_attCopy.q[2]; m_att.Q0 = m_attCopy.q[3];
	delete[] AttCopy; AttCopy = NULL;
}

//四元数顺序为0123，其中0为标量
void GeoBase::quatMult(double * q1, double * q2, double * q3)
{
	q3[1] = q1[0] * q2[1] - q1[3] * q2[2] + q1[2] * q2[3] + q1[1] * q2[0];
	q3[2] = q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0];
	q3[3] = -q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3] + q1[3] * q2[0];
	q3[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0];
}


////////////////////////////////////////////////////////
// 旋转矩阵与四元数的相互变换
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：从旋转矩阵获得四元数
// 输入:
//		double *R：		旋转矩阵3*3，行优先
// 输出：
//		double &q1:		四元数q1
//		double &q2:		四元数q2
//		double &q3:		四元数q3
//		double &q4:		四元数q4
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::Matrix2Quat(double *R, double &q1, double &q2, double &q3, double &q4)
{
 	int i, j;	double tq[4];
    tq[0] = 1 + R[0] + R[4] + R[8];	tq[1] = 1 + R[0] - R[4] - R[8];
    tq[2] = 1 - R[0] + R[4] - R[8];	tq[3] = 1 - R[0] - R[4] + R[8];
	// 寻找最大值
    j = 0;
    for(i=1;i<4;i++) j = (tq[i]>tq[j])? i : j;
	// 检查对角线
	switch(j)
	{
	case 0:{ q4 = tq[0];	q1 = R[5] - R[7]; q2 = R[6] - R[2];	q3 = R[1] - R[3];	break;	}
	case 1:{ q4 = R[5] - R[7];	q1 = tq[1];	q2 = R[1] + R[3];	q3 = R[6] + R[2];	break;	}
	case 2:{ q4 = R[6] - R[2];	q1 = R[1] + R[3];	q2 = tq[2];	q3 = R[5] + R[7];	break;	}
	case 3:{ q4 = R[1] - R[3];	q1 = R[6] + R[2];	q2 = R[5] + R[7];q3 = tq[3];	break;	}
	default:{	printf("Matrix2Quat Error\n");	return;	}
	}
    double s = sqrt(0.25/tq[j]);	q1 *= s;	q2 *= s;	q3 *= s;	q4 *= s;
}


//////////////////////////////////////
// 功能：从四元数获得旋转矩阵
// 输入:
//		double &q1:		四元数q1
//		double &q2:		四元数q2
//		double &q3:		四元数q3
//		double &q4:		四元数q4
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::Quat2Matrix(double q1, double q2, double q3, double q4, double *R)
{
	// 求取欧式空间长度
	double length2 = pow(q1,2) + pow(q2,2) + pow(q3,2) + pow(q4,2);
	// 非单位四元数
	if(fabs(length2-1.0) >= DBL_MIN*10)
	{
		if (fabs(length2) <= DBL_MIN*2)
		{
			printf("Quat2Matrix Error!\n");	// 输入不是单位四元数,并且欧式距离太短
			memset(R, 0, sizeof(double)*9);
			return;
		}
		double length = sqrt(length2);
		q1 /= length;  q2 /= length;  q3 /= length;  q4 /= length;
	}
	// 计算旋转矩阵
	R[0] = 1.0-2.0*(q2*q2+q3*q3);	R[1] = 2.0 * (q1*q2 + q3*q4);	R[2] = 2.0 * (q1*q3 -q2*q4);
	R[3] = 2.0 * (q1*q2 -q3*q4);	R[4] = 1.0-2.0*(q1*q1 + q3*q3);	R[5] = 2.0 * (q2*q3 + q1*q4);
	R[6] = 2.0 * (q1*q3 + q2*q4);	R[7] = 2.0 * (q2*q3 - q1*q4);	R[8] = 1.0-2.0*(q1*q1 + q2*q2);
}


////////////////////////////////////////////////////////
// 旋转矩阵与欧拉角的相互变换
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：从旋转矩阵获得欧拉角
// 输入:
//		double *R：		旋转矩阵3*3，行优先	
//		int ratateOrder:欧拉角转序
// 输出：
//		double &eulor1:	欧拉角1
//		double &eulor2:	欧拉角2
//		double &eulor3:	欧拉角3
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3)
{
	switch(rotateOrder)
	{
		// 第一类:第一次和第三次转动是绕同类坐标轴进行的,第二次转动是绕另两轴中的一类进行的
	case 121:  // 1
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[0]);					double temp = sin(eulor2);
			eulor1 = atan2(R[1]*temp, -R[2]*temp);	eulor3 = atan2(R[3]*temp, R[6]*temp);	break;
		}
	case 131:	// 2
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[0]);					double temp = sin(eulor2);
			eulor1 = atan2(R[2]*temp, R[1]*temp);	eulor3 = atan2(R[6]*temp, -R[3]*temp);	break;
		}
	case 212:	// 3
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[4]);					double temp = sin(eulor2);
			eulor1 = atan2(R[3]*temp, R[5]*temp);	eulor3 = atan2(R[1]*temp, -R[7]*temp);	break;
		}
	case 232:	// 4
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[4]);					double temp = sin(eulor2);
			eulor1 = atan2(R[5]*temp, -R[3]*temp);	eulor3 = atan2(R[7]*temp, R[1]*temp);	break;
		}
	case 313:	// 5
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[8]);					double temp = sin(eulor2);
			eulor1 = atan2(R[6]*temp, -R[7]*temp);	eulor3 = atan2(R[2]*temp, R[5]*temp);	break;
		}
	case 323:	// 6
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[8]);					double temp = sin(eulor2);
			eulor1 = atan2(R[7]*temp, R[6]*temp);	eulor3 = atan2(R[5]*temp, -R[2]*temp);	break;
		}
		// 第二类:每次转动是绕不同类别的坐标轴进行的
	case 123:	// 7
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(R[6]);					double temp = cos(eulor2);
			eulor1 = atan2(-R[7]*temp, R[8]*temp);	eulor3 = atan2(-R[3]*temp, R[0]*temp);	break;
		}
	case 132:	// 8
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(-R[3]);					double temp = cos(eulor2);
			eulor1 = atan2(R[5]*temp, R[4]*temp);	eulor3 = atan2(R[6]*temp, R[0]*temp);	break;
		}
	case 213:	// 9
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(-R[7]);					double temp = cos(eulor2);
			eulor1 = atan2(R[6]*temp, R[8]*temp);	eulor3 = atan2(R[1]*temp, R[4]*temp);	break;
		}
	case 231:	// 10
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(R[1]);					double temp = cos(eulor2);
			eulor1 = atan2(-R[2]*temp, R[0]*temp);	eulor3 = atan2(-R[7]*temp, R[4]*temp);	break;
		}
	case 312:	// 11
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(R[5]);					double temp = cos(eulor2);
			eulor1 = atan2(-R[3]*temp, R[4]*temp);	eulor3 = atan2(-R[2]*temp, R[8]*temp);	break;
		}
	case 321:	// 12
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(-R[2]);					double temp = cos(eulor2);
			eulor1 = atan2(R[1]*temp, R[0]*temp);	eulor3 = atan2(R[5]*temp, R[8]*temp);	break;
		}
	}
}


//////////////////////////////////////
// 功能：从欧拉角获得旋转矩阵
// 输入:
//		double &eulor1:	欧拉角1
//		double &eulor2:	欧拉角2
//		double &eulor3:	欧拉角3
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
//		int ratateOrder:欧拉角转序	
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R)
{
	double R1[9], R2[9], R3[9], Rtemp[9];
	switch(rotateOrder)
	{
		// 第一类:第一次和第三次转动是绕同类坐标轴进行的,第二次转动是绕另两轴中的一类进行的
	case 121:	// 1
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 131:	// 2
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 212:	// 3
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 232:	// 4
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 313:	// 5
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 323:	// 6
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
		// 第二类:每次转动是绕不同类别的坐标轴进行的
	case 123:	// 7
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 132:	// 8
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 213:	// 9
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 231:	// 10
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 312:	// 11
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 321:	// 12
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	default:
		{
			printf("Eulor2Matrix Error!\n");	break;	// 没有此种转序存在!
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// 绕轴的旋转
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：绕X轴转角angle的旋转矩阵
// [ 1          0         0     ]
// [ 0    cos(angle)  sin(angle)]
// [ 0   -sin(angle)  cos(angle)]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::RotationX(double angle, double *R)
{
	memset(R, 0, sizeof(double)*9);
	R[0] = 1.0;
	R[8] = R[4] = cos(angle);
	R[5] = sin(angle);
	R[7] = -R[5];
}


//////////////////////////////////////
// 功能：绕Y轴转角angle的旋转矩阵
// [cos(angle)  0    -sin(angle)]
// [     0      1         0     ]
// [sin(angle)  0     cos(angle)]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::RotationY(double angle, double *R)
{
	memset(R, 0, sizeof(double)*9);
	R[8] = R[0] = cos(angle);
	R[4] = 1.0;
	R[2] = -sin(angle);
	R[6] = -R[2];
}


//////////////////////////////////////
// 功能：绕Z轴转角angle的旋转矩阵
// [ cos(angle)  sin(angle)  0]
// [-sin(angle)  cos(angle)  0]
// [     0            0      1]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::RotationZ(double angle, double *R)
{
	memset(R, 0, sizeof(double)*9);
	R[4] = R[0] = cos(angle);
	R[8] = 1.0;
	R[1] = sin(angle);
	R[3] = -R[1];
}

void GeoBase::rot(double phi, double omg, double kap, double * R)
{
	memset(R, 0, 9 * sizeof(double));
	double RX[9], RY[9], RZ[9],Tmp[9];
	RotationX(-omg, RX);
	RotationY(-phi, RY);
	RotationZ(-kap, RZ);

	Multi(RY, RX, Tmp, 3, 3, 3);
	Multi(Tmp, RZ, R, 3, 3, 3);

}



////////////////////////////////////////////////////////////////////////////////////
// 矩阵的若干运算
////////////////////////////////////////////////////////////////////////////////////
// 求矩阵转置，形参m为行，n为列,A转置后存为B 
void GeoBase::Transpose(double *A,double *B, int m,int n)
{
    for (int i=0;i<n;i++)
       for (int j=0;j<m;j++)
           B[i*m+j]=A[j*n+i];
}

// 求矩阵转置，形参m为行，n为列,A转置后存为A 
void GeoBase::Transpose(double *A, int m, int n)
{
	long size = m*n;
	double *B = new double[size];
    for (int i=0;i<n;i++)
       for (int j=0;j<m;j++)
           B[i*m+j]=A[j*n+i];
	memcpy(A, B, sizeof(double)*size);
	if(B!=NULL) { delete []B; B=NULL; };
}

// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
void GeoBase::Multi(double *A,double *B,double *C ,int m,int p,int n)
{
     for (int i=0;i<m;i++)
        for (int j=0;j<n;j++)
        {
            double sum=0;
            for (int k=0;k<p;k++)
                sum=sum+A[i*p+k]*B[k*n+j];
            C[i*n+j]=sum;
        }
}

// 求矩阵和常数相乘,A矩阵为[m,n] 
void GeoBase::Multi(double *A, int m, int n, double p)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			A[i*n+j] *= p;
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
void GeoBase::Add(double *A,double *B, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			A[i*n+j] += B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
void GeoBase::Add(double *A,double *B, double *C, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			C[i*n+j] = A[i*n+j] + B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
void GeoBase::Add(double *A,double *B, int m,int n, int l, int k, int p, int q)
{
	for(int i=0; i<l; i++)
	{
		for(int j=0; j<k; j++)
		{
			A[(i+p)*n+j+q] += B[i*k+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
void GeoBase::Minus(double *A,double *B, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			A[i*n+j] -= B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
void GeoBase::Minus(double *A,double *B, double *C, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			C[i*n+j] = A[i*n+j] - B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
void GeoBase::Minus(double *A,double *B, int m,int n, int l, int k, int p, int q)
{
	for(int i=0; i<l; i++)
	{
		for(int j=0; j<k; j++)
		{
			A[(i+p)*n+j+q] -= B[i*k+j];
		}
	}
}


// 矩阵反号,A矩阵为[m,n],B矩阵为[m,n]
void GeoBase::RevSig(double *A,double *B, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			B[i*n+j] = -A[i*n+j];
		}
	}
}


// 求矩阵行列式
double GeoBase::Det(double *A,int m)
{
    int i=0,ii=0,j=0,jj=0,k=0,t=0,tt=1;
    double det=1, mk=0;
    double *pA=new double [m*m];
    double *pB=new double [m];
    for (i=0;i<m;i++)
	{
       pB[i]=0;
       for (j=0;j<m;j++)
          pA[i*m+j]=A[i*m+j];
	}
    for (k=0;k<m;k++)
	{
       for (j=k;j<m;j++)
          if (pA[k*m+j])
		  {
             for (i=0;i<m;i++)
			 {
                  pB[i]=pA[i*m+k];
                  pA[i*m+k]=pA[i*m+j];
                  pA[i*m+j]=pB[i];
			 }
             if (j-k)
                  tt=tt*(-1);
             t=t+1;
             break ;
		  } 
       if (t)
	   {
           for(ii=k+1;ii<m;ii++)
		   {
               mk=(-1)*pA[ii*m+k]/pA[k*m+k];
               pA[ii*m + k] = 0;
               for (jj=k+1;jj<m;jj++)
                   pA[ii*m+jj]=pA[ii*m+jj]+mk*pA[k*m+jj];
		   }
           det=det*pA[k*m+k];
           t=0;
	   }
       else
	   {
           det=0;
           break ;
	   }
	}
    det=det*tt;
    delete pA;	pA = NULL;
    delete pB;	pB = NULL;
    return det;
}

// 求A的逆矩阵C 
void GeoBase::Inv(double *A, double *C, int m)
{
     int i,j,x0,y0;
     double M=0;
     double *SP=new double [m*m];
     double *AB=new double [m*m];
     double *B=new double [m*m];
     M=Det(A,m);
     if(M==0.0)
         return;
     M=1/M;
     for(i=0;i<m;i++)
	 {
         for(j=0;j<m;j++)
		 {
            for(x0=0;x0<m;x0++)
                for (y0=0;y0<m;y0++)
                     B[x0*m+y0]=A[x0*m+y0];
            for(x0=0;x0<m;x0++)
                 B[x0*m+j]=0;
            for(y0=0;y0<m;y0++)
                 B[i*m+y0]=0;
            B[i*m+j]=1;
            SP[i*m+j]=Det(B,m);
            SP[i*m+j]=SP[i*m+j];
            AB[i*m+j]=M*SP[i*m+j];
		 }
	 }
     Transpose(AB,C,m,m);
     delete SP;		SP=NULL;
     delete AB;		AB=NULL;
     delete B;		B=NULL;   
}


// 对向量进行归一化
void GeoBase::NormVector(double *R, int num)
{
	double retVal = 0.0;
	for (int i=0; i < num; i++)
		retVal += pow(R[i],2);
	retVal = sqrt( retVal);
	for (int i=0; i < num; i++)
		R[i] /= retVal;
}


// 求取向量的模
double GeoBase::Norm(double *R, int num)
{
	double retVal = 0.0;
	for (int i=0; i<num; i++)
		retVal += pow(R[i],2);
	return sqrt(retVal);
}


// 两个向量点乘
double GeoBase::Dot(double *A, double *B, int num)
{
	double retVal = 0.0;
	for (int i=0; i < num; i++)
		retVal += A[i]*B[i];
	return retVal;
}


// 三阶向量叉乘
void GeoBase::CrossMult(double *u, double *v, double *w)
{
	w[0] = u[1]*v[2] - u[2]*v[1];
	w[1] = u[2]*v[0] - u[0]*v[2];
	w[2] = u[0]*v[1] - u[1]*v[0];
}

bool GeoBase::isExitlter(double * pData, double val, int num)
{
	for (int i = 0; i < num; i++)
		if (fabs(pData[i]) > val)
			return false;

	return true;
}

// 法化
void GeoBase::pNormal(double *a, int n, double b, double *aa, double *ab, double p)
{
	int i,j;
	for(i=0; i<n; i++) 
	{
		for (j=0; j<n; j++) 
			*aa++ += p*a[i]*a[j];
		*ab++ += p*a[i]*b;
	}
}

// 高斯求解
int GeoBase::Gauss(double *ATA,double *ATL,int n)
{
	double *ATAinv = new double[n*n];
	double *temp = new double[n];
	Inv(ATA, ATAinv, n);
	Multi(ATAinv, ATL, temp, n, n, 1);
	memcpy(ATL, temp, sizeof(double)*n);
	delete []ATAinv;	ATAinv = NULL;
	delete []temp;		temp = NULL;
	return 1;
}

// 3*3的高斯求解
bool GeoBase::solve33(double *A, double *al)
{
	double calresult[3];
	if(fabs(A[0])<1e-30)
		return false;
	double L10 = A[3]/A[0];
	double L20 = A[6]/A[0];
	double U11 = A[4]-L10*A[1];
	if(fabs(U11)<1e-30)
		return false;
	double L21 = (A[7]-(A[1]*L20))/U11;
	double U12 = A[5]-L10*A[2];
	double U22 = A[8]-L20*A[2]-L21*U12;
	double b0 = al[0];
	double b1 = al[1]-b0*L10;
	double b2 = al[2]-b0*L20-b1*L21;
	if(fabs(U22)<1e-30)
		return false;
	calresult[2] = b2/U22;
	calresult[1] = (b1-U12*calresult[2])/U11;
	calresult[0] = (b0-A[1]*calresult[1]-A[2]*calresult[2])/A[0];
	memcpy(al,calresult,sizeof(double)*3);
	return true;
}


//王新洲等，谱修正迭代法及其在测量数据处理中的应用
void GeoBase::GaussExt(double *ATA, double *ATL, double *x, int n)
{
	double *ATAinv = new double[n*n];
	long i;
	int num=0;
	for(int i=0;i<n;i++)
	{
		ATA[i*n+i]+=1;
	}
	Inv(ATA, ATAinv, n);	
	double *temp=new double[n];
	double *temp1=new double[n];
	double dx0=1e10,dx=1e10,dxx=0;
	do 
	{
		dx0=dx;
		memcpy(temp1,x,sizeof(double)*n);
		for(i=0;i<n;i++)
			temp[i]=ATL[i]+x[i];        
		Multi(ATAinv, temp, x, n, n, 1);		
		dx=0;
		for(i=0;i<n;i++)
		{
			dx+=(x[i]-temp1[i])*(x[i]-temp1[i]);
		}		
		num++;	
	} while(num<10000&&dx<dx0);;
	delete []temp;		temp = NULL;
	delete []temp1;		temp1 = NULL;
	delete []ATAinv;	ATAinv = NULL; 
}



//////////////////////////////////////
// 多项式拟合的各类操作
///////////////////////////////////////

///////////////////////////////////////
// 多项式拟合
//	在这里,一般x为时间,y为其对应的函数
//	n为观测值个数,order是多项式拟合的阶数
//  p为输出的多项式系数
///////////////////////////////////////
void GeoBase::PolynominalFitting(double *x, double *y, int n, int order, double *p)
{
	long i, j;
	double *a=new double[order];
	double *aa=new double[order*order];
	double *al=new double[order];
	memset(aa,0,sizeof(double)*order*order);
	memset(al,0,sizeof(double)*order);
	for(i=0; i<n; i++)
	{
		a[0] = 1.0;
		for(j=1; j<order; j++)
		{
			a[j]=a[j-1]*x[i];
		}
		pNormal(a, order, y[i], aa, al, 1.0);		
	}
	Gauss(aa, al, order);
	memcpy(p, al, sizeof(double)*order);

	delete []a;a=NULL;
	delete []aa;aa=NULL;
	delete []al;al=NULL;
}

///////////////////////////////////////
// 多项式拟合及其精度
///////////////////////////////////////
void GeoBase::PolynominalFittingError(double *x,double *y,int n,int order,double *error,double *p)
{
	PolynominalFitting(x, y, n, order, p);
    double *a=new double[order];
	long i, j;
	for(i=0;i<n;i++)
	{
		a[0] = 1.0;
		for(j=1; j<order; j++)
		{
			a[j]=a[j-1]*x[i];
		}
		error[i]=0;
		for(j=0; j<order; j++)
		{
			error[i]+=p[j]*a[j];
		}
		error[i]-=y[i];
	}
	delete []a;	a=NULL;
}


///////////////////////////////////////
// 多项式拟合求值
///////////////////////////////////////
void GeoBase::PolyValue(double *p, double *pDelta, double UT, double *value)
{
	long i;
	double order[POLYORDER];
	order[0]=1.0;
	for(i=1;i<POLYORDER;i++)
	{
		order[i]=order[i-1]*UT;
	}
    double Rvalue=0;
	for(i=0;i<POLYORDER;i++)
	{
		Rvalue+=(p[i]+pDelta[i])*order[i];
	}
	*value = Rvalue;
}


///////////////////////////////////////
// 求取归一化系数
///////////////////////////////////////
void GeoBase::Compute_avAnddx(double *a,int num,double &av,double &dx)
{
	int i;
	av=0;
	double amax=a[0],amin=a[0];
	for(i=0;i<num;i++)
	{
		av+=a[i];
		if(a[i]>amax) amax=a[i];
		if(a[i]<amin) amin=a[i];	
	}
	av /= num;
	dx = max(fabs(amax-av),fabs(amin-av));
}


///////////////////////////////////////
// 进行归一化
///////////////////////////////////////
void GeoBase::Normaliza_avAnddx(double *a, int num, double av, double dx)
{
	for(int i=0;i<num;i++)
	{
		a[i] = (a[i]-av)/dx;
	}
}

///////////////////////////////////////
// 进行反归一化
///////////////////////////////////////
void GeoBase::DNormaliza_avAnddx(double *a, int num, double av, double dx)
{
	for(int i=0;i<num;i++)
	{
		a[i] = a[i]*dx+av;
	}
}

///////////////////////////////////////
// 计算绝对值平均值
///////////////////////////////////////
double GeoBase::FabsAndAve(double *a, int num)
{
	double ave = 0;
	for(int i=0;i<num;i++)
	{
		ave += fabs(a[i]);
	}
	return (ave /= num);
}



// 产生一个符合正态分布的随机数
double GeoBase::GaussRand(double mean, double sigma)
{    
	static double v1, v2, s;   
	static int phase  = 0;    
	double x;    
	if (0 == phase)    
	{      
		do        
		{           
			double u1 = (double)rand()/RAND_MAX;      
			double u2 = (double)rand()/RAND_MAX;    
			v1 = 2 * u1 - 1;          
			v2 = 2 * u2 - 1;       
			s = v1 * v1 + v2 * v2;       
		} while ( 1 <= s || 0 == s);     
		x = v1 * sqrt(-2 * log(s) / s);  
	}   
	else    
	{      
		x = v2 * sqrt(-2 * log(s) / s);  
	}    
	phase = 1 - phase;   
	return (x*sigma+mean);    // 注意要加括号,否则返回值就是sigma了
}


// 产生一个符合正态分布的随机数
double GeoBase::GaussRand2(double mean, double sigma)
{    
	double v1, v2, s;   
	int phase  = 0;    
	double x;    
	if (0 == phase)    
	{      
		do        
		{           
			double u1 = (double)rand()/RAND_MAX;      
			double u2 = (double)rand()/RAND_MAX;    
			v1 = 2 * u1 - 1;          
			v2 = 2 * u2 - 1;       
			s = v1 * v1 + v2 * v2;       
		} while ( 1 <= s || 0 == s);     
		x = v1 * sqrt(-2 * log(s) / s);  
	}   
	else    
	{      
		x = v2 * sqrt(-2 * log(s) / s);  
	}    
	phase = 1 - phase;   
	return (x*sigma+mean);    // 注意要加括号,否则返回值就是sigma了
}

#include <time.h>
////////////////////////////////////////////////////////
// 生成随机误差
////////////////////////////////////////////////////////
void GeoBase::RandomDistribution(double mean, double sigma, int n, long randCount, double *a)
{
	// TODO: Add your implementation code here
	static unsigned int timetemp = 0;
	int extend = n/100;
	if(extend<20)	
		extend=20;
	int ntemp = n + extend;
	double *tempa = new double[ntemp];
	double mintemp = mean-2*sigma;
	double maxtemp = mean+2*sigma;
	if(randCount == 0)
	{
		// 注意要使用time(NULL),而不是GetTickcount
		// GetTickcount函数：它返回从操作系统启动到当前所经过的毫秒数，
		// 常常用来判断某个方法执行的时间，其函数原型是DWORD GetTickCount(void)，
		// 返回值以32位的双字类型DWORD存储，因此可以存储的最大值是2^32 ms约为49.71天，
		// 因此若系统运行时间超过49.71天时，这个数就会归0，MSDN中也明确的提到了:
		// "Retrieves the number of milliseconds that have elapsed since the 
		// system was started, up to 49.7 days."。因此，如果是编写服务器端程序，
		// 此处一定要万分注意，避免引起意外的状况。
		srand(time(NULL)+timetemp);  
		// 产生随机数
		for (int i=0; i<ntemp; i++)	
		{
			tempa[i] = GaussRand(mean, sigma);
			if((tempa[i]<mintemp)||(tempa[i]>maxtemp))
				i--;
		}
		if (fabs(a[0]) > 1)
		{			timetemp = (unsigned int)fabs(a[0]);		}
		else		
		{			timetemp = (unsigned int)1. / fabs(a[0]);		}
	}
	else
	{
		srand(randCount);
		// 产生随机数
		for (int i=0; i<ntemp; i++)	
		{
			tempa[i] = GaussRand2(mean, sigma);
			if((tempa[i]<mintemp)||(tempa[i]>maxtemp))
				i--;
		}
	}
	
	if(n>1)
	{
/*		// 让其变化不突兀
		double diffvalue, diffvaluetemp, temptemp;
		for(int i=1; i<n; i++)
		{
			diffvalue = fabs(tempa[i]-tempa[i-1]);
			for(int j=i+1; j<i+extend; j+=2)
			{
				diffvaluetemp = fabs(tempa[j]-tempa[i-1]);
				if(diffvaluetemp<diffvalue)
				{
					temptemp = tempa[i];
					tempa[i] = tempa[j];
					tempa[j] = temptemp;
					diffvalue = diffvaluetemp;
				}
			}
		}*/

		// 求取平均值
		double meantemp=0;
		for (int i=0; i<n; i++)
		{	
			meantemp+=tempa[i];	
		}
		meantemp/=n;
		// 对平均值进行修正
		double div = meantemp - mean;
		for (int i=0; i<n; i++)	
		{
			a[i] = tempa[i] - div;
		}
		// 释放内存
		if(tempa!=NULL)	delete []tempa;	tempa=NULL;
	}
	return;
}



/////////////////////////////////////////////////
// 从轨道六根数转化到轨道位置和速度
// 输入：
//		double a：				轨道高度
//		double e：				轨道偏心率
//		double i：				轨道倾角
//		double omega：			近地点幅角
//		double AscNode：		升交点赤经
//		double f：				真近点角/偏近点角
//		double istrueAnomaly:	真近点角/偏近点角标记
// 输出：
//		double *X：				轨道位置和速度
/////////////////////////////////////////////////
void GeoBase::OrbitEle2PosAndVel(double a, double e, double i, double omega, double AscNode, double f, double *X, bool istrueAnomaly)
{
	// 求取偏近点角
	double E;
	if(istrueAnomaly)
		E = 2*atan(sqrt((1-e)/(1+e))*tan(f/2));  
	else
		E = f;
	// 求取重力场阶数
	double n = sqrt(GM/pow(a, 3));
	double e2 = e*e;
	// 计算在轨道坐标系下的位置和速度
	double Rorbit[3], Vorbit[3], RJ2000[3], VJ2000[3];
	Rorbit[0] = a*(cos(E)-e);
	Rorbit[1] = a*(sin(E)*sqrt(1-e2));
	Rorbit[2] = 0;
	double temp = a*n/(1-e*cos(E));
	Vorbit[0] = -sin(E)*temp;
	Vorbit[1] = sqrt(1-e2)*cos(E)*temp;
	Vorbit[2] = 0;
	// 将轨道坐标系下的位置和速度转化到地心惯性系下
	double Rtemp1[9];
	Eulor2Matrix(-omega, -i, -AscNode, 313, Rtemp1);
	Multi(Rtemp1, Rorbit, RJ2000, 3, 3, 1);
	Multi(Rtemp1, Vorbit, VJ2000, 3, 3, 1);
	X[0] = RJ2000[0];	X[1] = RJ2000[1];	X[2] = RJ2000[2];
	X[3] = VJ2000[0];	X[4] = VJ2000[1];	X[5] = VJ2000[2];
}



/////////////////////////////////////////////////
// 从轨道位置和速度转化到轨道六根数（注意：暂时没考虑圆轨道、赤道轨道的情况）
// 输入：
//		double *X：				轨道位置和速度
//		double istrueAnomaly:	真近点角/偏近点角标记
// 输出：
//		double &a：				轨道高度
//		double &e：				轨道偏心率
//		double &i：				轨道倾角
//		double &omega：			近地点幅角
//		double &AscNode：		升交点赤经
//		double &f：				真近点角
// TestData
//		para[0] = 7008305.566631538800000;
//		para[1] = 0.001375159581061;
//		para[2] = 1.709373432680644;
//		para[3] = -0.998007301607373;
//		para[4] = 1.373884882951905;
//		para[5] = 2.968097348769756;
//		m_base.OrbitEle2PosAndVel(para[0], para[1], para[2], para[3], para[4], para[5], X);
//		m_base.PosAndVel2OrbitEle(X, para[0], para[1], para[2], para[3], para[4], para[5]);
//		342113.128047912560000    -2850305.537096649400000    6403764.992227509600000     
//		-1754.087741455078200     -6726.432356689453600       -2898.252371215820400
/////////////////////////////////////////////////
void GeoBase::PosAndVel2OrbitEle(double *X, double &a, double &e, double &i, double &omega, double &AscNode, double &f, bool istrueAnomaly)
{
	double X0[6];
	for(int j=0; j<6; j++)
		X0[j] = X[j]/1000;
	double eps = 1.e-10;								// 接近圆的阈值
	double r = Norm(&(X0[0]), 3);						// 位置矢量大小
	double v = Norm(&(X0[3]), 3);						// 速度矢量大小
	double vr = Dot(&(X0[0]), &(X0[3]), 3)/r;			// 径向速度分量
	double H[3];
	CrossMult(&(X0[0]), &(X0[3]), H);					// 角动量矢量
	double h = Norm(H, 3);								// 角动量大小
	i = acos(H[2]/h);									// 轨道倾角大小
	double N0[3], N[3];
	N0[0] = N0[1] = 0.0;		N0[2] = 1.0;
	CrossMult(N0, H, N);								// 升交点赤经矢量
	double n = Norm(N, 3);
	// 计算升交点赤经
	AscNode = 0.0;
	if(n!=0)
	{
		AscNode = acos(N[0]/n);
		if(N[1]<0)	AscNode = 2*PI - AscNode;
	}
	// 计算偏心率
	double E[3], temp1, temp2;
	temp1 = v*v/GM0-1/r;
	temp2 = r*vr/GM0;
	E[0] = temp1*X0[0] - temp2*X0[3];
	E[1] = temp1*X0[1] - temp2*X0[4];
	E[2] = temp1*X0[2] - temp2*X0[5];
	e = Norm(E, 3);
	// 计算近地点角距
	omega = 0.0;
	if((n!=0)&&(e>eps))
	{
		omega = acos(Dot(N, E, 3)/n/e);
		if(E[2]<0)	omega = 2*PI - omega;
	}
	// 计算真近点角
	double cp[3];
	if(e>eps)
	{
		f = acos(Dot(E, X0, 3)/e/r);
		if(vr<0)	f = 2*PI - f;
	}
	else
	{
		CrossMult(N, X0, cp);
		if(cp[2]>=0)
			f = acos(Dot(N, X0, 3)/n/r);
		else
			f = 2*PI - acos(Dot(N, X0, 3)/n/r);
	}
	// 计算长半轴
	a = h/GM0*(h/(1-e*e))*1000;  
	// 输出偏近点角
	if(!istrueAnomaly)
	{
		double E0 = 2*atan(sqrt((1-e)/(1+e))*tan(f/2));
		if(E0*f<0&&E0<0)
			E0 = 2*PI + E0;
		if(E0*f<0&&E0>0)
			E0 = 2*PI - E0;
		f = E0;
	}
}


/////////////////////////////////////////////////
// 偏近点角和平近点角转化
/////////////////////////////////////////////////
void GeoBase::TransMandE(double &M, double &E, double e, bool isM2E)
{
	// 从平近点角到偏近点角
	if(isM2E)
	{
		double dE;
		E = M;
		int count = 0;
		while(1)
		{
			dE = (M-E+e*sin(E))/(1-e*cos(E));
			E += dE;
			count++;
			if((fabs(dE)<10e-14)||count>100) break;
		}
	}
	// 从偏近点角到平近点角
	else
		M = E - e*sin(E);
}



/////////////////////////////////////////////////
// 有理函数模型的次数验证
/////////////////////////////////////////////////
void GeoBase::RFMdegreeTest1(int degree, string outfile)
{
	int step = 200;
	///////////////////////////////////////////
	// 根据次数来设置正弦函数
	///////////////////////////////////////////
	double *x = new double[step];
	double *y = new double[step];
	double *z = new double[step];
	for(int i=0; i<step; i++)
	{
		x[i] = i*PI/step;
		y[i] = sin((degree-1)*x[i]);
	}

	///////////////////////////////////////////
	// 根据正弦函数获得多项式系数
	///////////////////////////////////////////
	int num = degree+1;			// 系数个数
	double *X = new double[num];
	double *A = new double[num];
	double L;
	double *ATA = new double[num*num];
	double *ATL = new double[num];
	memset(ATA, 0, sizeof(double)*num*num);
	memset(ATL, 0, sizeof(double)*num);
	for(int i=0; i<num; i++)
		X[i] = 1.0;
	for(int i=0; i<step; i++)
	{
		x[i] = (double)i/step;
		A[0] = 1.0;
		for(int j=1; j<num; j++)
		{
			A[j] = A[j-1]*x[i];
		}
		L = y[i];
		pNormal(A, num, L, ATA, ATL, 1.0);
	}
	Gauss(ATA, ATL, num);
	memcpy(X, ATL, sizeof(double)*num);

	///////////////////////////////////////////
	// 根据多项式系数获取采样点
	///////////////////////////////////////////
	for(int i=0; i<step; i++)
	{
		x[i] = (double)i/step;
		L = X[0];
		for(int j=1; j<num; j++)
		{
			L += X[j]*pow(x[i], j);
		}
		y[i] = L;
	}

	///////////////////////////////////////////
	// 根据采样点获取有理函数式
	///////////////////////////////////////////
	double para[7];
	if(X!=NULL)		delete []X;		X = NULL;
	if(A!=NULL)		delete []A;		A = NULL;
	if(ATA!=NULL)	delete []ATA;	ATA = NULL;
	if(ATL!=NULL)	delete []ATL;	ATL = NULL;
	X = new double[7];
	A = new double[7];
	ATA = new double[7*7];
	ATL = new double[7];
	memset(ATA, 0, sizeof(double)*7*7);
	memset(ATL, 0, sizeof(double)*7);
	memset(X, 0, sizeof(double)*7);
	memset(para, 0, sizeof(double)*7);
	////////////////////
	// 求解一阶形式的
	////////////////////
	for(int i=0; i<step; i++)
	{
		A[0] = x[i];
		A[1] = 1;
		A[2] = -y[i]*x[i];
		L = y[i];
		pNormal(A, 3, L, ATA, ATL, 1.0);
	}
	Gauss(ATA, ATL, 3);
	para[2] = ATL[0];
	para[3] = ATL[1];
	para[6] = ATL[2];
	
	///////////////////////////////////////////
	// 根据有理函数结果获取对应点数值
	///////////////////////////////////////////
	for(int i=0; i<step; i++)
	{
		z[i] = (para[0]*pow(x[i], 3.0)+para[1]*pow(x[i], 2.0)+para[2]*x[i]+para[3])/
			   (para[4]*pow(x[i], 3.0)+para[5]*pow(x[i], 2.0)+para[6]*x[i]+1.0);
	}

	///////////////////////////////////////////
	// 输出结果
	///////////////////////////////////////////
	FILE *fp = fopen(outfile.c_str(), "w");
	for(int i=0; i<step; i++)
	{
		fprintf(fp, "%19.16lf\t%19.16lf\t%19.16lf\n", x[i], y[i], z[i]);
	}
	fclose(fp);		fp = NULL;

	fp = fopen(outfile.c_str(), "w");
	para[0] = 0.0;
	para[1] = 0.0;
	para[2] = 0.5;
	para[3] = 1.0;
	para[4] = 0.0;
	para[5] = 0.0;
	para[6] = 1.0;
	double temp;
	for(double i=-10; i<10; i+=0.1)
	{
		temp = (para[0]*pow(i, 3.0)+para[1]*pow(i, 2.0)+para[2]*i+para[3])/
			   (para[4]*pow(i, 3.0)+para[5]*pow(i, 2.0)+para[6]*i+1.0);
		fprintf(fp, "%19.16lf\t%19.16lf\n", i, temp);
	}
	fclose(fp);		fp = NULL;

	///////////////////////////////////////////
	// 释放内存
	///////////////////////////////////////////
	if(X!=NULL)		delete []X;		X = NULL;
	if(A!=NULL)		delete []A;		A = NULL;
	if(ATA!=NULL)	delete []ATA;	ATA = NULL;
	if(ATL!=NULL)	delete []ATL;	ATL = NULL;
	if(x!=NULL)		delete []x;		x = NULL;
	if(y!=NULL)		delete []y;		y = NULL;
	if(z!=NULL)		delete []z;		z = NULL;
}


/////////////////////////////////////////////////
// 有理函数模型的次数验证
/////////////////////////////////////////////////
void GeoBase::RFMdegreeTest(int degree, string outfile)
{
	int step = 200;
	///////////////////////////////////////////
	// 根据次数来设置正弦函数
	///////////////////////////////////////////
	double *x = new double[step];
	double *y = new double[step];
	double *z = new double[step];
	for(int i=0; i<step; i++)
	{
		x[i] = i*PI/step;
		y[i] = sin((degree-1)*x[i]);
	}

	///////////////////////////////////////////
	// 根据正弦函数获得多项式系数
	///////////////////////////////////////////
	int num = degree+1;			// 系数个数
	double *X = new double[num];
	double *A = new double[num];
	double L;
	double *ATA = new double[num*num];
	double *ATL = new double[num];
	memset(ATA, 0, sizeof(double)*num*num);
	memset(ATL, 0, sizeof(double)*num);
	for(int i=0; i<num; i++)
		X[i] = 1.0;
	for(int i=0; i<step; i++)
	{
		x[i] = (double)i/step;
		A[0] = 1.0;
		for(int j=1; j<num; j++)
		{
			A[j] = A[j-1]*x[i];
		}
		L = y[i];
		pNormal(A, num, L, ATA, ATL, 1.0);
	}
	Gauss(ATA, ATL, num);
	memcpy(X, ATL, sizeof(double)*num);

	///////////////////////////////////////////
	// 根据多项式系数获取采样点
	///////////////////////////////////////////
	for(int i=0; i<step; i++)
	{
		x[i] = (double)i/step;
		L = X[0];
		for(int j=1; j<num; j++)
		{
			L += X[j]*pow(x[i], j);
		}
		y[i] = L;
	}

	///////////////////////////////////////////
	// 根据采样点获取有理函数式
	///////////////////////////////////////////
	double para[7];
	if(X!=NULL)		delete []X;		X = NULL;
	if(A!=NULL)		delete []A;		A = NULL;
	if(ATA!=NULL)	delete []ATA;	ATA = NULL;
	if(ATL!=NULL)	delete []ATL;	ATL = NULL;
	X = new double[7];
	A = new double[7];
	ATA = new double[7*7];
	ATL = new double[7];
	memset(ATA, 0, sizeof(double)*7*7);
	memset(ATL, 0, sizeof(double)*7);
	memset(X, 0, sizeof(double)*7);
	memset(para, 0, sizeof(double)*7);
	////////////////////
	// 求解一阶形式的
	////////////////////
	for(int i=0; i<step; i++)
	{
		A[0] = x[i];
		A[1] = 1;
		A[2] = -y[i]*x[i];
		L = y[i];
		pNormal(A, 3, L, ATA, ATL, 1.0);
	}
	Gauss(ATA, ATL, 3);
	para[2] = ATL[0];
	para[3] = ATL[1];
	para[6] = ATL[2];
	
	///////////////////////////////////////////
	// 根据有理函数结果获取对应点数值
	///////////////////////////////////////////
	for(int i=0; i<step; i++)
	{
		z[i] = (para[0]*pow(x[i], 3.0)+para[1]*pow(x[i], 2.0)+para[2]*x[i]+para[3])/
			   (para[4]*pow(x[i], 3.0)+para[5]*pow(x[i], 2.0)+para[6]*x[i]+1.0);
	}

	///////////////////////////////////////////
	// 输出结果
	///////////////////////////////////////////
	FILE *fp = fopen(outfile.c_str(), "w");
	for(int i=0; i<step; i++)
	{
		fprintf(fp, "%19.16lf\t%19.16lf\t%19.16lf\n", x[i], y[i], z[i]);
	}
	fclose(fp);		fp = NULL;

	fp = fopen(outfile.c_str(), "w");
	para[0] = -0.4;
	para[1] = -0.8;
	para[2] = -0.5;
	para[3] = 1.0;
	para[4] = 0.02;
	para[5] = 1.0;
	para[6] = 0.02;
	double temp;
	for(double i=-5; i<5; i+=0.1)
	{
		temp = (para[0]*pow(i, 3.0)+para[1]*pow(i, 2.0)+para[2]*i+para[3])/
			   (para[4]*pow(i, 3.0)+para[5]*pow(i, 2.0)+para[6]*i+1.0);
		fprintf(fp, "%19.16lf\t%19.16lf\n", i, temp);
	}
	fclose(fp);		fp = NULL;

	///////////////////////////////////////////
	// 释放内存
	///////////////////////////////////////////
	if(X!=NULL)		delete []X;		X = NULL;
	if(A!=NULL)		delete []A;		A = NULL;
	if(ATA!=NULL)	delete []ATA;	ATA = NULL;
	if(ATL!=NULL)	delete []ATL;	ATL = NULL;
	if(x!=NULL)		delete []x;		x = NULL;
	if(y!=NULL)		delete []y;		y = NULL;
	if(z!=NULL)		delete []z;		z = NULL;
}


//////////////////////////////////////////////////////////////////////////
// L0和L1范数的若干求解
//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////
// 求取一个矩阵的范数
// 输入:
//		double *A：输入的矩阵
//		int m:     行数/列数
//		int index: 范数类型。0为无穷范数，1为1范数，2为2范数，3为F范数，默认为1
// 返回值:
//		double:	   范数
///////////////////////////////////////////
double GeoBase::Mat_Norm(double *A, int m, int index)
{
	////////////////////
	// 无穷范数的情况
	// 行和范数，即所有矩阵行向量绝对值之和的最大值
	////////////////////
	if(index==0)
	{
		double max0, max1;
		max0 = -1;
		for(int i=0; i<m; i++)
		{
			max1 = 0.0;
			for(int j=0; j<m; j++)
			{
				max1 += fabs(A[i*m+j]);
			}
			if(max1>max0)
				max0=max1;
		}
		return max0;
	}
	////////////////////
	// 1范数的情况
	//  列和范数，即所有矩阵列向量绝对值之和的最大值
	////////////////////
	else if(index==1)
	{
		double max0, max1;
		max0 = -1;
		for(int j=0; j<m; j++)
		{
			max1 = 0.0;
			for(int i=0; i<m; i++)
			{
				max1 += fabs(A[i*m+j]);
			}
			if(max1>max0)
				max0=max1;
		}
		return max0;
	}
	////////////////////
	// 2范数的情况
	// 谱范数，即A'A矩阵的最大特征值的开平方
	////////////////////
	else if(index==2)
	{
		double *AT = new double[m*m];
		double *ATA = new double[m*m];
		double *ATAval = new double[m*m];
		double *EigVal = new double[m];
		Transpose(A, AT, m, m);
		Multi(AT, A, ATA, m, m, m);
		JacbiCor(ATA, m, ATAval, EigVal, 10e-8, 100);
		double value = 0.0;
		for(int i=0; i<m; i++)
			if(EigVal[i]>value)	value = EigVal[i];
		if(AT!=NULL)		delete []AT;		AT = NULL;
		if(ATA!=NULL)		delete []ATA;		ATA = NULL;
		if(ATAval!=NULL)	delete []ATAval;	ATAval = NULL;
		if(EigVal!=NULL)	delete []EigVal;	EigVal = NULL;
		return sqrt(value);
	}
	////////////////////
	// F范数的情况
	// Frobenius范数，即矩阵元素绝对值的平方和再开平方
	////////////////////
	else
	{
		double value;
		value = 0;
		for(int j=0; j<m; j++)
		{
			for(int i=0; i<m; i++)
			{
				value += pow(A[i*m+j], 2.0);
			}
		}
		return value;
	}
}


///////////////////////////////////////////
// 求取一个矩阵的条件数
// 输入:
//		double *A：输入的矩阵
//		int m:     阶数
//		int index: 范数类型。0为无穷范数，1为1范数，2为2范数，3为F范数，默认为1
// 返回值:
//		double:	   范数
/////////////////////////////////////////// 
double GeoBase::Mat_Cov(double *A, int m, int index)
{
	double *Ainv = new double[m*m];
	Inv(A, Ainv, m);
	double temp1 = Mat_Norm(A, m, index);
	double temp2 = Mat_Norm(Ainv, m, index);
	if(Ainv!=NULL)	delete []Ainv;	Ainv = NULL;
	return temp1*temp2;
}


///////////////////////////////////////////
// 求实对称矩阵的特征值及特征向量的雅克比法
// 利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量 
// 输入:
//		double *A:			长度为n*n的数组，存放实对称矩阵
//		int n:				矩阵的阶数
//		double *pVec:		长度为n*n的数组，返回特征向量(按列存储) 
//		double *pEigVal:	特征值数组
//		double eps:			精度要求
//		int itenum：		迭代次数
// 返回值:
//		bool:	   
///////////////////////////////////////////
bool GeoBase::JacbiCor(double *A, int n, double *pVec, double *pEigVal, double eps, int itenum)
{
	double *At = new double[n*n];
	memcpy(At, A, sizeof(double)*n*n);
	// 初始化特征向量
	for(int i=0; i<n; i++) 
	{   
		pVec[i*n+i] = 1.0f; 
		for(int j=0; j<n; j++) 
		{ 
			if(i!=j)   
				pVec[i*n+j]=0.0f; 
		} 
	} 
	int count = 0;		//迭代次数
	while(1)
	{
		//在A的非对角线上找到最大元素
		double max = A[1];
		int row = 0;
		int col = 1;
		for (int i=0; i<n; i++)			//行
		{
			for (int j=0; j<n; j++)		//列
			{
				double d = fabs(A[i*n+j]); 
				if((i!=j) && (d>max)) 
				{ 
					max = d;   
					row = i;   
					col = j; 
				} 
			}
		}
		if(max<eps)				//精度符合要求 
			break;  
		if(count>itenum)		//迭代次数超过限制
			break;
		count++;
		double pp = A[row*n + row];
		double pq = A[row*n + col];
		double qq = A[col*n + col];
		//计算旋转角度
		double angle = 0.5*atan2(-2*pq, qq-pp);
		double st = sin(angle);
		double ct = cos(angle);
		double s2t = sin(2*angle);
		double c2t = cos(2*angle);
		A[row*n + row] = pp*ct*ct + qq*st*st + 2*pq*ct*st;
		A[col*n + col] = pp*st*st + qq*ct*ct - 2*pq*ct*st;
		A[row*n + col] = 0.5*(qq-pp)*s2t + pq*c2t;
		A[col*n + row] = A[row*n + col];
		for(int i=0; i<n; i++) 
		{ 
			if((i!=col)&&(i!=row)) 
			{ 
				int u = i*n + row;		//p  
				int w = i*n + col;		//q
				max = A[u]; 
				A[u]= A[w]*st + max*ct; 
				A[w]= A[w]*ct - max*st; 
			} 
		} 
		for(int j= 0; j<n; j++)
		{
			if((j!=col)&&(j!=row)) 
			{ 
				int u = row*n + j;		//p
				int w = col*n + j;		//q
				max = A[u]; 
				A[u]= A[w]*st + max*ct; 
				A[w]= A[w]*ct - max*st; 
			} 
		}
		//计算特征向量
		for(int i=0; i<n; i++) 
		{ 
			int u = i*n + row;		//p   
			int w = i*n + col;		//q
			max = pVec[u]; 
			pVec[u] = pVec[w]*st + max*ct; 
			pVec[w] = pVec[w]*ct - max*st; 
		} 
	}
	// 特征值即pMatrix主对角线上的元素
	for(int i=0; i<n; i++) 
	{   
		pEigVal[i] = A[i*n+i];
	} 
	//设定正负号
	for(int i=0; i<n; i++) 
	{
		double dSumVec = 0;
		for(int j=0; j<n; j++)
			dSumVec += pVec[j*n + i];
		if(dSumVec<0)
		{
			for(int j=0; j<n; j++)
				pVec[j*n + i] *= -1;
		}
	}
	// 内存操作
	memcpy(A, At, sizeof(double)*n*n);
	if(At!=NULL)		delete []At;		At = NULL;
	return 1;
}


///////////////////////////////////////////
// 岭估计法(L曲线求解)
///////////////////////////////////////////
void GeoBase::GaussL(double *ATA,double *ATL,int n, double LTL)
{
	double para = L_curve(ATA, ATL, n, LTL);
	for(int i=0;i<n;i++)
	{
		ATA[i*n+i] += para;
	}
	Gauss(ATA, ATL, n);
}


///////////////////////////////////////////
// L曲线函数
///////////////////////////////////////////
double GeoBase::L_curve(double *ATA, double *ATL, int n, double LTL)
{
	double reg_c;
	double eps = 2.2204e-016;
	double smin_ratio;
	int npoints = 200;	
	smin_ratio  = 16*eps;
	double *pEigVal = new double[n];
	double *ATAex = new double[n*n];
	// 求取特征值, 并获得最大最小值
	JacbiCor(ATA, n, ATAex, pEigVal, 0.0000001, 100);
	double max_eig,min_eig;
	max_eig = sqrt(pEigVal[0]);
	min_eig = sqrt(pEigVal[0]);
	for(int i = 1;i<n;i++)
	{
		max_eig = max(max_eig, sqrt(pEigVal[i]));
		min_eig = min(min_eig, sqrt(pEigVal[i]));
	}
	// 构建各个离散点
	double *para = new double[npoints];
	double *g = new double[npoints];
	memset(para, 0, npoints*sizeof(double));
	para[npoints-1] = max(min_eig, smin_ratio*max_eig);
	long double ratio = pow((max_eig/para[npoints-1]), double(1)/(npoints-1));
	for(int i=npoints-2; i>=0; i--)
	{
		para[i] = ratio*para[i+1];
	}	
	// 寻找曲率变化最大的点
	lcfun(para, npoints, ATA, ATL, n, LTL, g);	
	int gi;
	minvector(g, npoints, gi);
	double reg_max;
	double reg_min;
	double epsreg = 0.00000001;
	reg_min = para[min((gi+1), (npoints-1))];
	reg_max = para[max((gi-1), 0)];
	reg_c = fminbnd(reg_min, reg_max, ATA, ATL, n, LTL, epsreg);
	// 释放内存
	if(pEigVal!=NULL)	delete []pEigVal;	pEigVal = NULL;
	if(ATAex!=NULL)		delete []ATAex;		ATAex = NULL;
	if(para!=NULL)		delete []para;		para = NULL;
	delete []g;
	return reg_c;
}

///////////////////////////////////////////
// 以下为L曲线所使用的函数
///////////////////////////////////////////
double GeoBase::fminbnd(double reg_min,double reg_max,double *ata,double *atl,int n ,double ltl,double eps)
{
	int i;
	int npoints = 200;
	bool ISOVER = true;
	double reg_corner ;
	double ratio;
	double *reg_param = new double[npoints];
	double *g = new double[npoints];
	memset(reg_param,0,npoints*sizeof(double));	
	do {
		ratio = (reg_max - reg_min)/(npoints-1);
		for ( i = 0;i<npoints;i++)
		{
			reg_param[i] = reg_min + ratio * i ;
		}
		lcfun(reg_param,npoints,ata,atl,n,ltl,g);
		int gi;
		minvector(g,npoints,gi);
		reg_min = reg_param[max((gi - 1),0)];
		reg_max = reg_param[min((gi + 1),(npoints -1))];	 
		if ((reg_max - reg_min)<eps)
		{
			ISOVER = false;
			reg_corner = (reg_min  + reg_max)/2;
		}
	} while(ISOVER);
	delete []reg_param;
	delete []g;
	return reg_corner;
} 

void GeoBase::lcfun(double *lambda,int nump,double *ata,double *atl,int n ,double ltl,double *g)
{
	double eta;
	double rho;
	double deta;
	double drho;
	double ddrho;
	double ddeta;
	double dlogeta;
	double dlogrho;
	double ddlogeta;
	double ddlogrho;
	for (int i=0;i<nump;i++)
	{
		lamda2etarho(lambda[i],ata,atl,n,ltl,eta,rho);
		lamda2detadrho(lambda[i],ata,atl,n,ltl,deta,drho);
		lamda2ddetaddrho(lambda[i],ata,atl,n,ltl,ddeta,ddrho);
		dlogeta = deta/eta;
		dlogrho = drho/rho;
		ddlogeta = ddeta/eta - dlogeta*dlogeta;
		ddlogrho = ddrho/rho - dlogrho*dlogrho;
		g[i] = -1.0*(dlogrho*ddlogeta - ddlogrho*dlogeta)/pow((dlogeta*dlogeta+dlogrho*dlogrho),1.5); 
	}
}

void GeoBase::minvector(double *vec,int m,int &mini)
{
	double minm;
	minm = vec[0];
	mini = 0;
	for (int i = 0; i<m; i++)
	{
		if (minm>vec[i])
		{
			minm = vec[i];
			mini = i;
		}
	}
}

void GeoBase::lamda2etarho(double lambda,double *ata,double *atl,int n,double ltl,double &eta,double &rho)
{
	double *ataex = new double[n*n];
	double *x = new double[n];
	double *xtata = new double[n];
	double xtatax[1];
	double xtatl[1];
	memcpy(ataex,ata,n*n*sizeof(double));
	for (int i = 0;i<n;i++)
	{
		ataex[i*n + i] += lambda;
		x[i] = 0.90;
	}
	invers_matrix(ataex,n);
    trmul(ataex,atl,n,n,1,x);
	eta = vectornorm(x,n);
	trmul(x,ata,1,n,n,xtata);
	trmul(xtata,x,1,n,1,xtatax);
	trmul(x,atl,1,n,1,xtatl);
    rho = xtatax[0] + ltl -2*xtatl[0];
	delete []ataex;
	delete []x;
	delete []xtata;
}

double GeoBase::vectornorm(double *a,int m)
{
	double sum = 0;
	for (int i= 0;i<m;i++)
	{
		sum +=a[i];
	}
	return sqrt(sum);
}

void GeoBase::lamda2detadrho(double lambda,double *ata,double *atl,int n,double ltl,double &deta,double &drho)
{
	double lambda1 ,lambda2;
	double ratio; 
	ratio = 0.001;
	lambda1 = lambda - ratio/2;
	lambda2 = lambda + ratio/2; 
	double eta[2],rho[2]; 
	lamda2etarho(lambda1,ata,atl,n,ltl,eta[0],rho[0]);
	lamda2etarho(lambda2,ata,atl,n,ltl,eta[1],rho[1]); 
	deta = (eta[1]-eta[0])/ratio;
	drho = (rho[1]-rho[0])/ratio;
}

void GeoBase::lamda2ddetaddrho(double lambda,double *ata,double *atl,int n,double ltl,double &ddeta,double &ddrho)
{
	double lambda1 ,lambda2;
	double ratio; 
	ratio = 0.001;
	lambda1 = lambda - ratio/2;
	lambda2 = lambda + ratio/2;
	double deta[2],drho[2];	 
	lamda2detadrho(lambda1,ata,atl,n,ltl,deta[0],drho[0]);
	lamda2detadrho(lambda2,ata,atl,n,ltl,deta[1],drho[1]); 
	ddeta = (deta[1]-deta[0])/ratio;
	ddrho = (drho[1]-drho[0])/ratio;
}

void GeoBase::trmul(double *a, double *b, int m, int n,int k,double *c)
{ 
	int i,j,l,u;
	for (i=0; i<=m-1; i++)
		for (j=0; j<=k-1; j++)
		{ u=i*k+j; c[u]=0.0;
	for (l=0; l<=n-1; l++)
		c[u]=c[u]+a[i*n+l]*b[l*k+j];
	}
	return;
}

int GeoBase::invers_matrix(double *m1, int n)
{ 
	int *is,*js;
	int i,j,k,l,u,v;
	double temp,max_v;
	is=(int *)malloc(n*sizeof(int));
	js=(int *)malloc(n*sizeof(int));
	if(is==NULL||js==NULL)
	{  
		printf("out of memory!\n");  
		return(0);
	}
	for(k=0;k<n;k++){
		max_v=0.0;
		for(i=k;i<n;i++)
			for(j=k;j<n;j++){
				temp=fabs(m1[i*n+j]);
				if(temp>max_v){
					max_v=temp; is[k]=i; js[k]=j;
				}
			}
			if(max_v==0.0){
				free(is); free(js);
				printf("invers is not availble!\n");
				return(0);
			}
			if(is[k]!=k)
				for(j=0;j<n;j++){
					u=k*n+j; v=is[k]*n+j;
					temp=m1[u]; m1[u]=m1[v]; m1[v]=temp;
				}
				if(js[k]!=k)
					for(i=0;i<n;i++){
						u=i*n+k; v=i*n+js[k];
						temp=m1[u]; m1[u]=m1[v]; m1[v]=temp;
					}
					l=k*n+k;
					m1[l]=1.0/m1[l];
					for(j=0;j<n;j++)
						if(j!=k){
							u=k*n+j;
							m1[u]*=m1[l];
						}
						for(i=0;i<n;i++)
							if(i!=k)
								for(j=0;j<n;j++)
									if(j!=k){
										u=i*n+j;
										m1[u]-=m1[i*n+k]*m1[k*n+j];
									}
									for(i=0;i<n;i++)
										if(i!=k){
											u=i*n+k;
											m1[u]*=-m1[l];
										}
	}
	for(k=n-1;k>=0;k--){
		if(js[k]!=k)
			for(j=0;j<n;j++){
				u=k*n+j; v=js[k]*n+j;
				temp=m1[u]; m1[u]=m1[v]; m1[v]=temp;
			}
			if(is[k]!=k)
				for(i=0;i<n;i++){
					u=i*n+k; v=i*n+is[k];
					temp=m1[u]; m1[u]=m1[v]; m1[v]=temp;
				}
	}
	free(is); free(js);
	return(1);
}


///////////////////////////////////////////
// L0范数下贪婪算法：正交匹配跟踪求解方法(OrthMatchPursuit)/无迭代
// 输入：
//		double *A:		传感矩阵(m*n)
//		double *L:		目标向量(m*1)
//		int m:			传感矩阵行数
//		int n:			传感矩阵列数
//		double d_res:	模的差值
//		double min_res:	最小残差
// 返回值：
//		int:			非零系数个数
///////////////////////////////////////////
int GeoBase::OMP_Solve(double *A, double *L, int m, int n, double *x, double d_res, double min_res)
{
	// 获得传感矩阵的转置矩阵
	double *Atemp = new double[m*n];
	double *xtemp = new double[n];
	double *AT = new double[m*n];
	double *ATA = new double[m*m];
	double *ATL = new double[n];
	Transpose(A, Atemp, m, n);
	// 保存残差向量，初始残差向量即目标向量
	double *residual = new double[m];
	memcpy(residual, L, sizeof(double)*m);
	// 保存选择出来的系数
	vector<int> index;
	double max_coeff, coeff, res_norm, res_norm0;
	int indextemp, num;
	/////////////////////////////////////////
	// 开始进行循环
	/////////////////////////////////////////
	memset(xtemp, 0, sizeof(double)*n);
	while(1)
	{
		// 寻找最大投影向量
		max_coeff = 0.0;
		for(int i=0; i<n; i++)
		{
			// 计算投影大小
			coeff = Dot(&(Atemp[i*m]), residual, m)/Norm(residual, m);
			// 寻找最大值
			if(fabs(coeff)>fabs(max_coeff))
			{
				max_coeff = coeff;
				indextemp = i;
			}
		}
		// 添加选出的系数序号
		index.push_back(indextemp);
		// 构建子传感矩阵
		num = index.size();
		memcpy(&(AT[(num-1)*m]), &(Atemp[index[(num-1)]*m]), sizeof(double)*m);
		// 进行最小二乘求解
		Transpose(AT, A, num, m);
		Multi(AT, A, ATA, num, m, num);
		Multi(AT, L, ATL, num, m, 1);
		GaussExt(ATA, ATL, xtemp, num);
		// 计算剩余残差
		Multi(A, xtemp, ATL, m, num, 1);
		Minus(L, ATL, residual, m, 1);
		res_norm = Norm(residual, m)/sqrt((double)(m));
		// 迭代结束条件判断
		if(num>1)
		{
			if((num==n)||(fabs(res_norm-res_norm0)<d_res)||res_norm<min_res)
				break;
		}
		res_norm0 = res_norm;
	}
	// 进行各种赋值
	memset(x, 0, sizeof(double)*n);
	for(int i=0; i<num; i++)
	{
		x[index[i]] = xtemp[i];
	}
	// 释放内存
	if(Atemp!=NULL)		delete []Atemp;		Atemp = NULL;
	if(AT!=NULL)		delete []AT;		AT = NULL;
	if(ATA!=NULL)		delete []ATA;		ATA = NULL;
	if(ATL!=NULL)		delete []ATL;		ATL = NULL;
	if(residual!=NULL)	delete []residual;	residual = NULL;
	return num;
}


///////////////////////////////////////////
// L0范数下贪婪算法：正交匹配跟踪求解方法(OrthMatchPursuit)/有迭代
// 输入：
//		double *A:		传感矩阵(m*n)
//		double *L:		目标向量(m*1)
//		int m:			传感矩阵行数
//		int n:			传感矩阵列数
//		double d_res:	模的差值
//		double min_res:	最小残差
// 返回值：
//		int:			非零系数个数
///////////////////////////////////////////
int GeoBase::OMP_Solve2(double *A, double *L, int m, int n, double *x, double d_res, double min_res)
{
	// 获得传感矩阵的转置矩阵
	double *Atemp = new double[m*n];
	double *xtemp = new double[n];
	double *AT = new double[m*n];
	double *ATA = new double[m*m];
	double *ATL = new double[n];
	double *Ltemp = new double[n];
	double *Ax = new double[n];
	Transpose(A, Atemp, m, n);
	// 保存残差向量，初始残差向量即目标向量
	double *residual = new double[m];
	memcpy(residual, L, sizeof(double)*m);
	// 保存选择出来的系数
	vector<int> index;
	double max_coeff, coeff, res_norm, res_norm0;
	int indextemp, num;
	/////////////////////////////////////////
	// 开始进行循环
	/////////////////////////////////////////
	memset(xtemp, 0, sizeof(double)*n);
	memset(x, 0, sizeof(double)*n);
	while(1)
	{
		// 寻找最大投影向量
		max_coeff = 0.0;
		for(int i=0; i<n; i++)
		{
			// 计算投影大小
			coeff = Dot(&(Atemp[i*m]), residual, m)/Norm(residual, m);
			// 寻找最大值
			if(fabs(coeff)>fabs(max_coeff))
			{
				max_coeff = coeff;
				indextemp = i;
			}
		}
		// 添加选出的系数序号
		index.push_back(indextemp);
		// 构建子传感矩阵
		num = index.size();
		memcpy(&(AT[(num-1)*m]), &(Atemp[index[(num-1)]*m]), sizeof(double)*m);
		// 进行最小二乘迭代求解
		double sigma0, sigma1 = FLT_MAX;
		int iter = 0;
		while(1)
		{
			Transpose(AT, A, num, m);
			Multi(AT, A, ATA, num, m, num);
			Multi(A, x, Ax, m, num, 1);
			Minus(L, Ax, Ltemp, m, 1);
			Multi(AT, Ltemp, ATL, num, m, 1);
			GaussExt(ATA, ATL, xtemp, num);
			// 计算剩余残差
			Multi(A, xtemp, ATL, m, num, 1);
			Minus(L, ATL, residual, m, 1);
			sigma0 = Norm(residual, m)/sqrt((double)(m));
			if(sigma0>sigma1)
				break;
			sigma1 = sigma0;
			iter++;
			if(iter>100)
				break;
			for(int i=0; i<num; i++)
				x[i] += xtemp[i];
		}
		Multi(A, x, ATL, m, num, 1);
		Minus(L, ATL, residual, m, 1);
		res_norm = Norm(residual, m)/sqrt((double)(m));
		// 迭代结束条件判断
		if(num>1)
		{
			if((num==n)||(fabs(res_norm-res_norm0)<d_res)||res_norm<min_res)
				break;
		}
		res_norm0 = res_norm;
	}
	// 进行各种赋值
	memcpy(xtemp, x, sizeof(double)*n);
	memset(x, 0, sizeof(double)*n);
	for(int i=0; i<num; i++)
	{
		x[index[i]] = xtemp[i];
	}
	// 释放内存
	if(Atemp!=NULL)		delete []Atemp;		Atemp = NULL;
	if(AT!=NULL)		delete []AT;		AT = NULL;
	if(ATA!=NULL)		delete []ATA;		ATA = NULL;
	if(ATL!=NULL)		delete []ATL;		ATL = NULL;
	if(residual!=NULL)	delete []residual;	residual = NULL;
	if(Ltemp!=NULL)		delete []Ltemp;		Ltemp = NULL;
	if(Ax!=NULL)		delete []Ax;		Ax = NULL;
	return num;
}
