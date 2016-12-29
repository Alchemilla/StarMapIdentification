#include "stdafx.h"
#include "BaseFunc.h"

float ReverseQ (long Qtemp)
{
	float Q;
	long Qt;
	Qt=(((Qtemp&(0xff000000))>>24)|((Qtemp&(0x00ff0000))>>8)|
		((Qtemp&(0x0000ff00))<<8)|((Qtemp&(0x000000ff))<<24));
	Q=Qt/1073741824.0;
	return Q; 
}
float Reverse2 (long Qtemp)
{
	float Q;
	long Qt;
	Qt=(((Qtemp&(0xff000000))>>24)|((Qtemp&(0x00ff0000))>>8)|
		((Qtemp&(0x0000ff00))<<8)|((Qtemp&(0x000000ff))<<24));
	Q= (float&)Qt;
	return Q; 
}
double tran2double(unsigned char *pData)
{
	unsigned char transType_8byte[8];
	transType_8byte[0] = pData[7],transType_8byte[1] = pData[6],transType_8byte[2] = pData[5],transType_8byte[3] = pData[4],transType_8byte[4] = pData[3];
	transType_8byte[5] = pData[2],transType_8byte[6] = pData[1],transType_8byte[7] = pData[0];

	double val = (*(double*)transType_8byte);
	return val;
}
double RevDouble (unsigned char *x)
{		
	union Tpacket
	{
		unsigned char a[8];
		double b;
	}p;
	for (int i=0;i<8;i++)
	{
		p.a[i]=x[7-i];
	}	
	double y=p.b;
	return y;
}
void mult(double *m1,double *m2,double *result,int a,int b,int c)
{ 

	int i,j,k;
	for(i=0;i<a;i++)
		for(j=0;j<c;j++){
			result[i*c+j]=0.0;
			for(k=0;k<b;k++)
				result[i*c+j]+=m1[i*b+k]*m2[j+k*c];
		}
		return;
}
void quat2matrix(double q1,double q2,double q3,double q0,double *R)
{
	R[0]=q1*q1-q2*q2-q3*q3+q0*q0;
	R[1]=2*(q1*q2+q3*q0);
	R[2]=2*(q1*q3-q2*q0);

	R[3]=2*(q1*q2-q3*q0);
	R[4]=-q1*q1+q2*q2-q3*q3+q0*q0;
	R[5]=2*(q2*q3+q1*q0);

	R[6]=2*(q1*q3+q2*q0);
	R[7]=2*(q2*q3-q1*q0);
	R[8]=-q1*q1-q2*q2+q3*q3+q0*q0;

	return ;
}
void matrix2quat(double *R,double &q1,double &q2,double &q3,double &q0)
{
	double q1q2q3=sqrt((R[1]+R[3])*(R[2]+R[6])*(R[5]+R[7]));
	q1=q1q2q3/(R[5]+R[7])/2;
	q2=q1q2q3/(R[2]+R[6])/2;
	q3=q1q2q3/(R[1]+R[3])/2;	
	q0=sqrt((R[0]+R[4]+R[8]+1)/4);

	if(fabs(2*(q1*q2+q3*q0)-R[1])>0.0001)
		q0=-q0;
}
void crossmultnorm(double *x,double *y,double *z)
{
	z[0]=x[1]*y[2]-x[2]*y[1];
	z[1]=x[2]*y[0]-x[0]*y[2];
	z[2]=x[0]*y[1]-x[1]*y[0];

	double t;
	t=sqrt(z[0]*z[0]+z[1]*z[1]+z[2]*z[2]);
	if (fabs(t)<10e-8)
		return;
	z[0]/=t;z[1]/=t;z[2]/=t;
}
void normalvect(double *x,double *y)
{
	double t;
	t=sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
	if(fabs(t)<10e-8)
	{
		memcpy(y,x,sizeof(double)*3);
		return;
	}
	y[0]=x[0]/t;y[1]=x[1]/t;y[2]=x[2]/t;
}
int invers_matrix(double *m1,int n)
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
void quatmult(double *q1,double *q2,double *q3)//四元数顺序为0123，其中0为标量
{
	q3[1]=q1[0]*q2[1]-q1[3]*q2[2]+q1[2]*q2[3]+q1[1]*q2[0];
	q3[2]=q1[3]*q2[1]+q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0];
	q3[3]=-q1[2]*q2[1]+q1[1]*q2[2]+q1[0]*q2[3]+q1[3]*q2[0];
	q3[0]=-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]+q1[0]*q2[0];
}
void QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum,Quat *&m_att)
{
	if(AttNum<2)	{	printf("QuatInterpolation Error：AttNum is less than 2, can't interpolation!\n");	return;	}
	// 寻找临近的两个点(对分查找)
	Quat attleft, attright, att;
	long posstart, posend, pos;
	for(int i=0;i<interNum;i++)
	{
		posstart=0, posend=AttNum-1, pos=0;
		while(posstart<posend)  
		{   			
			pos = (posstart+posend)/2;
			if(pos==posstart) break;	// 记得加上这句判断,否则会陷入死循环
			if((Att[pos].UTC<=UTC[i])&&(Att[pos+1].UTC>UTC[i]))
				break;
			if (Att[pos].UTC <= UTC[i])  
				posstart = pos;  
			else
				posend = pos;
		}  
		if(pos < 0)	pos = 0;
		if(pos >= AttNum-1)		pos = AttNum-2;
		attleft = Att[pos];		attright = Att[pos+1];

		// 进行内插
		double sp,sq;
		double t = (UTC[i] - attleft.UTC)/(attright.UTC - attleft.UTC);
		double cosa = attleft.Q0*attright.Q0 + attleft.Q1*attright.Q1 + attleft.Q2*attright.Q2 + attleft.Q3*attright.Q3;
		// 这个错误需要注意了,防止邻近两个值互为反号的情况,需要确保length>0
		if(cosa<0)			
		{	
			cosa = -cosa;	
			attright.Q0 = -attright.Q0;	attright.Q1 = -attright.Q1;	attright.Q2 = -attright.Q2;	attright.Q3 = -attright.Q3;
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
		m_att[i].Q0 = sp*attleft.Q0 + sq*attright.Q0;	m_att[i].Q1 = sp*attleft.Q1 + sq*attright.Q1;
		m_att[i].Q2 = sp*attleft.Q2 + sq*attright.Q2;	m_att[i].Q3 = sp*attleft.Q3 + sq*attright.Q3;
		m_att[i].UTC = UTC[i];
	}
}

//////////////////////////////////////////////////////////////////////////
// 坐标转化函数
//////////////////////////////////////////////////////////////////////////
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


void LagrangianInterpolation(Orbit_Ep *Eph, long EphNum, double UTC, Orbit_Ep &m_point, int order)
{
	memset(&m_point, 0, sizeof(Orbit_Ep));
	m_point.UTC = UTC;
	// 搜索内插用的起始和结束点(对分查找)
	double up=1, down=1;
	long posstart, posend, pos;
	posstart=0, posend=EphNum-1, pos=0;
	while(posstart<posend)  
    {  
		pos = (posstart+posend)/2;
		if(pos==posstart) break;	// 记得加上这句判断,否则会陷入死循环
		if((Eph[pos].UTC<=UTC)&&(UTC<Eph[pos+1].UTC))
			break;  
        if (Eph[pos].UTC <= UTC)  
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
			if(i!=j)	{	up *= (UTC-Eph[i].UTC);	down *= (Eph[j].UTC-Eph[i].UTC);	}
			m_point.X += Eph[j].X*up/down;
			m_point.Y += Eph[j].Y*up/down;
			m_point.Z += Eph[j].Z*up/down;
			m_point.Xv += Eph[j].Xv*up/down;
			m_point.Yv += Eph[j].Yv*up/down;
			m_point.Zv += Eph[j].Zv*up/down;	
	}
}
void rotationZXY(double x,double y,double z,double *r)
{
	r[0]=cos(y)*cos(z)-sin(x)*sin(y)*sin(z);
	r[1]=cos(x)*sin(z)+sin(x)*sin(y)*cos(z);
	r[2]=-cos(x)*sin(y);
	r[3]=-cos(x)*sin(z);
	r[4]=cos(x)*cos(z);
	r[5]=sin(x);
	r[6]=sin(y)*cos(z)-cos(y)*sin(x)*sin(z);
	r[7]=sin(y)*sin(z)-sin(x)*cos(y)*cos(z);
	r[8]=cos(x)*cos(y);
}