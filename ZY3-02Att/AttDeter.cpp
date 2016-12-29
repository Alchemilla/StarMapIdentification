#include "AttDeter.h"

AttDeter::AttDeter(void){}
AttDeter::~AttDeter(void){}

double AttDeter::Ainstall[]=//Crb
{
	cos(29.13212/180*PI),cos(104.8425/180*PI),cos(65.5449/180*PI),
	cos(75.98529/180*PI),cos(120.6005/180*PI),cos(145.6867/180*PI),
	cos(65.0185/180*PI),cos(34.7422/180*PI),cos(112.497/180*PI)
};
double AttDeter::Binstall[9]=
{
	cos(21.68954/180*PI),cos(92.60966/180*PI),cos(111.5162/180*PI),
	cos(102.7403/180*PI),cos(149.8423/180*PI),cos(116.833/180*PI),
	cos(107.2508/180*PI),cos(59.97562/180*PI),cos(144.4336/180*PI)
};
double AttDeter::Cinstall[9]=
{
	cos(63.63085/180*PI),cos(92.72818/180*PI),cos(26.53156/180*PI),
	cos(68.95412/180*PI),cos(154.8806/180*PI),cos(103.0838/180*PI),
	cos(34.83029/180*PI),cos(65.0493/180*PI),cos(112.6469/180*PI)
};
//double AttDeter::GyroIns[9]=//Crb,ZY3-01星陀螺安装矩阵
//{
//	cos(54.7604/180*PI),	cos(90.0868/180*PI),cos(35.2731/180*PI),
//	cos(54.6596/180*PI),	cos(134.9521/180*PI),cos(114.0841/180*PI),
//	cos(54.8202/180*PI),	cos(45.0095/180*PI),cos(114.2353/180*PI)
//};
double AttDeter::GyroIns[9]=//Crb,ZY3-02星陀螺安装矩阵
{
	cos(54.72148/180*PI),	cos(89.9605/180*PI),cos(35.25769/180*PI),
	cos(54.72435/180*PI),	cos(135.0281/180*PI),cos(114.0297/180*PI),
	cos(54.77499/180*PI),	cos(44.97068/180*PI),cos(114.0914/180*PI)
};

//////////////////////////////////////////////////////////////////////////
//单星敏定姿，可以检查数据的问题
//输出定姿结果
//////////////////////////////////////////////////////////////////////////
bool AttDeter::SingleStar(vector<STGData> AttData,int StarTag,vector<Quat> &AttDet)
{
	int i,m;
	m=AttData.size();
	double Aalin[9],Balin[9],Calin[9];
	memcpy(Aalin,Ainstall,sizeof(Ainstall));
	memcpy(Balin,Binstall,sizeof(Binstall));
	memcpy(Calin,Cinstall,sizeof(Cinstall));
	double k,Crj[9],Cbj[9],q[4];
	Quat EKFres;	
	if (StarTag==1)//星敏1或A
	{
		invers_matrix(Aalin,3);//转换为Cbr	
		for (i=0;i<m;i++)
		{
			quat2matrix(AttData[i].StarA.Q1,AttData[i].StarA.Q2,AttData[i].StarA.Q3,AttData[i].StarA.Q0,Crj);
			mult(Aalin,Crj,Cbj,3,3,3);
			matrix2quat(Cbj,q[1],q[2],q[3],q[0]);
			k=pow(q[1],2)+pow(q[2],2)+pow(q[3],2)+pow(q[0],2)-1;
			if (k>0.01){return 0;}		
			EKFres.UTC=AttData[i].StarA.UTC,EKFres.Q0=q[0],EKFres.Q1=q[1],EKFres.Q2=q[2],EKFres.Q3=q[3];
			AttDet.push_back(EKFres);
		}
	} 
	else if(StarTag==2)//星敏2或B
	{
		invers_matrix(Balin,3);//转换为Cbr
		for (i=0;i<m;i++)
		{
			quat2matrix(AttData[i].StarB.Q1,AttData[i].StarB.Q2,AttData[i].StarB.Q3,AttData[i].StarB.Q0,Crj);		
			mult(Balin,Crj,Cbj,3,3,3);
			matrix2quat(Cbj,q[1],q[2],q[3],q[0]);
			k=pow(q[1],2)+pow(q[2],2)+pow(q[3],2)+pow(q[0],2)-1;
			if (k>0.01){return 0;}
		}
	}
	else if(StarTag==3)//星敏3或B
	{
		invers_matrix(Calin,3);//转换为Cbr
		for (i=0;i<m;i++)
		{
			quat2matrix(AttData[i].StarC.Q1,AttData[i].StarC.Q2,AttData[i].StarC.Q3,AttData[i].StarC.Q0,Crj);			
			mult(Calin,Crj,Cbj,3,3,3);
			matrix2quat(Cbj,q[1],q[2],q[3],q[0]);
			k=pow(q[1],2)+pow(q[2],2)+pow(q[3],2)+pow(q[0],2)-1;
			if (k>0.01){return 0;}
		}
	}
	return 1;
}

//////////////////////////////////////////////////////////////////////////
//计算STG中三个星敏的夹角
//////////////////////////////////////////////////////////////////////////
void AttDeter::StarAngle(vector<STGData> StarDat,string Res,int StarTag)
{	
	int m,n;
	m=StarDat.size();
	Quat *StarX = new Quat[m];
	Quat *StarY = new Quat[m];
	Quat *StarYi = new Quat[m];
	double *UTC = new double[m];
	string path = string(Res);
	path = path.substr(0,path.rfind('.'));
	string strpath;

	if (StarTag==23)
	{
		for (int i=0;i<m;i++)//星敏2和3数据
		{
			UTC[i]=StarDat.at(i).StarB.UTC;
			StarX[i].Q1=StarDat.at(i).StarB.Q1;StarX[i].Q2=StarDat.at(i).StarB.Q2;StarX[i].Q3=StarDat.at(i).StarB.Q3,StarX[i].Q0=StarDat.at(i).StarB.Q0;
			StarY[i].UTC=StarDat.at(i).StarC.UTC;
			StarY[i].Q1=StarDat.at(i).StarC.Q1;StarY[i].Q2=StarDat.at(i).StarC.Q2;StarY[i].Q3=StarDat.at(i).StarC.Q3,StarY[i].Q0=StarDat.at(i).StarC.Q0;
		}
		QuatInterpolation(StarY, m, UTC, m,StarYi);
		strpath =path+"_Star23Angle.txt";
	} 
	else if(StarTag==12)
	{
		for (int i=0;i<m;i++)//星敏1和2数据
		{
			UTC[i]=StarDat.at(i).StarA.UTC;
			StarX[i].Q1=StarDat.at(i).StarA.Q1;StarX[i].Q2=StarDat.at(i).StarA.Q2;StarX[i].Q3=StarDat.at(i).StarA.Q3,StarX[i].Q0=StarDat.at(i).StarA.Q0;
			StarY[i].UTC=StarDat.at(i).StarB.UTC;
			StarY[i].Q1=StarDat.at(i).StarB.Q1;StarY[i].Q2=StarDat.at(i).StarB.Q2;StarY[i].Q3=StarDat.at(i).StarB.Q3,StarY[i].Q0=StarDat.at(i).StarB.Q0;
		}
		QuatInterpolation(StarY, m, UTC, m,StarYi);
		strpath =path+"_Star12Angle.txt";
	} 
	else if(StarTag==13)
	{
		for (int i=0;i<m;i++)//星敏1和3数据
		{
			UTC[i]=StarDat.at(i).StarA.UTC;
			StarX[i].Q1=StarDat.at(i).StarA.Q1;StarX[i].Q2=StarDat.at(i).StarA.Q2;StarX[i].Q3=StarDat.at(i).StarA.Q3,StarX[i].Q0=StarDat.at(i).StarA.Q0;
			StarY[i].UTC=StarDat.at(i).StarC.UTC;
			StarY[i].Q1=StarDat.at(i).StarC.Q1;StarY[i].Q2=StarDat.at(i).StarC.Q2;StarY[i].Q3=StarDat.at(i).StarC.Q3,StarY[i].Q0=StarDat.at(i).StarC.Q0;
		}
		QuatInterpolation(StarY, m, UTC, m,StarYi);
		strpath =path+"_Star13Angle.txt";
	} 
		
	FILE *fpres=fopen(strpath.c_str(),"w");
	fprintf(fpres,"星敏%d定姿\n",StarTag);
	double mz[3]={0,0,1},RA[9],RB[9],za[3],zb[3],Angle;
	for(int i=0;i<m;i++)
	{
		quat2matrix(StarX[i].Q1,StarX[i].Q2,StarX[i].Q3,StarX[i].Q0,RA);
		quat2matrix(StarYi[i].Q1,StarYi[i].Q2,StarYi[i].Q3,StarYi[i].Q0,RB);//Crj
		invers_matrix(RA,3);
		invers_matrix(RB,3);//Cjr
		mult(RA,mz,za,3,3,1);
		mult(RB,mz,zb,3,3,1);//星敏光轴在惯性系中的坐标值
		Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
		fprintf(fpres,"%.9f\t%.9f\n",UTC[i],Angle);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：ZY3根据星敏和轨道得到光行差修正后，星敏之间的角度
//输入：星敏数据，轨道数据
//输出：星敏光行差修正结果
//注意：
//日期：2016.12.06
//////////////////////////////////////////////////////////////////////////
bool AttDeter::Aberration(vector<STGData> StarDat,vector<Orbit_Ep> EpDat, string Res,int StarTag)
{
	int i,m,n;
	m=StarDat.size();
	Quat *StarX = new Quat[m];
	Quat *StarY = new Quat[m];
	Quat *StarYi = new Quat[m];
	double *UTC = new double[m];
	string path = string(Res);
	path = path.substr(0,path.rfind('.'));
	string strpath;

	if(StarTag==12)
	{
		for (int i=0;i<m;i++)//星敏1和2数据
		{
			UTC[i]=StarDat.at(i).StarA.UTC;
			StarX[i].Q1=StarDat.at(i).StarA.Q1;StarX[i].Q2=StarDat.at(i).StarA.Q2;StarX[i].Q3=StarDat.at(i).StarA.Q3,StarX[i].Q0=StarDat.at(i).StarA.Q0;
			StarY[i].UTC=StarDat.at(i).StarB.UTC;
			StarY[i].Q1=StarDat.at(i).StarB.Q1;StarY[i].Q2=StarDat.at(i).StarB.Q2;StarY[i].Q3=StarDat.at(i).StarB.Q3,StarY[i].Q0=StarDat.at(i).StarB.Q0;
		}
		QuatInterpolation(StarY, m, UTC, m,StarYi);
		strpath =path+"_Star12Angle_Aberra.txt";
	} 
	else if(StarTag==13)
	{
		for (int i=0;i<m;i++)//星敏1和3数据
		{
			UTC[i]=StarDat.at(i).StarA.UTC;
			StarX[i].Q1=StarDat.at(i).StarA.Q1;StarX[i].Q2=StarDat.at(i).StarA.Q2;StarX[i].Q3=StarDat.at(i).StarA.Q3,StarX[i].Q0=StarDat.at(i).StarA.Q0;
			StarY[i].UTC=StarDat.at(i).StarC.UTC;
			StarY[i].Q1=StarDat.at(i).StarC.Q1;StarY[i].Q2=StarDat.at(i).StarC.Q2;StarY[i].Q3=StarDat.at(i).StarC.Q3,StarY[i].Q0=StarDat.at(i).StarC.Q0;
		}
		QuatInterpolation(StarY, m, UTC, m,StarYi);
		strpath =path+"_Star13Angle_Aberra.txt";
	} 
	double mz[3]={0,0,1},RA[9],RB[9],za[3],zb[3];//星敏光轴相关定义
	FILE *fp = fopen(strpath.c_str(),"w");
			
	Orbit_Ep *Epdata = new Orbit_Ep[m];
	Orbit_Ep *Epdatainter = new Orbit_Ep[m];
	n = EpDat.size();
	for(i=0;i<n;i++)
	{
		memcpy(&Epdata[i],&EpDat[i],sizeof(EpDat[i]));
	}
	double jd0,mjd,second,PosEarth[6],GCRS2ITRS[9],SatePos[3],SateVel[3],LightVelocity=299792458.0;
	int year, month, day, hour, minute;
	memset(PosEarth, 0, sizeof(double)*6);
	Cal2JD(2009, 1, 1, 0, &jd0, &mjd);
	//设置星历参数路径和EOP参数路径
	string path1 = "C:\\Users\\wcsgz\\Documents\\2-CProject\\9-ZY3\\Need\\2000_2020_421.txt";
	char *JPLPath = (char*)path1.data();
	string path2 = "C:\\Users\\wcsgz\\Documents\\2-CProject\\9-ZY3\\Need\\EOP00.txt";
	char* EOPPath = (char*)path2.data();
	for (i=0;i<m;i++)
	{
		quat2matrix(StarX[i].Q1,StarX[i].Q2,StarX[i].Q3,StarX[i].Q0,RA);
		quat2matrix(StarYi[i].Q1,StarYi[i].Q2,StarYi[i].Q3,StarYi[i].Q0,RB);//Crj
		invers_matrix(RA,3);
		invers_matrix(RB,3);//Cjr
		mult(RA,mz,za,3,3,1);
		mult(RB,mz,zb,3,3,1);//星敏光轴在惯性系中的坐标值
		LagrangianInterpolation(Epdata,n,UTC[i],Epdatainter[i],7);
		SatePos[0]=Epdatainter[i].X,SatePos[1]=Epdatainter[i].Y,SatePos[2]=Epdatainter[i].Z;
		SateVel[0]=Epdatainter[i].Xv,SateVel[1]=Epdatainter[i].Yv,SateVel[2]=Epdatainter[i].Zv;
		FromSecondtoYMD(mjd,UTC[i],year,month,day,hour,minute,second);
		PlanetEph(year,month,day,hour,minute,second,JPLPath,EOPPath,2,11,PosEarth);
		IAU2000ABaseCIOTerToCel(year,month,day,hour,minute,second,EOPPath,8,GCRS2ITRS,SatePos,SateVel);
		//卫星在GCRS坐标系下的速度(考虑了地球相对太阳的速度)
		SateVel[0]=SateVel[0]+PosEarth[3],SateVel[1]=SateVel[1]+PosEarth[4],SateVel[2]=SateVel[2]+PosEarth[5];
		double VelRa[3],VelRb[3],SateVelocity,costhetaA,costhetaB,sinthetaA,sinthetaB;
		crossmultnorm(za,SateVel,VelRa);//星敏A修正旋转轴
		crossmultnorm(zb,SateVel,VelRb);//星敏B修正旋转轴
		SateVelocity=sqrt(pow(SateVel[0],2)+pow(SateVel[1],2)+pow(SateVel[2],2));
		costhetaA=(za[0]*SateVel[0]+za[1]*SateVel[1]+za[2]*SateVel[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/SateVelocity;
		costhetaB=(zb[0]*SateVel[0]+zb[1]*SateVel[1]+zb[2]*SateVel[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2])/SateVelocity;
		sinthetaA=sqrt(1-pow(costhetaA,2));//星敏A光轴与速度夹角
		sinthetaB=sqrt(1-pow(costhetaB,2));//星敏B光轴与速度夹角
		double angleA,angleB,quatA[4],quatB[4],RotA[9],RotB[9];
		angleA=-(SateVelocity/LightVelocity)*sinthetaA;
		angleB=-(SateVelocity/LightVelocity)*sinthetaB;
		//修正星敏光轴的旋转四元数
		quatA[0]=cos(angleA/2);
		quatA[1]=VelRa[0]*sin(angleA/2),quatA[2]=VelRa[1]*sin(angleA/2),quatA[3]=VelRa[2]*sin(angleA/2);
		quatB[0]=cos(angleB/2);
		quatB[1]=VelRb[0]*sin(angleB/2),quatB[2]=VelRb[1]*sin(angleB/2),quatB[3]=VelRb[2]*sin(angleB/2);
		//转换为旋转矩阵
		quat2matrix(quatA[1],quatA[2],quatA[3],quatA[0],RotA);
		quat2matrix(quatB[1],quatB[2],quatB[3],quatB[0],RotB);
		double zafix[3],zbfix[3],StarSensorAngle,detza,detzb;
		mult(RotA,za,zafix,3,3,1);
		mult(RotB,zb,zbfix,3,3,1);
		detza=sqrt(zafix[0]*zafix[0]+zafix[1]*zafix[1]+zafix[2]*zafix[2]);
		detzb=sqrt(zbfix[0]*zbfix[0]+zbfix[1]*zbfix[1]+zbfix[2]*zbfix[2]);
		StarSensorAngle=acos((zafix[0]*zbfix[0]+zafix[1]*zbfix[1]+zafix[2]*zbfix[2])/detza/detzb)/PI*180;
		fprintf(fp,"%f\n",StarSensorAngle);
	}
	
	return true;
}

//////////////////////////////////////////////////////////////////////////
//双星敏定姿，可以与星上滤波姿态比较
/////////////////////////////////////////////////////////////////////////
void AttDeter::DoubleStar(vector<STGData> StarDat,vector<Quat> &AttDet,string Res,int StarTag)
{
	int i,m;
	m=StarDat.size();
	double Balin[9],Calin[9];//双星敏各自的安装矩阵
	Quat *StarX = new Quat[m];
	Quat *StarY = new Quat[m];
	Quat *StarXi = new Quat[m];
	Quat *StarYi = new Quat[m];
	double *UTC = new double[m];
	
	string path = string(Res);
	path = path.substr(0,path.rfind('.'));
	string strpath;
	double mz[3]={0,0,1},zc[3],zb[3],x[3],y[3],z[3],RC[9],RB[9],Cinstallz[3],Binstallz[3];
	Quat Q2Vec;
	if (StarTag==23)
	{	
		//根据选择的星敏确定安装矩阵
		memcpy(Balin,Binstall,sizeof(Binstall));
		memcpy(Calin,Cinstall,sizeof(Cinstall));
		invers_matrix(Balin,3);
		invers_matrix(Calin,3);
		for (int i=0;i<m;i++)
		{
			//内插星敏数据,此处为星敏2和星敏3
			UTC[i]=StarDat.at(i).utgyro;
			StarX[i].UTC=StarDat.at(i).StarB.UTC;
			StarX[i].Q1=StarDat.at(i).StarB.Q1;StarX[i].Q2=StarDat.at(i).StarB.Q2;StarX[i].Q3=StarDat.at(i).StarB.Q3,StarX[i].Q0=StarDat.at(i).StarB.Q0;
			StarY[i].UTC=StarDat.at(i).StarC.UTC;
			StarY[i].Q1=StarDat.at(i).StarC.Q1;StarY[i].Q2=StarDat.at(i).StarC.Q2;StarY[i].Q3=StarDat.at(i).StarC.Q3,StarY[i].Q0=StarDat.at(i).StarC.Q0;
		}
		QuatInterpolation(StarX, m, UTC, m,StarXi);
		QuatInterpolation(StarY, m, UTC, m,StarYi);
		strpath =path+"_2Vec_S2&S3.txt";
		FILE *fpres=fopen(strpath.c_str(),"w");
		
		mult(Balin,mz,Binstallz,3,3,1);//Cbr,星敏光轴在卫星本体系中的坐标值
		mult(Calin,mz,Cinstallz,3,3,1);
		normalvect(Binstallz,x); 
		crossmultnorm(Binstallz,Cinstallz,y);//此时产生的X和Y均是本体系下的值
		crossmultnorm(x,y,z);//Crb,以本体系下的星敏A和B的Z轴作为新坐标系的X和Y轴，此坐标系组成的旋转矩阵也在本体系下
		double Cbr[9],Crj[9],Cbj[9];
		Cbr[0]=x[0],Cbr[1]=x[1],Cbr[2]=x[2];
		Cbr[3]=y[0],Cbr[4]=y[1],Cbr[5]=y[2];
		Cbr[6]=z[0],Cbr[7]=z[1],Cbr[8]=z[2];
		invers_matrix(Cbr,3);
		for(int i=0;i<m;i++)
		{			
			quat2matrix(StarXi[i].Q1,StarXi[i].Q2,StarXi[i].Q3,StarXi[i].Q0,RB);//Crj
			quat2matrix(StarYi[i].Q1,StarYi[i].Q2,StarYi[i].Q3,StarYi[i].Q0,RC);
			invers_matrix(RC,3);
			invers_matrix(RB,3);//Cjr
			mult(RC,mz,zc,3,3,1);
			mult(RB,mz,zb,3,3,1);//星敏光轴在惯性系中的坐标值
			normalvect(zb,x); 
			crossmultnorm(zb,zc,y);	
			crossmultnorm(x,y,z);//Crj
			Crj[0] = x[0],Crj[1] = x[1],Crj[2] = x[2];
			Crj[3] = y[0],Crj[4] = y[1],Crj[5] = y[2];
			Crj[6] = z[0],Crj[7] = z[1],Crj[8] = z[2];
			mult(Cbr,Crj,Cbj,3,3,3);
			matrix2quat(Cbj,Q2Vec.Q1,Q2Vec.Q2,Q2Vec.Q3,Q2Vec.Q0);
			Q2Vec.UTC=UTC[i];
			AttDet.push_back(Q2Vec);
			fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",UTC[i],Q2Vec.Q0,Q2Vec.Q1,Q2Vec.Q2,Q2Vec.Q3);
		}
	} 
	else if(StarTag==12)
	{
	}
	else if(StarTag==13)
	{}
	
}

//////////////////////////////////////////////////////////////////////////
//功能：卡尔曼主程序
//输入：AttData:STG解析出的姿态数据	Res:结果输出路径		StarTag:星敏标识
//输出：AttDet:姿态确定结果
//注意：先进行双矢量再定姿
//日期：2015.11.15
//////////////////////////////////////////////////////////////////////////
void AttDeter::EKF6StateV2(vector<STGData> AttData,vector<Quat> &AttDet,string Res,int StarTag)
{
	int i;
	vector<Quat> AttDet2Vec;
	DoubleStar(AttData,AttDet2Vec,Res,StarTag);
	int m=AttDet2Vec.size();
	MatrixXd StarDat(m,4),Wgm(m,3),Qest(m,4);
	double GyDat[3],GyTran[3];
	invers_matrix(GyroIns,3);
	for (i=0;i<m;i++)
	{
		StarDat(i,0)=AttDet2Vec.at(i).Q1,StarDat(i,1)=AttDet2Vec.at(i).Q2;
		StarDat(i,2)=AttDet2Vec.at(i).Q3,StarDat(i,3)=AttDet2Vec.at(i).Q0;
		GyDat[0]=AttData.at(i).g1,GyDat[1]=AttData.at(i).g3,GyDat[2]=AttData.at(i).g5;
		mult(GyroIns,GyDat,GyTran,3,3,1);
		Wgm(i,0)=GyTran[0]/180*PI,Wgm(i,1)=GyTran[1]/180*PI,Wgm(i,2)=GyTran[2]/180*PI;
	}
	double dt=0.25;
	Qest(0,0)=StarDat(0,0),Qest(0,1)=StarDat(0,1),Qest(0,2)=StarDat(0,2),Qest(0,3)=StarDat(0,3);
	double a1=Qest(0,0),a2=Qest(0,1),a3=Qest(0,2),a4=Qest(0,3);

	double sig=1e-6;
	Matrix3d zero33,eye33,poa,pog,r,sigu,sigv;	
	eye33<<1,0,0,0,1,0,0,0,1;
	zero33<<MatrixXd::Zero(3,3);
	sigu<<2e-20*eye33;//陀螺漂移噪声
	sigv<<2e-14*eye33;//陀螺噪声
	poa<<1e-7*eye33;//初始姿态误差协方差
	pog<<1e-5*eye33;//初始陀螺误差协方差
	r<<pow(sig,2)*eye33;//星敏噪声
	MatrixXd p(6,6),Q(6,6),eye66(6,6),be(m,3),we(m,3),xest(m,6);
	eye66<<eye33,zero33,zero33,eye33;
	be.row(0)<<0,0,0;
	xest.row(0)<<0,0,0,0,0,0;
	p<<poa,zero33,zero33,pog;//过程协方差
	Q<<sigv,zero33,zero33,sigu;//过程噪声

	string path = string(Res);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_GyroBiasEstimate.txt";
	string strpath1 =path+"_EKF.txt";
	FILE *fpres=fopen(strpath.c_str(),"w");
	FILE *fpEKF=fopen(strpath1.c_str(),"w");

	int j=0;
	while(j<2)
	{
	for(i=0;i<m-1;i++)
	{	
		double qmm1,qmm2,qmm3;
		qmm1=-StarDat(i,3)*Qest(i,0)-StarDat(i,2)*Qest(i,1)+StarDat(i,1)*Qest(i,2)+StarDat(i,0)*Qest(i,3);
		qmm2= StarDat(i,2)*Qest(i,0)-StarDat(i,3)*Qest(i,1)-StarDat(i,0)*Qest(i,2)+StarDat(i,1)*Qest(i,3);
		qmm3=-StarDat(i,1)*Qest(i,0)+StarDat(i,0)*Qest(i,1)-StarDat(i,3)*Qest(i,2)+StarDat(i,2)*Qest(i,3);
		MatrixXd z(3,1);
		z<<2*qmm1,2*qmm2,2*qmm3;
		//cout<<"观测残差："<<z.transpose()<<endl;
		MatrixXd h(3,6),k(6,3);
		h<<eye33,zero33;
		k=p*h.transpose()*(h*p*h.transpose()+r).inverse();
		//cout<<k<<endl;
		p=(eye66-k*h)*p;
		//cout<<"p"<<p<<endl;
		xest.row(i)=xest.row(i)+(k*z).transpose();
		//cout<<xest.row(i);
		
		MatrixXd xe(1,3);
		xe=0.5*xest.row(i).head(3);
		double qe11,qe22,qe33,qe44;
		qe11=Qest(i,0)+xe(2)*Qest(i,1)-xe(1)*Qest(i,2)+xe(0)*Qest(i,3);
		qe22=-xe(2)*Qest(i,0)+Qest(i,1)+xe(0)*Qest(i,2)+xe(1)*Qest(i,3);
		qe33=xe(1)*Qest(i,0)-xe(0)*Qest(i,1)+Qest(i,2)+xe(2)*Qest(i,3);
		qe44=-xe(0)*Qest(i,0)-xe(1)*Qest(i,1)-xe(2)*Qest(i,2)+Qest(i,3);
		MatrixXd tempqe(4,1);
		tempqe<<qe11,qe22,qe33,qe44;
		tempqe.normalize();
		Qest.row(i)<<tempqe(0),tempqe(1),tempqe(2),tempqe(3);
		//cout<<Qest.row(i)<<endl;

		//Propagate Covariance
		//cout<<Wgm.row(i)<<endl;
		//cout<<xest.row(i).tail(3)<<endl;
		we.row(i)=Wgm.row(i)-xest.row(i).tail(3);
		double w=sqrt(we(i,0)*we(i,0)+we(i,1)*we(i,1)+we(i,2)*we(i,2));
		Matrix3d wa;
		//wa<<0,we(i,2),-we(i,1),-we(i,2),0,we(i,0),we(i,1),-we(i,0),0;
		wa<<0,-we(i,2),we(i,1),we(i,2),0,-we(i,0),-we(i,1),we(i,0),0;
		//cout<<wa<<endl;
		MatrixXd fmat(6,6),gmat(6,6),phi(6,6),gamma(6,6);
		fmat<<-wa,-eye33,zero33,zero33;
		gmat<<-eye33,zero33,zero33,eye33;
		phi=eye66+fmat*dt;
		//gamma=gmat*dt;
		gamma=(eye66*dt+fmat*dt*dt/2)*gmat;
		//cout<<phi<<endl;
		//cout<<gamma<<endl;

		//Propagate State
		double qw1,qw2,qw3,qw4;
		qw1=we(i,0)/w*sin(0.5*w*dt);
		qw2=we(i,1)/w*sin(0.5*w*dt);
		qw3=we(i,2)/w*sin(0.5*w*dt);
		qw4=cos(0.5*w*dt);
		MatrixXd om(4,4);
		om<<qw4,qw3,-qw2,qw1,-qw3,qw4,qw1,qw2,qw2,-qw1,qw4,qw3,-qw1,-qw2,-qw3,qw4;
		//cout<<om<<endl;
		Qest.row(i+1)=(om*Qest.row(i).transpose()).transpose();
		//cout<<Qest.row(i+1)<<endl;

		//Propagate Covariance
		p=phi*p*phi.transpose()+gamma*Q*gamma.transpose();
		xest.row(i+1)=xest.row(i);
		xest(i+1,0)=0;xest(i+1,1)=0;xest(i+1,2)=0;
		//cout<<xest.row(i)<<endl;
		
		fprintf(fpres,"%lf\t%lf\t%lf\n",xest(i,3)*180/PI*3600,
			xest(i,4)*180/PI*3600,xest(i,5)*180/PI*3600);
	}
	j++;
	xest(0,3)=xest(m-1,3),xest(0,4)=xest(m-1,4),xest(0,5)=xest(m-1,5);
	}
		
	Quat EKFres;
	fprintf(fpEKF,"%d\n",m);
	for (i=0;i<m;i++)
	{
		fprintf(fpEKF,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",AttData[i].utgyro,Qest(i,3),Qest(i,0),Qest(i,1),Qest(i,2));
		EKFres.UTC=AttData[i].utgyro,EKFres.Q0=Qest(i,3),EKFres.Q1=Qest(i,0),EKFres.Q2=Qest(i,1),EKFres.Q3=Qest(i,2);
		AttDet.push_back(EKFres);
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：将四元数格式改为JYH检校软件需要的姿态格式
//输入：姿态确定时间四元数
//输出：对应格式
//注意：
//日期：2016.12.05
//////////////////////////////////////////////////////////////////////////
void AttDeter::QuatForZY3JJ(vector<Quat> AttDet,string Res)
{
	int i=0,m;
	m = AttDet.size();
	string path = string(Res);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_APS_Format.txt";
	FILE *fp = fopen(strpath.c_str(),"w");
	for (i;i<m;i++)
	{
		fprintf(fp,"attData\n{\n");
		fprintf(fp,"timeCode = %lf\n",AttDet[i].UTC);
		if (AttDet[i].Q0>=0)
		{
			fprintf(fp,"q1 = %.9lf\nq2 = %.9lf\nq3 = %.9lf\n",AttDet[i].Q1,AttDet[i].Q2,AttDet[i].Q3);
		} 
		else
		{
			fprintf(fp,"q1 = %.9lf\nq2 = %.9lf\nq3 = %.9lf\n",-AttDet[i].Q1,-AttDet[i].Q2,-AttDet[i].Q3);
		}
		
		fprintf(fp,"}\n");
	}
}

//////////////////////////////////////////////////////////////////////////
//陀螺积分定姿模式
//////////////////////////////////////////////////////////////////////////
void AttDeter::GyroAtt(vector<STGData> AttData,vector<Quat> &AttDet,string Res,int StarTag)
{
	int i,num=0;
	int m=AttData.size();
	MatrixXd Wgm(m,3),W_Att(m,4);
	double Aalin[9],Balin[9],Calin[9],q[4];
	memcpy(Aalin,Ainstall,sizeof(Ainstall));
	memcpy(Balin,Binstall,sizeof(Binstall));
	memcpy(Calin,Cinstall,sizeof(Cinstall));
	invers_matrix(Balin,3);//转换为Cbr
	double GyDat[3],GyTran[3],Cbj[9],Crj[9];
	invers_matrix(GyroIns,3);
	double *UTC = new double[m];
	Quat *StarX = new Quat[m];
	Quat *StarXi = new Quat[m];

	for(i=0;i<m;i++)
	{
		quat2matrix(AttData[i].StarB.Q1,AttData[i].StarB.Q2,AttData[i].StarB.Q3,AttData[i].StarB.Q0,Crj);		
		mult(Balin,Crj,Cbj,3,3,3);
		matrix2quat(Cbj,q[1],q[2],q[3],q[0]);
		StarX[i].UTC=AttData[i].StarB.UTC,StarX[i].Q0=q[0],StarX[i].Q1=q[1],StarX[i].Q2=q[2],StarX[i].Q3=q[3];
		GyDat[0]=AttData.at(i).g1,GyDat[1]=AttData.at(i).g3,GyDat[2]=AttData.at(i).g5;
		mult(GyroIns,GyDat,GyTran,3,3,1);
		Wgm(i,0)=GyTran[0]/180*PI,Wgm(i,1)=GyTran[1]/180*PI,Wgm(i,2)=GyTran[2]/180*PI;
		UTC[i]=AttData.at(i).utgyro;
	}
	QuatInterpolation(StarX,m,UTC,m,StarXi);

	string path = string(Res);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_GyroAtt.txt";
	FILE *fpres=fopen(strpath.c_str(),"w");

	MatrixXd SAm1(4,1);
	SAm1<<StarXi[0].Q1,StarXi[0].Q2,StarXi[0].Q3,StarXi[0].Q0;
	double dt = 0.25;
	for(i=0;i<m;i++)
	{
		MatrixXd F(4,4);
		F<<0,Wgm(i,2),-Wgm(i,1),Wgm(i,0),
			-Wgm(i,2),0,Wgm(i,0),Wgm(i,1),
			Wgm(i,1),-Wgm(i,0),0,Wgm(i,2),
			-Wgm(i,0),-Wgm(i,1),-Wgm(i,2),0;
		double w0=sqrt(Wgm(i,0)*Wgm(i,0)+Wgm(i,1)*Wgm(i,1)+Wgm(i,2)*Wgm(i,2));
		MatrixXd eye44(4,4);
		eye44<<1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1;
		if(num==0)
		{
			W_Att.row(i).transpose()=(eye44*cos(w0*dt/2)+F*sin(w0*dt/2)/w0)*SAm1;
			//cout<<"Gyro Deter:"<<W_Att.row(i)<<endl;
			fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",UTC[i],W_Att(i,3),W_Att(i,0),W_Att(i,1),W_Att(i,2));
			num++;
		}
		else
		{
			W_Att.row(i).transpose()=(eye44*cos(w0*dt/2)+F*sin(w0*dt/2)/w0)*W_Att.row(i-1).transpose();
			fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",UTC[i],W_Att(i,3),W_Att(i,0),W_Att(i,1),W_Att(i,2));
			//cout<<"Gyro Deter:"<<W_Att.row(i)<<endl;
		}

	}

}

//////////////////////////////////////////////////////////////////////////
//和星上滤波结果的比较
//////////////////////////////////////////////////////////////////////////
void AttDeter::AttdeterCompareAOCC(vector<Quat> StarDat,string AOCC)
{
	FILE *fp=fopen(AOCC.c_str(),"r");
	int i,m,n;
	n=StarDat.size();//根据星敏原始数据计算的姿态数量
	Quat *StarDatOri = new Quat[n];
	for (i=0;i<n;i++)
	{
		StarDatOri[i].UTC=StarDat.at(i).UTC;
		StarDatOri[i].Q0=StarDat.at(i).Q0,StarDatOri[i].Q1=StarDat.at(i).Q1;
		StarDatOri[i].Q2=StarDat.at(i).Q2,StarDatOri[i].Q3=StarDat.at(i).Q3;
	}
	fscanf(fp,"%*f\t%*f\n%*s\n%d\n",&m);//星上滤波的姿态数量
	Quat *AOCCatt = new Quat[m];
	double *UTC = new double[m];
	for (i=0;i<m;i++)
	{
		fscanf(fp,"%*s\t%lf\t%lf\t%lf\t%lf\t%lf\n",&AOCCatt[i].UTC,&AOCCatt[i].Q0,&AOCCatt[i].Q1,&AOCCatt[i].Q2,&AOCCatt[i].Q3);
		UTC[i]=AOCCatt[i].UTC;
	}
	Quat *StarDatInt = new Quat[m];
	QuatInterpolation(StarDatOri,n,UTC,m,StarDatInt);

	string path = string(AOCC);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_Compare.txt";
	FILE *fpres=fopen(strpath.c_str(),"w");

	double dq1[4],dq2[4],dq3[4];
	SateEuler *EulerArray = new SateEuler[m];
	for (i=0;i<m;i++)
	{
		//注意dq1里的负号
		dq1[0]=-StarDatInt[i].Q0,dq1[1]=StarDatInt[i].Q1,dq1[2]=StarDatInt[i].Q2,dq1[3]=StarDatInt[i].Q3;
		dq2[0]=AOCCatt[i].Q0,dq2[1]=AOCCatt[i].Q1,dq2[2]=AOCCatt[i].Q2,dq2[3]=AOCCatt[i].Q3;
		quatmult(dq1,dq2,dq3);
		dq3[1]=dq3[1]*2/PI*180*3600,dq3[2]=dq3[2]*2/PI*180*3600,dq3[3]=dq3[3]*2/PI*180*3600;
		//fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\n",UTC[i],dq3[1],dq3[2],dq3[3]);
		EulerArray[i].R = dq3[1];	
		EulerArray[i].P = dq3[2];	
		EulerArray[i].Y = dq3[3];
		EulerArray[i].UTC = UTC[i];		
	}
	Sim.MatPlotDetQ(EulerArray,m);

}

//////////////////////////////////////////////////////////////////////////
//功能：和502精密定姿结果的比较
//输入：StarDat，星敏四元数；PAD，502精密定姿结果路径
//输出：与502路径相同的比较结果txt
//注意：
//日期：2016.07.19
//////////////////////////////////////////////////////////////////////////
void AttDeter::AttdeterCompare502(vector<Quat> StarDat,string PAD)
{
	int i,m,n;
	CMarkup xml;
	xml.Load(PAD);
	xml.FindElem("data");	
	xml.IntoElem();
	xml.FindElem("attitudeslist");	
	xml.FindChildElem("attitude_num");
	string strm = xml.GetChildData();
	m = atof(strm.c_str());
	Quat *Att = new Quat[m];	
	double *UTC = new double[m];		
	double mjd,jd0,fracsec,refsec,second;
	int year, month, day, hour, minute;
	string strdat;
	Cal2JD(2009,1,1,0,&jd0,&mjd);
	for(i=0;i<m;i++)
	{
		xml.FindChildElem("quaternion");
		xml.IntoElem();
		xml.FindChildElem("q1");
		strdat=xml.GetChildData();
		Att[i].Q1=atof(strdat.c_str());
		xml.FindChildElem("q2");
		strdat=xml.GetChildData();
		Att[i].Q2=atof(strdat.c_str());
		xml.FindChildElem("q3");
		strdat=xml.GetChildData();
		Att[i].Q3=atof(strdat.c_str());
		Att[i].Q0=sqrt(1-pow(Att[i].Q1,2)-pow(Att[i].Q2,2)-pow(Att[i].Q3,2));
		xml.FindChildElem("time");
		strdat=xml.GetChildData();
		sscanf(strdat.c_str(),"%4d%2d%2d%2d%2d%lf",&year,&month,&day,&hour,&minute,&second);
		FromYMDtoSecond(mjd,year,month,day,hour,minute,second,refsec);
		Att[i].UTC=refsec;
		UTC[i]=refsec;
		xml.OutOfElem();
	}
	
	//n=StarDat.size();//根据星敏原始数据计算的姿态数量
	//Quat *StarDatOri = new Quat[n];
	//for (i=0;i<n;i++)
	//{
	//	StarDatOri[i].UTC=StarDat.at(i).UTC;
	//	StarDatOri[i].Q0=StarDat.at(i).Q0,StarDatOri[i].Q1=StarDat.at(i).Q1;
	//	StarDatOri[i].Q2=StarDat.at(i).Q2,StarDatOri[i].Q3=StarDat.at(i).Q3;
	//}
	//		
	//Quat *StarDatInt = new Quat[m];
	//QuatInterpolation(StarDatOri,n,UTC,m,StarDatInt);

	string path = string(PAD);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_Compare502.txt";
	FILE *fpres=fopen(strpath.c_str(),"w");

	double dq1[4],dq2[4],dq3[4];
	SateEuler *EulerArray = new SateEuler[m];
	for (i=0;i<m;i++)
	{
		//注意dq1里的负号
		dq1[0]=-StarDat.at(i).Q0,dq1[1]=StarDat.at(i).Q1,dq1[2]=StarDat.at(i).Q2,dq1[3]=StarDat.at(i).Q3;
		dq2[0]=Att[i].Q0,dq2[1]=Att[i].Q1,dq2[2]=Att[i].Q2,dq2[3]=Att[i].Q3;
		quatmult(dq1,dq2,dq3);
		dq3[1]=dq3[1]*2/PI*180*3600,dq3[2]=dq3[2]*2/PI*180*3600,dq3[3]=dq3[3]*2/PI*180*3600;
		EulerArray[i].R = dq3[1];	
		EulerArray[i].P = dq3[2];	
		EulerArray[i].Y = dq3[3];
		EulerArray[i].UTC = UTC[i];
		//fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\n",UTC[i],dq3[1],dq3[2],dq3[3]);
	}
	Sim.MatPlotDetQ(EulerArray,m);	
}

//////////////////////////////////////////////////////////////////////////
//功能：502精密定姿与星上结果比较
//输入：AOCC:星上结果，PAD:502精密定姿结果路径
//输出：与502路径相同的比较结果txt
//注意：
//日期：2016.08.08
//////////////////////////////////////////////////////////////////////////
void AttDeter::AttdeterCompareAOCCwith502(string AOCC,string PAD)
{
	//读取502精密定姿结果
	int i,m,n;
	CMarkup xml;
	xml.Load(PAD);
	xml.FindElem("data");	
	xml.IntoElem();
	xml.FindElem("attitudeslist");	
	xml.FindChildElem("attitude_num");
	string strm = xml.GetChildData();
	m = atof(strm.c_str());
	Quat *Att = new Quat[m];			
	double mjd,jd0,fracsec,refsec,second;
	int year, month, day, hour, minute;
	string strdat;
	Cal2JD(2009,1,1,0,&jd0,&mjd);
	for(i=0;i<m;i++)
	{
		xml.FindChildElem("quaternion");
		xml.IntoElem();
		xml.FindChildElem("q1");
		strdat=xml.GetChildData();
		Att[i].Q1=atof(strdat.c_str());
		xml.FindChildElem("q2");
		strdat=xml.GetChildData();
		Att[i].Q2=atof(strdat.c_str());
		xml.FindChildElem("q3");
		strdat=xml.GetChildData();
		Att[i].Q3=atof(strdat.c_str());
		Att[i].Q0=sqrt(1-pow(Att[i].Q1,2)-pow(Att[i].Q2,2)-pow(Att[i].Q3,2));
		xml.FindChildElem("time");
		strdat=xml.GetChildData();
		sscanf(strdat.c_str(),"%4d%2d%2d%2d%2d%lf",&year,&month,&day,&hour,&minute,&second);
		FromYMDtoSecond(mjd,year,month,day,hour,minute,second,refsec);
		Att[i].UTC=refsec;
		xml.OutOfElem();
	}

	//读取星上滤波结果
	FILE *fp=fopen(AOCC.c_str(),"r");
	fscanf(fp,"%*f\t%*f\n%*s\n%d\n",&n);//星上滤波的姿态数量
	Quat *AOCCatt = new Quat[n];
	double *UTC = new double[n];
	for (i=0;i<n;i++)
	{
		fscanf(fp,"%*s\t%lf\t%lf\t%lf\t%lf\t%lf\n",&AOCCatt[i].UTC,&AOCCatt[i].Q0,&AOCCatt[i].Q1,&AOCCatt[i].Q2,&AOCCatt[i].Q3);
		UTC[i]=AOCCatt[i].UTC;
	}
	Quat *StarDatInt = new Quat[n];
	QuatInterpolation(Att,m,UTC,n,StarDatInt);
	
	string path = string(PAD);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_AOCC-502.txt";
	FILE *fpres=fopen(strpath.c_str(),"w");

	double dq1[4],dq2[4],dq3[4];
	SateEuler *EulerArray = new SateEuler[n];
	for (i=0;i<n;i++)
	{
		//注意dq1里的负号
		dq1[0]=-AOCCatt[i].Q0,dq1[1]=AOCCatt[i].Q1,dq1[2]=AOCCatt[i].Q2,dq1[3]=AOCCatt[i].Q3;
		dq2[0]=StarDatInt[i].Q0,dq2[1]=StarDatInt[i].Q1,dq2[2]=StarDatInt[i].Q2,dq2[3]=StarDatInt[i].Q3;
		quatmult(dq1,dq2,dq3);
		dq3[1]=dq3[1]*2/PI*180*3600,dq3[2]=dq3[2]*2/PI*180*3600,dq3[3]=dq3[3]*2/PI*180*3600;
		EulerArray[i].R = dq3[1];	
		EulerArray[i].P = dq3[2];	
		EulerArray[i].Y = dq3[3];
		EulerArray[i].UTC = UTC[i];
		//fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\n",UTC[i],dq3[1],dq3[2],dq3[3]);
	}
	
	Sim.MatPlotDetQ(EulerArray,n);	
}
//////////////////////////////////////////////////////////////////////////
//功能：读范城城姿态数据
//输入：AttData路径
//输出：
//注意：
//日期：2016.07.27
//////////////////////////////////////////////////////////////////////////
bool AttDeter::WMData(string AttData)
{
	FILE *fp = fopen(AttData.c_str(),"r");
	int i,m;
	fscanf(fp,"%d\n",&m);
	Quat *StarX = new Quat[m];
	Quat *StarY = new Quat[m];
	Quat *StarZ = new Quat[m];

	for (i=0;i<m;i++)
	{
		fscanf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",&StarX[i].Q1,&StarX[i].Q2,&StarX[i].Q3,
			&StarY[i].Q1,&StarY[i].Q2,&StarY[i].Q3,&StarZ[i].Q1,&StarZ[i].Q2,&StarZ[i].Q3);
		StarX[i].Q0=sqrt(1-pow(StarX[i].Q1,2)-pow(StarX[i].Q2,2)-pow(StarX[i].Q3,2));
		StarY[i].Q0=sqrt(1-pow(StarY[i].Q1,2)-pow(StarY[i].Q2,2)-pow(StarY[i].Q3,2));
		StarZ[i].Q0=sqrt(1-pow(StarZ[i].Q1,2)-pow(StarZ[i].Q2,2)-pow(StarZ[i].Q3,2));

	}

	string path = string(AttData);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_res.txt";
	FILE *fpres=fopen(strpath.c_str(),"w");
	double mz[3]={0,0,1},RA[9],RB[9],za[3],zb[3],Angle;
	for(int i=0;i<m;i++)
	{
		quat2matrix(StarY[i].Q1,StarY[i].Q2,StarY[i].Q3,StarY[i].Q0,RA);
		quat2matrix(StarZ[i].Q1,StarZ[i].Q2,StarZ[i].Q3,StarZ[i].Q0,RB);//Crj
		invers_matrix(RA,3);
		invers_matrix(RB,3);//Cjr
		mult(RA,mz,za,3,3,1);
		mult(RB,mz,zb,3,3,1);//星敏光轴在惯性系中的坐标值
		Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
		fprintf(fpres,"%.9f\n",Angle);
	}
	return 1;
}


//////////////////////////////////////////////////////////////////////////
//姿态确定主程序,根据各种情况选择
//////////////////////////////////////////////////////////////////////////
void AttDeter::AttSolution(vector<STGData>	AttData,string STGpath)
{
	ParseSTG ZY302;
	ZY302.ParseZY302_STG(STGpath,AttData);
	vector<Quat>AttDet;

	if (SingleStar(AttData,1,AttDet)!=1)
	{
		printf("星敏A没有数据或者有问题");
	}
	if (SingleStar(AttData,2,AttDet)!=1)
	{
		printf("星敏B没有数据或者有问题");
	}
	if (SingleStar(AttData,3,AttDet)!=1)
	{
		printf("星敏C没有数据或者有问题");
	}
	//DoubleStar(AttData,AttDet,STGpath,23);//23表示星敏2和3双矢量定姿
	//GyroAtt(AttData,AttDet,STGpath,2);

	//解析STG数据并定姿，然后与AOCC或者502定姿结果作对比
	string auxpath = STGpath;
	string sDir =  auxpath.substr(0,auxpath.rfind('\\'))+"\\NAD.ATT";
	//EKF6StateV2(AttData,AttDet,STGpath,23);
	//AttdeterCompareAOCC(AttDet,sDir);
	//sDir =  auxpath.substr(0,auxpath.rfind('\\'))+"\\502精密定姿.xml";
	//AttdeterCompare502(AttDet,sDir);
	//string sDir1 =  auxpath.substr(0,auxpath.rfind('\\'))+"\\NAD.ATT";
	//string sDir2 =  auxpath.substr(0,auxpath.rfind('\\'))+"\\502精密定姿.xml";
	//AttdeterCompareAOCCwith502(sDir1,sDir2);
}
