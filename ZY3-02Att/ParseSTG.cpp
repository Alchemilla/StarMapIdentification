#include "ParseSTG.h"


void ParseSTG::ParseZY302_STG(string STGpath,vector<STGData> &ZY3_02STGdata)
{
	ifstream in;
	in.open(STGpath.c_str(),ios::binary);

	in.seekg(0, ios::end);
	ios::pos_type ss = in.tellg();
	int nSize = (int)ss;
	nSize=nSize/120;
	in.seekg(0,ios::beg);
	
	int i;
	double ta,tb,tc;
	int ka=0,kb=0,kc=0;
	STGData *ZY_3Data=new STGData[nSize];
	memset(ZY_3Data,0,4096);
	for (i=0;i<nSize;i++)
	{
		in.seekg(4,ios::cur);
		unsigned char a[8];
		in.read((char*)&a,8);
			ta=RevDouble(a);
			ZY_3Data[i].StarA.UTC=ta;
			//Ϊ�������STG��������ӵ��ж�
			if (ka==0){ZY_3Data[i].StarA.UTC=ta;ka++;}
			else if (ta-ZY_3Data[i-1].StarA.UTC<-0.3){ZY_3Data[i].StarA.UTC=ta+1;}
			else if (ta-ZY_3Data[i-1].StarA.UTC>0.3){	ZY_3Data[i].StarA.UTC=ta-1;}
			else{ZY_3Data[i].StarA.UTC=ta;}
		in.seekg(1,ios::cur);
			long Q1temp,Q2temp,Q3temp;
			in.read((char*)&Q1temp,4);
			ZY_3Data[i].StarA.Q1=ReverseQ(Q1temp);
			in.read((char*)&Q2temp,4);
			ZY_3Data[i].StarA.Q2=ReverseQ(Q2temp);
			in.read((char*)&Q3temp,4);
			ZY_3Data[i].StarA.Q3=ReverseQ(Q3temp);
			ZY_3Data[i].StarA.Q0=sqrt(1-ZY_3Data[i].StarA.Q1*ZY_3Data[i].StarA.Q1-ZY_3Data[i].StarA.Q2*ZY_3Data[i].StarA.Q2-ZY_3Data[i].StarA.Q3*ZY_3Data[i].StarA.Q3);

		in.read((char*)&a,8);
			tb=RevDouble(a);
			ZY_3Data[i].StarB.UTC=tb;
			//Ϊ�������STG��������ӵ��ж�
			if (kb==0){ZY_3Data[i].StarB.UTC=tb;kb++;}
			else if (tb-ZY_3Data[i-1].StarB.UTC<-0.3){ZY_3Data[i].StarB.UTC=tb+1;}
			else if (tb-ZY_3Data[i-1].StarB.UTC>0.3){	ZY_3Data[i].StarB.UTC=tb-1;}
			else{ZY_3Data[i].StarB.UTC=tb;}
		in.seekg(1,ios::cur);
			in.read((char*)&Q1temp,4);
			ZY_3Data[i].StarB.Q1=ReverseQ(Q1temp);
			in.read((char*)&Q2temp,4);
			ZY_3Data[i].StarB.Q2=ReverseQ(Q2temp);
			in.read((char*)&Q3temp,4);
			ZY_3Data[i].StarB.Q3=ReverseQ(Q3temp);
			ZY_3Data[i].StarB.Q0=sqrt(1-ZY_3Data[i].StarB.Q1*ZY_3Data[i].StarB.Q1-ZY_3Data[i].StarB.Q2*ZY_3Data[i].StarB.Q2-ZY_3Data[i].StarB.Q3*ZY_3Data[i].StarB.Q3);

		in.read((char*)&a,8);
			tc=RevDouble(a);
			ZY_3Data[i].StarC.UTC=tc;
			//Ϊ�������STG��������ӵ��ж�
			if (kc==0){ZY_3Data[i].StarC.UTC=tc;kc++;}
			else if (tc-ZY_3Data[i-1].StarC.UTC<-0.3){ZY_3Data[i].StarC.UTC=tc+1;}
			else if (tc-ZY_3Data[i-1].StarC.UTC>0.3){	ZY_3Data[i].StarC.UTC=tc-1;}
			else{ZY_3Data[i].StarC.UTC=tc;}
		in.seekg(1,ios::cur);
			in.read((char*)&Q1temp,4);
			ZY_3Data[i].StarC.Q1=ReverseQ(Q1temp);
			in.read((char*)&Q2temp,4);
			ZY_3Data[i].StarC.Q2=ReverseQ(Q2temp);
			in.read((char*)&Q3temp,4);
			ZY_3Data[i].StarC.Q3=ReverseQ(Q3temp);
			ZY_3Data[i].StarC.Q0=sqrt(1-ZY_3Data[i].StarC.Q1*ZY_3Data[i].StarC.Q1-ZY_3Data[i].StarC.Q2*ZY_3Data[i].StarC.Q2-ZY_3Data[i].StarC.Q3*ZY_3Data[i].StarC.Q3);

		in.read((char*)&a,8);
			ZY_3Data[i].utgyro=RevDouble(a);
			short gy;
			in.read((char*)&gy,2);
			ZY_3Data[i].g1=gy*0.000022/0.25;
			//ZY_3Data[i].g1=gy*0.000022/0.25*1.115454545;//ZY3-01�����ݵı���ϵ��
			in.read((char*)&gy,2);
			ZY_3Data[i].g2=gy*0.000022/0.25;
			in.read((char*)&gy,2);
			ZY_3Data[i].g3=gy*0.000022/0.25;
			//ZY_3Data[i].g3=gy*0.000022/0.25*1.073181818;//ZY3-01�����ݵı���ϵ��
			in.read((char*)&gy,2);
			ZY_3Data[i].g4=gy*0.000022/0.25;
			in.read((char*)&gy,2);
			ZY_3Data[i].g5=gy*0.000022/0.25;
			//ZY_3Data[i].g5=gy*0.000022/0.25*1.103181818;//ZY3-01�����ݵı���ϵ��
			in.read((char*)&gy,2);
			ZY_3Data[i].g6=gy*0.000022/0.25;
			in.read((char*)&gy,2);
			ZY_3Data[i].g7=gy*0.00005/0.25;
			in.read((char*)&gy,2);
			ZY_3Data[i].g8=gy*0.00005/0.25;
			in.read((char*)&gy,2);
			ZY_3Data[i].g9=gy*0.00005/0.25;

			long bsx,bsy,bsz;
			in.read((char*)&bsx,4);
			in.read((char*)&bsy,4);
			in.read((char*)&bsz,4);
			ZY_3Data[i].bx=Reverse2(bsx);
			ZY_3Data[i].by=Reverse2(bsy);
			ZY_3Data[i].bz=Reverse2(bsz);
			
			in.seekg(3,ios::cur);
			in.read((char*)&gy,2);
			ZY_3Data[i].g10=gy*0.000025/0.25;
			in.read((char*)&gy,2);
			ZY_3Data[i].g11=gy*0.000025/0.25;
			in.read((char*)&gy,2);
			ZY_3Data[i].g12=gy*0.000025/0.25;	
		
			char sign;
			in.read((char*)&sign,1);
			//�ж�����B��C�ķ���
			if (sign==0)
			{
			} 
			else if(sign==1)
			{
				ZY_3Data[i].StarB.Q0=-ZY_3Data[i].StarB.Q0;
			}
			else if(sign==2)
			{
				ZY_3Data[i].StarC.Q0=-ZY_3Data[i].StarC.Q0;			
			}
			else if(sign==3)
			{
				ZY_3Data[i].StarB.Q0=-ZY_3Data[i].StarB.Q0;
				ZY_3Data[i].StarC.Q0=-ZY_3Data[i].StarC.Q0;
			}			
			ZY3_02STGdata.push_back(ZY_3Data[i]);
			in.seekg(5,ios::cur);
	}

	string path = string(STGpath);
	path = path.substr(0,path.rfind('.'));
	string strpath =path+"_STG_parse.txt";
	FILE *fpres=fopen(strpath.c_str(),"w");
	for (i=0;i<nSize;i++)
	{
		fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t",ZY_3Data[i].StarA.UTC,ZY_3Data[i].StarA.Q0,ZY_3Data[i].StarA.Q1,ZY_3Data[i].StarA.Q2,ZY_3Data[i].StarA.Q3);
		fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t",ZY_3Data[i].StarB.UTC,ZY_3Data[i].StarB.Q0,ZY_3Data[i].StarB.Q1,ZY_3Data[i].StarB.Q2,ZY_3Data[i].StarB.Q3);
		fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t",ZY_3Data[i].StarC.UTC,ZY_3Data[i].StarC.Q0,ZY_3Data[i].StarC.Q1,ZY_3Data[i].StarC.Q2,ZY_3Data[i].StarC.Q3);
		fprintf(fpres,"%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",ZY_3Data[i].utgyro,ZY_3Data[i].g1,
		ZY_3Data[i].g2,ZY_3Data[i].g3,ZY_3Data[i].g4,ZY_3Data[i].g5,ZY_3Data[i].g6,ZY_3Data[i].g7,ZY_3Data[i].g8,ZY_3Data[i].g9,ZY_3Data[i].g10,
		ZY_3Data[i].g11,ZY_3Data[i].g12,ZY_3Data[i].bx,ZY_3Data[i].by,ZY_3Data[i].bz);
	}

}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ݺ��ǳྭ��γ��������̬������������������
//���룺starCatlog�����ź��ǲ���������J2000ϵ�µĵ�λʸ��V[3]
//		R����������ϵ��J2000ϵ����ת��ϵ��Crj
//�����x,y����������
//ע�⣺������ΪAPS��������
//���ڣ�2016.11.01
//////////////////////////////////////////////////////////////////////////
void ParseSTG::FromLL2XY(Star starCatlog, double *R, double &x, double &y)
{
	double V[3],W[3];
	V[0]=starCatlog.V[0];V[1]=starCatlog.V[1];V[2]=starCatlog.V[2];
	mult(R,V,W,3,3,1);
	//x0=y0=512,f=43.3mm,��Ԫ��С0.015mm
	if(W[2]>0)
	{
		x = (512*0.015-W[0]/W[2]*43.3)/0.015;
		y = (512*0.015-W[1]/W[2]*43.3)/0.015;
	}
	else
	{x=-1,y=-1;}//����������ж��Ƿ������ָ��İ�����һ��
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ݺ������ȵȼ������������ͼ������DNֵ
//���룺Mag:�����ǵȣ�
//�����DNֵ
//ע�⣺�����ǵ�3Ϊ255���ȣ�С���ǵ�3��ҲΪ255
//���ڣ�2016.12.08
//////////////////////////////////////////////////////////////////////////
int ParseSTG::Mag2DN(double Mag)
{
	int DN = 255*pow(100,(4-Mag)/5);
	return DN;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���������APS������Ԫ��������ͼ
//���룺ZY3_02STGdata��STG����������̬����
//�������Ԫ����Ӧ��ͼ
//ע�⣺����AΪAPS��������
//���ڣ�2016.10.31
//////////////////////////////////////////////////////////////////////////
void ParseSTG::StarMap(vector<STGData> ZY3_02STGdata)
{
	//���Ǳ��ļ�
	string starCatlogpath = "D:\\2_ImageData\\ZY3-02\\��ͼ����\\star1";	
	FILE *fp;
	fp = fopen(starCatlogpath.c_str(), "r");
	if (fp == NULL)
	{
		printf("��%s�ļ�ʧ��,��ȷ���ļ�·���Ƿ���ȷ!\n", starCatlogpath.c_str());
		return;
	}
	int i,n;
	fscanf(fp,"%d",&n);//��ȡ�Ǳ�����
	Star *starCatlog = new Star[n];
	for(i=0;i<n;i++)
	{
		fscanf(fp,"%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d\n",&starCatlog[i].ID,&starCatlog[i].phiX,&starCatlog[i].phiY,
			&starCatlog[i].mag,&starCatlog[i].V[0],&starCatlog[i].V[1],&starCatlog[i].V[2],&starCatlog[i].DN);
	}
	
	int j,m = ZY3_02STGdata.size();
	double R[9],za[3]={0,0,1},zc[3];
	double px,py,x,y;
	// Ӱ��Ŀ�Ⱥ͸߶�
	long width = 1024, height = 1024;
	//������ƽ������ֵ����
	byte *UnitData = new byte[1024 * 1024];

	//������ͼ�ļ���
	string imgtmp = workpath+"��ͼ";
	char * imgpath = (char *)imgtmp.data();
	if (_mkdir(imgpath) == 0);	
	string imgtxt = workpath + "��ͼ\\��������.txt";
	FILE *fptxt = fopen(imgtxt.c_str(),"w");

	//������һ֡����������������	
	quat2matrix(ZY3_02STGdata[0].StarA.Q1,ZY3_02STGdata[0].StarA.Q2,
		ZY3_02STGdata[0].StarA.Q3,ZY3_02STGdata[0].StarA.Q0,R);//Crj
	for (j=0;j<n;j++)
	{
		FromLL2XY(starCatlog[j],R,x,y);//���Ǳ�ÿ���Ǳ�����������������
		if (x>0&&x<1024&&y>0&&y<1024)
		{
			int xPixel = int(x+0.5);
			int yPixel = int(y+0.5);
			UnitData[yPixel*width + xPixel] = starCatlog[j].DN;
			fprintf(fptxt,"%d\t%d\t%.1f\n",xPixel,yPixel,starCatlog[j].mag);
		}
	}
	fclose(fptxt);

	//������̬�³���ͼ
	//for (i=0;i<m;i++)
	for (i=900;i<1100;i++)
	{
		memset(UnitData, 0, sizeof(byte)* 1024 * 1024);//Ӱ������ֵ��Ϊ0
		//����A��APS��������
		//quat2matrix(ZY3_02STGdata[i].StarA.Q1,ZY3_02STGdata[i].StarA.Q2,
		//	ZY3_02STGdata[i].StarA.Q3,ZY3_02STGdata[i].StarA.Q0,R);//Crj
		//����B����
		quat2matrix(ZY3_02STGdata[i].StarB.Q1,ZY3_02STGdata[i].StarB.Q2,
			ZY3_02STGdata[i].StarB.Q3,ZY3_02STGdata[i].StarB.Q0,R);//Crj
		for (j=0;j<n;j++)
		{
			FromLL2XY(starCatlog[j],R,x,y);//���Ǳ�ÿ���Ǳ�����������������
			if (x>2&&x<1022&&y>2&&y<1022)
			{
				int xPixel = int(x+0.5)-2;
				int yPixel = int(y+0.5)-2;
				for (int ii=0;ii<5;ii++)//����5�����ش�С����ͼ
				{
					for (int jj=0;jj<5;jj++)
					{
						//UnitData[yPixel*width + xPixel] = starCatlog[j].DN;
						//�� X��Y�� ת��Ϊ X��Y��
						int xTrans = yPixel;
						int yTrans = 1024 - xPixel;
						//int DN = Mag2DN(starCatlog[j].mag);
						UnitData[yTrans*width + xTrans] = Mag2DN(starCatlog[j].mag);
						xPixel++;
					}		
					xPixel-=5;
					yPixel++;
				}				
			}
		}
		//����Ӱ��
		char tempath[100];
		// ���Ӱ�����
		string outdriver = "GTiff";
		//ͶӰ
		string tarProject = "PROJCS[\"UTM_Zone_50N\", GEOGCS[\"GCS_WGS_1984\", DATUM[\"WGS_1984\", SPHEROID[\"WGS_1984\", 6378137.0, 298.2572235630016],TOWGS84[0,0,0,0,0,0,0]], PRIMEM[\"Greenwich\", 0.0], UNIT[\"Degree\", 0.0174532925199433]], PROJECTION[\"Transverse_Mercator\"], PARAMETER[\"False_Easting\", 500000.0], PARAMETER[\"False_Northing\", 0.0], PARAMETER[\"Central_Meridian\", 117.0], PARAMETER[\"Scale_Factor\", 0.9996], PARAMETER[\"Latitude_Of_Origin\", 0.0], UNIT[\"Meter\", 1.0]]";
		//����任
		double minx = 0, maxy = 1024, resolution = 1;
		double adfGeoTransform[6] = { minx, resolution, 0, maxy, 0, -resolution };
		//sprintf(tempath, "������ͼӰ��%02d_UT%.4lf.tif", i+1,ZY3_02STGdata[i].StarA.UTC);
		sprintf(tempath, "����B����%02d_UT%.4lf.tif", i+1,ZY3_02STGdata[i].StarB.UTC);
		string imgpath = workpath +"��ͼ\\" +tempath;
		GeoReadImage m_out;
		m_out.New(imgpath, outdriver, GDT_Byte, width, height, 1);
		m_out.poDataset->SetProjection(tarProject.c_str());
		m_out.poDataset->SetGeoTransform(adfGeoTransform);
		//���·�ʽ��Ӱ��
		m_out.Open(imgpath, GA_Update);
		if (m_out.m_isopen == true)
			printf("\rUpdate Img (%s)", imgpath.c_str());
		else
		{
			printf("\rUpdate Img (%s) Failed", imgpath.c_str());
			return ;
		}
		//����out������
		m_out.SetBuffer(0, 0, width, height, m_out.pBuffer[0]);
		double gray;
		for (int yPixel = 0; yPixel <height; yPixel++)       //y����
		{
			for (int xPixel = 0; xPixel < width; xPixel++)   //x����
			{
				//��������
				gray = UnitData[yPixel*width + xPixel];
				//gray = 0;
				m_out.SetDataValue(xPixel, yPixel, gray, 0);    //��ֵ
			}
		}
		//д������
		bool iswrite = true;
		iswrite *= m_out.WriteBlock(0, 0, width, height, 0, m_out.pBuffer[0]);
		//�ر�Ӱ��
		m_out.Destroy();	
	}	
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����APS��ͼʶ�����������´�STG���ݶԱ�
//���룺ZY3_02STGdata��STG����������̬����
//      IDpath����ͼʶ��õ���APS������Ԫ��
//�����ͬĿ¼�¶ԱȽ��
//ע�⣺����AΪAPS��������
//���ڣ�2016.11.02
//////////////////////////////////////////////////////////////////////////
void ParseSTG::StarIDComp(vector<STGData> ZY3_02STGdata,string IDpath)
{
	FILE *fp;
	fp = fopen(IDpath.c_str(),"r");
	int i,m,n;
	fscanf(fp,"%d",&m);//��ͼʶ����Ԫ������ 2Hz
	n = ZY3_02STGdata.size();//STG����������Ԫ������ 4Hz
	Quat *Qid = new Quat[m];
	Quat *Qidint = new Quat[n];
	double *UT = new double[n];
	for (i=0;i<m;i++)
	{
		fscanf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\n",&Qid[i].UTC,&Qid[i].Q0,&Qid[i].Q1,&Qid[i].Q2,&Qid[i].Q3);
	}
	for (i=0;i<n;i++)
	{
		UT[i] = ZY3_02STGdata[i].StarA.UTC;
	}
	QuatInterpolation(Qid,m,UT,n,Qidint);//��̬�ڲ�
	
	string resPath = workpath + "StarId_Compare.txt";
	FILE *fpres;
	fpres = fopen(resPath.c_str(),"w");
	//Ȼ����бȽ����
	double qL[4],qR[4],dQ[4];
	SateEuler *EulerArray = new SateEuler[n];
	for (i=0;i<n;i++)
	{
		qL[0]=ZY3_02STGdata[i].StarA.Q0,qL[1]=ZY3_02STGdata[i].StarA.Q1,
		qL[2]=ZY3_02STGdata[i].StarA.Q2,qL[3]=ZY3_02STGdata[i].StarA.Q3;
		qR[0]=-Qidint[i].Q0,qR[1]=Qidint[i].Q1,qR[2]=Qidint[i].Q2,qR[3]=Qidint[i].Q3;
		quatmult(qL,qR,dQ);
		dQ[1]=2*dQ[1]/PI*180*3600;dQ[2]=2*dQ[2]/PI*180*3600;dQ[3]=2*dQ[3]/PI*180*3600;
		fprintf(fpres,"%.9lf\t%.9lf\t%.9lf\n",dQ[1],dQ[2],dQ[3]);
		EulerArray[i].R = dQ[1];	
		EulerArray[i].P = dQ[2];	
		EulerArray[i].Y = dQ[3];
		EulerArray[i].UTC = UT[i]-97499271;
	}
	Sim.MatPlotDetQ(EulerArray,n);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�APS��ͼʶ������Ԫ��������B������C����֮��ļнǹ�ϵ
//���룺ZY3_02STGdata��STG����������̬����
//      IDpath����ͼʶ��õ���APS������Ԫ��
//      StarTag��intֵ��12��ʾ����A��B
//�����ͬĿ¼�£�����нǶԱȽ��
//ע�⣺����AΪAPS��������
//���ڣ�2016.11.02
//////////////////////////////////////////////////////////////////////////
void ParseSTG::StarAngleAPS_B_C(vector<STGData> ZY3_02STGdata,string IDpath, int StarTag)
{
	int i,m,n;
	m=ZY3_02STGdata.size();	
	FILE *fp = fopen(IDpath.c_str(),"r");
	fscanf(fp,"%d",&n);
	Quat *StarA = new Quat[n];
	Quat *StarB = new Quat[m];
	Quat *StarC = new Quat[m];
	Quat *StarAi = new Quat[m];
	Quat *StarCi = new Quat[m];
	double *UT = new double[m];
	for (i=0;i<n;i++)
	{
		fscanf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\n",&StarA[i].UTC,&StarA[i].Q0,&StarA[i].Q1,&StarA[i].Q2,&StarA[i].Q3);
	}
	double mz[3]={0,0,1},RA[9],RB[9],za[3],zb[3],Angle;
	string strpath;
	if (StarTag==23)
	{
		for (i=0;i<m;i++)//����B��C���ݣ���BΪʱ���׼
		{
			UT[i]=ZY3_02STGdata.at(i).StarB.UTC;
			StarB[i].Q1=ZY3_02STGdata.at(i).StarB.Q1;StarB[i].Q2=ZY3_02STGdata.at(i).StarB.Q2;
			StarB[i].Q3=ZY3_02STGdata.at(i).StarB.Q3,StarB[i].Q0=ZY3_02STGdata.at(i).StarB.Q0;
			StarC[i].UTC=ZY3_02STGdata.at(i).StarC.UTC;
			StarC[i].Q1=ZY3_02STGdata.at(i).StarC.Q1;StarC[i].Q2=ZY3_02STGdata.at(i).StarC.Q2;
			StarC[i].Q3=ZY3_02STGdata.at(i).StarC.Q3,StarC[i].Q0=ZY3_02STGdata.at(i).StarC.Q0;
		}
		QuatInterpolation(StarC, m, UT, m,StarCi);
		strpath = workpath+"_Star23Angle.txt";	
		FILE *fpres=fopen(strpath.c_str(),"w");
		for(int i=0;i<m;i++)
		{
			quat2matrix(StarB[i].Q1,StarB[i].Q2,StarB[i].Q3,StarB[i].Q0,RA);
			quat2matrix(StarCi[i].Q1,StarCi[i].Q2,StarCi[i].Q3,StarCi[i].Q0,RB);//Crj
			invers_matrix(RA,3);
			invers_matrix(RB,3);//Cjr
			mult(RA,mz,za,3,3,1);
			mult(RB,mz,zb,3,3,1);//���������ڹ���ϵ�е�����ֵ
			Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
			fprintf(fpres,"%.9f\t%.9f\n",UT[i],Angle);
		}
	} 
	else if(StarTag==12)
	{
		for (i=0;i<m;i++)//����A��B���ݣ���BΪʱ���׼
		{
			UT[i]=ZY3_02STGdata.at(i).StarB.UTC;
			StarB[i].Q1=ZY3_02STGdata.at(i).StarB.Q1;StarB[i].Q2=ZY3_02STGdata.at(i).StarB.Q2;
			StarB[i].Q3=ZY3_02STGdata.at(i).StarB.Q3,StarB[i].Q0=ZY3_02STGdata.at(i).StarB.Q0;
		}
		QuatInterpolation(StarA, n, UT, m, StarAi);
		strpath =workpath+"_Star12Angle.txt";
		FILE *fpres=fopen(strpath.c_str(),"w");
		for(int i=0;i<m;i++)
		{
			quat2matrix(StarAi[i].Q1,StarAi[i].Q2,StarAi[i].Q3,StarAi[i].Q0,RA);
			quat2matrix(StarB[i].Q1,StarB[i].Q2,StarB[i].Q3,StarB[i].Q0,RB);//Crj
			invers_matrix(RA,3);
			invers_matrix(RB,3);//Cjr
			mult(RA,mz,za,3,3,1);
			mult(RB,mz,zb,3,3,1);//���������ڹ���ϵ�е�����ֵ
			Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
			fprintf(fpres,"%.9f\t%.9f\n",UT[i],Angle);
		}
	} 
	else if(StarTag==13)
	{
		for (i=0;i<m;i++)//����A��C���ݣ���CΪʱ���׼
		{
			UT[i]=ZY3_02STGdata.at(i).StarC.UTC;
			StarC[i].Q1=ZY3_02STGdata.at(i).StarC.Q1;StarC[i].Q2=ZY3_02STGdata.at(i).StarC.Q2;
			StarC[i].Q3=ZY3_02STGdata.at(i).StarC.Q3,StarC[i].Q0=ZY3_02STGdata.at(i).StarC.Q0;
		}
		QuatInterpolation(StarA, n, UT, m, StarAi);
		strpath =workpath+"_Star13Angle.txt";
		FILE *fpres=fopen(strpath.c_str(),"w");
		for(int i=0;i<m;i++)
		{
			quat2matrix(StarAi[i].Q1,StarAi[i].Q2,StarAi[i].Q3,StarAi[i].Q0,RA);
			quat2matrix(StarC[i].Q1,StarC[i].Q2,StarC[i].Q3,StarC[i].Q0,RB);//Crj
			invers_matrix(RA,3);
			invers_matrix(RB,3);//Cjr
			mult(RA,mz,za,3,3,1);
			mult(RB,mz,zb,3,3,1);//���������ڹ���ϵ�е�����ֵ
			Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
			fprintf(fpres,"%.9f\t%.9f\n",UT[i],Angle);
		}
	} 	
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡAPS������ͼʶ����
//���룺IDpath��ʶ�����ļ�·��
//�����APS��Ӧ��Ԫ��
//ע�⣺ֻ�õ�������A
//���ڣ�2016.12.05
//////////////////////////////////////////////////////////////////////////
void ParseSTG::ReadStarID(string IDpath,vector<STGData>&APSQ)
{
	FILE *fp;
	fp = fopen(IDpath.c_str(),"r");
	int i,m;
	fscanf(fp,"%d",&m);//��ͼʶ����Ԫ������ 2Hz
	STGData APStemp;
	for (i=0;i<m;i++)
	{
		fscanf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\n",&APStemp.StarA.UTC,&APStemp.StarA.Q0,
			&APStemp.StarA.Q1,&APStemp.StarA.Q2,&APStemp.StarA.Q3);
		APSQ.push_back(APStemp);
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡSTG��ȡ������̬����STA
//���룺STA����·��
//�������ȡ��STA��������
//ע�⣺
//���ڣ�2016.12.08
//////////////////////////////////////////////////////////////////////////
void ParseSTG::ReadSTAtxt(string STApath,vector<STGData>&STAdat)
{
	FILE *fp;
	fp = fopen(STApath.c_str(),"r");
	int i,m;
	STGData STAtemp,STAtemp2;
	//fscanf(fp,"%*s\t%*s\t\n%*s\t%*s\t\n%d\n",&m);//����ǰ���ж�ȡ������
	fscanf(fp,"%*s\t%*s\t\n%*s\t%*s\t\n%d\n%*s\n",&m);//����ǰ���ж�ȡ�����У�˳������������
	fscanf(fp,"%*s\t%lf\t%*d\t%lf\t%lf\t%lf\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\n",
			&STAtemp2.StarA.UTC,	&STAtemp2.StarA.Q1,	&STAtemp2.StarA.Q2,	&STAtemp2.StarA.Q3,
			&STAtemp2.StarB.UTC,	&STAtemp2.StarB.Q1,	&STAtemp2.StarB.Q2,	&STAtemp2.StarB.Q3,
			&STAtemp2.StarC.UTC,	&STAtemp2.StarC.Q1,	&STAtemp2.StarC.Q2,	&STAtemp2.StarC.Q3);
	for (i=0;i<m;i++)
	{
		fscanf(fp,"%*s\t%lf\t%*d\t%lf\t%lf\t%lf\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\n",
			&STAtemp.StarA.UTC,	&STAtemp.StarA.Q1,	&STAtemp.StarA.Q2,	&STAtemp.StarA.Q3,
			&STAtemp.StarB.UTC,	&STAtemp.StarB.Q1,	&STAtemp.StarB.Q2,	&STAtemp.StarB.Q3,
			&STAtemp.StarC.UTC,	&STAtemp.StarC.Q1,	&STAtemp.StarC.Q2,	&STAtemp.StarC.Q3);
		if (STAtemp.StarA.UTC<STAtemp2.StarA.UTC)
		{
			STAtemp.StarA.UTC +=1;
		}
		STAtemp2.StarA.UTC=STAtemp.StarA.UTC;
		if (STAtemp.StarB.UTC<STAtemp2.StarB.UTC)
		{
			STAtemp.StarB.UTC -=1;
		}
		STAtemp2.StarB.UTC=STAtemp.StarB.UTC;
		if (STAtemp.StarC.UTC<STAtemp2.StarC.UTC)
		{
			STAtemp.StarC.UTC -=1;
		}
		STAtemp2.StarC.UTC=STAtemp.StarC.UTC;
		STAtemp.StarA.Q0 = sqrt(1-pow(STAtemp.StarA.Q1,2)-pow(STAtemp.StarA.Q2,2)-pow(STAtemp.StarA.Q3,2));
		STAtemp.StarB.Q0 = sqrt(1-pow(STAtemp.StarB.Q1,2)-pow(STAtemp.StarB.Q2,2)-pow(STAtemp.StarB.Q3,2));
		STAtemp.StarC.Q0 = sqrt(1-pow(STAtemp.StarC.Q1,2)-pow(STAtemp.StarC.Q2,2)-pow(STAtemp.StarC.Q3,2));
		STAdat.push_back(STAtemp);
	}
}

