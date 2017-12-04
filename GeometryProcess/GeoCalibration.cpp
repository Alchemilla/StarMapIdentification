#include "GeoCalibration.h"


GeoCalibration::GeoCalibration(void)
{
}


GeoCalibration::~GeoCalibration(void)
{
}

/////////////////////////////////////////////
//功能：检校线性模型
//输入：姿态、轨道、行时、相机参数、模型参数、控制点vector
//输出：更新Ru参数
//注意：目前未知参数为三个角度
//日期：2016.12.21
////////////////////////////////////////////
void GeoCalibration::ExtOrientCali(GeoOrbit *orbit, GeoAttitude *att, GeoTime *time, GeoCamera *cam, StrModelParamInput input,vector<StrGCP>ZY3_GCP)
{
	m_orbit = orbit;
	m_att = att;
	m_time = time;
	m_cam = cam;
	m_input = input;
	// 获取模型影像长度和宽度
	m_xnum = m_time->get_num();
	m_ynum = m_cam->get_ynum();
	// 获取模型所定位的基准椭球参数
	m_datum = m_orbit->get_datum();
	// 计算姿轨多项式模型以及整数位的姿轨以及常见的安装
	m_att->get_ROff(m_body2att.R);//相对于对Ru赋初值单位阵
	ReCalPosAndAtt();

	//下面开始定标过程
	double xpixel, ypixel, X, Y, Z, Xs, Ys, Zs, t, Xt[3],Xr[3];
	double Xbar, Ybar, Zbar;
	int i,j, num = ZY3_GCP.size();
	//构建Ru并设初值为0；
	double phi=0,omg=0,kap=0,sinphi,sinomg,sinkap,cosphi,cosomg,coskap;	
	//设置迭代判断初值
	double iter_phi = PI, iter_omg = PI, iter_kap = PI, iter_count = 0;
	//有关法化计算的数组
	double ATA[9], ATL[3], L;

	while (ZY3_GCP.size())
	{
		memset(ATA, 0, sizeof(double)*9);	memset(ATL, 0, sizeof(double)*3);
		sinphi = sin(phi); sinomg = sin(omg); sinkap = sin(kap);
		cosphi = cos(phi); cosomg = cos(omg); coskap = cos(kap);		
		m_base.Eulor2Matrix(phi,omg,kap,213,Ru);	
		for (i=0; i<num; i++)
		{
			double Orb[3], Att[9], Inner[3];
			xpixel = ZY3_GCP[i].y;
			ypixel = ZY3_GCP[i].x;
			//根据像素位置得到轨道姿态和内方位元素
			GetOrbitAttitudeInnerBaseXY(xpixel, ypixel, Orb, Att, Inner);
			m_base.Geograph2Rect(m_datum,ZY3_GCP[i].lat/180*PI,ZY3_GCP[i].lon/180*PI,ZY3_GCP[i].h,X,Y,Z);
			//根据地面点位置和卫星位置计算摄影光线指向
			Xt[0] = X - Orb[0];	Xt[1] = Y - Orb[1];	Xt[2] = Z - Orb[2];
			m_base.invers_matrix(Att,3);
			m_base.Multi(Att,Xt,Xr,3,3,1);
			
			//组合为X_ Y_ Z_
			Xbar = Ru[0]*Xr[0] + Ru[3]*Xr[1] + Ru[6]*Xr[2];
			Ybar = Ru[1]*Xr[0] + Ru[4]*Xr[1] + Ru[7]*Xr[2];
			Zbar = Ru[2]*Xr[0] + Ru[5]*Xr[1] + Ru[8]*Xr[2];
	
			//旋转矩阵对各项的偏微分
			//对phi角的偏微分
			double partial_a1phi = -sinphi*coskap + cosphi*sinomg*sinkap;
			double partial_a2phi = 0;
			double partial_a3phi = -cosphi*coskap - sinphi*sinomg*sinkap;
			double partial_b1phi = sinphi*sinkap + cosphi*sinomg*coskap;
			double partial_b2phi = 0;
			double partial_b3phi = cosphi*sinkap - sinphi*sinomg*coskap;
			double partial_c1phi = cosphi*cosomg;
			double partial_c2phi = 0;
			double partial_c3phi = -sinphi*cosomg;
			//对omega角的偏微分
			double partial_a1omg = sinphi*cosomg*sinkap;
			double partial_a2omg = -sinomg*sinkap;
			double partial_a3omg = cosphi*cosomg*sinkap;
			double partial_b1omg = sinphi*cosomg*coskap;
			double partial_b2omg = -sinomg*coskap;
			double partial_b3omg = cosphi*cosomg*coskap;
			double partial_c1omg = -sinphi*sinomg;
			double partial_c2omg = -cosomg;
			double partial_c3omg = -cosphi*sinomg;
			//对kappa角的偏微分
			double partial_a1kap = -cosphi*sinkap + sinphi*sinomg*coskap;
			double partial_a2kap = cosomg*coskap;
			double partial_a3kap = sinphi*sinkap + cosphi*sinomg*coskap;
			double partial_b1kap = -cosphi*coskap - sinphi*sinomg*sinkap;
			double partial_b2kap = -cosomg*sinkap;
			double partial_b3kap = sinphi*coskap - cosphi*sinomg*sinkap;
			double partial_c1kap = 0;
			double partial_c2kap = 0;
			double partial_c3kap = 0;
			//求Xbar,Ybar,Zbar三者的偏导数
			double partial_Xbar_phi = Xr[0]*partial_a1phi + Xr[1]*partial_b1phi + Xr[2]*partial_c1phi;
			double partial_Xbar_omg = Xr[0]*partial_a1omg + Xr[1]*partial_b1omg + Xr[2]*partial_c1omg;
			double partial_Xbar_kap = Xr[0]*partial_a1kap + Xr[1]*partial_b1kap + Xr[2]*partial_c1kap;
			double partial_Ybar_phi = Xr[0]*partial_a2phi + Xr[1]*partial_b2phi + Xr[2]*partial_c2phi;
			double partial_Ybar_omg = Xr[0]*partial_a2omg + Xr[1]*partial_b2omg + Xr[2]*partial_c2omg;
			double partial_Ybar_kap = Xr[0]*partial_a2kap + Xr[1]*partial_b2kap + Xr[2]*partial_c2kap;
			double partial_Zbar_phi = Xr[0]*partial_a3phi + Xr[1]*partial_b3phi + Xr[2]*partial_c3phi;
			double partial_Zbar_omg = Xr[0]*partial_a3omg + Xr[1]*partial_b3omg + Xr[2]*partial_c3omg;
			double partial_Zbar_kap = Xr[0]*partial_a3kap + Xr[1]*partial_b3kap + Xr[2]*partial_c3kap;
			//方程Vx法化
			double Ax[3],Ay[3];
			Ax[0] = 1/Zbar*partial_Xbar_phi - Xbar/Zbar/Zbar*partial_Zbar_phi;
			Ax[1] = 1/Zbar*partial_Xbar_omg - Xbar/Zbar/Zbar*partial_Zbar_omg;
			Ax[2] = 1/Zbar*partial_Xbar_kap - Xbar/Zbar/Zbar*partial_Zbar_kap;
			L = Inner[0] - Xbar/Zbar;
			m_base.pNormal(Ax, 3, L, ATA, ATL, 1.0);
			//方程Vy法化
			Ay[0] = 1/Zbar*partial_Ybar_phi - Ybar/Zbar/Zbar*partial_Zbar_omg;
			Ay[1] = 1/Zbar*partial_Ybar_omg - Ybar/Zbar/Zbar*partial_Zbar_omg;
			Ay[2] = 1/Zbar*partial_Ybar_kap - Ybar/Zbar/Zbar*partial_Zbar_kap;			
			L = Inner[1] - Ybar/Zbar;
			m_base.pNormal(Ay, 3, L, ATA, ATL, 1.0);
		}
		//迭代求解
		m_base.solve33(ATA,ATL);
		if (abs(ATL[0]>iter_phi&&ATL[1]>iter_omg&&ATL[3]>iter_kap||iter_count>20))
			break;
		iter_count++;
		phi += ATL[0]; omg += ATL[1]; 	kap += ATL[2];
		m_base.Eulor2Matrix(phi,omg,kap,213,Ru);
		iter_phi = abs(ATL[0]), iter_omg = abs(ATL[1]), iter_kap = abs(ATL[2]);
	}
	OutputRu(phi, omg, kap);

	//// 开始计算精度
	//double mean = 0;
	//m_base.Eulor2Matrix(phi, omg, kap, 213, Ru);
	//double Ru0[9];
	//m_base.Multi(m_body2att.R, Ru, Ru0, 3, 3, 3);
	//memcpy(m_body2att.R, Ru0, sizeof(double)*9);
	//ComputerGlobalParam();
	//double dxy, dx, dy;
	//for(int i=0; i<num; i++)
	//{
	//	FromLatLon2XY(ZY3_GCP[i].lat/180*PI,ZY3_GCP[i].lon/180*PI, ZY3_GCP[i].h, dx, dy);
	//	dx = ZY3_GCP[i].y - dx;
	//	dy = ZY3_GCP[i].x - dy;
	//	mean += (dx*dx + dy*dy);
	//}
	//mean = sqrt(mean/num);
	//printf("偏置补偿后精度：%lf像素\n", mean);	
}

void GeoCalibration::OutputRu(double phi, double omg, double kap)
{
	char* ruPath = "D:\\2_ImageData\\ZY3-02\\1-定位精度\\503\\标定参数\\2016-07-02GZC.txt";
	FILE *fp = fopen(ruPath, "w");
	fprintf(fp, "%s\n%.9f\n%.9f\n%.9f\n", "phi omg kap", phi, omg, kap);
}
/////////////////////////////////////////////
//功能：检校线性模型2
//输入：面阵几何模型，控制点
//输出：更新Ru参数
//注意：
//作者：JYH
//日期：2017.08.18
////////////////////////////////////////////
bool GeoCalibration::calcOffsetMatrix(GeoModelArray* pModel, StrGCP* pGCP, int numGCP, OffsetAngle &angle)
{
	//int UnknowNum = 3;
	double A_T_P_A[9], A_T_P_L[3];
	int i;

	angle.RuKappa = angle.RuOmega = angle.RuPhi = 0;

	double pParams[3];
	double RCam2wgs84[9], Rwgs842cam[9], Ru[9], XYZ[3], Rinstall[9];
	int inter = 0;

	pModel->GetCam2WGS84(RCam2wgs84);
	memcpy(Rwgs842cam, RCam2wgs84, sizeof(double) * 9);
	m_base.invers_matrix(Rwgs842cam, 3);
	pModel->GetCamPos(XYZ);
	pModel->GetCamInstall(Rinstall);

	do
	{
		memset(A_T_P_A, 0, sizeof(double) * 9);
		memset(A_T_P_L, 0, sizeof(double) * 3);
		//memset(pX,0,sizeof(double)*UnknowNum);	

		double x, y, X, Y, Z, XX, YY, ZZ;
		m_base.rot(angle.RuPhi, angle.RuOmega, angle.RuKappa, Ru);
		for (int i = 0; i < numGCP; i++)
		{
			memset(pParams, 0, sizeof(double) * 3);
			x = pGCP[i].x;
			y = pGCP[i].y;
			StrDATUM WGS84;
			m_base.Geograph2Rect(WGS84, pGCP[i].lat, pGCP[i].lon, pGCP[i].h, X, Y, Z);

			double POS[3] = { X - XYZ[0],Y - XYZ[1],Z - XYZ[2] };
			double POSCam[3];
			m_base.Multi(Rwgs842cam, POS, POSCam, 3, 3, 1);
			XX = Ru[0] * POSCam[0] + Ru[3] * POSCam[1] + Ru[6] * POSCam[2];
			YY = Ru[1] * POSCam[0] + Ru[4] * POSCam[1] + Ru[7] * POSCam[2];
			ZZ = Ru[2] * POSCam[0] + Ru[5] * POSCam[1] + Ru[8] * POSCam[2];
			pParams[0] = ((Ru[5] * YY - Ru[4] * ZZ)*ZZ - (Ru[4] * XX - Ru[3] * YY)*XX) / (ZZ*ZZ);
			pParams[1] = (sin(angle.RuKappa)*ZZ*ZZ + (sin(angle.RuKappa)*XX +
				cos(angle.RuKappa)*YY)*XX) / (ZZ*ZZ);
			pParams[2] = (ZZ*YY) / (ZZ*ZZ);

			double L, phiX, phiY;

			pModel->GetInner(x, y, phiX, phiY);

			double tv1[3] = { phiX,phiY,1 };
			double tv2[3];
			m_base.Multi(Rinstall, tv1, tv2, 3, 3, 1);

			phiX = tv2[0] / tv2[2];
			phiY = tv2[1] / tv2[2];

			L = phiX - XX / ZZ;
			m_base.pNormal(pParams, 3, L, A_T_P_A, A_T_P_L, 1.);

			
			memset(pParams, 0, sizeof(double) * 3);
			pParams[0] = ((Ru[3] * ZZ - Ru[5] * XX)*ZZ - (Ru[4] * XX - Ru[3] * YY)*YY) / (ZZ*ZZ);
			pParams[1] = (cos(angle.RuKappa)*ZZ*ZZ + (sin(angle.RuKappa)*XX +
				cos(angle.RuKappa)*YY)*YY) / (ZZ*ZZ);
			pParams[2] = -(ZZ*XX) / (ZZ*ZZ);
			L = phiY - YY / ZZ;
			m_base.pNormal(pParams, 3, L, A_T_P_A, A_T_P_L, 1.);
		}

		m_base.Gauss(A_T_P_A, A_T_P_L, 3);
		angle.RuPhi += A_T_P_L[0];
		angle.RuOmega += A_T_P_L[1];
		angle.RuKappa += A_T_P_L[2];
		//GaussExt(A_T_P_A,A_T_P_L,pX,UnknowNum);
		//for(int j = 0;j<UnknowNum;j++)
		//	pUnknow[j] += pX[j];
		inter++;
		if (inter > 10)
		{
			printf("=>out of  convergence for offset matrix!\n");
			break;
		}

	} while (!m_base.isExitlter(A_T_P_L, 1e-9, 3));

	return true;
}

/////////////////////////////////////////////
//功能：计算两景影像间的真实角速度
//输入：面阵几何模型，控制点
//输出：精度文件
//注意：
//作者：JYH
//日期：2017.08.30
////////////////////////////////////////////
void GeoCalibration::CalcRealOmega(GeoModelArray *pModel,OffsetAngle Ru, structEOP* Eop,  Gyro &omega)
{
	double Rbody2wgs84L[9], Rbody2wgs84R[9], Rbody2wgs84Rt[9], dRu[9], Res[9];

	Attitude attL = pModel[0].GetQuatCam2wgs84();
	m_base.Quat2Matrix(attL.Q1, attL.Q2, attL.Q3, attL.Q0, Rbody2wgs84L);
	m_base.invers_matrix(Rbody2wgs84L, 3);

	Attitude attR = pModel[1].GetQuatCam2wgs84();
	m_base.Quat2Matrix(attR.Q1, attR.Q2, attR.Q3, attR.Q0, Rbody2wgs84Rt);
	m_base.rot(Ru.RuPhi, Ru.RuOmega, Ru.RuKappa, dRu);
	m_base.Multi(Rbody2wgs84Rt, dRu, Rbody2wgs84R, 3, 3, 3);
	//m_base.invers_matrix(Rbody2wgs84R, 3);//Cbj

	double dt = attR.UTC - attL.UTC;
	double om1[3];
	m_base.Multi(Rbody2wgs84R, Rbody2wgs84L, Res, 3, 3, 3);//先转左边，再转右边
	omega.UT = attL.UTC;
	omega.wx= (Res[5] - Res[7]) / 2 / dt;
	omega.wy = (Res[6] - Res[2]) / 2 / dt;
	omega.wz = (Res[1] - Res[3]) / 2 / dt;

	string out = "D:\\2_ImageData\\0.1\\Point1\\LAC\\2.影像文件\\omega.txt";
	FILE *fp = fopen(out.c_str(), "a");
	fprintf(fp, "%.9f\t%.9f\t%.9f\n", omega.wx, omega.wy, omega.wz);
	fclose(fp);
}

/////////////////////////////////////////////
//功能：残差计算
//输入：面阵几何模型，控制点
//输出：精度文件
//注意：
//作者：JYH
//日期：2017.08.22
////////////////////////////////////////////
void GeoCalibration::calcRMS(GeoModelArray* pModel,string out,StrGCP *pGCP,int numGCP)
{
	FILE* fp = fopen(out.c_str(), "w");
	fprintf(fp, "%d\n", numGCP);
	double maxx, maxy, minx, miny, rmsx, rmsy, plane;
	rmsx = rmsy = 0.;
	maxx = maxy = 0;
	minx = miny = 999999999.;
	for (int i = 0; i < numGCP; i++)
	{
		double x, y, ex, ey;
		pModel->FromLatLon2XY(pGCP[i].lat, pGCP[i].lon, pGCP[i].h, x, y);
		ex = x - pGCP[i].x;
		ey = y - pGCP[i].y;

		minx = min(minx, fabs(ex));
		miny = min(miny, fabs(ey));

		maxx = max(maxx, fabs(ex));
		maxy = max(maxy, fabs(ey));

		rmsx += ex*ex / numGCP;
		rmsy += ey*ey / numGCP;

		fprintf(fp, "%04d\t%lf\t%lf\t0.\t0.\t%lf\t%lf\n", i, pGCP[i].x, pGCP[i].y, ex, ey);
	}
	plane = sqrt(rmsx + rmsy);
	rmsx = sqrt(rmsx);
	rmsy = sqrt(rmsy);
	fprintf(fp, "x: %lf\t%lf\t%lf\n", minx, maxx, rmsx);
	fprintf(fp, "y: %lf\t%lf\t%lf\n", miny, maxy, rmsy);
	fprintf(fp, "plane: %lf", plane);
	fclose(fp);
}

/////////////////////////////////////////////
//功能：真实控制点残差计算
//输入：面阵几何模型，控制点
//输出：精度比较文件
//注意：
//作者：GZC
//日期：2017.08.23
////////////////////////////////////////////
void GeoCalibration::calcGCPerr(GeoModelArray* pModel, string strImg,string out,vector<strRMS>&acc,bool isPlus1)
{
	string tmp1 = strImg.substr(strImg.rfind('_') - 4, 4);
	string tmp2 = strImg.substr(0, strImg.rfind('_') - 5);
	int tmp3 = atoi(tmp1.c_str());// +1;
	if (isPlus1==1)
	{		tmp3 = atoi(tmp1.c_str()) +1;	}//正序递推
	else
	{		tmp3 = atoi(tmp1.c_str());	}//倒序递推
	
	char strgcp[512];
	sprintf(strgcp, "%s_%04d%s",tmp2.c_str(), tmp3, ".ctl");
	FILE* fp = fopen(strgcp, "r");
	//string strRes = strImg.substr(0, strImg.rfind('_')) + out + "_rms.txt";
	//FILE* fpres = fopen(strRes.c_str(), "w");
	int numGCP;
	fscanf(fp, "%d\n", &numGCP);
	double maxx, maxy, minx, miny, rmsx, rmsy, plane;
	rmsx = rmsy = 0.;
	maxx = maxy = 0;
	minx = miny = 999999999.;
	for (int i = 0; i < numGCP; i++)
	{
		double lx, ly, rx, ry, ex, ey, lat, lon, h;
		fscanf(fp, "%*d\t%lf\t%lf\t%lf\t%lf\t%lf\n",  &ly, &lx,&lat, &lon, &h);
		pModel->FromLatLon2XY(lat/180.*PI, lon / 180.*PI, h, rx, ry);
		ex = rx - lx;
		ey = ry - ly;

		minx = min(minx, fabs(ex));
		miny = min(miny, fabs(ey));

		maxx = max(maxx, fabs(ex));
		maxy = max(maxy, fabs(ey));

		rmsx += ex*ex / numGCP;
		rmsy += ey*ey / numGCP;

		//fprintf(fpres, "%04d\t%lf\t%lf\t0.\t0.\t%lf\t%lf\n", i, lx, ly, ex, ey);
	}
	plane = sqrt(rmsx + rmsy);
	rmsx = sqrt(rmsx);
	rmsy = sqrt(rmsy);
	//fprintf(fpres, "x: %lf\t%lf\t%lf\n", minx, maxx, rmsx);
	//fprintf(fpres, "y: %lf\t%lf\t%lf\n", miny, maxy, rmsy);
	//fprintf(fpres, "plane: %lf", plane);
	strRMS tmpRms;
	tmpRms.rmsall = plane; tmpRms.rmsx = rmsx; tmpRms.rmsy = rmsy;
	acc.push_back(tmpRms);
	fclose(fp);
	//fclose(fpres);
}

// 立体定位精度分析接口
void GeoCalibration::Cal3DAccuracy(long step, long times, long ctlnum, string outpath)
{	
	// 以下参数通过用户传参传进,这儿赋值是为了让外界能够访问
	m_step = step;
	m_times = times;
	if (m_model.size()<2)
	{
		printf("成像模型个数少于2个,无法进行立体定位精度分析!");
	}

	// 刺点误差
	double tmpX, tmpY;

	// 控制点选项
	vector<GeoTranslation> m_transall;
	m_transall.resize(m_model.size());
	// 计算控制点仿射变换系数
	double *pointx0, *pointy0, *pointx1, *pointy1, *index0;
	pointx0 = pointy0 = pointx1 = pointy1 = index0 = NULL;
	if (ctlnum>0)
	{
		pointx0 = new double[ctlnum];	pointy0 = new double[ctlnum];
		pointx1 = new double[ctlnum];	pointy1 = new double[ctlnum];
		index0 = new double[2 * ctlnum];
		srand(_time64(NULL) + times);
		for (int i = 0; i<2 * ctlnum; i++)
			index0[i] = (double)rand() / RAND_MAX;
	}
	long height, width;
	double lat, lon;
	for (int k = 0; k<m_model.size(); k++)
	{
		tmpX = m_model[k]->get_m_height();
		tmpY = m_model[k]->get_m_width();
		double *pointX = new double[ctlnum];
		m_base.RandomDistribution(0, tmpX, ctlnum, 0, pointX);
		double *pointY = new double[ctlnum];
		m_base.RandomDistribution(0, tmpY, ctlnum, 0, pointY);
		for (int i = 0; i<ctlnum; i++)
		{
			height = m_model[k]->get_m_height();
			width = m_model[k]->get_m_width();
			pointx0[i] = (height - 1)*index0[2 * i + 0];
			pointy0[i] = (width - 1)*index0[2 * i + 1];
			double hh = 0;
			m_model[k]->FromXY2LatLon(pointx0[i], pointy0[i], hh, lat, lon);
			m_model[k]->FromLatLon2XY(lat, lon, 0, pointx1[i], pointy1[i]);
			pointx1[i] += pointX[i];
			pointy1[i] += pointY[i];
		}
		m_transall[k].CalAffineParam(pointx0, pointy0, pointx1, pointy1, ctlnum);
		if (pointX != NULL)	delete[]pointX;	pointX = NULL;
		if (pointY != NULL)	delete[]pointY;	pointY = NULL;
	}
	if (pointx0 != NULL)	delete[]pointx0;	pointx0 = NULL;
	if (pointx1 != NULL)	delete[]pointx1;	pointx1 = NULL;
	if (pointy0 != NULL)	delete[]pointy0;	pointy0 = NULL;
	if (pointy1 != NULL)	delete[]pointy1;	pointy1 = NULL;
	if (index0 != NULL)	delete[]index0;	index0 = NULL;

	// 开始计算几个模型的公共重叠部分
	long level;
	OGRLinearRing *m_Ring = new OGRLinearRing[m_model.size()];   // 这儿会造成内存泄露,将来应该想办法解决
	OGRPolygon *m_Polygon = new OGRPolygon[m_model.size()];
	for (int i = 0; i<m_model.size(); i++)
	{
		// 获得积分级数和宽、高
		//m_model[i]->GetLevel(&level);
		height = m_model[i]->get_m_height();
		width = m_model[i]->get_m_width();
		// 计算四个角点的坐标并保存
		double hh = 0;
		m_model[i]->FromXY2LatLon(0, 0, hh, lat, lon);
		m_Ring[i].addPoint(lat, lon);		// 左上角
		m_model[i]->FromXY2LatLon(0, width - 1, hh, lat, lon);
		m_Ring[i].addPoint(lat, lon);		// 右上角
		m_model[i]->FromXY2LatLon(height - 1, width - 1, hh, lat, lon);
		m_Ring[i].addPoint(lat, lon);		// 右下角
		m_model[i]->FromXY2LatLon(height - 1, 0, hh, lat, lon);
		m_Ring[i].addPoint(lat, lon);		// 左下角
		m_Ring[i].closeRings();				// 将环闭合
		m_Polygon[i].addRing(&m_Ring[i]);	//转化成Polygon
	}
	OGRPolygon *m_PolygonTemp;
	for (int i = 0; i<m_model.size() - 1; i++)
	{
		//如果没有相交部分,则无法进行立体定位精度分析
		if (m_Polygon[i].Intersect(&m_Polygon[i + 1]) == 0)
		{
			printf("三维精度分析:没有重叠部分");
		}
		m_PolygonTemp = (OGRPolygon *)m_Polygon[i].Intersection(&m_Polygon[i + 1]);
		OGRLinearRing *m_RingTemp = m_PolygonTemp->getExteriorRing();
		m_Polygon[i + 1].empty();
		m_Polygon[i + 1].addRing(m_RingTemp);
	}

	OGRLinearRing *m_LinearRing = m_PolygonTemp->getExteriorRing();
	int ringNum = m_LinearRing->getNumPoints();   // 获得点个数
	double *x1 = new double[ringNum];
	double *y1 = new double[ringNum];
	double minx, maxx, miny, maxy;
	int indexminx, indexmaxx, indexminy, indexmaxy;
	minx = miny = DBL_MAX;
	maxx = maxy = DBL_MIN;
	OGRPoint m_ogrpoint;
	for (int i = 0; i<ringNum; i++)
	{
		m_LinearRing->getPoint(i, &m_ogrpoint);
		x1[i] = m_ogrpoint.getX();
		if (x1[i]>maxx) { maxx = x1[i]; indexmaxx = i; }
		if (x1[i]<minx) { minx = x1[i]; indexminx = i; }
		y1[i] = m_ogrpoint.getY();
		if (y1[i]>maxy) { maxy = y1[i]; indexmaxy = i; }
		if (y1[i]<miny) { miny = y1[i]; indexminy = i; }
	}
	// 对重叠区域划分格网
	GeoTranslation m_trans;
	double xtemp[4], ytemp[4], lattemp[4], lontemp[4];
	xtemp[0] = -1;					 ytemp[0] = -1;
	cornerlat[0] = lattemp[0] = x1[indexminx];
	cornerlon[0] = lontemp[0] = y1[indexminx];
	xtemp[1] = m_step + 1;			 ytemp[1] = -1;
	cornerlat[1] = lattemp[1] = x1[indexmaxy];
	cornerlon[1] = lontemp[1] = y1[indexmaxy];
	xtemp[2] = m_step + 1;			 ytemp[2] = m_step + 1;
	cornerlat[2] = lattemp[2] = x1[indexmaxx];
	cornerlon[2] = lontemp[2] = y1[indexmaxx];
	xtemp[3] = -1;					 ytemp[3] = m_step + 1;
	cornerlat[3] = lattemp[3] = x1[indexminy];
	cornerlon[3] = lontemp[3] = y1[indexminy];
	m_trans.CalAffineParam(xtemp, ytemp, lattemp, lontemp, 4);
	//清空之前的数据,并分配新的空间
	if (pStatis != NULL)
	{
		delete[]pStatis;
		pStatis = NULL;
	}

	pStatis = new Str3DAccuracyData[m_step*m_step];
	memset(pStatis, 0, sizeof(Str3DAccuracyData)*m_step*m_step);
	//给统计值赋初值
	for (int l = 0; l< m_step; l += 1)
	{
		for (int s = 0; s < m_step; s += 1)
		{
			pStatis[l*m_step + s].min = DBL_MAX;   //存储平面最小值
			pStatis[l*m_step + s].max = DBL_MIN;   //存储平面最大值
			pStatis[l*m_step + s].minh = DBL_MAX;  //存储高程最小值
			pStatis[l*m_step + s].maxh = DBL_MIN;  //存储高程最大值
		}
	}
	
	double m_step2 = (double)m_step*m_step;
	// 循环计算多次定位误差
	for (int time = 0; time<times; time++)
	{
		// 重新计算模型
		for (int k = 0; k<m_model.size(); k++)
		{
			//m_model[k]->ReCalPosAndAtt(m_model);
		}
		// 划分格网分别进行计算
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				long index = l*m_step + s;
				m_trans.GetValueBaseAffine(l, s, lat, lon);
				struct Str3DAccuracyData data;
				CalPointAccuracy(lat, lon, 0, ctlnum, data, m_transall);
				// 当定位误差小于0.001m时,几乎可以认为是没有定位误差
				if (fabs(data.exy)<0.001&&fabs(data.eh)<0.001)
				{
					data.ex = 0;
					data.ey = 0;
					data.eh = 0;
					data.exy = 0;
				}
				// 存储下来
				pStatis[index].lat += lat*180.0 / PI;
				pStatis[index].lon += lon*180.0 / PI;
				pStatis[index].eh += data.eh*data.eh;
				pStatis[index].ex += data.ex*data.ex;
				pStatis[index].ey += data.ey*data.ey;
				if (pStatis[index].min>fabs(data.exy))	pStatis[index].min = fabs(data.exy);	// 平面最小值
				if (pStatis[index].max<fabs(data.exy))	pStatis[index].max = fabs(data.exy);   // 平面最大值
				if (pStatis[index].minh>fabs(data.eh))	pStatis[index].minh = fabs(data.eh);	// 高程最小值
				if (pStatis[index].maxh<fabs(data.eh))	pStatis[index].maxh = fabs(data.eh);	// 高程最大值
			}		
		}
	}
	for (int l = 0; l< m_step; l += 1)
	{
		for (int s = 0; s < m_step; s += 1)
		{
			long index = l*m_step + s;
			pStatis[index].ex = sqrt(pStatis[index].ex / times);
			pStatis[index].ey = sqrt(pStatis[index].ey / times);
			pStatis[index].eh = sqrt(pStatis[index].eh / times);
			pStatis[index].exy = sqrt(pow(pStatis[index].ex, 2) + pow(pStatis[index].ey, 2));
			pStatis[index].lat = pStatis[index].lat / times;
			pStatis[index].lon = pStatis[index].lon / times;
		}
	}
	// 计算平面RMS
	RMS = 0.0;
	for (int l = 0; l< m_step; l += 1)
	{
		for (int s = 0; s < m_step; s += 1)
		{
			long index = l*m_step + s;
			RMS += pow(pStatis[index].exy, 2);
		}
	}
	RMS = sqrt(RMS / (m_step2 - 1));
	// 计算RMSx
	RMSx = 0.0;
	for (int l = 0; l< m_step; l += 1)
	{
		for (int s = 0; s < m_step; s += 1)
		{
			long index = l*m_step + s;
			RMSx += pow(pStatis[index].ex, 2);
		}
	}
	RMSx = sqrt(RMSx / (m_step2 - 1));
	// 计算平面RMSy
	RMSy = 0.0;
	for (int l = 0; l< m_step; l += 1)
	{
		for (int s = 0; s < m_step; s += 1)
		{
			long index = l*m_step + s;
			RMSy += pow(pStatis[index].ey, 2);
		}
	}
	RMSy = sqrt(RMSy / (m_step2 - 1));
	// 计算高程RMSh
	RMSh = 0.0;
	for (int l = 0; l< m_step; l += 1)
	{
		for (int s = 0; s < m_step; s += 1)
		{
			long index = l*m_step + s;
			RMSh += pow(pStatis[index].eh, 2);
		}
	}
	RMSh = sqrt(RMSh / (m_step2 - 1));


	string temp1 = outpath + "\\3DAccuracy.txt";
	Write3DAccuracyResult(temp1);
}

////////////////////////////////////////////////////////////////
// 对单独一个点进行法化和前方交会
////////////////////////////////////////////////////////////////
bool GeoCalibration::CalPointAccuracy(double lat, double lon, double h, int ctlnum,
	struct Str3DAccuracyData &data, vector<GeoTranslation> m_trans)
{
	// 刺点误差
	double tmpX, tmpY;
	// 计算三维点的初始值
	double elat, elon, eh;
	// 求取具有多少个模型
	long num = m_model.size();
	long level = 0;
	// 中间变量
	struct StrOrbitPoint pos;			// WGS84系下的坐标
	struct StrAttPoint att;				// cam2wgs84
	double innerx, innery;
	double x1, y1, x2, y2;
	double a[3], aa[9], al[3], L, temp;
	memset(aa, 0, sizeof(double) * 9);		memset(al, 0, sizeof(double) * 3);
	// 对带误差的模型进行前方交会
	for (long i = 0; i<num; i++)
	{
		tmpX = m_model[i]->get_m_height();
		tmpY = m_model[i]->get_m_width();
		double *pointX = new double[10];
		m_base.RandomDistribution(0, tmpX, 10, 0, pointX);
		double *pointY = new double[10];
		m_base.RandomDistribution(0, tmpY, 10, 0, pointY);
		// 首先将物方点通过正确的模型投影到像面上
		m_model[i]->FromLatLon2XY(lat, lon, h, x1, y1);
		x1 += pointX[5];
		y1 += pointY[5];
		m_trans[i].GetValueBaseAffine(x1, y1, x2, y2);		// 仿射修正
		// 获取对应的姿轨和内方位元素
		m_model[i]->GetPosAndInner(x2, y2, &pos, &att, &innerx, &innery);
		// 第一个方程
		a[0] = att.R[0] - att.R[2] * innerx;
		a[1] = att.R[3] - att.R[5] * innerx;
		a[2] = att.R[6] - att.R[8] * innerx;
		L = a[0] * pos.X[0] + a[1] * pos.X[1] + a[2] * pos.X[2];
		m_base.pNormal(a, 3, L, aa, al, 1.0);
		// 第二个方程
		a[0] = att.R[1] - att.R[2] * innery;
		a[1] = att.R[4] - att.R[5] * innery;
		a[2] = att.R[7] - att.R[8] * innery;
		L = a[0] * pos.X[0] + a[1] * pos.X[1] + a[2] * pos.X[2];
		m_base.pNormal(a, 3, L, aa, al, 1.0);

		if (pointX != NULL)	delete[]pointX;	pointX = NULL;
		if (pointY != NULL)	delete[]pointY;	pointY = NULL;
	}
	m_base.solve33(aa, al);
	StrDATUM datum;
	m_base.Rect2Geograph(datum, al[0], al[1], al[2], elat, elon, eh);
	// 开始计算定位误差
	data.eh = eh - h;
	data.ex = (elat - lat) * 6378140;
	data.ey = (elon - lon)*cos(lat) * 6378140;
	data.exy = sqrt(pow(data.ex, 2) + pow(data.ey, 2));

	return true;
}


////////////////////////////////////////////////////////////////
// 定位精度分析：输出分析结果
////////////////////////////////////////////////////////////////
void GeoCalibration::Write3DAccuracyResult(string outpath)
{
	FILE *fsave;
	fsave = fopen(outpath.c_str(), "w");
	if (fsave != NULL)
	{
		fprintf(fsave, "#Geo3DAccuracyResult#\n");
		fprintf(fsave, "分析次数：%10d\n", m_times);
		fprintf(fsave, "垂轨向分析点个数：%10d\n", m_step);
		fprintf(fsave, "沿轨向分析点个数：%10d\n", m_step);
		fprintf(fsave, "四个角点信息(经度,纬度)：\n");
		for (int i = 0; i<4; i++)
		{
			fprintf(fsave, "%29.19lf%29.19lf\n", cornerlon[i] * 180.0 / PI, cornerlat[i] * 180.0 / PI);
		}
		fprintf(fsave, "计算得到的总体RMS(x, y, 平面, 高程)\n");
		fprintf(fsave, "%29.19lf%29.19lf%29.19lf%29.19lf\n", RMSx, RMSy, RMS, RMSh);
		fprintf(fsave, "\n");
		int index = 0;
		fprintf(fsave, "计算得到的各个采样点地面平面误差的平均值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].exy);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面平面误差的最大值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].max);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面平面误差的最小值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].min);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面高程误差的平均值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].eh);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面高程误差的最大值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].maxh);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面高程误差的最小值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].minh);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面平面误差在纬线上的投影的平均值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].ex);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面平面误差在经线上的投影的平均值(单位为m):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].ey);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面的纬度(单位为°):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].lat);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
		index = 0;
		fprintf(fsave, "计算得到的各个采样点地面的经度(单位为°):\n");
		for (int l = 0; l< m_step; l += 1)
		{
			for (int s = 0; s < m_step; s += 1)
			{
				fprintf(fsave, "%29.19lf\t", pStatis[index].lon);
				index++;
			}
			fprintf(fsave, "\n");
		}
		fprintf(fsave, "\n");
	}
	fclose(fsave);
}
