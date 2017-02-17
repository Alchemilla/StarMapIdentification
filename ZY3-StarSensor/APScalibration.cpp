#include "APScalibration.h"



APScalibration::APScalibration()
{
}


APScalibration::~APScalibration()
{
}

//////////////////////////////////////////////////////////////////////////
//功能：星敏相机定标函数，采用最小二乘迭代求解
//输入：星点提取点和对应恒星赤经赤纬，作为控制点
//输出：待估计的3个参数
//注意：
//作者：GZC
//日期：2017.01.11
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate3Param(vector<StarGCP> getGCP, int index)
{
	//是否对控制点进行优化，小于20个像素角距的不要
	//OptimizeGCP(getGCP,20);
	//未知参数个数
	const int Param = 3;
	//定义Xest为待估计的6个量，单位都化成毫米mm吧
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//有关法化计算的数组
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//设置迭代判断初值
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_count = 0;
	int num = getGCP.size();
	double Xa, Ya, Xb, Yb, DetX, DetY;
	while (true)
	{
		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
		for (int a = 0; a < num - 1; a++)
		{
			for (int b = a + 1; b < num; b++)
			{
				//真实角距
				//double  VTV1, VTV2;
				//mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV1, 1, 3, 1);
				////测量角距
				//double V1[3],V2[3];
				//V1[0] = (getGCP[a].x - 512)*0.015;	V1[1] = (getGCP[a].y - 512)*0.015;	V1[2] = 43.3;
				//V2[0] = (getGCP[b].x - 512)*0.015;	V2[1] = (getGCP[b].y - 512)*0.015;	V2[2] = 43.3;
				//mBase.NormVector(V1, 3);
				//mBase.NormVector(V2, 3);
				//mBase.Multi(V1, V2, &VTV2, 1, 3, 1);
				//double det = VTV1 - VTV2;
				//if (det > 0.0001)
				//	continue;

				//赋值
				/*Xa = getGCP[a].x ;		Ya = getGCP[a].y ;
				Xb = getGCP[b].x;		Yb = getGCP[b].y;*/
				Xa = 1024 - getGCP[a].y - 512;		Ya = getGCP[a].x - 512;
				Xb = 1024 - getGCP[b].y - 512;		Yb = getGCP[b].x - 512;
				/*Xa = 512 - getGCP[a].x;		Ya = 512 - getGCP[a].y;
				Xb = 512 - getGCP[b].x;		Yb = 512 - getGCP[b].y;*/
				//畸变模型
				DetX = Xest[0];
				DetY = Xest[1];
				double N = (Xa - DetX)*(Xb - DetX) + (Ya - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
				double D1 = (Xa - DetX)*(Xa - DetX) + (Ya - DetY)*(Ya - DetY) + Xest[2] * Xest[2];
				double D2 = (Xb - DetX)*(Xb - DetX) + (Yb - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
				double D0 = sqrt(D1*D2);

				//第一层偏导
				double D1x0_Par = -2 * (Xa - DetX);
				double D1y0_Par = -2 * (Ya - DetY);
				double D1f_Par = 2 * Xest[2];
				double D2x0_Par = -2 * (Xb - DetX);
				double D2y0_Par = -2 * (Yb - DetY);
				double D2f_Par = 2 * Xest[2];

				//第二层偏导第一部分
				double D0x0_Par = 0.5*sqrt(D2 / D1)*D1x0_Par + 0.5*sqrt(D1 / D2)*D2x0_Par;
				double D0y0_Par = 0.5*sqrt(D2 / D1)*D1y0_Par + 0.5*sqrt(D1 / D2)*D2y0_Par;
				double D0f_Par = 0.5*sqrt(D2 / D1)*D1f_Par + 0.5*sqrt(D1 / D2)*D2f_Par;

				//第二层偏导第二部分
				double Nx0_Par = 2 * DetX - Xa - Xb;
				double Ny0_Par = 2 * DetY - Ya - Yb;
				double Nf_Par = 2 * Xest[2];

				//第三层偏导
				double G[Param];
				G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
				G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
				G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;

				//真实角距
				double  VTV;
				mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV, 1, 3, 1);
				L = VTV - N / D0;
				mBase.pNormal(G, Param, L, ATA, ATL, 1.0);
			}
		}
		//迭代求解
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-20&&abs(ATL[1] - iter_y0) < 1e-20&&abs(ATL[2] - iter_f) < 1e-20 
			|| iter_count > 20)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
	}
	//精度评估
	//CaliAccuracy(getGCP, Xest, 3);
	char output[512];
	sprintf(output, "角距3参数定标第%d景", index);
	OutputFile(Xest,3,output);
}

//////////////////////////////////////////////////////////////////////////
//功能：星敏相机定标函数，采用最小二乘迭代求解，涉及多幅影像叠加求解
//输入：星点提取点和对应恒星赤经赤纬，作为控制点
//输出：待估计的3个参数
//注意：
//作者：GZC
//日期：2017.01.18
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate3ParamMultiImg(vector<vector<StarGCP>>getGCP, int index)
{
	//未知参数个数
	const int Param = 3;
	//定义Xest为待估计的6个量，单位都化成毫米mm吧
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//有关法化计算的数组
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//设置迭代判断初值
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_count = 0;
	int numIMG,numGCP;
	double Xa, Ya, Xb, Yb, DetX, DetY;
	while (true)
	{
		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
		numIMG = getGCP.size();
		for (size_t i = 0; i < numIMG; i++)
		{
			numGCP = getGCP[i].size();
			for (int a = 0; a < numGCP - 1; a++)
					{
						for (int b = a + 1; b < numGCP; b++)
						{
							//赋值仿真
							/*Xa = 1024 - getGCP[i][a].y - 512 + 2;		Ya = getGCP[i][a].x - 512 - 3;
							Xb = 1024 - getGCP[i][b].y - 512 + 2;		Yb = getGCP[i][b].x - 512 - 3;*/
							//赋值
							Xa = 1024 - getGCP[i][a].y - 512;		Ya = getGCP[i][a].x - 512;
							Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;
							//畸变模型
							DetX = Xest[0];
							DetY = Xest[1];
							double N = (Xa - DetX)*(Xb - DetX) + (Ya - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
							double D1 = (Xa - DetX)*(Xa - DetX) + (Ya - DetY)*(Ya - DetY) + Xest[2] * Xest[2];
							double D2 = (Xb - DetX)*(Xb - DetX) + (Yb - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
							double D0 = sqrt(D1*D2);

							//第一层偏导
							double D1x0_Par = -2 * (Xa - DetX);
							double D1y0_Par = -2 * (Ya - DetY);
							double D1f_Par = 2 * Xest[2];
							double D2x0_Par = -2 * (Xb - DetX);
							double D2y0_Par = -2 * (Yb - DetY);
							double D2f_Par = 2 * Xest[2];

							//第二层偏导第一部分
							double D0x0_Par = 0.5*sqrt(D2 / D1)*D1x0_Par + 0.5*sqrt(D1 / D2)*D2x0_Par;
							double D0y0_Par = 0.5*sqrt(D2 / D1)*D1y0_Par + 0.5*sqrt(D1 / D2)*D2y0_Par;
							double D0f_Par = 0.5*sqrt(D2 / D1)*D1f_Par + 0.5*sqrt(D1 / D2)*D2f_Par;

							//第二层偏导第二部分
							double Nx0_Par = 2 * DetX - Xa - Xb;
							double Ny0_Par = 2 * DetY - Ya - Yb;
							double Nf_Par = 2 * Xest[2];

							//第三层偏导
							double G[Param];
							G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
							G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
							G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;

							//真实角距
							double  VTV;
							mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
							L = VTV - N / D0;
							mBase.pNormal(G, Param, L, ATA, ATL, 1.0);
						}
					}
		}
		
		//迭代求解
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
			|| iter_count > 20)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
	}
	//精度评估
	//CaliAccuracy(getGCP, Xest, 3);
	char output[512];
	sprintf(output, "角距3参数定标第%d景", index);
	OutputFile(Xest, 3, output);
}

//////////////////////////////////////////////////////////////////////////
//功能：星敏相机定标函数，采用最小二乘迭代求解，涉及多幅影像叠加求解
//输入：星点提取点和对应恒星赤经赤纬，作为控制点
//输出：待估计的3个参数
//注意：
//作者：GZC
//日期：2017.01.18
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate3ParamKalman(vector<vector<StarGCP>>getGCP, int index)
{
	//未知参数个数
	const int Param = 3;
	//定义Xest为待估计的6个量，单位都化成毫米mm吧
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//有关法化计算的数组
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//设置迭代判断初值
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_count = 0;
	int numIMG, numGCP;
	double Xa, Ya, Xb, Yb, DetX, DetY;
	while (true)
	{
		
		numIMG = getGCP.size();
		for (size_t i = 0; i < numIMG; i++)
		{
			double G[Param];//在此处定义，可以在跳出循环时得到最后一次状态转移矩阵H;
			while (true)
			{
				numGCP = getGCP[i].size();
				memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
				//构建法方程
				for (int a = 0; a < numGCP - 1; a++)
				{
					for (int b = a + 1; b < numGCP; b++)
					{
						//赋值
						Xa = 1024 - getGCP[i][a].y - 512;		Ya = getGCP[i][a].x - 512;
						Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;
						//畸变模型
						DetX = Xest[0];
						DetY = Xest[1];
						double N = (Xa - DetX)*(Xb - DetX) + (Ya - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
						double D1 = (Xa - DetX)*(Xa - DetX) + (Ya - DetY)*(Ya - DetY) + Xest[2] * Xest[2];
						double D2 = (Xb - DetX)*(Xb - DetX) + (Yb - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
						double D0 = sqrt(D1*D2);

						//第一层偏导
						double D1x0_Par = -2 * (Xa - DetX);
						double D1y0_Par = -2 * (Ya - DetY);
						double D1f_Par = 2 * Xest[2];
						double D2x0_Par = -2 * (Xb - DetX);
						double D2y0_Par = -2 * (Yb - DetY);
						double D2f_Par = 2 * Xest[2];

						//第二层偏导第一部分
						double D0x0_Par = 0.5*sqrt(D2 / D1)*D1x0_Par + 0.5*sqrt(D1 / D2)*D2x0_Par;
						double D0y0_Par = 0.5*sqrt(D2 / D1)*D1y0_Par + 0.5*sqrt(D1 / D2)*D2y0_Par;
						double D0f_Par = 0.5*sqrt(D2 / D1)*D1f_Par + 0.5*sqrt(D1 / D2)*D2f_Par;

						//第二层偏导第二部分
						double Nx0_Par = 2 * DetX - Xa - Xb;
						double Ny0_Par = 2 * DetY - Ya - Yb;
						double Nf_Par = 2 * Xest[2];

						//第三层偏导
						G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
						G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
						G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;

						//真实角距
						double  VTV;
						mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
						L = VTV - N / D0;
						mBase.pNormal(G, Param, L, ATA, ATL, 1.0);
					}
				}
				//迭代求解
				mBase.Gauss(ATA, ATL, Param);
				if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
					|| iter_count > 20)
					break;
				iter_count++;
				Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
				iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
			}		
			//Kalman迭代求解
			int iter_count3 = 0;
			Vector3d Xk, Hk, Xk1, Kal;
			Matrix3d Pk, Pk1;
			Pk = Matrix3d::Identity(3, 3);
			double Qe = 0.2;//星点提取误差
			Xk << Xest[0], Xest[1], Xest[2];
			while (true)
			{
				for (int a = 0; a < numGCP - 1; a++)
				{
					for (int b = a + 1; b < numGCP; b++)
					{
						//赋值
						Xa = 1024 - getGCP[i][a].y - 512;		Ya = getGCP[i][a].x - 512;
						Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;
						//畸变模型
						DetX = Xk(0);
						DetY = Xk(1);
						double N = (Xa - DetX)*(Xb - DetX) + (Ya - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
						double D1 = (Xa - DetX)*(Xa - DetX) + (Ya - DetY)*(Ya - DetY) + Xest[2] * Xest[2];
						double D2 = (Xb - DetX)*(Xb - DetX) + (Yb - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
						double D0 = sqrt(D1*D2);

						//第一层偏导
						double D1x0_Par = -2 * (Xa - DetX);
						double D1y0_Par = -2 * (Ya - DetY);
						double D1f_Par = 2 * Xest[2];
						double D2x0_Par = -2 * (Xb - DetX);
						double D2y0_Par = -2 * (Yb - DetY);
						double D2f_Par = 2 * Xest[2];

						//第二层偏导第一部分
						double D0x0_Par = 0.5*sqrt(D2 / D1)*D1x0_Par + 0.5*sqrt(D1 / D2)*D2x0_Par;
						double D0y0_Par = 0.5*sqrt(D2 / D1)*D1y0_Par + 0.5*sqrt(D1 / D2)*D2y0_Par;
						double D0f_Par = 0.5*sqrt(D2 / D1)*D1f_Par + 0.5*sqrt(D1 / D2)*D2f_Par;

						//第二层偏导第二部分
						double Nx0_Par = 2 * DetX - Xa - Xb;
						double Ny0_Par = 2 * DetY - Ya - Yb;
						double Nf_Par = 2 * Xest[2];

						//第三层偏导
						G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
						G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
						G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
						
						double  VTV;
						mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
						L = VTV - N / D0;

						Hk << G[0], G[1], G[2];
						cout << Hk;
						Kal = Pk*Hk / (Hk.transpose()*Pk*Hk + Qe);
						cout << Kal;
						Xk1 = Xk + Kal*(L - Hk.transpose()*Xk);
						cout << Xk1;
						Pk1 = (Matrix3d::Identity(3, 3) - Kal*Hk.transpose());
						cout << Pk1;
						Pk = Pk1;
						if (abs(Xk(0) - Xk1(0))<1e-8&&abs(Xk(1) - Xk1(1))<1e-8&&abs(Xk(2) - Xk1(2))<1e-8
							|| iter_count3>50)
						{
							Xest[0] = Xk1(0), Xest[1] = Xk1(1), Xest[2] = Xk1(2);
							break;
						}
						Xk = Xk1;
					}
				}
			}
			
		}		
	}
	//精度评估
	//CaliAccuracy(getGCP, Xest, 3);
	char output[512];
	sprintf(output, "角距3参数定标第%d景", index);
	OutputFile(Xest, 3, output);
}

//////////////////////////////////////////////////////////////////////////
//功能：星敏相机定标函数，采用最小二乘迭代求解
//输入：星点提取点和对应恒星赤经赤纬，作为控制点
//输出：待估计的6个参数
//注意：
//作者：GZC
//日期：2017.01.10
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate6Param(vector<StarGCP> getGCP, int index)
{
	//是否对控制点进行优化，小于20个像素角距的不要
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	//未知参数个数
	const int Param = 6;
	//定义Xest为待估计的6个量，单位都化成毫米mm吧
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//有关法化计算的数组
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//设置迭代判断初值
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3/0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0, iter_count = 0;
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	while (true)
	{
		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
		for (int a = 0; a < num - 1; a++)
		{
			for (int b = a + 1; b < num; b++)
			{				
				Xa = 1024 - getGCP[a].y - 512;		Ya = getGCP[a].x - 512;
				Xb = 1024 - getGCP[b].y - 512;		Yb = getGCP[b].x - 512;

				//畸变模型
				double ra = sqrt(Xa * Xa + Ya * Ya);
				DetXa = Xest[0] + Xest[3] * Xa * ra*ra + Xest[4] * (3 * Xa * Xa + Ya * Ya) + 2 * Xest[5] * Xa * Ya;
				DetYa = Xest[1] + Xest[3] * Ya * ra*ra + Xest[5] * (3 * Ya * Ya + Xa * Xa) + 2 * Xest[4] * Xa * Ya;
				double rb = sqrt(Xb * Xb + Yb * Yb);
				DetXb = Xest[0] + Xest[3] * Xb * rb*rb + Xest[4] * (3 * Xb * Xb + Yb * Yb) + 2 * Xest[5] * Xb * Yb;
				DetYb = Xest[1] + Xest[3] * Yb * rb*rb + Xest[5] * (3 * Yb * Yb + Xb * Xb) + 2 * Xest[4] * Xb * Yb;
				double N = (Xa - DetXa)*(Xb - DetXb) + (Ya - DetYa)*(Yb - DetYb) + Xest[2] * Xest[2];
				double D1 = (Xa - DetXa)*(Xa - DetXa) + (Ya - DetYa)*(Ya - DetYa) + Xest[2] * Xest[2];
				double D2 = (Xb - DetXb)*(Xb - DetXb) + (Yb - DetYb)*(Yb - DetYb) + Xest[2] * Xest[2];
				double D0 = sqrt(D1*D2);

				//第一层偏导
				double D1x0_Par = -2 * (Xa - DetXa);
				double D1y0_Par = -2 * (Ya - DetYa);
				double D1f_Par = 2 * Xest[2];
				double D1k1_Par = -2 * Xa * ra*ra*(Xa - DetXa) - 2 * Ya * ra*ra*(Ya - DetYa);
				double D1p1_Par = -2 * (Xa - DetXa)*(3 * Xa * Xa + Ya * Ya) - 4 * (Ya - DetYa)*Xa * Ya;
				double D1p2_Par = -2 * (Ya - DetYa)*(3 * Ya * Ya + Xa * Xa) - 4 * (Xa - DetXa)*Xa * Ya;
				double D2x0_Par = -2 * (Xb - DetXb);
				double D2y0_Par = -2 * (Yb - DetYb);
				double D2f_Par = 2 * Xest[2];
				double D2k1_Par = -2 * Xb * ra*ra*(Xb - DetXb) - 2 * Yb * ra*ra*(Yb - DetYb);
				double D2p1_Par = -2 * (Xb - DetXb)*(3 * Xb * Xb + Yb * Yb) - 4 * (Yb - DetYb)*Xb * Yb;
				double D2p2_Par = -2 * (Yb - DetYb)*(3 * Yb * Yb + Xb * Xb) - 4 * (Xb - DetXb)*Xb * Yb;

				//第二层偏导第一部分
				double D0x0_Par = 0.5*(sqrt(D2 / D1)*D1x0_Par + sqrt(D1 / D2)*D2x0_Par);
				double D0y0_Par = 0.5*(sqrt(D2 / D1)*D1y0_Par + sqrt(D1 / D2)*D2y0_Par);
				double D0f_Par = 0.5*(sqrt(D2 / D1)*D1f_Par + sqrt(D1 / D2)*D2f_Par);
				double D0k1_Par = 0.5*(sqrt(D2 / D1)*D1k1_Par + sqrt(D1 / D2)*D2k1_Par);
				double D0p1_Par = 0.5*(sqrt(D2 / D1)*D1p1_Par + sqrt(D1 / D2)*D2p1_Par);
				double D0p2_Par = 0.5*(sqrt(D2 / D1)*D1p2_Par + sqrt(D1 / D2)*D2p2_Par);

				//第二层偏导第二部分
				double Nx0_Par = DetXa + DetXb - Xa - Xb;
				double Ny0_Par = DetYa + DetYb - Ya - Yb;
				double Nf_Par = 2 * Xest[2];
				double Nk1_Par = Xa * ra*ra*DetXb + Xb * rb*rb*DetXa - Xa * ra*ra*Xb - Xb * rb*rb*Xa
					+ Ya * ra*ra*DetYb + Yb * rb*rb*DetYa - Ya * ra*ra*Yb - Yb * rb*rb*Ya;
				double Np1_Par = (3 * Xa * Xa + Ya * Ya)*DetXb + (3 * Xb * Xb + Yb * Yb)*DetXa -
					(3 * Xa * Xa + Ya * Ya)*Xb - (3 * Xb * Xb + Yb * Yb)*Xa + 2 * Xa * Ya *
					DetYb + 2 * Xb * Yb * DetYa - 2 * Xa * Ya * Yb - 2 * Xb * Yb * Ya;
				double Np2_Par = (3 * Ya * Ya + Xa * Xa)*DetYb + (3 * Yb * Yb + Xb * Xb)*DetYa -
					(3 * Ya * Ya + Xa * Xa)*Yb - (3 * Yb * Yb + Xb * Xb)*Ya + 2 * Xa * Ya *
					DetXb + 2 * Xb * Yb * DetXa - 2 * Xa * Ya * Xb - 2 * Xb * Yb * Xa;

				//第三层偏导
				double G[6];
				G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
				G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
				G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
				G[3] = (Nk1_Par*D0 - D0k1_Par*N) / D0 / D0;
				G[4] = (Np1_Par*D0 - D0p1_Par*N) / D0 / D0;
				G[5] = (Np2_Par*D0 - D0p2_Par*N) / D0 / D0;

				//真实角距
				double  VTV;
				mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV, 1, 3, 1);
				L = VTV - N / D0;
				mBase.pNormal(G, 6, L, ATA, ATL, 1.0);
			}
		}
		//迭代求解
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-20&&abs(ATL[1] - iter_y0) < 1e-20&&abs(ATL[2] - iter_f) < 1e-20
			&&abs(ATL[3] - iter_k1) < 1e-20 &&abs(ATL[4] - iter_p1) < 1e-20 &&abs(ATL[5] - iter_p2) < 1e-20
			|| iter_count > 20)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		Xest[3] += ATL[3]; Xest[4] += ATL[4]; Xest[5] += ATL[5];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
		iter_k1 = abs(ATL[3]), iter_p1 = abs(ATL[4]), iter_p2 = abs(ATL[5]);
	}
	//精度评估
	//CaliAccuracy(getGCP,Xest,6);
	//原始精度评估
	printf("未定标精度为:\n");
	memset(Xest, 0, sizeof(double) * 6);
	Xest[2] = 43.3 / 0.015;
	//CaliAccuracy(getGCP, Xest, 6);
	//输出
	char output[512];
	sprintf(output, "角距6参数定标第%d景", index);
	OutputFile(Xest, 6, output);
}

//////////////////////////////////////////////////////////////////////////
//功能：得到定标后精度
//输入：getGCP，控制点，X：估计的参数，num，数量
//输出：精度评估结果，单位角秒
//注意：
//作者：GZC
//日期：2017.01.11
//////////////////////////////////////////////////////////////////////////
void APScalibration::CaliAccuracy(vector<StarGCP> getGCP, double *X, int num)
{
	double LL = 0;
	double Xest[6];
	if (num==3)
	{
		Xest[0] = X[0]; Xest[1] = X[1]; Xest[2] = X[2];
		Xest[3] = 0; Xest[4] = 0; Xest[5] = 0;
	}
	else if(num==6)
	{
		memcpy(Xest, X, sizeof(double)*6);
	}
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	for (int a = 0; a < num - 1; a++)
	{
		for (int b = a + 1; b < num; b++)
		{
			//赋值
			Xa = (getGCP[a].x - 512);		Ya = (getGCP[a].y - 512);
			Xb = (getGCP[b].x - 512);	Yb = (getGCP[b].y - 512);

			//畸变模型
			double ra = sqrt(Xa * Xa + Ya * Ya);
			DetXa = Xest[0] + Xest[3] * Xa * ra*ra + Xest[4] * (3 * Xa * Xa + Ya * Ya) + 2 * Xest[5] * Xa * Ya;
			DetYa = Xest[1] + Xest[3] * Ya * ra*ra + Xest[5] * (3 * Ya * Ya + Xa * Xa) + 2 * Xest[4] * Xa * Ya;
			double rb = sqrt(Xb * Xb + Yb * Yb);
			DetXb = Xest[0] + Xest[3] * Xb * rb*rb + Xest[4] * (3 * Xb * Xb + Yb * Yb) + 2 * Xest[5] * Xb * Yb;
			DetYb = Xest[1] + Xest[3] * Yb * rb*rb + Xest[5] * (3 * Yb * Yb + Xb * Xb) + 2 * Xest[4] * Xb * Yb;
			double N = (Xa - DetXa)*(Xb - DetXb) + (Ya - DetYa)*(Yb - DetYb) + Xest[2] * Xest[2];
			double D1 = (Xa - DetXa)*(Xa - DetXa) + (Ya - DetYa)*(Ya - DetYa) + Xest[2] * Xest[2];
			double D2 = (Xb - DetXb)*(Xb - DetXb) + (Yb - DetYb)*(Yb - DetYb) + Xest[2] * Xest[2];
			double D0 = sqrt(D1*D2);
			//真实角距
			double  VTV;
			mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV, 1, 3, 1);
			if (abs(acos(VTV) - acos(N / D0))<1)
			{
				LL += (acos(VTV) - acos(N / D0))*(acos(VTV) - acos(N / D0));
			}
		}
	}
	double RMS = sqrt(LL * 2 / num / (num + 1)) / PI * 180 * 3600;
	printf("%d参数精度为%.9f角秒\n", num,RMS);
}

//////////////////////////////////////////////////////////////////////////
//功能：去除较小角距的星点，得到优化后的控制点
//输入：getGCP，控制点；pixel，小于pixel个像素的星点不要
//输出：getGCP，优化后的控制点
//注意：星点存储格式满足StarGCP结构体的要求
//作者：GZC
//日期：2017.01.12
//////////////////////////////////////////////////////////////////////////
void APScalibration::OptimizeGCP(vector<StarGCP> &getGCP,int pixel)
{
	int num = getGCP.size();
	vector<StarGCP>getGCPchoose(getGCP);
	int *GCPa = new int[num];  memset(GCPa, 0, num * sizeof(int));
	getGCP.clear();
	//筛选星点
	while (true)
	{
		int jumpout = 0;
		for (int a = 0; a < num - 1; a++)
		{
			for (int b = a + 1; b < num; b++)
			{
				//真实角距
				double  VTV1;
				mBase.Multi(getGCPchoose[a].V, getGCPchoose[b].V, &VTV1, 1, 3, 1);				
				if (VTV1>cos(atan(pixel * 0.015 / 43.3)))//两个星点小于这个角度不要，后面这个不加abs(det) > 0.00002 || 
				{
					GCPa[a]++;
					GCPa[b]++;
					jumpout++;
				}
			}
		}
		int pos = max_element(GCPa, GCPa + num) - GCPa;
		getGCPchoose.erase(getGCPchoose.begin() + pos, getGCPchoose.begin() + pos + 1);
		num = getGCPchoose.size();
		memset(GCPa, 0, num * sizeof(int));
		if (jumpout == 0)
		{
			break;
		}
	}
	getGCP.assign(getGCPchoose.begin(), getGCPchoose.end());
}

//////////////////////////////////////////////////////////////////////////
//功能：根据控制点仿真带一定误差的控制点
//输入：getGCP，控制点；ZY3_02STGdata，此刻星敏惯性姿态，index时间指示
//输出：getGCP，仿真出的带误差控制点
//注意：星点存储格式满足StarGCP结构体的要求
//作者：GZC
//日期：2017.01.18
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP(vector<StarGCP> &getGCP, vector<STGData> ZY3_02STGdata,int index)
{
	int num = getGCP.size();
	Star getGCPsim;
	double V[3], alpha, sigma, R[9], x, y,W[3];
	index = 1;
	mBase.quat2matrix(ZY3_02STGdata[index].StarA.Q1, ZY3_02STGdata[index].StarA.Q2,
		ZY3_02STGdata[index].StarA.Q3, ZY3_02STGdata[index].StarA.Q0, R);//Crj
	double *randx = new double[num];
	double *randy = new double[num];
	mBase.RandomDistribution(0, 0.1, num, 0, randx);
	mBase.RandomDistribution(0, 0.1, num, 0, randy);
	for (int a = 0; a < num; a++)
	{
		V[0] = getGCP[a].V[0];
		V[1] = getGCP[a].V[1];
		V[2] = getGCP[a].V[2];			
		mBase.Multi(R, V, W, 3, 3, 1);
		//x0=y0=512,f=43.3mm,像元大小0.015mm
		if (W[2] > 0)
		{
			x = (512 - W[0] / W[2] * 43.3 / 0.015);
			y = (512 - W[1] / W[2] * 43.3 / 0.015);
		}
		else
		{
			x = -513, y = -513;
		}//这个是用来判断是否和星敏指向的半球方向一致
		getGCP[a].x = x + randx[a];
		getGCP[a].y = y + randy[a];
	}	
	
	/*getGCP[0].V[0] = 0.2; getGCP[0].V[1] = 0.3; getGCP[0].V[2] =sqrt(1-0.2*0.2-0.3*0.3);
	V[0] = getGCP[0].V[0];
	V[1] = getGCP[0].V[1];
	V[2] = getGCP[0].V[2];
	mBase.Multi(R, V, W, 3, 3, 1);
	if (W[2] > 0)
	{
		x = (512 - W[0] / W[2] * 43.3 / 0.015);
		y = (512 - W[1] / W[2] * 43.3 / 0.015);
	}
	getGCP[0].x = x;
	getGCP[0].y = y;
	getGCP[1].V[0] = 0.22; getGCP[1].V[1] = 0.32; getGCP[1].V[2] = sqrt(1 - 0.22*0.22 - 0.32*0.32);
	V[0] = getGCP[1].V[0];
	V[1] = getGCP[1].V[1];
	V[2] = getGCP[1].V[2];
	mBase.Multi(R, V, W, 3, 3, 1);
	if (W[2] > 0)
	{
		x = (512-W[0] / W[2] * 43.3 / 0.015);
		y = (512 - W[1] / W[2] * 43.3 / 0.015);
	}
	getGCP[1].x = x;
	getGCP[1].y = y;*/
}

//////////////////////////////////////////////////////////////////////////
//功能：根据控制点仿真带一定误差的控制点
//输入：getGCP，控制点；ZY3_02STGdata，此刻星敏惯性姿态，index时间指示
//输出：getGCP，仿真出的带误差控制点
//注意：星点存储格式满足StarGCP结构体的要求
//作者：GZC
//日期：2017.02.16
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP_PreRand(vector<vector<StarGCP>> &getGCP, 
	vector<STGData> ZY3_02STGdata, int *gcpNum, int index, double *randx, double *randy)
{
	double V[3], alpha, sigma, R[9], x, y, W[3];
	int pNum = 0;
	for (int b = 0; b < index-1; b++)
	{
		mBase.quat2matrix(ZY3_02STGdata[b].StarA.Q1, ZY3_02STGdata[b].StarA.Q2,
			ZY3_02STGdata[b].StarA.Q3, ZY3_02STGdata[b].StarA.Q0, R);//Crj
		int num = getGCP[b].size();
		for (int a = 0; a < num; a++)
		{
			V[0] = getGCP[b][a].V[0];
			V[1] = getGCP[b][a].V[1];
			V[2] = getGCP[b][a].V[2];
			mBase.Multi(R, V, W, 3, 3, 1);
			//x0=y0=512,f=43.3mm,像元大小0.015mm
			if (W[2] > 0)
			{
				x = (512 - W[0] / W[2] * 43.3 / 0.015);
				y = (512 - W[1] / W[2] * 43.3 / 0.015);
			}
			else
			{
				x = -513, y = -513;
			}//这个是用来判断是否和星敏指向的半球方向一致
			getGCP[b][a].x = x + randx[a+ pNum];
			getGCP[b][a].y = y + randy[a+ pNum];
		}
		pNum = pNum + gcpNum[b];
	}

}


//////////////////////////////////////////////////////////////////////////
//功能：根据严密几何，建立的定标模型
//输入：getGCP，控制点
//输出：严密几何模型得到3参数：主点f，主距x0, y0
//注意：getGCP得到的点坐标系方向为 X→Y↑, 而星敏的坐标系为X↓Y→
//作者：GZC
//日期：2017.01.13
//////////////////////////////////////////////////////////////////////////
void APScalibration::CalibrateRigorous3(vector<StarGCP> getGCP, int index)
{	
	//是否对控制点进行优化，小于20个像素角距的不要
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	const int Param = 3;
	//有关法化计算的数组
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param], V[3];
	memset(Xest, 0, sizeof(double) * Param);
	//设置初值
	Xest[0] = 512-21.3; Xest[1] = 512-11.9;
	Xest[2] = 43.3 / 0.015;	
	double quater[4], Crj[9];
	qMethod(getGCP, Xest, Param, quater);
	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
	double DetX, DetY;
	//设置迭代判断初值
	double iter_x0 = 512, iter_y0 = 512, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0;
	int iter_count = 0, iter_count2 = 0;
	while (true)
	{	
		while (true)
		{
			memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
			for (int i = 0; i < num; i++)
			{
				double Vtemp[3] = { getGCP[i].V[0],getGCP[i].V[1], getGCP[i].V[2] };
				mBase.Multi(Crj, Vtemp, V, 3, 3, 1);
				double X, Y;
				X = 1024 - getGCP[i].y;		Y = getGCP[i].x;
				/*DetX = Xest[0] + Xest[3] * X * r2 + Xest[4] * (3 * X * X + Y * Y) + 2 * Xest[5] * X * Y;
				DetY = Xest[1] + Xest[3] * Y * r2 + Xest[5] * (3 * Y * Y + X * X) + 2 * Xest[4] * X * Y;*/

				double FX[Param],FY[Param];
				FX[0] = 1;
				FX[1] = 0;
				FX[2] = - V[0] / V[2];			
				L = X - Xest[0] + Xest[2] * V[0] / V[2];
				mBase.pNormal(FX, Param, L, ATA, ATL, 1.0);
				FY[0] = 0;
				FY[1] = 1;
				FY[2] = -V[1] / V[2];
				L = Y - Xest[1] + Xest[2] * V[1] / V[2];
				mBase.pNormal(FY, Param, L, ATA, ATL, 1.0);
			}
			//迭代求解
			/*double aa[Param];
			mBase.GaussExt(ATA, ATL, aa, Param);
			if (abs(aa[0] - iter_x0) < 1e-20&&abs(aa[1] - iter_y0) < 1e-20&&abs(aa[2] - iter_f) < 1e-20
				|| iter_count > 20)
				break;
			iter_count++;
			Xest[0] += aa[0]; Xest[1] += aa[1]; Xest[2] += aa[2];
			iter_x0 = abs(aa[0]), iter_y0 = abs(aa[1]), iter_f = abs(aa[2]);*/
			mBase.Gauss(ATA, ATL, Param);
			if (abs(ATL[0] - iter_x0) < 1e-20&&abs(ATL[1] - iter_y0) < 1e-20&&abs(ATL[2] - iter_f) < 1e-20
				||iter_count > 20)
				break;
			iter_count++;
			Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];		
			iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);					
		}
		qMethod(getGCP, Xest, Param, quater);
		mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
		iter_count2++;
		iter_count = 0;
		if (iter_count2>= 50)
		{
			break;
		}
	}
	//输出
	char output[512];
	sprintf(output, "严密3参数定标第%d景", index);
	OutputFile(Xest, 3, output);
}

//////////////////////////////////////////////////////////////////////////
//功能：根据严密几何，建立的定标模型
//输入：getGCP，控制点
//输出：严密几何模型得到3参数：主点f，主距x0, y0，径向畸变k1，切向畸变p1，p2
//注意：getGCP得到的点坐标系方向为 X→Y↑, 而星敏的坐标系为X↓Y→
//作者：GZC
//日期：2017.01.13
//////////////////////////////////////////////////////////////////////////
void APScalibration::CalibrateRigorous6(vector<StarGCP> getGCP, int index)
{
	//是否对控制点进行优化，小于20个像素角距的不要
	//OptimizeGCP(getGCP, 20);
	int num = getGCP.size();
	const int Param = 6;
	//有关法化计算的数组
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param], V[3];
	memset(Xest, 0, sizeof(double) * Param);
	//设置初值
	Xest[0] = 512; Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	/*Xest[0] = 512*0.015; Xest[1] = 512 * 0.015;
	Xest[2] = 43.3;*/
	double quater[4], Crj[9];
	qMethod6(getGCP, Xest, Param, quater);
	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
	double DetX, DetY;
	//设置迭代判断初值
	double iter_x0 = 512, iter_y0 = 512, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0;
	int iter_count = 0, iter_count2 = 0;
	while (true)
	{
		while (true)
		{
			memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
			for (int i = 0; i < num; i++)
			{
				double Vtemp[3] = { getGCP[i].V[0],getGCP[i].V[1], getGCP[i].V[2] };
				mBase.Multi(Crj, Vtemp, V, 3, 3, 1);
				double Xp, Yp;
				Xp = 1024 - getGCP[i].y;		Yp = getGCP[i].x;
				//畸变模型
				double X = Xp - Xest[0];
				double Y = Yp - Xest[1];
				/*double D = sqrt(X*X + Y*Y + Xest[2] * Xest[2]);
				X = X / D; Y = Y / D; Xest[2] = Xest[2] / D;*/
				double r2 = X*X + Y*Y;
				DetX = Xest[3] * X * r2 + Xest[4] * (3 * X * X + Y * Y) + 2 * Xest[5] * X * Y;
				DetY = Xest[3] * Y * r2 + Xest[5] * (3 * Y * Y + X * X) + 2 * Xest[4] * X * Y;

				//
				double Rx0 = -(Xp - Xest[0]) / sqrt(r2);
				double Ry0 = -(Yp - Xest[1]) / sqrt(r2);
				double FX[Param], FY[Param];
				/*FX[0] = -Xest[3] * r2 + 2 * Xest[3] * X*sqrt(r2)*Rx0 - 6 * Xest[4] * X - 2 * Xest[5] * Y;
				FX[1] = 2 * Xest[3] * X*sqrt(r2)*Ry0 - 2 * Xest[4] * Y - 2 * Xest[5] * X;*/
				FX[0] = 1;
				FX[1] = 0;
				FX[2] = -V[0] / V[2];
				FX[3] = X*r2;
				FX[4] = 3 * X*X + Y*Y;
				FX[5] = 2 * X*Y;
				//L = Xp - DetX - Xest[0] + Xest[2] * V[0] / V[2];
				L = X - DetX + Xest[2] * V[0] / V[2];
				mBase.pNormal(FX, Param, L, ATA, ATL, 1.0);

				/*FY[0] = 2 * Xest[3] * Y*sqrt(r2)*Rx0 - 2 * Xest[5] * X - 2 * Xest[4] * Y;
				FY[1] = -Xest[3] * r2 + 2 * Xest[3] * Y*sqrt(r2)*Ry0 - 6 * Xest[5] * Y - 2 * Xest[4] * X;*/
				FY[0] = 0;
				FY[1] = 1;
				FY[2] = -V[1] / V[2];
				FY[3] = Y*r2;
				FY[4] = 2 * X*Y;
				FY[5] = 3 * Y*Y + X*X;
				L = Y - DetY  + Xest[2] * V[1] / V[2];
				mBase.pNormal(FY, Param, L, ATA, ATL, 1.0);
			}
			//迭代求解
			mBase.Gauss(ATA, ATL, Param);
			if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
			&&abs(ATL[3] - iter_k1) < 1e-10 &&abs(ATL[4] - iter_p1) < 1e-10 &&abs(ATL[5] - iter_p2) < 1e-10 || iter_count > 20)
			break;
			iter_count++;
			Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
			Xest[3] += ATL[3]; Xest[4] += ATL[4]; Xest[5] += ATL[5];
			iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
			iter_k1 = abs(ATL[3]), iter_p1 = abs(ATL[4]), iter_p2 = abs(ATL[5]);
		}
		qMethod6(getGCP, Xest, Param, quater);
		mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
		iter_count2++;
		iter_count = 0;
		if (iter_count2 >= 20)
		{
			break;
		}
	}
	//输出
	char output[512];
	sprintf(output, "严密6参数定标第%d景", index);
	OutputFile(Xest, 6, output);
}

//////////////////////////////////////////////////////////////////////////
//功能：根据严密几何，建立的定标模型
//输入：getGCP，控制点
//输出：严密几何模型得到3参数：主点f，主距x0, y0，径向畸变k1，切向畸变p1，p2
//注意：getGCP得到的点坐标系方向为 X→Y↑, 而星敏的坐标系为X↓Y→
//作者：GZC
//日期：2017.01.14
//////////////////////////////////////////////////////////////////////////
void APScalibration::CalibrateRigorousRPY(vector<StarGCP> getGCP, int index)
{
	int num = getGCP.size();
	const int Param = 6;
	//有关法化计算的数组
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param], V[3];
	memset(Xest, 0, sizeof(double) * Param);
	//设置初值
	Xest[0] = 512; Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	/*Xest[0] = 512*0.015; Xest[1] = 512 * 0.015;
	Xest[2] = 43.3;*/
	double quater[4], Crj[9];
	qMethod6(getGCP, Xest, Param, quater);
	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
	double DetX, DetY;
	//设置迭代判断初值
	double iter_x0 = 512, iter_y0 = 512, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0;
	int iter_count = 0, iter_count2 = 0;
	//构建Ru并设初值为0；
	double phi = atan(Crj[6] / Crj[8]), omg = -asin(Crj[7]), kap = atan(Crj[1] / Crj[4]);
	double sinphi, sinomg, sinkap, cosphi, cosomg, coskap;
	while (true)
		{
			memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
			sinphi = sin(phi); sinomg = sin(omg); sinkap = sin(kap);
			cosphi = cos(phi); cosomg = cos(omg); coskap = cos(kap);
			for (int i = 0; i < num; i++)
			{
				double V[3] = { getGCP[i].V[0],getGCP[i].V[1], getGCP[i].V[2] };
				//mBase.Multi(Crj, Vtemp, V, 3, 3, 1);
				double Xp, Yp;
				Xp = 1024 - getGCP[i].y;		Yp = getGCP[i].x;
				double Xbar = (cosphi*coskap + sinphi*sinomg*sinkap)*V[0] + cosomg*sinkap*V[1] +
											(-sinphi*coskap + cosphi*sinomg*sinkap)*V[2];
				double Ybar = (-cosphi*sinkap + sinphi*sinomg*coskap)*V[0] + cosomg*coskap*V[1] +
											(sinphi*sinkap + cosphi*sinomg*coskap)*V[2];
				double Zbar = sinphi*cosomg*V[0] - sinomg*V[1] + cosphi*cosomg*V[2];
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
				double partial_Xbar_phi = V[0] * partial_a1phi + V[1] * partial_b1phi + V[2] * partial_c1phi;
				double partial_Xbar_omg = V[0] * partial_a1omg + V[1] * partial_b1omg + V[2] * partial_c1omg;
				double partial_Xbar_kap = V[0] * partial_a1kap + V[1] * partial_b1kap + V[2] * partial_c1kap;
				double partial_Ybar_phi = V[0] * partial_a2phi + V[1] * partial_b2phi + V[2] * partial_c2phi;
				double partial_Ybar_omg = V[0] * partial_a2omg + V[1] * partial_b2omg + V[2] * partial_c2omg;
				double partial_Ybar_kap = V[0] * partial_a2kap + V[1] * partial_b2kap + V[2] * partial_c2kap;
				double partial_Zbar_phi = V[0] * partial_a3phi + V[1] * partial_b3phi + V[2] * partial_c3phi;
				double partial_Zbar_omg = V[0] * partial_a3omg + V[1] * partial_b3omg + V[2] * partial_c3omg;
				double partial_Zbar_kap = V[0] * partial_a3kap + V[1] * partial_b3kap + V[2] * partial_c3kap;
				//				
				double FX[Param], FY[Param];			
				FX[0] = 1;
				FX[1] = 0;
				FX[2] = -Xbar / Zbar;
				FX[3] = Xest[2]*(1 / Zbar*partial_Xbar_phi - Xbar / Zbar / Zbar*partial_Zbar_phi);
				FX[4] = Xest[2] * (1 / Zbar*partial_Xbar_omg - Xbar / Zbar / Zbar*partial_Zbar_omg);
				FX[5] = Xest[2] * (1 / Zbar*partial_Xbar_kap - Xbar / Zbar / Zbar*partial_Zbar_kap);			
				L = Xp - Xest[0]+ Xest[2] * Xbar / Zbar;
				mBase.pNormal(FX, Param, L, ATA, ATL, 1.0);

				FY[0] = 0;
				FY[1] = 1;
				FY[2] = -Ybar / Zbar;
				FY[3] = Xest[2] * (1 / Zbar*partial_Ybar_phi - Ybar / Zbar / Zbar*partial_Zbar_phi);
				FY[4] = Xest[2] * (1 / Zbar*partial_Ybar_omg - Ybar / Zbar / Zbar*partial_Zbar_omg);
				FY[5] = Xest[2] * (1 / Zbar*partial_Ybar_kap - Ybar / Zbar / Zbar*partial_Zbar_kap);
				L = Yp - Xest[1] + Xest[2] * Ybar / Zbar;
				mBase.pNormal(FY, Param, L, ATA, ATL, 1.0);
			}
			//迭代求解
			mBase.Gauss(ATA, ATL, Param);
			if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
				&&abs(ATL[3] - iter_k1) < 1e-10 &&abs(ATL[4] - iter_p1) < 1e-10 &&abs(ATL[5] - iter_p2) < 1e-10
				|| iter_count > 5)
				break;
			iter_count++;
			Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
			Xest[3] += ATL[3]; Xest[4] += ATL[4]; Xest[5] += ATL[5];
			iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
			iter_k1 = abs(ATL[3]), iter_p1 = abs(ATL[4]), iter_p2 = abs(ATL[5]);
		}		
	//输出
	char output[512];
	sprintf(output, "严密RPY参数定标第%d景", index);
	OutputFile(Xest, 6, output);
}


//////////////////////////////////////////////////////////////////////////
//功能：根据严密几何，建立的定标模型
//输入：getGCP，控制点
//输出：严密几何模型得到3参数：主点f，主距x0, y0，径向畸变k1，切向畸变p1，p2
//注意：getGCP得到的点坐标系方向为 X→Y↑, 而星敏的坐标系为X↓Y→
//作者：GZC
//日期：2017.01.14
//////////////////////////////////////////////////////////////////////////
//void APScalibration::CalibrateRigorousQuat(vector<StarGCP> getGCP)
//{
//	int num = getGCP.size();
//	const int Param = 6;
//	//有关法化计算的数组
//	double *ATA = new double[Param*Param];
//	double *ATL = new double[Param];
//	double L;
//	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
//	//Detx0	Dety0	 f			  k1		   p1         p2
//	double Xest[Param], V[3];
//	memset(Xest, 0, sizeof(double) * Param);
//	//设置初值
//	Xest[0] = 512; Xest[1] = 512;
//	Xest[2] = 43.3 / 0.015;
//	double quater[4], Crj[9];
//	qMethod6(getGCP, Xest, Param, quater);
//	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
//	double DetX, DetY;
//	//设置迭代判断初值
//	double iter_x0 = 512, iter_y0 = 512, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0;
//	int iter_count = 0, iter_count2 = 0;
//	//构建Ru并设初值为0；
//	double phi = atan(Crj[6] / Crj[8]), omg = -asin(Crj[7]), kap = atan(Crj[1] / Crj[4]);
//	double sinphi, sinomg, sinkap, cosphi, cosomg, coskap;
//	while (true)
//	{
//		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
//		sinphi = sin(phi); sinomg = sin(omg); sinkap = sin(kap);
//		cosphi = cos(phi); cosomg = cos(omg); coskap = cos(kap);
//		for (int i = 0; i < num; i++)
//		{
//			double V[3] = { getGCP[i].V[0],getGCP[i].V[1], getGCP[i].V[2] };
//			//mBase.Multi(Crj, Vtemp, V, 3, 3, 1);
//			double Xp, Yp;
//			Xp = 1024 - getGCP[i].y;		Yp = getGCP[i].x;
//			double Xbar = (cosphi*coskap + sinphi*sinomg*sinkap)*V[0] + cosomg*sinkap*V[1] +
//				(-sinphi*coskap + cosphi*sinomg*sinkap)*V[2];
//			double Ybar = (-cosphi*sinkap + sinphi*sinomg*coskap)*V[0] + cosomg*coskap*V[1] +
//				(sinphi*sinkap + cosphi*sinomg*coskap)*V[2];
//			double Zbar = sinphi*cosomg*V[0] - sinomg*V[1] + cosphi*cosomg*V[2];
//			//q4对q1q2q3的偏微分
//			double 
//			double q4q1 = 0.5*sqrt(1-)
//			//对q1的偏微分
//			double a1q1 = 
//			//对omega角的偏微分
//			double partial_a1omg = sinphi*cosomg*sinkap;
//			double partial_a2omg = -sinomg*sinkap;
//			double partial_a3omg = cosphi*cosomg*sinkap;
//			double partial_b1omg = sinphi*cosomg*coskap;
//			double partial_b2omg = -sinomg*coskap;
//			double partial_b3omg = cosphi*cosomg*coskap;
//			double partial_c1omg = -sinphi*sinomg;
//			double partial_c2omg = -cosomg;
//			double partial_c3omg = -cosphi*sinomg;
//			//对kappa角的偏微分
//			double partial_a1kap = -cosphi*sinkap + sinphi*sinomg*coskap;
//			double partial_a2kap = cosomg*coskap;
//			double partial_a3kap = sinphi*sinkap + cosphi*sinomg*coskap;
//			double partial_b1kap = -cosphi*coskap - sinphi*sinomg*sinkap;
//			double partial_b2kap = -cosomg*sinkap;
//			double partial_b3kap = sinphi*coskap - cosphi*sinomg*sinkap;
//			double partial_c1kap = 0;
//			double partial_c2kap = 0;
//			double partial_c3kap = 0;
//			//求Xbar,Ybar,Zbar三者的偏导数
//			double partial_Xbar_phi = V[0] * partial_a1phi + V[1] * partial_b1phi + V[2] * partial_c1phi;
//			double partial_Xbar_omg = V[0] * partial_a1omg + V[1] * partial_b1omg + V[2] * partial_c1omg;
//			double partial_Xbar_kap = V[0] * partial_a1kap + V[1] * partial_b1kap + V[2] * partial_c1kap;
//			double partial_Ybar_phi = V[0] * partial_a2phi + V[1] * partial_b2phi + V[2] * partial_c2phi;
//			double partial_Ybar_omg = V[0] * partial_a2omg + V[1] * partial_b2omg + V[2] * partial_c2omg;
//			double partial_Ybar_kap = V[0] * partial_a2kap + V[1] * partial_b2kap + V[2] * partial_c2kap;
//			double partial_Zbar_phi = V[0] * partial_a3phi + V[1] * partial_b3phi + V[2] * partial_c3phi;
//			double partial_Zbar_omg = V[0] * partial_a3omg + V[1] * partial_b3omg + V[2] * partial_c3omg;
//			double partial_Zbar_kap = V[0] * partial_a3kap + V[1] * partial_b3kap + V[2] * partial_c3kap;
//			//				
//			double FX[Param], FY[Param];
//			FX[0] = 1;
//			FX[1] = 0;
//			FX[2] = -Xbar / Zbar;
//			FX[3] = Xest[2] * (1 / Zbar*partial_Xbar_phi - Xbar / Zbar / Zbar*partial_Zbar_phi);
//			FX[4] = Xest[2] * (1 / Zbar*partial_Xbar_omg - Xbar / Zbar / Zbar*partial_Zbar_omg);
//			FX[5] = Xest[2] * (1 / Zbar*partial_Xbar_kap - Xbar / Zbar / Zbar*partial_Zbar_kap);
//			L = Xp - Xest[0] + Xest[2] * Xbar / Zbar;
//			mBase.pNormal(FX, Param, L, ATA, ATL, 1.0);
//
//			FY[0] = 0;
//			FY[1] = 1;
//			FY[2] = -Ybar / Zbar;
//			FY[3] = Xest[2] * (1 / Zbar*partial_Ybar_phi - Ybar / Zbar / Zbar*partial_Zbar_phi);
//			FY[4] = Xest[2] * (1 / Zbar*partial_Ybar_omg - Ybar / Zbar / Zbar*partial_Zbar_omg);
//			FY[5] = Xest[2] * (1 / Zbar*partial_Ybar_kap - Ybar / Zbar / Zbar*partial_Zbar_kap);
//			L = Yp - Xest[1] + Xest[2] * Ybar / Zbar;
//			mBase.pNormal(FY, Param, L, ATA, ATL, 1.0);
//		}
//		//迭代求解
//		mBase.Gauss(ATA, ATL, Param);
//		if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
//			&&abs(ATL[3] - iter_k1) < 1e-10 &&abs(ATL[4] - iter_p1) < 1e-10 &&abs(ATL[5] - iter_p2) < 1e-10 || iter_count > 20)
//			break;
//		iter_count++;
//		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
//		Xest[3] += ATL[3]; Xest[4] += ATL[4]; Xest[5] += ATL[5];
//		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
//		iter_k1 = abs(ATL[3]), iter_p1 = abs(ATL[4]), iter_p2 = abs(ATL[5]);
//	}
//
//}

//////////////////////////////////////////////////////////////////////////
//功能：q_Method定姿方式
//输入：getGCP：获取的星点控制，Xest：相机内畸变参数，num：参数数量
//输出：四元数Q
//注意：采用Eigen库进行编写
//作者：GZC
//日期：2017.01.12
//////////////////////////////////////////////////////////////////////////
bool APScalibration::qMethod6(vector<StarGCP> getGCP, double *Xest, int Param, double *quater)
{
	MatrixXd obs_in_startrackerframe(getGCP.size(), 3), stars_in_celestialframeV(getGCP.size(), 3);
	double Wob[3];
	double X, Y, DetX, DetY;
	for (int a = 0; a < getGCP.size(); a++)
	{
		double Xp, Yp;
		Xp = 1024 - getGCP[a].y;		Yp = getGCP[a].x;
		//畸变模型
		double X = Xp - Xest[0];
		double Y = Yp - Xest[1];
		double r2 = sqrt(X*X + Y*Y);
		DetX = Xest[3] * X * r2 + Xest[4] * (3 * X * X + Y * Y) + 2 * Xest[5] * X * Y;
		DetY = Xest[3] * Y * r2 + Xest[5] * (3 * Y * Y + X * X) + 2 * Xest[4] * X * Y;
		//赋值，将像方坐标系转到星敏坐标系，星敏坐标系为-(X-X0)，-(Y-Y0)，f；然后
		double D = sqrt(pow((X - DetX - Xest[0]),2) + pow((Y - DetY - Xest[1]),2) + pow(Xest[2],2));
		//double Wob[3];
		Wob[0] = -(X - DetX) / D;
		Wob[1] = -(Y - DetY) / D;
		Wob[2] = Xest[2] / D;
		obs_in_startrackerframe.row(a) << Wob[0], Wob[1], Wob[2];
		stars_in_celestialframeV.row(a) << getGCP[a].V[0], getGCP[a].V[1], getGCP[a].V[2];
	}
	MatrixXd W(3, getGCP.size()), V(3, getGCP.size());	
	W = obs_in_startrackerframe.transpose();
	V = stars_in_celestialframeV.transpose();
	Matrix3d B, S;
	B = W*V.transpose();
	S = B + B.transpose();
	Vector3d Z;
	Z << B(1, 2) - B(2, 1), B(2, 0) - B(0, 2), B(0, 1) - B(1, 0);
	double SIGMA = B.trace();
	Matrix<double, 4, 4>K;
	K << S - MatrixXd::Identity(3, 3)*SIGMA, Z, Z.transpose(), SIGMA;
	EigenSolver<Matrix<double, 4, 4>> es(K);
	MatrixXcd evecs = es.eigenvectors();
	MatrixXcd evals = es.eigenvalues();
	MatrixXd evalsReal;
	evalsReal = evals.real();
	MatrixXf::Index evalsMax;
	evalsReal.rowwise().sum().maxCoeff(&evalsMax);
	Vector4d q;
	q << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax), evecs.real()(3, evalsMax);
	//输出顺序为0123，标量为0
	quater[0] = q(3); quater[1] = q(0); quater[2] = q(1); quater[3] = q(2);
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//功能：输出定标结果
//输入：
//输出：
//注意：
//作者：GZC
//日期：2017.01.14
//////////////////////////////////////////////////////////////////////////
void APScalibration::OutputFile(double * Xest, int num,string description)
{
	string path = workpath + "定标结果！！！.txt";
	FILE *fp = fopen(path.c_str(), "a+");
	fprintf(fp,"%s\t", description.c_str());
	for (int i = 0; i < num; i++)
	{
		fprintf(fp, "%.9f\t", Xest[i]);
	}
	fprintf(fp, "\n");
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//功能：q_Method定姿方式
//输入：getGCP：获取的星点控制，Xest：相机内畸变参数，num：参数数量
//输出：四元数Q
//注意：采用Eigen库进行编写
//作者：GZC
//日期：2017.01.12
//////////////////////////////////////////////////////////////////////////
bool APScalibration::qMethod(vector<StarGCP> getGCP, double *Xest, int Param, double *quater)
{
	MatrixXd obs_in_startrackerframe(getGCP.size(), 3), stars_in_celestialframeV(getGCP.size(), 3);
	double Wob[3];
	double X, Y, DetX, DetY;
	for (int a = 0; a < getGCP.size(); a++)
	{		
		//赋值，将像方坐标系转到星敏坐标系，星敏坐标系为-(X-X0)，-(Y-Y0)，f；然后
		X =  1024-getGCP[a].y;		Y = getGCP[a].x;		
		double D = sqrt((X - Xest[0])*(X - Xest[0]) + (Y - Xest[1])*(Y - Xest[1]) + Xest[2] * Xest[2]);
		//double Wob[3];
		Wob[0] = -(X - Xest[0])/D;
		Wob[1] = -(Y - Xest[1])/D;
		Wob[2] = Xest[2] / D;	
		obs_in_startrackerframe.row(a) << Wob[0], Wob[1], Wob[2];
		stars_in_celestialframeV.row(a) << getGCP[a].V[0], getGCP[a].V[1], getGCP[a].V[2];
	}
	MatrixXd W(3, getGCP.size()), V(3, getGCP.size());
	/*MatrixXd obs_in_startrackerframe(6, 3), stars_in_celestialframeV(6, 3);
	double WW[18] = {
	0.0623983740354064,	0.0326875431675405,	0.997515898339173,
		0.0466218623533174,	0.0307762213743818,	0.998438393767299,
		- 0.0514466419108713,	0.0317300918171719,	0.998171550540972,
		- 0.0670591218971525, - 0.0143664675871821,	0.997645567714030,
		0.0465261487754611,	0.0392349537174110,	0.998146249748461,
		0.0464370594121891,	0.0429966052309510,	0.997995436588646 };
		double VV[18]{
		-0.370678862615461,	0.472075338442427,	0.799838768530618,
		- 0.379972004238474,	0.459434550983920,	0.802833213910088,
		- 0.443691485579987,	0.385051726777690,	0.809242258739213,
		- 0.423223585177919	,0.349743732574587,	0.835799688007805,
		- 0.385487451849472	,0.463507336729176,	0.797847337066953,
		- 0.387985835419495	,0.465289521484083,	0.795595784749360 };
		for (int i = 0; i < 6; i++)
		{
			obs_in_startrackerframe.row(i) << WW[3 * i], WW[3 * i + 1], WW[3 * i + 2];
			stars_in_celestialframeV.row(i) << VV[3 * i], VV[3 * i + 1], VV[3 * i + 2];
		}		
	MatrixXd W(3, 6), V(3, 6);*/
	W = obs_in_startrackerframe.transpose();
	V = stars_in_celestialframeV.transpose();
	Matrix3d B, S;
	B = W*V.transpose();
	S = B + B.transpose();
	Vector3d Z;
	Z << B(1, 2) - B(2, 1), B(2, 0) - B(0, 2), B(0, 1) - B(1, 0);
	double SIGMA = B.trace();
	Matrix<double, 4, 4>K;
	K << S - MatrixXd::Identity(3, 3)*SIGMA, Z, Z.transpose(), SIGMA;
	EigenSolver<Matrix<double, 4, 4>> es(K);
	MatrixXcd evecs = es.eigenvectors();
	MatrixXcd evals = es.eigenvalues();
	MatrixXd evalsReal;
	evalsReal = evals.real();
	MatrixXf::Index evalsMax;
	evalsReal.rowwise().sum().maxCoeff(&evalsMax);
	Vector4d q;
	q << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax), evecs.real()(3, evalsMax);
	//输出顺序为0123，标量为0
	quater[0] = q(3); quater[1] = q(0); quater[2] = q(1); quater[3] = q(2);
	return 0;
}