#include "APScalibration.h"



APScalibration::APScalibration()
{
}


APScalibration::~APScalibration()
{
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����������꺯����������С���˵������
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�3������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.01.11
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate3Param(vector<StarGCP> getGCP, int index)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	//δ֪��������
	const int Param = 3;
	//����XestΪ�����Ƶ�6��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
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
				//��ʵ�Ǿ�
				//double  VTV1, VTV2;
				//mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV1, 1, 3, 1);
				////�����Ǿ�
				//double V1[3],V2[3];
				//V1[0] = (getGCP[a].x - 512)*0.015;	V1[1] = (getGCP[a].y - 512)*0.015;	V1[2] = 43.3;
				//V2[0] = (getGCP[b].x - 512)*0.015;	V2[1] = (getGCP[b].y - 512)*0.015;	V2[2] = 43.3;
				//mBase.NormVector(V1, 3);
				//mBase.NormVector(V2, 3);
				//mBase.Multi(V1, V2, &VTV2, 1, 3, 1);
				//double det = VTV1 - VTV2;
				//if (det > 0.0001)
				//	continue;

				//��ֵ
				/*Xa = getGCP[a].x ;		Ya = getGCP[a].y ;
				Xb = getGCP[b].x;		Yb = getGCP[b].y;*/
				Xa = 1024 - getGCP[a].y - 512;		Ya = getGCP[a].x - 512;
				Xb = 1024 - getGCP[b].y - 512;		Yb = getGCP[b].x - 512;
				/*Xa = 512 - getGCP[a].x;		Ya = 512 - getGCP[a].y;
				Xb = 512 - getGCP[b].x;		Yb = 512 - getGCP[b].y;*/
				//����ģ��
				DetX = Xest[0];
				DetY = Xest[1];
				double N = (Xa - DetX)*(Xb - DetX) + (Ya - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
				double D1 = (Xa - DetX)*(Xa - DetX) + (Ya - DetY)*(Ya - DetY) + Xest[2] * Xest[2];
				double D2 = (Xb - DetX)*(Xb - DetX) + (Yb - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
				double D0 = sqrt(D1*D2);

				//��һ��ƫ��
				double D1x0_Par = -2 * (Xa - DetX);
				double D1y0_Par = -2 * (Ya - DetY);
				double D1f_Par = 2 * Xest[2];
				double D2x0_Par = -2 * (Xb - DetX);
				double D2y0_Par = -2 * (Yb - DetY);
				double D2f_Par = 2 * Xest[2];

				//�ڶ���ƫ����һ����
				double D0x0_Par = 0.5*sqrt(D2 / D1)*D1x0_Par + 0.5*sqrt(D1 / D2)*D2x0_Par;
				double D0y0_Par = 0.5*sqrt(D2 / D1)*D1y0_Par + 0.5*sqrt(D1 / D2)*D2y0_Par;
				double D0f_Par = 0.5*sqrt(D2 / D1)*D1f_Par + 0.5*sqrt(D1 / D2)*D2f_Par;

				//�ڶ���ƫ���ڶ�����
				double Nx0_Par = 2 * DetX - Xa - Xb;
				double Ny0_Par = 2 * DetY - Ya - Yb;
				double Nf_Par = 2 * Xest[2];

				//������ƫ��
				double G[Param];
				G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
				G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
				G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;

				//��ʵ�Ǿ�
				double  VTV;
				mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV, 1, 3, 1);
				L = VTV - N / D0;
				mBase.pNormal(G, Param, L, ATA, ATL, 1.0);
			}
		}
		//�������
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-20&&abs(ATL[1] - iter_y0) < 1e-20&&abs(ATL[2] - iter_f) < 1e-20
			|| iter_count > 20)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
	}
	//��������
	//CaliAccuracy(getGCP, Xest, 3);
	char output[512];
	sprintf_s(output, "�Ǿ�3���������%d��", index);
	OutputFile(Xest, 3, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����������꺯����������С���˵�����⣬�漰���Ӱ��������
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�3������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.01.18
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate3ParamMultiImg(vector<vector<StarGCP>>getGCP, int index)
{
	//δ֪��������
	const int Param = 3;
	//����XestΪ�����Ƶ�6��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_count = 0;
	int numIMG, numGCP;
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
					//��ֵ����
					/*Xa = 1024 - getGCP[i][a].y - 512 + 2;		Ya = getGCP[i][a].x - 512 - 3;
					Xb = 1024 - getGCP[i][b].y - 512 + 2;		Yb = getGCP[i][b].x - 512 - 3;*/
					//��ֵ
					Xa = 1024 - getGCP[i][a].y - 512;		Ya = getGCP[i][a].x - 512;
					Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;
					//����ģ��
					DetX = Xest[0];
					DetY = Xest[1];
					double N = (Xa - DetX)*(Xb - DetX) + (Ya - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
					double D1 = (Xa - DetX)*(Xa - DetX) + (Ya - DetY)*(Ya - DetY) + Xest[2] * Xest[2];
					double D2 = (Xb - DetX)*(Xb - DetX) + (Yb - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
					double D0 = sqrt(D1*D2);

					//��һ��ƫ��
					double D1x0_Par = -2 * (Xa - DetX);
					double D1y0_Par = -2 * (Ya - DetY);
					double D1f_Par = 2 * Xest[2];
					double D2x0_Par = -2 * (Xb - DetX);
					double D2y0_Par = -2 * (Yb - DetY);
					double D2f_Par = 2 * Xest[2];

					//�ڶ���ƫ����һ����
					double D0x0_Par = 0.5*sqrt(D2 / D1)*D1x0_Par + 0.5*sqrt(D1 / D2)*D2x0_Par;
					double D0y0_Par = 0.5*sqrt(D2 / D1)*D1y0_Par + 0.5*sqrt(D1 / D2)*D2y0_Par;
					double D0f_Par = 0.5*sqrt(D2 / D1)*D1f_Par + 0.5*sqrt(D1 / D2)*D2f_Par;

					//�ڶ���ƫ���ڶ�����
					double Nx0_Par = 2 * DetX - Xa - Xb;
					double Ny0_Par = 2 * DetY - Ya - Yb;
					double Nf_Par = 2 * Xest[2];

					//������ƫ��
					double G[Param];
					G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
					G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
					G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;

					//��ʵ�Ǿ�
					double  VTV;
					mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
					L = VTV - N / D0;
					mBase.pNormal(G, Param, L, ATA, ATL, 1.0);
				}
			}
		}

		//�������
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
			|| iter_count > 20)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
	}
	//��������
	//CaliAccuracy(getGCP, Xest, 3);
	char output[512];
	sprintf_s(output, "�Ǿ�3���������%d��", index);
	OutputFile(Xest, 3, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�������������������˲�����
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�3������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.04.11
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate3ParamKalman(vector<vector<StarGCP>>getGCP)
{
	//δ֪��������
	const int Param = 3;
	//����XestΪ�����Ƶ�6��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[0] = 512; Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_count = 0;
	int numIMG, numGCP;
	double Xa, Ya, Xb, Yb, DetX, DetY;

	//Kalman���
	int mNum = 3;//�۲ⷽ������
	Vector3d Xk, Xk1, Zk;
	Matrix3d Pk1;
	MatrixXd Kal(3, 3);
	MatrixXd Pk = Matrix3d::Identity(3, 3);
	MatrixXd Hk = Matrix3d::Identity(3, 3);
	MatrixXd Rk = Matrix3d::Identity(3, 3);//�ǵ���ȡ���
	Xk << Xest[0], Xest[1], Xest[2];

	numIMG = getGCP.size();
	for (size_t i = 0; i < numIMG; i++)
	{
		double G[Param];//�ڴ˴����壬����������ѭ��ʱ�õ����һ��״̬ת�ƾ���H;
		while (true)
		{
			numGCP = getGCP[i].size();
			memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
			//����������
			for (int a = 0; a < numGCP - 1; a++)
			{
				for (int b = a + 1; b < numGCP; b++)
				{
					//��ʵ����ʹ��
					Xa = 1024 - getGCP[i][a].y;		Ya = getGCP[i][a].x;
					Xb = 1024 - getGCP[i][b].y;		Yb = getGCP[i][b].x;
					//��ֵ
					//Xa = 1024 - getGCP[i][a].y - 512;		Ya = getGCP[i][a].x - 512;
					//Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;
					//����ģ��
					DetX = Xest[0];
					DetY = Xest[1];
					double N = (Xa - DetX)*(Xb - DetX) + (Ya - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
					double D1 = (Xa - DetX)*(Xa - DetX) + (Ya - DetY)*(Ya - DetY) + Xest[2] * Xest[2];
					double D2 = (Xb - DetX)*(Xb - DetX) + (Yb - DetY)*(Yb - DetY) + Xest[2] * Xest[2];
					double D0 = sqrt(D1*D2);

					//��һ��ƫ��
					double D1x0_Par = -2 * (Xa - DetX);
					double D1y0_Par = -2 * (Ya - DetY);
					double D1f_Par = 2 * Xest[2];
					double D2x0_Par = -2 * (Xb - DetX);
					double D2y0_Par = -2 * (Yb - DetY);
					double D2f_Par = 2 * Xest[2];

					//�ڶ���ƫ����һ����
					double D0x0_Par = 0.5*sqrt(D2 / D1)*D1x0_Par + 0.5*sqrt(D1 / D2)*D2x0_Par;
					double D0y0_Par = 0.5*sqrt(D2 / D1)*D1y0_Par + 0.5*sqrt(D1 / D2)*D2y0_Par;
					double D0f_Par = 0.5*sqrt(D2 / D1)*D1f_Par + 0.5*sqrt(D1 / D2)*D2f_Par;

					//�ڶ���ƫ���ڶ�����
					double Nx0_Par = 2 * DetX - Xa - Xb;
					double Ny0_Par = 2 * DetY - Ya - Yb;
					double Nf_Par = 2 * Xest[2];

					//������ƫ��
					G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
					G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
					G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;

					//��ʵ�Ǿ�
					double  VTV;
					mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
					L = VTV - N / D0;
					mBase.pNormal(G, Param, L, ATA, ATL, 1.0);
				}
			}
			//�������
			mBase.Gauss(ATA, ATL, Param);
			Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];

			if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
				|| iter_count > 20)
				break;
			iter_count++;
			iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
		}

		////////////////////////////////////////////////
		//	�������˲�
		////////////////////////////////////////////////
		Kal = Pk*Hk.transpose()*(Hk*Pk*Hk.transpose() + Rk).inverse();		
		Zk << Xest[0], Xest[1], Xest[2];		
		Xk1 = Xk + Kal*(Zk - Hk*Xk);
		Pk1 = (Matrix3d::Identity(3, 3) - Kal*Hk)*Pk;
		/*cout << Xk1 << endl;
		cout << Zk << endl;
		cout << Kal << endl;
		cout << Pk1<<endl;*/
		Pk = Pk1;
		Xk = Xk1;

		//��������
		//printf("\r�����У���%d��", i);
		//char output[512];
		//sprintf_s(output, "�Ǿ�3���������%d��", i);
		//double Xe[3] = { Xk(0),Xk(1), Xk(2) };
		//OutputFile(Xe, 3, output);

		//��������
		double Xoutput[5];
		double Xe[3] = { Xk(0),Xk(1), Xk(2) };
		memcpy(Xoutput, Xe, sizeof(Xe));//sizeof(Xest)��ʾ�ľ�������Xest�ˣ�����Ҫ�ٳ���5
		vector<vector<StarGCP>>getGCPaccu;
		getGCPaccu.assign(getGCP.begin(), getGCP.begin() + i + 1);
		CaliAccuracy3Param(getGCPaccu, Xe, Xoutput[3]);
		Xe[0] = 512;	Xe[1] = 512;	Xe[2] = 43.3 / 0.015;
		CaliAccuracy3Param(getGCPaccu, Xe, Xoutput[4]);
		getGCPaccu.clear();
		//���
		char output[512];
		sprintf_s(output, "�Ǿ�3���������%d��", i);
		OutputFile(Xoutput, 5, output);
		printf("\r�����У���%d��", i);
	}

}

//////////////////////////////////////////////////////////////////////////
//���ܣ�������������������˲�����
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�3������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.04.11
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate5ParamKalman2(vector<vector<StarGCP>>getGCP)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	//δ֪��������
	const int Param = 5;
	//����XestΪ�����Ƶ�5��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  
	//Detx0	Dety0	 f			  k1		   k1       
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	//Xest[0] = 512;
	//Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_k2 = 0, iter_count = 0;
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	//Kalman���
	MatrixXd Xk(Param,1), Xk1(Param, 1);
	MatrixXd Pk1(Param,Param);
	MatrixXd Pk = MatrixXd::Identity(Param, Param);		
	Xk << Xest[0], Xest[1], Xest[2], Xest[3], Xest[4];

	for (size_t i = 0; i < getGCP.size(); i++)
	{
		iter_count = 0;
		int hNum = getGCP[i].size();
		int j = 0;
		hNum = hNum*(hNum - 1) / 2;
		MatrixXd Hk(hNum, Param);
		MatrixXd Zk(hNum, 1);
		MatrixXd Rk = MatrixXd::Identity(hNum, hNum)*1/0.3;//�ǵ���ȡ���		
		MatrixXd Kal(Param, hNum);
		while (true)
		{
			memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
			for (int a = 0; a < getGCP[i].size() - 1; a++)
			{
				for (int b = a + 1; b < getGCP[i].size(); b++)
				{
					//��ʵ����ʹ��
					Xa = 1024 - getGCP[i][a].y;		Ya = getGCP[i][a].x;
					Xb = 1024 - getGCP[i][b].y;		Yb = getGCP[i][b].x;
					//��ʵ����ʹ��
					/*Xa = 1024 - getGCP[i][a].y - 512;			Ya = getGCP[i][a].x - 512;
					Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;*/
					//�����ʱ����
					/*Xa = getGCP[i][a].x;		Ya = getGCP[i][a].y;
					Xb = getGCP[i][b].x;		Yb = getGCP[i][b].y;*/

					//ģ��
					double ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
					double rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
					double Ta = 1 - Xest[3] * ra2 - Xest[4] * ra2*ra2;
					double Tb = 1 - Xest[3] * rb2 - Xest[4] * rb2*rb2;
					double N = (Xa - Xest[0])*(Xb - Xest[0])*Ta*Tb + (Ya - Xest[1])*(Yb - Xest[1])*Ta*Tb + Xest[2] * Xest[2];
					double D1 = (Xa - Xest[0])*(Xa - Xest[0])*Ta*Ta + (Ya - Xest[1])*(Ya - Xest[1])*Ta*Ta + Xest[2] * Xest[2];
					double D2 = (Xb - Xest[0])*(Xb - Xest[0])*Tb*Tb + (Yb - Xest[1])*(Yb - Xest[1])*Tb*Tb + Xest[2] * Xest[2];
					double D0 = sqrt(D1*D2);

					//��һ��ƫ��
					double Tak1_Par = -ra2;
					double Tbk1_Par = -rb2;
					double Tak2_Par = -ra2*ra2;
					double Tbk2_Par = -rb2*rb2;

					double D1x0_Par = -2 * Ta*Ta * (Xa - Xest[0]);
					double D1y0_Par = -2 * Ta*Ta* (Ya - Xest[1]);
					double D1f_Par = 2 * Xest[2];
					double D1k1_Par = 2 * Ta*ra2*Tak1_Par;
					double D1k2_Par = 2 * Ta*ra2*Tak2_Par;

					double D2x0_Par = -2 * Tb*Tb * (Xb - Xest[0]);
					double D2y0_Par = -2 * Tb*Tb* (Yb - Xest[1]);
					double D2f_Par = 2 * Xest[2];
					double D2k1_Par = 2 * Tb*rb2*Tbk1_Par;
					double D2k2_Par = 2 * Tb*rb2*Tbk2_Par;

					//�ڶ���ƫ����һ����
					double D0x0_Par = 0.5*(sqrt(D2 / D1)*D1x0_Par + sqrt(D1 / D2)*D2x0_Par);
					double D0y0_Par = 0.5*(sqrt(D2 / D1)*D1y0_Par + sqrt(D1 / D2)*D2y0_Par);
					double D0f_Par = 0.5*(sqrt(D2 / D1)*D1f_Par + sqrt(D1 / D2)*D2f_Par);
					double D0k1_Par = 0.5*(sqrt(D2 / D1)*D1k1_Par + sqrt(D1 / D2)*D2k1_Par);
					double D0k2_Par = 0.5*(sqrt(D2 / D1)*D1k2_Par + sqrt(D1 / D2)*D2k2_Par);

					//�ڶ���ƫ���ڶ�����
					double Nx0_Par = 2 * Ta*Tb*Xest[0] - Ta*Tb*Xa - Ta*Tb*Xb;
					double Ny0_Par = 2 * Ta*Tb*Xest[1] - Ta*Tb*Ya - Ta*Tb*Yb;
					double Nf_Par = 2 * Xest[2];
					double Nk1_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk1_Par + Tb*Tak1_Par);
					double Nk2_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk2_Par + Tb*Tak2_Par);

					//������ƫ��
					double G[5];
					G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
					G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
					G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
					G[3] = (Nk1_Par*D0 - D0k1_Par*N) / D0 / D0;
					G[4] = (Nk2_Par*D0 - D0k2_Par*N) / D0 / D0;

					//��ʵ�Ǿ�
					double  VTV;
					mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);						
					L = VTV - N / D0;
					mBase.pNormal(G, 5, L, ATA, ATL, 1.0);			
					//�������˲��۲�����ת�ƾ���
					Zk(j) = L;
					Hk.row(j) << G[0], G[1], G[2], G[3], G[4];					
					j++;
				}
			}
			j = 0;
			
			//�������
			mBase.Gauss(ATA, ATL, Param);
			if (abs(ATL[0] - iter_x0) < 1e-7&&abs(ATL[1] - iter_y0) < 1e-7&&abs(ATL[2] - iter_f) < 1e-9
				&&abs(ATL[3] - iter_k1) < 1e-15 &&abs(ATL[4] - iter_k2) < 1e-20 || iter_count > 50)
				break;
			iter_count++;
			Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
			Xest[3] += ATL[3]; Xest[4] += ATL[4];
			iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
			iter_k1 = abs(ATL[3]), iter_k2 = abs(ATL[4]);
		}
		////////////////////////////////////////////////
		//	�������˲�
		////////////////////////////////////////////////
		Xk << Xest[0], Xest[1], Xest[2], Xest[3], Xest[4];
		Pk = (Pk.inverse() + Hk.transpose()*Rk*Hk).inverse();
		Kal = Pk*Hk.transpose()*Rk;
		cout << Xk << endl;
		Xk = Xk + Kal*(Zk - Hk*Xk);
		//cout << Hk << endl;
		//cout << Pk << endl;
		//cout << Kal << endl;
		cout << Xk << endl;
		Xest[0] = Xk(0), Xest[1] = Xk(1), Xest[2] = Xk(2), Xest[3] = Xk(3), Xest[4] = Xk(4);	
		//��������
		//printf("\r�����У���%d��", i);
		char output[512];
		sprintf_s(output, "�Ǿ�3���������%d��", i);
		OutputFile(Xest, 5, output);
	}

}

//////////////////////////////////////////////////////////////////////////
//���ܣ�������������������˲�����
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�5������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.04.12
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate5ParamKalman(vector<vector<StarGCP>>getGCP)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	//δ֪��������
	const int Param = 5;
	//����XestΪ�����Ƶ�5��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  
	//Detx0	Dety0	 f			  k1		   k1       
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[0] = 512;
	Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_k2 = 0, iter_count = 0;
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	//Kalman���
	int mNum = 3;//�۲ⷽ������
	VectorXd Xk(Param, 1), Xk1(Param, 1), Zk(Param, 1);
	MatrixXd Pk1(Param, Param);
	MatrixXd Kal(Param, Param);
	MatrixXd Pk = MatrixXd::Identity(Param, Param);
	MatrixXd Hk = MatrixXd::Identity(Param, Param);
	MatrixXd Rk = MatrixXd::Identity(Param, Param);//�ǵ���ȡ���
	Xk << Xest[0], Xest[1], Xest[2], Xest[3], Xest[4];
	
	//ȥ���������ļ��������
	string path = workpath + "������������.txt";
	FILE *fp = fopen(path.c_str(), "w");
	fclose(fp);	

	for (size_t i = 0; i < getGCP.size(); i++)
	{
		iter_count = 0;
		while (true)
		{
			memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
			for (int a = 0; a < getGCP[i].size() - 1; a++)
			{
				for (int b = a + 1; b < getGCP[i].size(); b++)
				{
					//��ʵ����ʹ��
					Xa = 1024 - getGCP[i][a].y;		Ya = getGCP[i][a].x;
					Xb = 1024 - getGCP[i][b].y;		Yb = getGCP[i][b].x;
					//��ʵ����ʹ��
					//Xa = 1024 - getGCP[i][a].y - 512;			Ya = getGCP[i][a].x - 512;
					//Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;
					//�����ʱ����
					//Xa = getGCP[i][a].x;		Ya = getGCP[i][a].y;
					//Xb = getGCP[i][b].x;		Yb = getGCP[i][b].y;

					//ģ��
					double ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
					double rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
					double Ta = 1 - Xest[3] * ra2 - Xest[4] * ra2*ra2;
					double Tb = 1 - Xest[3] * rb2 - Xest[4] * rb2*rb2;
					double N = (Xa - Xest[0])*(Xb - Xest[0])*Ta*Tb + (Ya - Xest[1])*(Yb - Xest[1])*Ta*Tb + Xest[2] * Xest[2];
					double D1 = (Xa - Xest[0])*(Xa - Xest[0])*Ta*Ta + (Ya - Xest[1])*(Ya - Xest[1])*Ta*Ta + Xest[2] * Xest[2];
					double D2 = (Xb - Xest[0])*(Xb - Xest[0])*Tb*Tb + (Yb - Xest[1])*(Yb - Xest[1])*Tb*Tb + Xest[2] * Xest[2];
					double D0 = sqrt(D1*D2);

					//��һ��ƫ��
					double Tak1_Par = -ra2;
					double Tbk1_Par = -rb2;
					double Tak2_Par = -ra2*ra2;
					double Tbk2_Par = -rb2*rb2;

					double D1x0_Par = -2 * Ta*Ta * (Xa - Xest[0]);
					double D1y0_Par = -2 * Ta*Ta* (Ya - Xest[1]);
					double D1f_Par = 2 * Xest[2];
					double D1k1_Par = 2 * Ta*ra2*Tak1_Par;
					double D1k2_Par = 2 * Ta*ra2*Tak2_Par;

					double D2x0_Par = -2 * Tb*Tb * (Xb - Xest[0]);
					double D2y0_Par = -2 * Tb*Tb* (Yb - Xest[1]);
					double D2f_Par = 2 * Xest[2];
					double D2k1_Par = 2 * Tb*rb2*Tbk1_Par;
					double D2k2_Par = 2 * Tb*rb2*Tbk2_Par;

					//�ڶ���ƫ����һ����
					double D0x0_Par = 0.5*(sqrt(D2 / D1)*D1x0_Par + sqrt(D1 / D2)*D2x0_Par);
					double D0y0_Par = 0.5*(sqrt(D2 / D1)*D1y0_Par + sqrt(D1 / D2)*D2y0_Par);
					double D0f_Par = 0.5*(sqrt(D2 / D1)*D1f_Par + sqrt(D1 / D2)*D2f_Par);
					double D0k1_Par = 0.5*(sqrt(D2 / D1)*D1k1_Par + sqrt(D1 / D2)*D2k1_Par);
					double D0k2_Par = 0.5*(sqrt(D2 / D1)*D1k2_Par + sqrt(D1 / D2)*D2k2_Par);

					//�ڶ���ƫ���ڶ�����
					double Nx0_Par = 2 * Ta*Tb*Xest[0] - Ta*Tb*Xa - Ta*Tb*Xb;
					double Ny0_Par = 2 * Ta*Tb*Xest[1] - Ta*Tb*Ya - Ta*Tb*Yb;
					double Nf_Par = 2 * Xest[2];
					double Nk1_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk1_Par + Tb*Tak1_Par);
					double Nk2_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk2_Par + Tb*Tak2_Par);

					//������ƫ��
					double G[5];
					G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
					G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
					G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
					G[3] = (Nk1_Par*D0 - D0k1_Par*N) / D0 / D0;
					G[4] = (Nk2_Par*D0 - D0k2_Par*N) / D0 / D0;

					//��ʵ�Ǿ�
					double  VTV;
					mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
					L = VTV - N / D0;
					mBase.pNormal(G, 5, L, ATA, ATL, 1.0);					
				}
			}

			//�������
			mBase.Gauss(ATA, ATL, Param);
			if (abs(ATL[0] - iter_x0) < 1e-7&&abs(ATL[1] - iter_y0) < 1e-7&&abs(ATL[2] - iter_f) < 1e-9
				&&abs(ATL[3] - iter_k1) < 1e-15 &&abs(ATL[4] - iter_k2) < 1e-20 || iter_count > 50)
				break;
			iter_count++;
			Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
			Xest[3] += ATL[3]; Xest[4] += ATL[4];
			iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
			iter_k1 = abs(ATL[3]), iter_k2 = abs(ATL[4]);
		}
		////////////////////////////////////////////////
		//	�������˲�
		////////////////////////////////////////////////
		Kal = Pk*Hk.transpose()*(Hk*Pk*Hk.transpose() + Rk).inverse();
		Zk << Xest[0], Xest[1], Xest[2], Xest[3], Xest[4];
		Xk1 = Xk + Kal*(Zk - Hk*Xk);
		Pk1 = (MatrixXd::Identity(5, 5) - Kal*Hk)*Pk;
		/*cout << Xk1 << endl;
		cout << Zk << endl;
		cout << Kal << endl;
		cout << Pk1<<endl;*/
		Pk = Pk1;
		Xk = Xk1;
		Xest[0] = Xk(0), Xest[1] = Xk(1), Xest[2] = Xk(2), Xest[3] = Xk(3), Xest[4] = Xk(4);
		//��������
		double Xoutput[7];
		memcpy(Xoutput, Xest, sizeof(Xest));//sizeof(Xest)��ʾ�ľ�������Xest�ˣ�����Ҫ�ٳ���5
		vector<vector<StarGCP>>getGCPaccu;
		getGCPaccu.assign(getGCP.begin(), getGCP.begin() + i + 1);
		CaliAccuracy5Param(getGCPaccu, Xest, Xoutput[5]);
		Xest[0] = 512;	Xest[1] = 512;	Xest[2] = 43.3 / 0.015; Xest[3] = 0; Xest[4] = 0;
		CaliAccuracy5Param(getGCPaccu, Xest, Xoutput[6]);
		getGCPaccu.clear();
		//���
		char output[512];
		sprintf_s(output, "�Ǿ�5���������%d��", i);
		OutputFile(Xoutput, 7, output);
		printf("\r�����У���%d��", i);
		//char output[512];
		//sprintf_s(output, "�Ǿ�3���������%d��", i);
		//OutputFile(Xest, 5, output);
	}

}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����������꺯����������С���˹���
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�5������
//ע�⣺�޸���Xa��Ya�ĳ�ʼֵ
//���ߣ�GZC
//���ڣ�2017.04.04
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate5Param(vector<StarGCP>getGCP, int index)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	//δ֪��������
	const int Param = 5;
	//����XestΪ�����Ƶ�5��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  
	//Detx0	Dety0	 f			  k1		   k1       
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[0] = 512;
	Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_k2 = 0, iter_count = 0;
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	while (true)
	{
		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
		for (int a = 0; a < getGCP.size() - 1; a++)
		{
			for (int b = a + 1; b < getGCP.size(); b++)
			{
				//��ʵ����ʹ��
				Xa = 1024 - getGCP[a].y;		Ya = getGCP[a].x;
				Xb = 1024 - getGCP[b].y;		Yb = getGCP[b].x;
				//�����ʱ����
				/*Xa = getGCP[i][a].x;		Ya = getGCP[i][a].y;
				Xb = getGCP[i][b].x;		Yb = getGCP[i][b].y;*/

				//ģ��
				double ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
				double rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
				double Ta = 1 - Xest[3] * ra2 - Xest[4] * ra2*ra2;
				double Tb = 1 - Xest[3] * rb2 - Xest[4] * rb2*rb2;
				double N = (Xa - Xest[0])*(Xb - Xest[0])*Ta*Tb + (Ya - Xest[1])*(Yb - Xest[1])*Ta*Tb + Xest[2] * Xest[2];
				double D1 = (Xa - Xest[0])*(Xa - Xest[0])*Ta*Ta + (Ya - Xest[1])*(Ya - Xest[1])*Ta*Ta + Xest[2] * Xest[2];
				double D2 = (Xb - Xest[0])*(Xb - Xest[0])*Tb*Tb + (Yb - Xest[1])*(Yb - Xest[1])*Tb*Tb + Xest[2] * Xest[2];
				double D0 = sqrt(D1*D2);

				//��һ��ƫ��
				double Tak1_Par = -ra2;
				double Tbk1_Par = -rb2;
				double Tak2_Par = -ra2*ra2;
				double Tbk2_Par = -rb2*rb2;

				double D1x0_Par = -2 * Ta*Ta * (Xa - Xest[0]);
				double D1y0_Par = -2 * Ta*Ta* (Ya - Xest[1]);
				double D1f_Par = 2 * Xest[2];
				double D1k1_Par = 2 * Ta*ra2*Tak1_Par;
				double D1k2_Par = 2 * Ta*ra2*Tak2_Par;

				double D2x0_Par = -2 * Tb*Tb * (Xb - Xest[0]);
				double D2y0_Par = -2 * Tb*Tb* (Yb - Xest[1]);
				double D2f_Par = 2 * Xest[2];
				double D2k1_Par = 2 * Tb*rb2*Tbk1_Par;
				double D2k2_Par = 2 * Tb*rb2*Tbk2_Par;

				//�ڶ���ƫ����һ����
				double D0x0_Par = 0.5*(sqrt(D2 / D1)*D1x0_Par + sqrt(D1 / D2)*D2x0_Par);
				double D0y0_Par = 0.5*(sqrt(D2 / D1)*D1y0_Par + sqrt(D1 / D2)*D2y0_Par);
				double D0f_Par = 0.5*(sqrt(D2 / D1)*D1f_Par + sqrt(D1 / D2)*D2f_Par);
				double D0k1_Par = 0.5*(sqrt(D2 / D1)*D1k1_Par + sqrt(D1 / D2)*D2k1_Par);
				double D0k2_Par = 0.5*(sqrt(D2 / D1)*D1k2_Par + sqrt(D1 / D2)*D2k2_Par);

				//�ڶ���ƫ���ڶ�����
				double Nx0_Par = 2 * Ta*Tb*Xest[0] - Ta*Tb*Xa - Ta*Tb*Xb;
				double Ny0_Par = 2 * Ta*Tb*Xest[1] - Ta*Tb*Ya - Ta*Tb*Yb;
				double Nf_Par = 2 * Xest[2];
				double Nk1_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk1_Par + Tb*Tak1_Par);
				double Nk2_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk2_Par + Tb*Tak2_Par);

				//������ƫ��
				double G[5];
				G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
				G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
				G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
				G[3] = (Nk1_Par*D0 - D0k1_Par*N) / D0 / D0;
				G[4] = (Nk2_Par*D0 - D0k2_Par*N) / D0 / D0;

				//��ʵ�Ǿ�
				double  VTV;
				mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV, 1, 3, 1);
				double LL = abs(acos(VTV) - acos(N / D0));// PI * 180 * 3600;
				ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
				rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
				double Threashold = atan(1e-8*(pow(sqrt(ra2), 3) + pow(sqrt(rb2), 3)) / Xest[2]) + atan(0.5 / Xest[2]);
				if (LL < Threashold)
				{
					L = VTV - N / D0;
					mBase.pNormal(G, 5, L, ATA, ATL, 1.0);
				}
			}

		}
		//�������
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-7&&abs(ATL[1] - iter_y0) < 1e-7&&abs(ATL[2] - iter_f) < 1e-9
			&&abs(ATL[3] - iter_k1) < 1e-15 &&abs(ATL[4] - iter_k2) < 1e-20 || iter_count > 50)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		Xest[3] += ATL[3]; Xest[4] += ATL[4];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
		iter_k1 = abs(ATL[3]), iter_k2 = abs(ATL[4]);
	}
	//�����������
	ZY302CaliParam.x0 = Xest[0];
	ZY302CaliParam.y0 = Xest[1];
	ZY302CaliParam.f = Xest[2];
	ZY302CaliParam.k1 = Xest[3];
	ZY302CaliParam.k2 = Xest[4];
	//��������
	//double Xoutput[7];
	//memcpy(Xoutput, Xest, sizeof(Xest));//sizeof(Xest)��ʾ�ľ�������Xest�ˣ�����Ҫ�ٳ���5
	//CaliAccuracy5Param(getGCP, Xest, Xoutput[5]);
	//Xest[0] = 512;	Xest[1] = 512;	Xest[2] = 43.3 / 0.015; Xest[3] = 0; Xest[4] = 0;
	//CaliAccuracy5Param(getGCP, Xest, Xoutput[6]);
	//���
	char output[512];
	sprintf_s(output, "�Ǿ�5���������%d��", index);
	OutputFile(Xest, 5, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����������꺯����������С���˵������
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�6������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.01.10
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate6Param(vector<StarGCP> getGCP, int index)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	//δ֪��������
	const int Param = 6;
	//����XestΪ�����Ƶ�6��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0, iter_count = 0;
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

				//����ģ��
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

				//��һ��ƫ��
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

				//�ڶ���ƫ����һ����
				double D0x0_Par = 0.5*(sqrt(D2 / D1)*D1x0_Par + sqrt(D1 / D2)*D2x0_Par);
				double D0y0_Par = 0.5*(sqrt(D2 / D1)*D1y0_Par + sqrt(D1 / D2)*D2y0_Par);
				double D0f_Par = 0.5*(sqrt(D2 / D1)*D1f_Par + sqrt(D1 / D2)*D2f_Par);
				double D0k1_Par = 0.5*(sqrt(D2 / D1)*D1k1_Par + sqrt(D1 / D2)*D2k1_Par);
				double D0p1_Par = 0.5*(sqrt(D2 / D1)*D1p1_Par + sqrt(D1 / D2)*D2p1_Par);
				double D0p2_Par = 0.5*(sqrt(D2 / D1)*D1p2_Par + sqrt(D1 / D2)*D2p2_Par);

				//�ڶ���ƫ���ڶ�����
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

				//������ƫ��
				double G[6];
				G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
				G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
				G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
				G[3] = (Nk1_Par*D0 - D0k1_Par*N) / D0 / D0;
				G[4] = (Np1_Par*D0 - D0p1_Par*N) / D0 / D0;
				G[5] = (Np2_Par*D0 - D0p2_Par*N) / D0 / D0;

				//��ʵ�Ǿ�
				double  VTV;
				mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV, 1, 3, 1);
				L = VTV - N / D0;
				mBase.pNormal(G, 6, L, ATA, ATL, 1.0);
			}
		}
		//�������
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
	//��������
	//CaliAccuracy(getGCP,Xest,6);
	//ԭʼ��������
	printf("δ���꾫��Ϊ:\n");
	memset(Xest, 0, sizeof(double) * 6);
	Xest[2] = 43.3 / 0.015;
	//CaliAccuracy(getGCP, Xest, 6);
	//���
	char output[512];
	sprintf_s(output, "�Ǿ�6���������%d��", index);
	OutputFile(Xest, 6, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����������꺯����������С���˵������
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�6������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.02.23
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate6ParamMultiImg(vector<vector<StarGCP>>getGCP, int index)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	//δ֪��������
	const int Param = 6;
	//����XestΪ�����Ƶ�6��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0, iter_count = 0;
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	while (true)
	{
		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
		for (size_t i = 0; i < getGCP.size(); i++)
		{
			for (int a = 0; a < getGCP[i].size() - 1; a++)
			{
				for (int b = a + 1; b < getGCP[i].size(); b++)
				{
					Xa = 1024 - getGCP[i][a].y - 512;		Ya = getGCP[i][a].x - 512;
					Xb = 1024 - getGCP[i][b].y - 512;		Yb = getGCP[i][b].x - 512;

					//����ģ��
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

					//��һ��ƫ��
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

					//�ڶ���ƫ����һ����
					double D0x0_Par = 0.5*(sqrt(D2 / D1)*D1x0_Par + sqrt(D1 / D2)*D2x0_Par);
					double D0y0_Par = 0.5*(sqrt(D2 / D1)*D1y0_Par + sqrt(D1 / D2)*D2y0_Par);
					double D0f_Par = 0.5*(sqrt(D2 / D1)*D1f_Par + sqrt(D1 / D2)*D2f_Par);
					double D0k1_Par = 0.5*(sqrt(D2 / D1)*D1k1_Par + sqrt(D1 / D2)*D2k1_Par);
					double D0p1_Par = 0.5*(sqrt(D2 / D1)*D1p1_Par + sqrt(D1 / D2)*D2p1_Par);
					double D0p2_Par = 0.5*(sqrt(D2 / D1)*D1p2_Par + sqrt(D1 / D2)*D2p2_Par);

					//�ڶ���ƫ���ڶ�����
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

					//������ƫ��
					double G[6];
					G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
					G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
					G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
					G[3] = (Nk1_Par*D0 - D0k1_Par*N) / D0 / D0;
					G[4] = (Np1_Par*D0 - D0p1_Par*N) / D0 / D0;
					G[5] = (Np2_Par*D0 - D0p2_Par*N) / D0 / D0;

					//��ʵ�Ǿ�
					double  VTV;
					mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
					L = VTV - N / D0;
					mBase.pNormal(G, 6, L, ATA, ATL, 1.0);
				}
			}
		}
		//�������
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
			&&abs(ATL[3] - iter_k1) < 1e-10 &&abs(ATL[4] - iter_p1) < 1e-10 &&abs(ATL[5] - iter_p2) < 1e-10
			|| iter_count > 50)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		Xest[3] += ATL[3]; Xest[4] += ATL[4]; Xest[5] += ATL[5];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
		iter_k1 = abs(ATL[3]), iter_p1 = abs(ATL[4]), iter_p2 = abs(ATL[5]);
	}
	//��������
	//CaliAccuracy(getGCP,Xest,6);
	//ԭʼ��������
	/*printf("δ���꾫��Ϊ:\n");
	memset(Xest, 0, sizeof(double) * 6);
	Xest[2] = 43.3 / 0.015;*/
	//CaliAccuracy(getGCP, Xest, 6);
	//���
	char output[512];
	sprintf_s(output, "�Ǿ�6���������%d��", index);
	OutputFile(Xest, 6, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����������꺯����������С���˵������
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�5������
//ע�⣺�޸���Xa��Ya�ĳ�ʼֵ
//���ߣ�GZC
//���ڣ�2017.03.02
//////////////////////////////////////////////////////////////////////////
void APScalibration::Calibrate5ParamMultiImg(vector<vector<StarGCP>>getGCP, int index)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	//δ֪��������
	const int Param = 5;
	//����XestΪ�����Ƶ�5��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  
	//Detx0	Dety0	 f			  k1		   k1       
	double Xest[Param];
	memset(Xest, 0, sizeof(double) * Param);
	Xest[0] = 512;
	Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_k2 = 0, iter_count = 0;
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	while (true)
	{
		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
		for (size_t i = 0; i < getGCP.size(); i++)
		{
			for (int a = 0; a < getGCP[i].size() - 1; a++)
			{
				for (int b = a + 1; b < getGCP[i].size(); b++)
				{
					//��ʵ����ʹ��
					Xa = 1024 - getGCP[i][a].y;		Ya = getGCP[i][a].x;
					Xb = 1024 - getGCP[i][b].y;		Yb = getGCP[i][b].x;
					//�����ʱ����
					/*Xa = getGCP[i][a].x;		Ya = getGCP[i][a].y;
					Xb = getGCP[i][b].x;		Yb = getGCP[i][b].y;*/

					//ģ��
					double ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
					double rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
					double Ta = 1 - Xest[3] * ra2 - Xest[4] * ra2*ra2;
					double Tb = 1 - Xest[3] * rb2 - Xest[4] * rb2*rb2;
					double N = (Xa - Xest[0])*(Xb - Xest[0])*Ta*Tb + (Ya - Xest[1])*(Yb - Xest[1])*Ta*Tb + Xest[2] * Xest[2];
					double D1 = (Xa - Xest[0])*(Xa - Xest[0])*Ta*Ta + (Ya - Xest[1])*(Ya - Xest[1])*Ta*Ta + Xest[2] * Xest[2];
					double D2 = (Xb - Xest[0])*(Xb - Xest[0])*Tb*Tb + (Yb - Xest[1])*(Yb - Xest[1])*Tb*Tb + Xest[2] * Xest[2];
					double D0 = sqrt(D1*D2);

					//��һ��ƫ��
					double Tak1_Par = -ra2;
					double Tbk1_Par = -rb2;
					double Tak2_Par = -ra2*ra2;
					double Tbk2_Par = -rb2*rb2;

					double D1x0_Par = -2 * Ta*Ta * (Xa - Xest[0]);
					double D1y0_Par = -2 * Ta*Ta* (Ya - Xest[1]);
					double D1f_Par = 2 * Xest[2];
					double D1k1_Par = 2 * Ta*ra2*Tak1_Par;
					double D1k2_Par = 2 * Ta*ra2*Tak2_Par;

					double D2x0_Par = -2 * Tb*Tb * (Xb - Xest[0]);
					double D2y0_Par = -2 * Tb*Tb* (Yb - Xest[1]);
					double D2f_Par = 2 * Xest[2];
					double D2k1_Par = 2 * Tb*rb2*Tbk1_Par;
					double D2k2_Par = 2 * Tb*rb2*Tbk2_Par;

					//�ڶ���ƫ����һ����
					double D0x0_Par = 0.5*(sqrt(D2 / D1)*D1x0_Par + sqrt(D1 / D2)*D2x0_Par);
					double D0y0_Par = 0.5*(sqrt(D2 / D1)*D1y0_Par + sqrt(D1 / D2)*D2y0_Par);
					double D0f_Par = 0.5*(sqrt(D2 / D1)*D1f_Par + sqrt(D1 / D2)*D2f_Par);
					double D0k1_Par = 0.5*(sqrt(D2 / D1)*D1k1_Par + sqrt(D1 / D2)*D2k1_Par);
					double D0k2_Par = 0.5*(sqrt(D2 / D1)*D1k2_Par + sqrt(D1 / D2)*D2k2_Par);

					//�ڶ���ƫ���ڶ�����
					double Nx0_Par = 2 * Ta*Tb*Xest[0] - Ta*Tb*Xa - Ta*Tb*Xb;
					double Ny0_Par = 2 * Ta*Tb*Xest[1] - Ta*Tb*Ya - Ta*Tb*Yb;
					double Nf_Par = 2 * Xest[2];
					double Nk1_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk1_Par + Tb*Tak1_Par);
					double Nk2_Par = ((Xa - Xest[0])*(Xb - Xest[0]) + (Ya - Xest[1])*(Yb - Xest[1]))*(Ta*Tbk2_Par + Tb*Tak2_Par);

					//������ƫ��
					double G[5];
					G[0] = (Nx0_Par*D0 - D0x0_Par*N) / D0 / D0;
					G[1] = (Ny0_Par*D0 - D0y0_Par*N) / D0 / D0;
					G[2] = (Nf_Par*D0 - D0f_Par*N) / D0 / D0;
					G[3] = (Nk1_Par*D0 - D0k1_Par*N) / D0 / D0;
					G[4] = (Nk2_Par*D0 - D0k2_Par*N) / D0 / D0;

					//��ʵ�Ǿ�
					double  VTV;
					mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
					double LL = abs(acos(VTV) - acos(N / D0));// PI * 180 * 3600;
					ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
					rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
					double Threashold = atan(1e-8*(pow(sqrt(ra2), 3) + pow(sqrt(rb2), 3)) / Xest[2]) + atan(0.5 / Xest[2]);
					if (LL < Threashold)
					{
						L = VTV - N / D0;
						mBase.pNormal(G, 5, L, ATA, ATL, 1.0);
					}
				}
			}
		}
		//�������
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-7&&abs(ATL[1] - iter_y0) < 1e-7&&abs(ATL[2] - iter_f) < 1e-9
			&&abs(ATL[3] - iter_k1) < 1e-15 &&abs(ATL[4] - iter_k2) < 1e-20 || iter_count > 50)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		Xest[3] += ATL[3]; Xest[4] += ATL[4];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
		iter_k1 = abs(ATL[3]), iter_k2 = abs(ATL[4]);
	}
	//�����������
	ZY302CaliParam.x0 = Xest[0];
	ZY302CaliParam.y0 = Xest[1];
	ZY302CaliParam.f = Xest[2];
	ZY302CaliParam.k1 = Xest[3];
	ZY302CaliParam.k2 = Xest[4];
	//��������
	double Xoutput[7];
	memcpy(Xoutput, Xest, sizeof(Xest));//sizeof(Xest)��ʾ�ľ�������Xest�ˣ�����Ҫ�ٳ���5
	CaliAccuracy5Param(getGCP, Xest, Xoutput[5]);
	Xest[0] = 512;	Xest[1] = 512;	Xest[2] = 43.3 / 0.015; Xest[3] = 0; Xest[4] = 0;
	CaliAccuracy5Param(getGCP, Xest, Xoutput[6]);
	//���
	char output[512];
	sprintf_s(output, "�Ǿ�5���������%d��", index);
	OutputFile(Xoutput, 7, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��õ�����󾫶�
//���룺getGCP�����Ƶ㣬X�����ƵĲ�����num������
//��������������������λ����
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.01.11
//////////////////////////////////////////////////////////////////////////
void APScalibration::CaliAccuracy(vector<StarGCP> getGCP, double *X, int num)
{
	double LL = 0;
	double Xest[6];
	if (num == 3)
	{
		Xest[0] = X[0]; Xest[1] = X[1]; Xest[2] = X[2];
		Xest[3] = 0; Xest[4] = 0; Xest[5] = 0;
	}
	else if (num == 6)
	{
		memcpy(Xest, X, sizeof(double) * 6);
	}
	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	for (int a = 0; a < num - 1; a++)
	{
		for (int b = a + 1; b < num; b++)
		{
			//��ֵ
			Xa = (getGCP[a].x - 512);		Ya = (getGCP[a].y - 512);
			Xb = (getGCP[b].x - 512);	Yb = (getGCP[b].y - 512);

			//����ģ��
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
			//��ʵ�Ǿ�
			double  VTV;
			mBase.Multi(getGCP[a].V, getGCP[b].V, &VTV, 1, 3, 1);
			if (abs(acos(VTV) - acos(N / D0)) < 1)
			{
				LL += (acos(VTV) - acos(N / D0))*(acos(VTV) - acos(N / D0));
			}
		}
	}
	double RMS = sqrt(LL * 2 / num / (num + 1)) / PI * 180 * 3600;
	printf("%d��������Ϊ%.9f����\n", num, RMS);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��õ�����󾫶�
//���룺getGCP�����Ƶ㣬X�����ƵĲ���
//��������������������λ����
//ע�⣺Kalman�˲���������������
//���ߣ�GZC
//���ڣ�2017.04.27
//////////////////////////////////////////////////////////////////////////
void APScalibration::CaliAccuracy3Param(vector<vector<StarGCP>> getGCP, double *X, double &RMS)
{
	string path;
	if (X[0] != 512)
	{
		path = workpath + "���Ƶ�\\����-�����.txt";
	}
	else
	{
		path = workpath + "���Ƶ�\\����-����ǰ.txt";
	}
	FILE *fp = fopen(path.c_str(), "w");

	double LL = 0, num = 0;
	double Xest[5];
	memcpy(Xest, X, sizeof(double)*3);
	Xest[3] = 0; Xest[4] = 0;


	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	for (size_t i = 0; i < getGCP.size(); i++)
	{
		for (int a = 0; a < getGCP[i].size() - 1; a++)
		{
			for (int b = a + 1; b < getGCP[i].size(); b++)
			{
				//��ʵ����ʹ��
				Xa = 1024 - getGCP[i][a].y;		Ya = getGCP[i][a].x;
				Xb = 1024 - getGCP[i][b].y;		Yb = getGCP[i][b].x;

				//ģ��
				double ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
				double rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
				double Ta = 1 - Xest[3] * ra2 - Xest[4] * ra2*ra2;
				double Tb = 1 - Xest[3] * rb2 - Xest[4] * rb2*rb2;
				double N = (Xa - Xest[0])*(Xb - Xest[0])*Ta*Tb + (Ya - Xest[1])*(Yb - Xest[1])*Ta*Tb + Xest[2] * Xest[2];
				double D1 = (Xa - Xest[0])*(Xa - Xest[0])*Ta*Ta + (Ya - Xest[1])*(Ya - Xest[1])*Ta*Ta + Xest[2] * Xest[2];
				double D2 = (Xb - Xest[0])*(Xb - Xest[0])*Tb*Tb + (Yb - Xest[1])*(Yb - Xest[1])*Tb*Tb + Xest[2] * Xest[2];
				double D0 = sqrt(D1*D2);

				//��ʵ�Ǿ�
				double  VTV;
				mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
				if (abs(acos(VTV) - acos(N / D0)) < 1)
				{
					LL += (acos(VTV) - acos(N / D0))*(acos(VTV) - acos(N / D0));
					double tmp = (acos(VTV) - acos(N / D0)) / PI * 180 * 3600;
					fprintf(fp, "%.9f\n", tmp);
					num++;
				}
			}
		}
	}
	RMS = sqrt(LL / num) / PI * 180 * 3600;
	fclose(fp);
	//printf("%d��������Ϊ%.9f����\n", num, RMS);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��õ�����󾫶�
//���룺getGCP�����Ƶ㣬X�����ƵĲ���
//��������������������λ����
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.03.03
//////////////////////////////////////////////////////////////////////////
void APScalibration::CaliAccuracy5Param(vector<vector<StarGCP>> getGCP, double *X, double &RMS)
{
	string path;
	if (X[4] != 0)
	{
		path = workpath + "����-�����.txt";
	}
	else
	{
		path = workpath + "����-����ǰ.txt";
	}
	FILE *fp = fopen(path.c_str(), "w");

	double LL = 0, num = 0;
	double Xest[5];
	memcpy(Xest, X, sizeof(double) * 5);


	double Xa, Ya, Xb, Yb, DetXa, DetYa, DetXb, DetYb;
	for (size_t i = 0; i < getGCP.size(); i++)
	{
		for (int a = 0; a < getGCP[i].size() - 1; a++)
		{
			for (int b = a + 1; b < getGCP[i].size(); b++)
			{
				//��ʵ����ʹ��
				Xa = 1024 - getGCP[i][a].y;		Ya = getGCP[i][a].x;
				Xb = 1024 - getGCP[i][b].y;		Yb = getGCP[i][b].x;

				//ģ��
				double ra2 = (Xa - Xest[0]) * (Xa - Xest[0]) + (Ya - Xest[1]) * (Ya - Xest[1]);
				double rb2 = (Xb - Xest[0]) * (Xb - Xest[0]) + (Yb - Xest[1]) * (Yb - Xest[1]);
				double Ta = 1 - Xest[3] * ra2 - Xest[4] * ra2*ra2;
				double Tb = 1 - Xest[3] * rb2 - Xest[4] * rb2*rb2;
				double N = (Xa - Xest[0])*(Xb - Xest[0])*Ta*Tb + (Ya - Xest[1])*(Yb - Xest[1])*Ta*Tb + Xest[2] * Xest[2];
				double D1 = (Xa - Xest[0])*(Xa - Xest[0])*Ta*Ta + (Ya - Xest[1])*(Ya - Xest[1])*Ta*Ta + Xest[2] * Xest[2];
				double D2 = (Xb - Xest[0])*(Xb - Xest[0])*Tb*Tb + (Yb - Xest[1])*(Yb - Xest[1])*Tb*Tb + Xest[2] * Xest[2];
				double D0 = sqrt(D1*D2);

				//��ʵ�Ǿ�
				double  VTV;
				mBase.Multi(getGCP[i][a].V, getGCP[i][b].V, &VTV, 1, 3, 1);
				if (abs(acos(VTV) - acos(N / D0)) < 1)
				{
					LL += (acos(VTV) - acos(N / D0))*(acos(VTV) - acos(N / D0));
					double tmp = (acos(VTV) - acos(N / D0)) / PI * 180 * 3600;
					fprintf(fp, "%.9f\n", tmp);
					num++;
				}
			}
		}
	}
	RMS = sqrt(LL / num) / PI * 180 * 3600;
	fclose(fp);
	//printf("%d��������Ϊ%.9f����\n", num, RMS);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�ȥ����С�Ǿ���ǵ㣬�õ��Ż���Ŀ��Ƶ�
//���룺getGCP�����Ƶ㣻pixel��С��pixel�����ص��ǵ㲻Ҫ
//�����getGCP���Ż���Ŀ��Ƶ�
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ�󣻸��º�����ȥ������ͬ�������������ѡ�����
//���ߣ�GZC
//���ڣ�2017.01.12�����£�2017.02.17
//////////////////////////////////////////////////////////////////////////
void APScalibration::OptimizeGCP(vector<StarGCP> &getGCP, int pixel, int index)
{
	int num = getGCP.size();
	//��ɾ������ͬxy�ĵ㣬��Ϊ��֪���ĸ�׼ȷ�������㶼ɾ��
	vector<StarGCP>getGCPdelSame(getGCP);
	int label = 0;
	for (vector<StarGCP>::iterator a = getGCPdelSame.begin(); a <= getGCPdelSame.end() - 1; )
	{
		for (vector<StarGCP>::iterator b = a + 1; b != getGCPdelSame.end();)
		{
			if ((*a).x == (*b).x && (*a).y == (*b).y)
			{
				b = getGCPdelSame.erase(b);
				label = 1;
			}
			else
			{
				++b;
			}
		}
		if (label == 1)
		{
			a = getGCPdelSame.erase(a);
			label = 0;
		}
		else
		{
			++a;
		}
	}

	//���ݸ��������ؾ���ȥ������̫���ĵ�
	vector<StarGCP>getGCPchoose(getGCPdelSame);
	num = getGCPdelSame.size();
	int *GCPa = new int[num];
	memset(GCPa, 0, num * sizeof(int));
	getGCP.clear();
	while (true)
	{
		int jumpout = 0;
		for (int a = 0; a < num - 1; a++)
		{
			for (int b = a + 1; b < num; b++)
			{
				//��ʵ�Ǿ�
				double  VTV1;
				mBase.Multi(getGCPchoose[a].V, getGCPchoose[b].V, &VTV1, 1, 3, 1);
				if (VTV1 > cos(atan(pixel * 0.015 / 43.3)))//�����ǵ�С������ǶȲ�Ҫ�������������abs(det) > 0.00002 || 
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

	char GCPpathtmp[100];
	sprintf_s(GCPpathtmp, "\\���Ƶ�\\����GCP%d.pts", index);
	string GCPpath = workpath + GCPpathtmp;
	FILE *fp2 = fopen(GCPpath.c_str(), "w");
	for (int j = 0; j < getGCP.size(); j++)
	{
		fprintf(fp2, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", getGCP[j].x, getGCP[j].y,
			getGCP[j].V[0], getGCP[j].V[1], getGCP[j].V[2]);
		/*fprintf(fp2, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", 1024 - getGCP[j].y, getGCP[j].x,
			getGCP[j].V[0], getGCP[j].V[1], getGCP[j].V[2]);*/
	}
	fclose(fp2);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����ENVI��ʽ������Ƶ�
//���룺getGCP�����Ƶ㣻indexʱ��ָʾ
//��������Ƶ��ļ�.pts
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ��
//���ߣ�GZC
//���ڣ�2017.02.18
//////////////////////////////////////////////////////////////////////////
void APScalibration::OutputGCP(vector<StarGCP> &getGCP, int index)
{
	//������Ƶ�
	int num = getGCP.size();
	char GCPpathtmp[100];
	sprintf_s(GCPpathtmp, "���Ƶ��Ż�\\GCP(%d).pts", index);
	string GCPpath = workpath + GCPpathtmp;
	FILE *fp = fopen(GCPpath.c_str(), "w");
	string str1 = "; ENVI Image to Image GCP File";
	string str2 = "; base file : D:\\2_ImageData\\ZY3 - 02\\��ͼ����\\0830\\��ͼ\\";
	sprintf_s(GCPpathtmp, "��ͼ (%d).tiff", index);
	str2 = str2 + GCPpathtmp;
	string str3 = "; warp file : D:\\2_ImageData\\ZY3 - 02\\��ͼ����\\0830\\��ͼ\\";
	sprintf_s(GCPpathtmp, "�ǵ���ȡ��� (%d).tiff", index);
	str3 = str3 + GCPpathtmp;
	string str4 = "; Base Image(x, y), Warp Image(x, y)";
	fprintf(fp, "%s\n%s\n%s\n%s\n", str1.c_str(), str2.c_str(), str3.c_str(), str4.c_str());

	for (int k = 0; k < num; k++)
	{
		fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", getGCP[k].x, getGCP[k].y,
			getGCP[k].V[0], getGCP[k].V[1], getGCP[k].V[2]);
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��洢���еĿ��Ƶ�
//���룺getGCP�����еĿ��Ƶ�
//�����Allgcp.txt���ڿ��Ƶ�Ŀ¼��
//ע�⣺������ķ�ʽ�洢���Ƶ�
//���ߣ�GZC
//���ڣ�2017.03.16
//////////////////////////////////////////////////////////////////////////
void APScalibration::OutputAllGCP(vector<vector<StarGCP>> getGCP)
{
	string GCPpath = workpath + "���Ƶ�\\Allgcp.txt";
	FILE *fp = fopen(GCPpath.c_str(), "w");
	string STItime = workpath.substr(0, workpath.rfind('\\'));
	STItime = STItime.substr(0, STItime.rfind('\\')) + "\\��ʱ.txt";
	FILE *fp2 = fopen(STItime.c_str(), "r");
	int num;
	fscanf(fp2, "%d\n", &num);
	double *time = new double[num];
	for (int i = 0; i < num; i++)
	{
		fscanf(fp2, "%*d\t%lf\n", &time[i]);
	}
	for (int a = 0; a < getGCP.size(); a++)
	{
		fprintf(fp, "%d\t%.9f\n", getGCP[a].size(), time[a + 4]);
		for (int b = 0; b < getGCP[a].size(); b++)
		{
			fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", getGCP[a][b].x, getGCP[a][b].y,
				getGCP[a][b].V[0], getGCP[a][b].V[1], getGCP[a][b].V[2]);
		}
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡ���еĿ��Ƶ�
//���룺Allgcp.txt���ڿ��Ƶ�Ŀ¼��
//�����getGCP�����еĿ��Ƶ�
//ע�⣺12.04��workpathΪ����·��
//���ߣ�GZC
//���ڣ�2017.03.16
//////////////////////////////////////////////////////////////////////////
void APScalibration::ReadAllGCP(vector<vector<StarGCP>> &getGCP)
{
	//string GCPpath = workpath + "���Ƶ�\\Allgcp.txt";
	FILE *fp = fopen(workpath.c_str(), "r");
	if (fp == NULL) perror("Error opening file");
	int num;
	vector<StarGCP > getGCPtmp;
	StarGCP getGCPtmptmp;
	while (!feof(fp))
	{
		double UTC;
		fscanf(fp, "%d\t%lf\n", &num, &UTC);
		for (int b = 0; b < num; b++)
		{
			getGCPtmptmp.UTC = UTC-0.1;
			fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", &getGCPtmptmp.x, &getGCPtmptmp.y,
				&getGCPtmptmp.V[0], &getGCPtmptmp.V[1], &getGCPtmptmp.V[2]);
			getGCPtmp.push_back(getGCPtmptmp);
		}
		getGCP.push_back(getGCPtmp);
		getGCPtmp.clear();
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�������Ƶ�в�
//���룺getGCP����ά���Ƶ�����
//��������Ƶ��ļ�.txt
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ��
//���ߣ�GZC
//���ڣ�2017.03.07
//////////////////////////////////////////////////////////////////////////
void APScalibration::OutputErr(vector<vector<StarGCP>> getGCP, vector<STGData>ZY302STG, int index)
{
	//������Ƶ�
	char GCPpathtmp[100];
	sprintf_s(GCPpathtmp, "�в�\\�ۻ�(%d)��.txt", 1000 - index);
	string GCPpath = workpath + GCPpathtmp;
	FILE *fp = fopen(GCPpath.c_str(), "w");

	double R[9], xr[2], yr[2], V[3], W[3];
	index = 2 * index - 5;//0702��
	mBase.quat2matrix(ZY302STG[index].StarA.Q1, ZY302STG[index].StarA.Q2,
		ZY302STG[index].StarA.Q3, ZY302STG[index].StarA.Q0, R);//Crj
	for (int a = 0; a < getGCP.size(); a++)
	{
		for (int b = 0; b < getGCP[a].size(); b++)
		{
			V[0] = getGCP[a][b].V[0];
			V[1] = getGCP[a][b].V[1];
			V[2] = getGCP[a][b].V[2];
			mBase.Multi(R, V, W, 3, 3, 1);
			mParse.FromLL2XY(W, ZY302CaliParam, xr, yr);
			fprintf(fp, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", getGCP[a][b].x, getGCP[a][b].y, xr[0] - getGCP[a][b].x,
				yr[0] - getGCP[a][b].y, xr[1] - getGCP[a][b].x, yr[1] - getGCP[a][b].y);
		}
	}

	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ݿ��Ƶ�����һ�����Ŀ��Ƶ�
//���룺getGCP�����Ƶ㣻ZY3_02STGdata���˿�����������̬��indexʱ��ָʾ
//�����getGCP��������Ĵ������Ƶ�
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ��
//���ߣ�GZC
//���ڣ�2017.01.18
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP(vector<StarGCP> &getGCP, vector<STGData> ZY3_02STGdata, int index)
{
	int num = getGCP.size();
	Star getGCPsim;
	double V[3], alpha, sigma, R[9], x, y, W[3];
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
		//x0=y0=512,f=43.3mm,��Ԫ��С0.015mm
		if (W[2] > 0)
		{
			x = (512 - W[0] / W[2] * 43.3 / 0.015);
			y = (512 - W[1] / W[2] * 43.3 / 0.015);
		}
		else
		{
			x = -513, y = -513;
		}//����������ж��Ƿ������ָ��İ�����һ��
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
//���ܣ����ݿ��Ƶ�����һ�����Ŀ��Ƶ�
//���룺getGCP�����Ƶ㣻ZY3_02STGdata���˿�����������̬��indexʱ��ָʾ
//�����getGCP��������Ĵ������Ƶ�
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ��
//���ߣ�GZC
//���ڣ�2017.02.16
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP_PreRand(vector<vector<StarGCP>> &getGCP,
	vector<STGData> ZY3_02STGdata, int *gcpNum, int index)
{
	int gcpNumAll = 0;
	for (int i = 0; i < index; i++)
	{
		gcpNumAll = gcpNumAll + gcpNum[i];
	}
	double *randx = new double[gcpNumAll];
	double *randy = new double[gcpNumAll];
	//����x��y��������������Ҽ�����ʵ������
	mBase.RandomDistribution(1, 0.05, gcpNumAll, 0, randx);
	mBase.RandomDistribution(3, 0.05, gcpNumAll, 0, randy);
	double V[3], alpha, sigma, R[9], x, y, W[3];
	int pNum = 0;
	for (int b = 0; b < index - 1; b++)
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
			//x0=y0=512,f=43.3mm,��Ԫ��С0.015mm
			if (W[2] > 0)
			{
				x = (512 - W[0] / W[2] * 43.3 / 0.015);
				y = (512 - W[1] / W[2] * 43.3 / 0.015);
			}
			else
			{
				x = -513, y = -513;
			}//����������ж��Ƿ������ָ��İ�����һ��
			getGCP[b][a].x = x + randx[a + pNum];
			getGCP[b][a].y = y + randy[a + pNum];
		}
		pNum = pNum + gcpNum[b];
	}

}

//////////////////////////////////////////////////////////////////////////
//���ܣ��������һϵ���ǵ���Ƶ�
//���룺index����ͼ����
//�����getGCP��������Ŀ��Ƶ�
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ��
//���ߣ�GZC
//���ڣ�2017.02.28
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP_RandomXY(int index, vector<vector<StarGCP>>& getGCP)
{
	//���ȹ���50������������x,y
	int num = 50;
	double V[3], x, y;
	double *xy = new double[num*index * 2];
	mBase.AverageRand(0, 1024, num*index * 2, xy);
	vector<StarGCP> getGCPtmp(50);
	//������
	int gcpNumAll = num*index;
	double *randx = new double[gcpNumAll];
	double *randy = new double[gcpNumAll];
	double *k1 = new double[gcpNumAll];
	double *p1 = new double[gcpNumAll];
	double *p2 = new double[gcpNumAll];
	//����x��y��������������Ҽ�����ʵ������
	mBase.RandomDistribution(1, 0.2, gcpNumAll, 0, randx);
	mBase.RandomDistribution(3, 0.2, gcpNumAll, 0, randy);
	mBase.RandomDistribution(5, 0.02, gcpNumAll, 0, k1);
	mBase.RandomDistribution(5, 0.02, gcpNumAll, 0, p1);
	mBase.RandomDistribution(5, 0.02, gcpNumAll, 0, p2);
	StarCaliParam ZY302;
	ZY302.f = 43.3 / 0.015;
	ZY302.x0 = 512 + 1;
	ZY302.y0 = 512 + 3;
	ZY302.k1 = 5e-9;
	ZY302.k2 = 5e-14;
	for (int a = 0; a < index; a++)
	{
		for (int b = 0; b < num; b++)
		{
			x = xy[a*num + b];
			y = xy[num*index + a*num + b];
			//����x,y����������õ��ǵ�ָ��V[3]
			mParse.FromXY2LL(x, y, ZY302, V);
			getGCPtmp[b].V[0] = V[0]; getGCPtmp[b].V[1] = V[1]; getGCPtmp[b].V[2] = V[2];
			//���û���ģ�����
			double r2 = x*x + y*y;
			/*getGCPtmp[b].x = x + randx[a + num];
			getGCPtmp[b].y = y + randy[a + num];*/
			getGCPtmp[b].x = x + randx[a*num + b] + k1[a*num + b] * 1e-11*x*r2
				+ p1[a*num + b] * 1e-8*(3 * x*x + y*y) + 2 * p2[a*num + b] * 1e-8*x*y;
			getGCPtmp[b].y = y + randy[a*num + b] + k1[a*num + b] * 1e-11*y*r2
				+ p2[a*num + b] * 1e-8*(3 * y*y + x*x) + 2 * p1[a*num + b] * 1e-8*x*y;
			/*getGCPtmp[b].x = x + randx[a + num] + 1e-12*x*r2 + 1e-9*(3 * x*x + y*y) + 4 * 1e-9*x*y;
			getGCPtmp[b].y = y + randy[a + num] + 1e-12*y*r2 + 2e-9*(3 * y*y + x*x) + 2 * 1e-9*x*y;*/
		}
		getGCP.push_back(getGCPtmp);
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��������һϵ���ǵ���Ƶ�
//���룺index����ͼ����
//�����getGCP��������Ŀ��Ƶ�
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ��
//���ߣ�GZC
//���ڣ�2017.02.28
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP_RandomXY5Param(int index, vector<vector<StarGCP>>& getGCP)
{
	//���ȹ���50������������x,y
	int num = 50;
	double V[3], x, y;
	double *xy = new double[num*index * 2];
	mBase.AverageRand(0, 1024, num*index * 2, xy);
	vector<StarGCP> getGCPtmp(50);
	//������
	int gcpNumAll = num*index;
	double *randxy = new double[gcpNumAll * 2];
	//����x��y��������������Ҽ�����ʵ������
	mBase.RandomDistribution(0, 0.3, gcpNumAll * 2, 0, randxy);

	double x0 = ZY302CaliParam.x0;
	double y0 = ZY302CaliParam.y0;
	double f = ZY302CaliParam.f;
	for (int a = 0; a < index; a++)
	{
		for (int b = 0; b < num; b++)
		{
			x = xy[a*num + b];
			y = xy[num*index + a*num + b];
			//����������ʱ���ڱ�
			//ZY302CaliParam.x0 = x0 +  1e-2*a;
			//ZY302CaliParam.y0 = y0 -  2e-2*a;
			ZY302CaliParam.f = f -  2e-2*a;
			//����x,y����������õ��ǵ�ָ��V[3]
			mParse.FromXY2LL(x, y, ZY302CaliParam, V);
			getGCPtmp[b].V[0] = V[0]; getGCPtmp[b].V[1] = V[1]; getGCPtmp[b].V[2] = V[2];
			//���������ȡ���
			getGCPtmp[b].x = x + randxy[a*num + b];
			getGCPtmp[b].y = y + randxy[gcpNumAll + a*num + b];
		}
		getGCP.push_back(getGCPtmp);
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ݿ��Ƶ�����һ�����Ŀ��Ƶ㣬����������������
//���룺getGCP�����Ƶ㣻ZY3_02STGdata���˿�����������̬��indexʱ��ָʾ
//�����getGCP��������Ĵ������Ƶ�
//ע�⣺�ǵ�洢��ʽ����StarGCP�ṹ���Ҫ��
//���ߣ�GZC
//���ڣ�2017.02.28
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP_PreRand6Param(vector<vector<StarGCP>>& getGCP, vector<STGData> ZY3_02STGdata, int * gcpNum, int index)
{
	int gcpNumAll = 0;
	for (int i = 0; i < index; i++)
	{
		gcpNumAll = gcpNumAll + gcpNum[i];
	}
	double *randx = new double[gcpNumAll];
	double *randy = new double[gcpNumAll];
	double *k1 = new double[gcpNumAll];
	double *p1 = new double[gcpNumAll];
	double *p2 = new double[gcpNumAll];
	//����x��y��������������Ҽ�����ʵ������
	mBase.RandomDistribution(1, 0.02, gcpNumAll, 0, randx);
	mBase.RandomDistribution(3, 0.02, gcpNumAll, 0, randy);
	mBase.RandomDistribution(2, 0.02, gcpNumAll, 0, k1);
	mBase.RandomDistribution(5, 0.02, gcpNumAll, 0, p1);
	mBase.RandomDistribution(5, 0.02, gcpNumAll, 0, p2);
	double V[3], alpha, sigma, R[9], x, y, W[3];
	int pNum = 0;
	for (int b = 0; b < index - 1; b++)
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
			//x0=y0=512,f=43.3mm,��Ԫ��С0.015mm
			if (W[2] > 0)
			{
				x = (512 - W[0] / W[2] * 43.3 / 0.015);
				y = (512 - W[1] / W[2] * 43.3 / 0.015);
			}
			else
			{
				x = -513, y = -513;
			}//����������ж��Ƿ������ָ��İ�����һ��
			double r2 = sqrt(x*x + y*y);
			getGCP[b][a].x = x + randx[a + pNum] + k1[a + pNum] * 1e-9*x*r2
				+ p1[a + pNum] * 1e-9*(3 * x*x + y*y) + 2 * p2[a + pNum] * 1e-9*x*y;
			getGCP[b][a].y = y + randy[a + pNum] + k1[a + pNum] * 1e-9*y*r2
				+ p2[a + pNum] * 1e-9*(3 * y*y + x*x) + 2 * p1[a + pNum] * 1e-9*x*y;
		}
		pNum = pNum + gcpNum[b];
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ݿ��Ƶ�����һ�����Ŀ��Ƶ㣬�����������
//���룺getGCP�����Ƶ㣻ZY3_02STGdata���˿�����������̬��indexʱ��ָʾ
//�����getGCP��������Ĵ������Ƶ�
//ע�⣺�����õ�����ʵ�ǵ�ķֲ���������Ϊ�Լ��µ��ǵ�ֲ��ȽϾ��ȡ�
//���ߣ�GZC
//���ڣ�2017.03.02	���£�2017.03.30
//////////////////////////////////////////////////////////////////////////
void APScalibration::SimulateGCP_PreRand5Param(vector<vector<StarGCP>>& getGCP, vector<STGData> ZY3_02STGdata, int index)
{
	int gcpNumAll = 0;
	for (int i = 0; i < getGCP.size(); i++)
	{
		gcpNumAll = gcpNumAll + getGCP[i].size();
	}
	double *randxy = new double[gcpNumAll * 2];
	//����x��y��������������Ҽ�����ʵ������
	mBase.RandomDistribution(0, 0.3, gcpNumAll * 2, 0, randxy);
	double V[3], alpha, sigma, R[9], x, y, W[3];
	int pNum = 0;
	vector < vector<StarGCP> > getGCPchange;
	double x0 = ZY302CaliParam.x0;
	double y0 = ZY302CaliParam.y0;
	double f = ZY302CaliParam.f;
	for (int a = 0; a < getGCP.size(); a++)
	{
		vector<StarGCP> getGCPtmp(getGCP[a].size());
		for (int b = 0; b < getGCP[a].size(); b++)
		{
			//����������ȡ�ǵ�ֲ�������ʵ�ǵ�����
			x = getGCP[a][b].x;
			y = getGCP[a][b].y;
			//����������ʱ���ڱ�
			ZY302CaliParam.x0 = x0 +  1e-2*a;
			ZY302CaliParam.y0 = y0 -  3e-2*a;
			ZY302CaliParam.f = f -  2e-2*a;
			mParse.FromXY2LL(x, y, ZY302CaliParam, V);
			getGCPtmp[b].V[0] = V[0]; getGCPtmp[b].V[1] = V[1]; getGCPtmp[b].V[2] = V[2];
			//���������ȡ���
			getGCPtmp[b].x = x + randxy[pNum + b];
			getGCPtmp[b].y = y + randxy[gcpNumAll + pNum + b];

			//�����õ�����ʵ��̬���ǵ�ʸ����
			//mBase.quat2matrix(ZY3_02STGdata[b].StarA.Q1, ZY3_02STGdata[b].StarA.Q2,
			//	ZY3_02STGdata[b].StarA.Q3, ZY3_02STGdata[b].StarA.Q0, R);//Crj
			//int num = getGCP[b].size();
			//for (int a = 0; a < num; a++)
			//{
			//	V[0] = getGCP[b][a].V[0];
			//	V[1] = getGCP[b][a].V[1];
			//	V[2] = getGCP[b][a].V[2];
			//	mBase.Multi(R, V, W, 3, 3, 1);
			//	//x0=y0=512,f=43.3mm,��Ԫ��С0.015mm
			//	if (W[2] > 0)
			//	{
			//		x = (512 - W[0] / W[2] * 43.3 / 0.015);
			//		y = (512 - W[1] / W[2] * 43.3 / 0.015);
			//	}
			//	else
			//	{
			//		x = -513, y = -513;
			//	}//����������ж��Ƿ������ָ��İ�����һ��
			//	double r2 = sqrt(x*x + y*y);
			//	getGCP[b][a].x = x + randx[a + pNum] + k1[a + pNum] * 1e-8*x*r2 + k2[a + pNum] * 1e-14*x*r2*r2;
			//	getGCP[b][a].y = y + randy[a + pNum] + k1[a + pNum] * 1e-8*y*r2 + k2[a + pNum] * 1e-14*y*r2*r2;
		}
		getGCPchange.push_back(getGCPtmp);
		pNum = pNum + getGCP[a].size();
	}
	//getGCP.clear();
	//getGCP.assign(getGCPchange.begin(), getGCPchange.end());
	getGCP.swap(getGCPchange);
}


//////////////////////////////////////////////////////////////////////////
//���ܣ��������ܼ��Σ������Ķ���ģ��
//���룺getGCP�����Ƶ�
//��������ܼ���ģ�͵õ�3����������f������x0, y0
//ע�⣺getGCP�õ��ĵ�����ϵ����Ϊ X��Y��, ������������ϵΪX��Y��
//���ߣ�GZC
//���ڣ�2017.01.13
//////////////////////////////////////////////////////////////////////////
void APScalibration::CalibrateRigorous3(vector<StarGCP> getGCP, int index)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP,20);
	int num = getGCP.size();
	const int Param = 3;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param], V[3];
	memset(Xest, 0, sizeof(double) * Param);
	//���ó�ֵ
	Xest[0] = 512 - 21.3; Xest[1] = 512 - 11.9;
	Xest[2] = 43.3 / 0.015;
	double quater[4], Crj[9];
	qMethod(getGCP, Xest, Param, quater);
	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
	double DetX, DetY;
	//���õ����жϳ�ֵ
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

				double FX[Param], FY[Param];
				FX[0] = 1;
				FX[1] = 0;
				FX[2] = -V[0] / V[2];
				L = X - Xest[0] + Xest[2] * V[0] / V[2];
				mBase.pNormal(FX, Param, L, ATA, ATL, 1.0);
				FY[0] = 0;
				FY[1] = 1;
				FY[2] = -V[1] / V[2];
				L = Y - Xest[1] + Xest[2] * V[1] / V[2];
				mBase.pNormal(FY, Param, L, ATA, ATL, 1.0);
			}
			//�������
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
				|| iter_count > 20)
				break;
			iter_count++;
			Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
			iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
		}
		qMethod(getGCP, Xest, Param, quater);
		mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
		iter_count2++;
		iter_count = 0;
		if (iter_count2 >= 50)
		{
			break;
		}
	}
	//���
	char output[512];
	sprintf_s(output, "����3���������%d��", index);
	OutputFile(Xest, 3, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��������ܼ��Σ������Ķ���ģ��
//���룺getGCP�����Ƶ�
//��������ܼ���ģ�͵õ�3����������f������x0, y0���������k1���������p1��p2
//ע�⣺getGCP�õ��ĵ�����ϵ����Ϊ X��Y��, ������������ϵΪX��Y��
//���ߣ�GZC
//���ڣ�2017.01.13
//////////////////////////////////////////////////////////////////////////
void APScalibration::CalibrateRigorous6(vector<StarGCP> getGCP, int index)
{
	//�Ƿ�Կ��Ƶ�����Ż���С��20�����ؽǾ�Ĳ�Ҫ
	//OptimizeGCP(getGCP, 20);
	int num = getGCP.size();
	const int Param = 6;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param], V[3];
	memset(Xest, 0, sizeof(double) * Param);
	//���ó�ֵ
	Xest[0] = 512; Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	/*Xest[0] = 512*0.015; Xest[1] = 512 * 0.015;
	Xest[2] = 43.3;*/
	double quater[4], Crj[9];
	qMethod6(getGCP, Xest, Param, quater);
	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
	double DetX, DetY;
	//���õ����жϳ�ֵ
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
				//����ģ��
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
				L = Y - DetY + Xest[2] * V[1] / V[2];
				mBase.pNormal(FY, Param, L, ATA, ATL, 1.0);
			}
			//�������
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
	//���
	char output[512];
	sprintf_s(output, "����6���������%d��", index);
	OutputFile(Xest, 6, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��������ܼ��Σ������Ķ���ģ��
//���룺getGCP�����Ƶ�
//��������ܼ���ģ�͵õ�3����������f������x0, y0���������k1���������p1��p2
//ע�⣺getGCP�õ��ĵ�����ϵ����Ϊ X��Y��, ������������ϵΪX��Y��
//���ߣ�GZC
//���ڣ�2017.01.14
//////////////////////////////////////////////////////////////////////////
void APScalibration::CalibrateRigorousRPY(vector<StarGCP> getGCP, int index)
{
	int num = getGCP.size();
	const int Param = 6;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param], V[3];
	memset(Xest, 0, sizeof(double) * Param);
	//���ó�ֵ
	Xest[0] = 512; Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	/*Xest[0] = 512*0.015; Xest[1] = 512 * 0.015;
	Xest[2] = 43.3;*/
	double quater[4], Crj[9];
	qMethod6(getGCP, Xest, Param, quater);
	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
	double DetX, DetY;
	//���õ����жϳ�ֵ
	double iter_x0 = 512, iter_y0 = 512, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0;
	int iter_count = 0, iter_count2 = 0;
	//����Ru�����ֵΪ0��
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
			//��ת����Ը����ƫ΢��
			//��phi�ǵ�ƫ΢��
			double partial_a1phi = -sinphi*coskap + cosphi*sinomg*sinkap;
			double partial_a2phi = 0;
			double partial_a3phi = -cosphi*coskap - sinphi*sinomg*sinkap;
			double partial_b1phi = sinphi*sinkap + cosphi*sinomg*coskap;
			double partial_b2phi = 0;
			double partial_b3phi = cosphi*sinkap - sinphi*sinomg*coskap;
			double partial_c1phi = cosphi*cosomg;
			double partial_c2phi = 0;
			double partial_c3phi = -sinphi*cosomg;
			//��omega�ǵ�ƫ΢��
			double partial_a1omg = sinphi*cosomg*sinkap;
			double partial_a2omg = -sinomg*sinkap;
			double partial_a3omg = cosphi*cosomg*sinkap;
			double partial_b1omg = sinphi*cosomg*coskap;
			double partial_b2omg = -sinomg*coskap;
			double partial_b3omg = cosphi*cosomg*coskap;
			double partial_c1omg = -sinphi*sinomg;
			double partial_c2omg = -cosomg;
			double partial_c3omg = -cosphi*sinomg;
			//��kappa�ǵ�ƫ΢��
			double partial_a1kap = -cosphi*sinkap + sinphi*sinomg*coskap;
			double partial_a2kap = cosomg*coskap;
			double partial_a3kap = sinphi*sinkap + cosphi*sinomg*coskap;
			double partial_b1kap = -cosphi*coskap - sinphi*sinomg*sinkap;
			double partial_b2kap = -cosomg*sinkap;
			double partial_b3kap = sinphi*coskap - cosphi*sinomg*sinkap;
			double partial_c1kap = 0;
			double partial_c2kap = 0;
			double partial_c3kap = 0;
			//��Xbar,Ybar,Zbar���ߵ�ƫ����
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
			FX[3] = Xest[2] * (1 / Zbar*partial_Xbar_phi - Xbar / Zbar / Zbar*partial_Zbar_phi);
			FX[4] = Xest[2] * (1 / Zbar*partial_Xbar_omg - Xbar / Zbar / Zbar*partial_Zbar_omg);
			FX[5] = Xest[2] * (1 / Zbar*partial_Xbar_kap - Xbar / Zbar / Zbar*partial_Zbar_kap);
			L = Xp - Xest[0] + Xest[2] * Xbar / Zbar;
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
		//�������
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
	//���
	char output[512];
	sprintf_s(output, "����RPY���������%d��", index);
	OutputFile(Xest, 6, output);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ������ǵ�͹���ĽǾ�������
//���룺�ǵ���ȡ��Ͷ�Ӧ���ǳྭ��γ����Ϊ���Ƶ�
//����������Ƶ�3������
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.03.30
//////////////////////////////////////////////////////////////////////////
void APScalibration::CalibrateOpticAxisMultiImg(vector<vector<StarGCP>> getGCP, int index)
{
	//δ֪��������
	const int Param = 3;
	//����XestΪ�����Ƶ�6��������λ�����ɺ���mm��
	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
	//Detx0	Dety0	 f			  k1		   p1         p2
	double Xest[Param];
	Xest[0] = 512;
	Xest[1] = 512;
	Xest[2] = 43.3 / 0.015;
	//�йط������������
	double *ATA = new double[Param*Param];
	double *ATL = new double[Param];
	double L;
	//���õ����жϳ�ֵ
	double iter_x0 = 0, iter_y0 = 0, iter_f = 43.3 / 0.015, iter_count = 0;
	int numIMG, numGCP;
	double Xa, Ya, Xb, Yb, DetX, DetY;
	while (true)
	{
		memset(ATA, 0, sizeof(double) * Param*Param);	memset(ATL, 0, sizeof(double) * Param);
		numIMG = getGCP.size();
		for (size_t i = 0; i < numIMG; i++)
		{
			//����qMethod���ɳ�ʼ��̬
			double quater[4], Crj[9], Opt[3] = { 0,0,1 }, Vopt[3];
			qMethod(getGCP[i], Xest, Param, quater);
			mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
			mBase.invers_matrix(Crj, 3);//Cjr
			mBase.Multi(Crj, Opt, Vopt, 3, 3, 1);
			for (int a = 0; a < getGCP[i].size(); a++)
			{
				//��ʵ����ʹ��
				Xa = 1024 - getGCP[i][a].y;		Ya = getGCP[i][a].x;
				//ģ��
				double dim = (Xa - Xest[0])*(Xa - Xest[0]) + (Ya - Xest[1])*(Ya - Xest[1]) + Xest[2] * Xest[2];
				double thetaX0 = Xest[2] * pow(dim, -1.5)*(Xa - Xest[0]);
				double thetaY0 = Xest[2] * pow(dim, -1.5)*(Ya - Xest[1]);
				double focalPar = (sqrt(dim) - Xest[2] / sqrt(dim)) / dim;
				double G[3] = { thetaX0,thetaY0,focalPar };

				//��ʵ�Ǿ�
				double  VTV;
				mBase.Multi(getGCP[i][a].V, Vopt, &VTV, 1, 3, 1);
				L = VTV - Xest[2] / sqrt(dim);
				mBase.pNormal(G, Param, L, ATA, ATL, 1.0);
			}
		}
		//�������
		mBase.Gauss(ATA, ATL, Param);
		if (abs(ATL[0] - iter_x0) < 1e-10&&abs(ATL[1] - iter_y0) < 1e-10&&abs(ATL[2] - iter_f) < 1e-10
			|| iter_count > 100)
			break;
		iter_count++;
		Xest[0] += ATL[0]; Xest[1] += ATL[1]; Xest[2] += ATL[2];
		iter_x0 = abs(ATL[0]), iter_y0 = abs(ATL[1]), iter_f = abs(ATL[2]);
	}
	//��������
	//CaliAccuracy(getGCP, Xest, 3);
	char output[512];
	sprintf_s(output, "�Ǿ�3���������%d��", index);
	OutputFile(Xest, 3, output);
}


//////////////////////////////////////////////////////////////////////////
//���ܣ��������ܼ��Σ������Ķ���ģ��
//���룺getGCP�����Ƶ�
//��������ܼ���ģ�͵õ�3����������f������x0, y0���������k1���������p1��p2
//ע�⣺getGCP�õ��ĵ�����ϵ����Ϊ X��Y��, ������������ϵΪX��Y��
//���ߣ�GZC
//���ڣ�2017.01.14
//////////////////////////////////////////////////////////////////////////
//void APScalibration::CalibrateRigorousQuat(vector<StarGCP> getGCP)
//{
//	int num = getGCP.size();
//	const int Param = 6;
//	//�йط������������
//	double *ATA = new double[Param*Param];
//	double *ATL = new double[Param];
//	double L;
//	//Xest[0]  Xest[1]  Xest[2]  Xest[3]  Xest[4]  Xest[5]
//	//Detx0	Dety0	 f			  k1		   p1         p2
//	double Xest[Param], V[3];
//	memset(Xest, 0, sizeof(double) * Param);
//	//���ó�ֵ
//	Xest[0] = 512; Xest[1] = 512;
//	Xest[2] = 43.3 / 0.015;
//	double quater[4], Crj[9];
//	qMethod6(getGCP, Xest, Param, quater);
//	mBase.quat2matrix(quater[1], quater[2], quater[3], quater[0], Crj);
//	double DetX, DetY;
//	//���õ����жϳ�ֵ
//	double iter_x0 = 512, iter_y0 = 512, iter_f = 43.3 / 0.015, iter_k1 = 0, iter_p1 = 0, iter_p2 = 0;
//	int iter_count = 0, iter_count2 = 0;
//	//����Ru�����ֵΪ0��
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
//			//q4��q1q2q3��ƫ΢��
//			double 
//			double q4q1 = 0.5*sqrt(1-)
//			//��q1��ƫ΢��
//			double a1q1 = 
//			//��omega�ǵ�ƫ΢��
//			double partial_a1omg = sinphi*cosomg*sinkap;
//			double partial_a2omg = -sinomg*sinkap;
//			double partial_a3omg = cosphi*cosomg*sinkap;
//			double partial_b1omg = sinphi*cosomg*coskap;
//			double partial_b2omg = -sinomg*coskap;
//			double partial_b3omg = cosphi*cosomg*coskap;
//			double partial_c1omg = -sinphi*sinomg;
//			double partial_c2omg = -cosomg;
//			double partial_c3omg = -cosphi*sinomg;
//			//��kappa�ǵ�ƫ΢��
//			double partial_a1kap = -cosphi*sinkap + sinphi*sinomg*coskap;
//			double partial_a2kap = cosomg*coskap;
//			double partial_a3kap = sinphi*sinkap + cosphi*sinomg*coskap;
//			double partial_b1kap = -cosphi*coskap - sinphi*sinomg*sinkap;
//			double partial_b2kap = -cosomg*sinkap;
//			double partial_b3kap = sinphi*coskap - cosphi*sinomg*sinkap;
//			double partial_c1kap = 0;
//			double partial_c2kap = 0;
//			double partial_c3kap = 0;
//			//��Xbar,Ybar,Zbar���ߵ�ƫ����
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
//		//�������
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
//���ܣ�q_Method���˷�ʽ
//���룺getGCP����ȡ���ǵ���ƣ�Xest������ڻ��������num����������
//�������Ԫ��Q
//ע�⣺����Eigen����б�д
//���ߣ�GZC
//���ڣ�2017.01.12
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
		//����ģ��
		double X = Xp - Xest[0];
		double Y = Yp - Xest[1];
		double r2 = sqrt(X*X + Y*Y);
		DetX = Xest[3] * X * r2 + Xest[4] * (3 * X * X + Y * Y) + 2 * Xest[5] * X * Y;
		DetY = Xest[3] * Y * r2 + Xest[5] * (3 * Y * Y + X * X) + 2 * Xest[4] * X * Y;
		//��ֵ����������ϵת����������ϵ����������ϵΪ-(X-X0)��-(Y-Y0)��f��Ȼ��
		double D = sqrt(pow((X - DetX - Xest[0]), 2) + pow((Y - DetY - Xest[1]), 2) + pow(Xest[2], 2));
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
	//���˳��Ϊ0123������Ϊ0
	quater[0] = q(3); quater[1] = q(0); quater[2] = q(1); quater[3] = q(2);
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����������
//���룺�����Ƶ�num������Xest��description��ʾ��һ���������Ϣ
//�������workpath�����������������.txt
//ע�⣺
//���ߣ�GZC
//���ڣ�2017.01.14
//////////////////////////////////////////////////////////////////////////
void APScalibration::OutputFile(double * Xest, int num, string description)
{
	string path = workpath + "������������.txt";
	FILE *fp = fopen(path.c_str(), "a+");
	fprintf(fp, "%s\t", description.c_str());
	for (int i = 0; i < num; i++)
	{
		fprintf(fp, "%.20f\t", Xest[i]);
	}
	fprintf(fp, "\n");
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�q_Method���˷�ʽ
//���룺getGCP����ȡ���ǵ���ƣ�Xest������ڻ��������num����������
//�������Ԫ��Q
//ע�⣺����Eigen����б�д
//���ߣ�GZC
//���ڣ�2017.01.12
//////////////////////////////////////////////////////////////////////////
bool APScalibration::qMethod(vector<StarGCP> getGCP, double *Xest, int Param, double *quater)
{
	MatrixXd obs_in_startrackerframe(getGCP.size(), 3), stars_in_celestialframeV(getGCP.size(), 3);
	double Wob[3];
	double X, Y, DetX, DetY;
	for (int a = 0; a < getGCP.size(); a++)
	{
		//��ֵ����������ϵת����������ϵ����������ϵΪ-(X-X0)��-(Y-Y0)��f��Ȼ��
		X = 1024 - getGCP[a].y;		Y = getGCP[a].x;
		double D = sqrt((X - Xest[0])*(X - Xest[0]) + (Y - Xest[1])*(Y - Xest[1]) + Xest[2] * Xest[2]);
		//double Wob[3];
		Wob[0] = -(X - Xest[0]) / D;
		Wob[1] = -(Y - Xest[1]) / D;
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
	//���˳��Ϊ0123������Ϊ0
	quater[0] = q(3); quater[1] = q(0); quater[2] = q(1); quater[3] = q(2);
	return 0;
}