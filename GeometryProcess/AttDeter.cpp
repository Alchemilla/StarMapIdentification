#include "AttDeter.h"



AttDeter::AttDeter()
{
}


AttDeter::~AttDeter()
{
}

void AttDeter::ExtendedKalmanFilter(vector<Attitude>&qMeas, vector<Gyro> wMeas)
{
	int nGyro = wMeas.size();
	int nQuat = qMeas.size();
	double sig = 8. / 3600 * PI / 180;
	double w, dt, qw1, qw2, qw3, qw4, qmm1, qmm2, qmm3, qe11, qe22, qe33, qe44;
	Matrix3d zero33, eye33, poa, pog, r, sigu33, sigv33, wa;
	MatrixXd p(6, 6), Q(6, 6), eye66(6, 6), xe(1, 3), z(3, 1), h(3, 6), k(6, 3), tempqe(4, 1), om(4, 4),
		fmat(6, 6), gmat(6, 6), phi(6, 6), gamma(6, 6);
	eye33 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	zero33 << MatrixXd::Zero(3, 3);
	sigu33 << 1e-20*eye33;//陀螺漂移噪声
	sigv33 << 1e-13*eye33;//陀螺噪声
	poa << 3e-6*eye33;//初始姿态误差协方差
	pog << 1e-12*eye33;//初始陀螺误差协方差
	r << pow(sig, 2)*eye33;//星敏噪声	
	eye66 << eye33, zero33, zero33, eye33;

	//预先计算估计四元数的数量
	double utStart = qMeas[0].UTC;
	int a = 1, b = 0;
	for (int i = 1; i < wMeas.size();)
	{
		if (a < wMeas.size() && (qMeas[a].UTC - utStart) <= (wMeas[i].UT - utStart))
		{
			utStart = qMeas[a].UTC;	 a++;		b++;
		}
		else
		{
			utStart = wMeas[i].UT;	 i++;		 b++;
		}
	}
	MatrixXd Qest(b + 1, 4), we(b + 1, 3), xest(b + 1, 6);

	//设置递推初始值
	a = 1, b = 0;
	utStart = qMeas[0].UTC;
	//biasOut[0] = biasOut[1] = biasOut[2] = 0;
	xest.row(0) << 0, 0, 0, 0, 0, 0;//状态初始值
	p << poa, zero33, zero33, pog;//过程协方差
	Q << sigv33, zero33, zero33, sigu33;//过程噪声
	Qest(0, 0) = qMeas[0].Q1, Qest(0, 1) = qMeas[0].Q2;
	Qest(0, 2) = qMeas[0].Q3, Qest(0, 3) = qMeas[0].Q0;
	vector<Attitude>quatEst(qMeas);
	//quatEst[0].UTC = 0;
	quatEst[0].Q1 = Qest(b, 0), quatEst[0].Q2 = Qest(b, 1);
	quatEst[0].Q3 = Qest(b, 2), quatEst[0].Q0 = Qest(b, 3);
	for (int i = 1; i < nGyro;)
	{
		if (a < nQuat && (qMeas[a].UTC - utStart) <= (wMeas[i].UT - utStart))
		{
			/****************陀螺测量值预测***************/
			dt = qMeas[a].UTC - utStart;
			utStart = qMeas[a].UTC;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0; //xest(b + 1, 3) = 0; xest(b + 1, 4) = 0; xest(b + 1, 5) = 0;
			//cout << xest.row(b + 1) << endl;
			//xest(b + 1, 3) = xest(b, 3); xest(b + 1, 4) = xest(b, 4); xest(b + 1, 5) = xest(b, 5);
			b++;

			/****************星敏测量值更新***************/
			qmm1 = -qMeas[a].Q0*Qest(b, 0) - qMeas[a].Q3*Qest(b, 1) + qMeas[a].Q2*Qest(b, 2) + qMeas[a].Q1*Qest(b, 3);
			qmm2 = qMeas[a].Q3*Qest(b, 0) - qMeas[a].Q0*Qest(b, 1) - qMeas[a].Q1*Qest(b, 2) + qMeas[a].Q2*Qest(b, 3);
			qmm3 = -qMeas[a].Q2*Qest(b, 0) + qMeas[a].Q1*Qest(b, 1) - qMeas[a].Q0*Qest(b, 2) + qMeas[a].Q3*Qest(b, 3);
			z << 2 * qmm1, 2 * qmm2, 2 * qmm3;
			//cout << z << endl;
			if (a < 20 || qmm1 < 0.01&&qmm2 < 0.01&&qmm3 < 0.01)//这里加个四元数容错
			{
				h << eye33, zero33;
				k = p*h.transpose()*(h*p*h.transpose() + r).inverse();
				p = (eye66 - k*h)*p;
				xest.row(b) = xest.row(b) + (k*z).transpose();
				xe = 0.5*xest.row(b).head(3);
				qe11 = Qest(b, 0) + xe(2)*Qest(b, 1) - xe(1)*Qest(b, 2) + xe(0)*Qest(b, 3);
				qe22 = -xe(2)*Qest(b, 0) + Qest(b, 1) + xe(0)*Qest(b, 2) + xe(1)*Qest(b, 3);
				qe33 = xe(1)*Qest(b, 0) - xe(0)*Qest(b, 1) + Qest(b, 2) + xe(2)*Qest(b, 3);
				qe44 = -xe(0)*Qest(b, 0) - xe(1)*Qest(b, 1) - xe(2)*Qest(b, 2) + Qest(b, 3);
				tempqe << qe11, qe22, qe33, qe44;
				tempqe.normalize();
				Qest.row(b) << tempqe(0), tempqe(1), tempqe(2), tempqe(3);
			}
			a++;
		}
		else
		{
			/****************陀螺测量值预测***************/
			dt = wMeas[i].UT - utStart;
			utStart = wMeas[i].UT;
			we(b, 0) = wMeas[i - 1].wx - xest(b, 3);//注意是i-1，因为此刻的四元数是上一刻陀螺递推而来
			we(b, 1) = wMeas[i - 1].wy - xest(b, 4);
			we(b, 2) = wMeas[i - 1].wz - xest(b, 5);
			w = sqrt(we(b, 0)*we(b, 0) + we(b, 1)*we(b, 1) + we(b, 2)*we(b, 2));
			wa << 0, -we(b, 2), we(b, 1), we(b, 2), 0, -we(b, 0), -we(b, 1), we(b, 0), 0;
			fmat << -wa, -eye33, zero33, zero33;
			gmat << -eye33, zero33, zero33, eye33;
			phi = eye66 + fmat*dt;
			gamma = (eye66*dt + fmat*dt*dt / 2)*gmat;
			//Propagate State
			qw1 = we(b, 0) / w*sin(0.5*w*dt);
			qw2 = we(b, 1) / w*sin(0.5*w*dt);
			qw3 = we(b, 2) / w*sin(0.5*w*dt);
			qw4 = cos(0.5*w*dt);
			om << qw4, qw3, -qw2, qw1, -qw3, qw4, qw1, qw2, qw2, -qw1, qw4, qw3, -qw1, -qw2, -qw3, qw4;
			Qest.row(b + 1) = (om*Qest.row(b).transpose()).transpose();
			//Propagate Covariance
			p = phi*p*phi.transpose() + gamma*Q*gamma.transpose();
			xest.row(b + 1) = xest.row(b);
			xest(b + 1, 0) = 0; xest(b + 1, 1) = 0; xest(b + 1, 2) = 0;// xest(b + 1, 3) = 0; xest(b + 1, 4) = 0; xest(b + 1, 5) = 0;
			//xest(b + 1, 3) = xest(b, 3); xest(b + 1, 4) = xest(b, 4); xest(b + 1, 5) = xest(b, 5);

			quatEst[i].UTC = wMeas[i].UT;
			quatEst[i].Q1 = Qest(b + 1, 0), quatEst[i].Q2 = Qest(b + 1, 1);
			quatEst[i].Q3 = Qest(b + 1, 2), quatEst[i].Q0 = Qest(b + 1, 3);		
			b++;
			i++;
		}
	}
	qMeas.clear();
	qMeas.assign(quatEst.begin(), quatEst.end());
}
