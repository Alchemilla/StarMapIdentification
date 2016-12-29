#include "Simulation.h"

Simulation::Simulation(void)
{
	//Engine *ep; //����Matlab����ָ�롣
	if (!(ep=engOpen(NULL))) //�����Ƿ�����Matlab����ɹ���
	{
		cout <<"Can't start Matlab engine!" <<endl;
		exit(1);
	}
	else
	{
		cout <<"Begin to Draw angle residual"<<endl;
	}
}
Simulation::~Simulation(void)
{
	engClose(ep); //�ر�Matlab���档
}
//////////////////////////////////////////////////////////////////////////
//���ܣ�����matlab����ͼ��ʾ����
//���룺��
//�������
//ע�⣺�����ĵ��÷�ʽ
//���ڣ�2016.10.11
//////////////////////////////////////////////////////////////////////////
void Simulation::MatlabExample()
{
	const int N = 50;
	double x[N],y[N];
	int j = 1;
	for (int i=0; i<N; i++) //��������x��y
	{
		x[i] = (i+1);
		y[i] = sin(x[i]) + j * log(x[i]); //������֮������������xx[i];
		j*= -1;
	}
	Engine *ep; //����Matlab����ָ�롣
	if (!(ep=engOpen(NULL))) //�����Ƿ�����Matlab����ɹ���
	{
		cout <<"Can't start Matlab engine!" <<endl;
		exit(1);
	}

	//����mxArray��Ϊ�У�N�е�ʵ�����顣
	mxArray *xx = mxCreateDoubleMatrix(1,N, mxREAL);
	mxArray *yy = mxCreateDoubleMatrix(1,N, mxREAL); //ͬ�ϡ�

	memcpy(mxGetPr(xx), x, N*sizeof(double)); //������x���Ƶ�mxarray����xx�С�
	memcpy(mxGetPr(yy), y, N*sizeof(double)); //������x���Ƶ�mxarray����yy�С�

	engPutVariable(ep, "xx",xx); //��mxArray����xxд�뵽Matlab�����ռ䣬����Ϊxx��
	engPutVariable(ep, "yy",yy); //��mxArray����yyд�뵽Matlab�����ռ䣬����Ϊyy��

	//��Matlab���淢�ͻ�ͼ���plotΪMatlab�Ļ�ͼ�������μ�Matlab����ĵ���
	engEvalString(ep, "plot(xx, yy); ");

	mxDestroyArray(xx); //����mxArray����xx��yy��
	mxDestroyArray(yy);

	cout <<"Press any key to exit!" <<endl;
	cin.get();
	engClose(ep); //�ر�Matlab���档

	/*Engine* EnginePt;
	EnginePt = engOpen(NULL);
	if( EnginePt==NULL ) { 
	fprintf(stderr,"engine not found!\n"); exit(-1);
	}
	engEvalString(EnginePt, "x = 0:0.01:pi;");
	engEvalString(EnginePt, "y = sin(x);");
	engEvalString(EnginePt, "plot(x,y);");
	system("pause");
	engClose(EnginePt);*/
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ڱȽϸ��ֶ��˷���֮������𣻵���matlab��ŷ���ǲв�
//���룺ŷ���ǽṹ�壨����ʱ���RPY�����ǣ����ṹ������N
//������в�ͼ��3*1��ͼ��
//ע�⣺�ں������ж����ݽ���ƽ��
//���ڣ�2016.10.11
//////////////////////////////////////////////////////////////////////////
void Simulation::MatPlotDetQ(SateEuler *EulerArray,long N)
{	
	//����mxArray��Ϊ�У�N�е�ʵ�����顣
	mxArray *UTC = mxCreateDoubleMatrix(1,N, mxREAL);
	mxArray *Roll = mxCreateDoubleMatrix(1,N, mxREAL); //ͬ�ϡ�
	mxArray *Pitch = mxCreateDoubleMatrix(1,N, mxREAL);
	mxArray *Yaw = mxCreateDoubleMatrix(1,N, mxREAL);
	int i;
	double cal[4]={0,0,0,0};
	double *a = new double[N];
	double *b = new double[N];
	double *c = new double[N];
	double *d = new double[N];
	
	//�����ݽ���ƽ������
	for (i=0;i<N;i++)
	{
		cal[1]+=EulerArray[i].R;
		cal[2]+=EulerArray[i].P;
		cal[3]+=EulerArray[i].Y;
	}
	cal[1]=cal[1]/N; cal[2]=cal[2]/N;	cal[3]=cal[3]/N;
	//���ṹ����ֵ����������
	for (i=0;i<N;i++)
	{
		a[i]=EulerArray[i].UTC;
		b[i]=EulerArray[i].R-cal[1];
		c[i]=EulerArray[i].P-cal[2];
		d[i]=EulerArray[i].Y-cal[3];
	}
	

	memcpy(mxGetPr(UTC), a, N*sizeof(double)); //������x���Ƶ�mxarray����xx�С�
	memcpy(mxGetPr(Roll), b, N*sizeof(double)); //������x���Ƶ�mxarray����yy�С�
	memcpy(mxGetPr(Pitch), c, N*sizeof(double)); //������x���Ƶ�mxarray����yy�С�
	memcpy(mxGetPr(Yaw), d, N*sizeof(double)); //������x���Ƶ�mxarray����yy�С�

	engPutVariable(ep, "UTC",UTC); //��mxArray����xxд�뵽Matlab�����ռ䣬����Ϊxx��
	engPutVariable(ep, "Roll",Roll); 
	engPutVariable(ep, "Pitch",Pitch); 
	engPutVariable(ep, "Yaw",Yaw); 

	//��Matlab���淢�ͻ�ͼ���plotΪMatlab�Ļ�ͼ�������μ�Matlab����ĵ���
	engEvalString(ep, "subplot 311;plot(UTC,Roll,'r'); ");
	engEvalString(ep, "subplot 312;plot(UTC,Pitch,'g'); ");
	engEvalString(ep, "subplot 313;plot(UTC,Yaw,'b'); ");

	mxDestroyArray(UTC); //����mxArray����xx��yy��
	mxDestroyArray(Roll);
	mxDestroyArray(Pitch);
	mxDestroyArray(Yaw);

	cout <<"Press any key to exit!" <<endl;
	cin.get();
//	engClose(ep); //�ر�Matlab���档
}

//////////////////////////////////////////////////////////////////////////
//���ܣ������������ֲ���
//���룺����x������N
//����������ֲ�ͼ
//ע�⣺
//���ڣ�2016.10.12
//////////////////////////////////////////////////////////////////////////
void Simulation::MatGaussDist(double *x, long N)
{
	mxArray *xx = mxCreateDoubleMatrix(1,N, mxREAL);

	memcpy(mxGetPr(xx), x, N*sizeof(double)); //������x���Ƶ�mxarray����xx�С�

	engPutVariable(ep, "x",xx); //��mxArray����xxд�뵽Matlab�����ռ䣬����Ϊxx��
	//ת��.m�ļ��Ĵ��·���������޸������ʽ����ֱ�Ӹ�m�ļ�
	engEvalString(ep,"cd C:\\Users\\wcsgz\\Documents\\1-MATLAB\\1-��Ŀʵ��\\ZY_3��������");
	engEvalString(ep, "GaussDist(x)");//��matlab��hist��������ͼ
	mxDestroyArray(xx); //����mxArray����xx��yy��	
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����һ��������̬�ֲ��������
//���룺mean:��ֵ��sigma:����
//����������ֲ�ͼ
//ע�⣺
//���ڣ�2016.10.12 by HWC
//////////////////////////////////////////////////////////////////////////
double GaussRand(double mean, double sigma, int &phase)
{
	static double v1, v2, s;
	double x;
	if (0 == phase)
	{
		do
		{
			double u1 = (double)rand() / RAND_MAX;
			double u2 = (double)rand() / RAND_MAX;
			v1 = 2 * u1 - 1;
			v2 = 2 * u2 - 1;
			s = v1 * v1 + v2 * v2;
		} while (1 <= s || 0 == s);
		x = v1 * sqrt(-2 * log(s) / s);
	}
	else
	{
		x = v2 * sqrt(-2 * log(s) / s);
	}
	phase = 1 - phase;
	return (x*sigma + mean);    // ע��Ҫ������,���򷵻�ֵ����sigma��
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����һ�������̬�ֲ��������
//���룺mean:��ֵ��sigma:���n:���������randCount:����0ÿ���������һ��
//�����a:����n���������a
//ע�⣺randCountһ��Ϊ0
//���ڣ�2016.10.12 by HWC
//////////////////////////////////////////////////////////////////////////
double RandomDistribution(double mean, double sigma, int n, long randCount, double *a)
{
	if (n <= 0)
		return !0;
	static int phase1 = 0;
	int phase2 = 0;
	static unsigned int timetemp = 0;
	// ����ÿ�ζ���ͬ�������
	if (randCount == 0)
	{
		double min = mean - 3 * sigma;
		double max = mean + 3 * sigma;
		// ע��Ҫʹ��time(NULL),������GetTickcount
		// GetTickcount�����������شӲ���ϵͳ��������ǰ�������ĺ�������
		// ���������ж�ĳ������ִ�е�ʱ�䣬�亯��ԭ����DWORD GetTickCount(void)��
		// ����ֵ��32λ��˫������DWORD�洢����˿��Դ洢�����ֵ��2^32 msԼΪ49.71�죬
		// �����ϵͳ����ʱ�䳬��49.71��ʱ��������ͻ��0��MSDN��Ҳ��ȷ���ᵽ��:
		// "Retrieves the number of milliseconds that have elapsed since the 
		// system was started, up to 49.7 days."����ˣ�����Ǳ�д�������˳���
		// �˴�һ��Ҫ���ע�⣬�������������״����
		srand(time(NULL) + timetemp);
		// ���������
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase1);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
		timetemp = (unsigned int)fabs(a[0]);
	}
	// ����ÿ�ζ���ͬ�������,�����ȶ��Ⱦ���Ҫ�õ�
	else
	{
		double min = mean - sigma;
		double max = mean + sigma;
		srand(randCount);
		// ���������
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase2);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
	}
	// ����̫�پͲ�����
	if (n>3)
	{
		// ��ȡƽ��ֵ
		double meantemp = 0;
		for (int i = 0; i<n; i++)
			meantemp += a[i];
		meantemp /= n;
		// ��ƽ��ֵ��������
		double div = meantemp - mean;
		for (int i = 0; i<n; i++)
			a[i] -= div;
	}
	return 0;
}


//////////////////////////////////////////////////////////////////////////
//���ܣ�������������
//���룺
//�����
//ע�⣺
//���ڣ�2016.10.12
//////////////////////////////////////////////////////////////////////////
void Simulation::SimStar()
{		
	//����һ����ֵΪ0������ΪN���������
	long N=10000;
	double *x = new double[N];
	RandomDistribution(0,1,N,0,x);
	MatGaussDist(x,N);//��matlab�����ֲ�ͼ

	cout <<"Press any key to exit!" <<endl;
	cin.get();    
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�������������
//���룺
//�����
//ע�⣺
//���ڣ�2016.10.11
//////////////////////////////////////////////////////////////////////////
void Simulation::SimGyro()
{}


