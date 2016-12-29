#include "Simulation.h"

Simulation::Simulation(void)
{
	//Engine *ep; //定义Matlab引擎指针。
	if (!(ep=engOpen(NULL))) //测试是否启动Matlab引擎成功。
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
	engClose(ep); //关闭Matlab引擎。
}
//////////////////////////////////////////////////////////////////////////
//功能：调用matlab来画图（示例）
//输入：无
//输出：无
//注意：函数的调用方式
//日期：2016.10.11
//////////////////////////////////////////////////////////////////////////
void Simulation::MatlabExample()
{
	const int N = 50;
	double x[N],y[N];
	int j = 1;
	for (int i=0; i<N; i++) //计算数组x和y
	{
		x[i] = (i+1);
		y[i] = sin(x[i]) + j * log(x[i]); //产生－之间的随机数赋给xx[i];
		j*= -1;
	}
	Engine *ep; //定义Matlab引擎指针。
	if (!(ep=engOpen(NULL))) //测试是否启动Matlab引擎成功。
	{
		cout <<"Can't start Matlab engine!" <<endl;
		exit(1);
	}

	//定义mxArray，为行，N列的实数数组。
	mxArray *xx = mxCreateDoubleMatrix(1,N, mxREAL);
	mxArray *yy = mxCreateDoubleMatrix(1,N, mxREAL); //同上。

	memcpy(mxGetPr(xx), x, N*sizeof(double)); //将数组x复制到mxarray数组xx中。
	memcpy(mxGetPr(yy), y, N*sizeof(double)); //将数组x复制到mxarray数组yy中。

	engPutVariable(ep, "xx",xx); //将mxArray数组xx写入到Matlab工作空间，命名为xx。
	engPutVariable(ep, "yy",yy); //将mxArray数组yy写入到Matlab工作空间，命名为yy。

	//向Matlab引擎发送画图命令。plot为Matlab的画图函数，参见Matlab相关文档。
	engEvalString(ep, "plot(xx, yy); ");

	mxDestroyArray(xx); //销毁mxArray数组xx和yy。
	mxDestroyArray(yy);

	cout <<"Press any key to exit!" <<endl;
	cin.get();
	engClose(ep); //关闭Matlab引擎。

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
//功能：用于比较各种定姿方法之间的区别；调用matlab来欧拉角残差
//输入：欧拉角结构体（包括时间和RPY三个角），结构体数量N
//输出：残差图（3*1的图）
//注意：在函数内有对数据进行平均
//日期：2016.10.11
//////////////////////////////////////////////////////////////////////////
void Simulation::MatPlotDetQ(SateEuler *EulerArray,long N)
{	
	//定义mxArray，为行，N列的实数数组。
	mxArray *UTC = mxCreateDoubleMatrix(1,N, mxREAL);
	mxArray *Roll = mxCreateDoubleMatrix(1,N, mxREAL); //同上。
	mxArray *Pitch = mxCreateDoubleMatrix(1,N, mxREAL);
	mxArray *Yaw = mxCreateDoubleMatrix(1,N, mxREAL);
	int i;
	double cal[4]={0,0,0,0};
	double *a = new double[N];
	double *b = new double[N];
	double *c = new double[N];
	double *d = new double[N];
	
	//对数据进行平均处理
	for (i=0;i<N;i++)
	{
		cal[1]+=EulerArray[i].R;
		cal[2]+=EulerArray[i].P;
		cal[3]+=EulerArray[i].Y;
	}
	cal[1]=cal[1]/N; cal[2]=cal[2]/N;	cal[3]=cal[3]/N;
	//将结构体数值赋到数组中
	for (i=0;i<N;i++)
	{
		a[i]=EulerArray[i].UTC;
		b[i]=EulerArray[i].R-cal[1];
		c[i]=EulerArray[i].P-cal[2];
		d[i]=EulerArray[i].Y-cal[3];
	}
	

	memcpy(mxGetPr(UTC), a, N*sizeof(double)); //将数组x复制到mxarray数组xx中。
	memcpy(mxGetPr(Roll), b, N*sizeof(double)); //将数组x复制到mxarray数组yy中。
	memcpy(mxGetPr(Pitch), c, N*sizeof(double)); //将数组x复制到mxarray数组yy中。
	memcpy(mxGetPr(Yaw), d, N*sizeof(double)); //将数组x复制到mxarray数组yy中。

	engPutVariable(ep, "UTC",UTC); //将mxArray数组xx写入到Matlab工作空间，命名为xx。
	engPutVariable(ep, "Roll",Roll); 
	engPutVariable(ep, "Pitch",Pitch); 
	engPutVariable(ep, "Yaw",Yaw); 

	//向Matlab引擎发送画图命令。plot为Matlab的画图函数，参见Matlab相关文档。
	engEvalString(ep, "subplot 311;plot(UTC,Roll,'r'); ");
	engEvalString(ep, "subplot 312;plot(UTC,Pitch,'g'); ");
	engEvalString(ep, "subplot 313;plot(UTC,Yaw,'b'); ");

	mxDestroyArray(UTC); //销毁mxArray数组xx和yy。
	mxDestroyArray(Roll);
	mxDestroyArray(Pitch);
	mxDestroyArray(Yaw);

	cout <<"Press any key to exit!" <<endl;
	cin.get();
//	engClose(ep); //关闭Matlab引擎。
}

//////////////////////////////////////////////////////////////////////////
//功能：用来画函数分布的
//输入：数组x，数量N
//输出：函数分布图
//注意：
//日期：2016.10.12
//////////////////////////////////////////////////////////////////////////
void Simulation::MatGaussDist(double *x, long N)
{
	mxArray *xx = mxCreateDoubleMatrix(1,N, mxREAL);

	memcpy(mxGetPr(xx), x, N*sizeof(double)); //将数组x复制到mxarray数组xx中。

	engPutVariable(ep, "x",xx); //将mxArray数组xx写入到Matlab工作空间，命名为xx。
	//转入.m文件的存放路径，后续修改输出方式可以直接改m文件
	engEvalString(ep,"cd C:\\Users\\wcsgz\\Documents\\1-MATLAB\\1-项目实现\\ZY_3仿真试验");
	engEvalString(ep, "GaussDist(x)");//用matlab的hist函数来画图
	mxDestroyArray(xx); //销毁mxArray数组xx和yy。	
}

//////////////////////////////////////////////////////////////////////////
//功能：产生一个符合正态分布的随机数
//输入：mean:均值，sigma:方差
//输出：函数分布图
//注意：
//日期：2016.10.12 by HWC
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
	return (x*sigma + mean);    // 注意要加括号,否则返回值就是sigma了
}

//////////////////////////////////////////////////////////////////////////
//功能：产生一组符合正态分布的随机数
//输入：mean:均值，sigma:方差，n:随机数量，randCount:输入0每组随机数不一样
//输出：a:生成n组随机数组a
//注意：randCount一般为0
//日期：2016.10.12 by HWC
//////////////////////////////////////////////////////////////////////////
double RandomDistribution(double mean, double sigma, int n, long randCount, double *a)
{
	if (n <= 0)
		return !0;
	static int phase1 = 0;
	int phase2 = 0;
	static unsigned int timetemp = 0;
	// 产生每次都不同的随机数
	if (randCount == 0)
	{
		double min = mean - 3 * sigma;
		double max = mean + 3 * sigma;
		// 注意要使用time(NULL),而不是GetTickcount
		// GetTickcount函数：它返回从操作系统启动到当前所经过的毫秒数，
		// 常常用来判断某个方法执行的时间，其函数原型是DWORD GetTickCount(void)，
		// 返回值以32位的双字类型DWORD存储，因此可以存储的最大值是2^32 ms约为49.71天，
		// 因此若系统运行时间超过49.71天时，这个数就会归0，MSDN中也明确的提到了:
		// "Retrieves the number of milliseconds that have elapsed since the 
		// system was started, up to 49.7 days."。因此，如果是编写服务器端程序，
		// 此处一定要万分注意，避免引起意外的状况。
		srand(time(NULL) + timetemp);
		// 产生随机数
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase1);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
		timetemp = (unsigned int)fabs(a[0]);
	}
	// 产生每次都相同的随机数,比如稳定度就需要用到
	else
	{
		double min = mean - sigma;
		double max = mean + sigma;
		srand(randCount);
		// 产生随机数
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase2);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
	}
	// 个数太少就不进行
	if (n>3)
	{
		// 求取平均值
		double meantemp = 0;
		for (int i = 0; i<n; i++)
			meantemp += a[i];
		meantemp /= n;
		// 对平均值进行修正
		double div = meantemp - mean;
		for (int i = 0; i<n; i++)
			a[i] -= div;
	}
	return 0;
}


//////////////////////////////////////////////////////////////////////////
//功能：仿真星敏数据
//输入：
//输出：
//注意：
//日期：2016.10.12
//////////////////////////////////////////////////////////////////////////
void Simulation::SimStar()
{		
	//建立一个均值为0，方差为N的随机数组
	long N=10000;
	double *x = new double[N];
	RandomDistribution(0,1,N,0,x);
	MatGaussDist(x,N);//用matlab画出分布图

	cout <<"Press any key to exit!" <<endl;
	cin.get();    
}

//////////////////////////////////////////////////////////////////////////
//功能：仿真陀螺数据
//输入：
//输出：
//注意：
//日期：2016.10.11
//////////////////////////////////////////////////////////////////////////
void Simulation::SimGyro()
{}


