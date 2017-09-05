#pragma once

#include<algorithm>
#include <math.h>

#define MAXIMUM 1024000000
#define MINIMUM -1024000000

#define PI 3.14159265358979323846264338327950288419716939937510



template<class T>
void  __declspec(dllexport) VectorMinus(T *M,T *M1,T *M2)
{
	M2[0]=M[0]-M1[0];  
	M2[1]=M[1]-M1[1]; 
	M2[2]=M[2]-M1[2]; 
}


//matrix  M2=M-M1   (double , float) - (double , float) = (double , float) 矩阵相减
template<class T1,class T2,class T3>
void  __declspec(dllexport) MatrixMinus(T1* M,T2* M1,T3* M2,int nrows,int ncols)
{//M - nrows*ncols    M1 - nrows*ncols   M2 - nrows*ncols
	for(int i=0;i<nrows;++i)
		for(int j=0;j<ncols;++j)
			M2[i*ncols+j]=M[i*ncols+j]-M1[i*ncols+j];   
}

// (float,double) * (float,double) =  (double , float) 矩阵相乘
template<class T1,class T2,class T3>
void  __declspec(dllexport) MultMatrix(T1* M,T2* M1,T3* M2,int rows,int cols)
{//M - rows*cols    M1 - cols*rows   M2 - rows*rows
	for(int i=0;i<rows;++i)
		for(int j=0;j<rows;++j)
		{
			M2[i*rows+j]=0.0;
			for(int k=0;k<cols;++k)
				M2[i*rows+j]+=M[i*cols+k]*M1[k*rows+j];
		}  
}


// (float,double) * (float,double) =  (double , float) 矩阵相乘
template<class T1,class T2,class T3>
void  __declspec(dllexport) MultMatrix(T1* M,T2* M1,T3* M2,int rows,int soms,int cols)
{
	//M - rows*soms    M1 - soms*cols   M2 - rows*cols
	for(int i=0;i<rows;++i)
		for(int j=0;j<cols;++j)
		{
			M2[i*cols+j]=0.0;
			for(int k=0;k<soms;++k)
				M2[i*cols+j]+=M[i*soms+k]*M1[k*cols+j];
		} 
}

// (float,double) * (float,double) =  (double , float) 矩阵相乘
template<class T1,class T2,class T3>
void  __declspec(dllexport) MultMatrix(T1** M,T2* M1,T3* M2,int rows,int soms,int cols)
{
	//M - rows*soms    M1 - soms*cols   M2 - rows*cols
	for(int i=0;i<rows;++i)
		for(int j=0;j<cols;++j)
		{
			M2[i*cols+j]=0.0;
			for(int k=0;k<soms;++k)
				M2[i*cols+j]+=M[i][k]*M1[k*cols+j];
		} 
}


//matrix[3][4] change to  matrix0[4][3] 矩阵转置
template<class T,class T0>
void __declspec(dllexport) MatrixTranspose(T* matrix,T0* matrix0,int nrows,int ncols)
{// matrix - nrows*ncols    matrix0 - ncols*nrows
	for(int i=0;i<nrows;++i)
		for(int j=0;j<ncols;++j)
			matrix0[j*nrows+i]=matrix[i*ncols+j];
}

/// <summary>   
/// 得到单位矩阵
/// </summary>     
/// <param name="matrix">矩阵</param> 
/// <param name="nrows">阶数</param>
template<class T>
void MatrixIdentity(T* matrix,int nrows)
{ //  matrix -  nrows*nrows
	memset(matrix,0,nrows*nrows*sizeof(T));
	for(int i=0;i<nrows;++i)
		matrix[i*nrows+i]=1;
}



//DooliTle max elements solve equations 
//DooliTle列主元消元法求解多个多元一次线性方程组  求解矩阵 
template<class T1,class T2,class T3>//>
void DooliTle(T1* aa,T2* bb,T3* xx,int rows)
{// aa * xx = bb        root - xx[rows][rows]
	int k,i,j,t,ik;
	int* M=new int[rows];
	double  *s,*l,*u,*a,*b;
	double temp,smax=0,*y,*x;
	s=new double[rows];
	l=new double[rows*rows];
	u=new double[rows*rows];
	a=new double[rows*rows];
	b=new double[rows];
	y=new double[rows];
	x=new double[rows];
	//  QA  =  LU
	for(i=0;i<rows;++i)
	{
		M[i]=0;
		for(j=0;j<rows;++j)
		{
			a[i*rows+j]=aa[i*rows+j];
		}
	}
	for(k=0;k<rows;++k)
	{
		for(i=k;i<rows;++i)
		{
			s[i]=a[i*rows+k];
			for(t=0;t<k;++t)
				s[i]-=l[i*rows+t]*u[t*rows+k];
			if(i==k)
			{
				smax=s[i];
				ik=i;
			}
			if(fabs(smax)<fabs(s[i]))
			{
				smax=s[i];
				ik=i;
			}
		}
		M[k]=ik;
		if(ik!=k)
		{
			for(t=0;t<k;++t)
			{
				temp=l[k*rows+t];
				l[k*rows+t]=l[ik*rows+t];
				l[ik*rows+t]=temp;
			}
			for(t=k;t<rows;++t)
			{
				temp=a[k*rows+t];
				a[k*rows+t]=a[ik*rows+t];
				a[ik*rows+t]=temp;
			}
			temp=s[k];
			s[k]=s[ik];
			s[ik]=temp;
		}
		u[k*rows+k]=s[k];
		if(k<rows-1)
		{
			for(j=k+1;j<rows;++j)
			{
				u[k*rows+j]=a[k*rows+j];
				for(t=0;t<k;++t)
					u[k*rows+j]-=l[k*rows+t]*u[t*rows+j];
			}
			for(i=k+1;i<rows;++i)
				l[i*rows+k]=s[i]/(u[k*rows+k]+0.00001);
		}
	}
	//Qb  =  Ly   AND   Ux  =   y
	for(j=0;j<rows;++j)
	{
		for(i=0;i<rows;++i)
			b[i]=bb[i*rows+j];
		for(k=0;k<rows-1;++k)
		{
			t=M[k];
			temp=b[k];
			b[k]=b[t];
			b[t]=temp;
		}
		y[0]=b[0];
		for(i=1;i<rows;++i)
		{
			y[i]=b[i];
			for(t=0;t<i;++t)
				y[i]-=l[i*rows+t]*y[t];     
		}
		x[rows-1]=y[rows-1]/(u[rows*rows-1]+0.00001);
		for(i=rows-2;i>-1;--i)
		{
			x[i]=y[i];
			for(t=i+1;t<rows;++t)
				x[i]-=u[i*rows+t]*x[t];
			x[i]/=(u[i*rows+i]+0.00001);
		} 
		for(i=0;i<rows;++i)
		{
			xx[i*rows+j]=x[i];
		}           
	}
	delete[]M;
	delete[]s;
	delete[]l;
	delete[]u;
	delete[]a;
	delete[]b;
	delete[]y;
	delete[]x;
	M=NULL;
	s=NULL;
	l=NULL;
	u=NULL;
	a=NULL;
	b=NULL;
	y=NULL;
	x=NULL;
}


/// <summary>   
/// 矩阵求逆
/// </summary>     
/// <param name="Matrix">矩阵</param> 
/// <param name="MatrixA">逆矩阵</param>
/// <param name="rows">阶数</param>
//////////////////////////////////////////////////////////////////////////
// invertible matrice
template<class T1,class T2>
void MatrixAnti(T1* Matrix,T2* MatrixA,int rows)
{//  Matrix * MatrixA = I          I = E
	double* E=new double[rows*rows]; 
	MatrixIdentity(E,rows); 

	//DooliTle solution
	DooliTle(Matrix,E,MatrixA,rows); 
	delete[]E;
	E=NULL;
}

template<class T>
void GetWu1(T Wu1[], const T *matrixB, const T *l, int N, int U)
{
	T *transposedB = new T[N*U];
	MatrixTranspose(matrixB, transposedB, N, U);
	MultMatrix(transposedB, l, Wu1, U, N, 1);

	delete[] transposedB;
}

template<class T>
void GetNBB(T nbb[], const T *matrixB, int N, int U)
{
	T *transposedB = new T[N*U];
	MatrixTranspose(matrixB, transposedB, N, U);
	MultMatrix(transposedB, matrixB, nbb, U, N, U);

	delete[] transposedB;
}

template<class T>
void GetNcc(T Ncc[], const T matrixA[], const T nbb[], int S, int U)
{
	T *inverseForNbb = new T[U*U];
	T *transposedA = new T[S*U];
	T *transit = new T[S*U];
	MatrixTranspose(matrixA, transposedA, S, U);
	MatrixAnti(nbb, inverseForNbb, U);
	MultMatrix(matrixA, inverseForNbb, transit, S, U, U);
	MultMatrix(transit, transposedA, Ncc, S, U, S);

	delete[] inverseForNbb;
	delete[] transposedA;
	delete[] transit;
}

/// <summary>   
/// 间接平差
/// </summary>     
/// <param name="correction">返回的改正数</param> 
/// <param name="matrixB"></param>
/// <param name="l"></param>
/// <param name="N"></param>
/// <param name="U"></param>
//////////////////////////////////////////////////////////////////////////
template<class T>
void GetCorrection(T correction[], const T *matrixB, const T *l, int N, int U)
{
	T *nbb = new T[U*U];
	T *inverseForNbb = new T[U*U];
	T *Wu1 = new T[U];

	GetNBB(nbb, matrixB, N, U);
	MatrixAnti(nbb, inverseForNbb, U);
	GetWu1(Wu1, matrixB, l, N, U);

	MultMatrix(inverseForNbb, Wu1, correction, U, U, 1);

	delete[] nbb;
	delete[] inverseForNbb;
	delete[] Wu1;
}

/// <summary>   
/// 附有限制条件的间接平差
/// </summary>     
/// <param name="correction">返回的改正数</param> 
/// <param name="MatrixA"></param>
/// <param name="w"></param>
/// <param name="matrixB"></param>
/// <param name="l"></param>
/// <param name="N"></param>
/// <param name="U"></param>
/// <param name="S"></param>
//////////////////////////////////////////////////////////////////////////
template<class T>
void GetCorrectionWithCondition(T correction[], const T matrixA[], const T w[], const T *matrixB, const T *l, int N, int U, int S)
{
	T *nbb = new T[U*U];
	T *inverseForNbb = new T[U*U];
	T *transposedA = new T[S*U];
	T *Ncc = new T[S*S];
	T *inverseForNcc = new T[S*S];
	T *Wu1 = new T[U];

	GetNBB(nbb, matrixB, N, U);
	MatrixAnti(nbb, inverseForNbb, U);
	MatrixTranspose(matrixA, transposedA, S, U);
	GetNcc(Ncc, matrixA, nbb, S, U);
	MatrixAnti(Ncc, inverseForNcc, S);
	GetWu1(Wu1, matrixB, l, N, U);

	T *transit1 = new T[S*U];

	MultMatrix(inverseForNbb, transposedA, transit1, U, U, S);

	T *transit2 = new T[S*U];

	MultMatrix(transit1, inverseForNcc, transit2, U, S, S);

	T *transit3 = new T[U*U];

	MultMatrix(transit2, matrixA, transit3, U, S, U);

	T *transit4 = new T[U*U];

	MultMatrix(transit3, inverseForNbb, transit4, U, U);

	T *transit5 = new T[U*U];

	MatrixMinus(inverseForNbb, transit4, transit5, U, U);

	T *transit6 = new T[U];

	MultMatrix(transit5, Wu1, transit6, U, U, 1);

	T *transit7 = new T[S*U];

	MultMatrix(inverseForNbb, transposedA, transit7, U, U, S);

	T *transit8 = new T[S*U];

	MultMatrix(transit7, inverseForNcc, transit8, U, S, S);

	T *transit9 = new T[U];

	MultMatrix(transit8, w, transit9, U, S, 1);

	MatrixMinus(transit6, transit9, correction, U, 1);

	delete[] nbb;
	delete[] inverseForNbb;
	delete[] transposedA;
	delete[] Ncc;
	delete[] inverseForNcc;
	delete[] Wu1;
	delete[] transit1;
	delete[] transit2;
	delete[] transit3;
	delete[] transit4;
	delete[] transit5;
	delete[] transit6;
	delete[] transit7;
	delete[] transit8;
	delete[] transit9;
}