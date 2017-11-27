#include"GeoHarris.h"
#include "math.h"
#include "stdio.h"

GeoHarris::GeoHarris(){}
GeoHarris::~GeoHarris(void){}


////////////////////////////////////////////////////////////////////////// 
// Harris角点提取
// 输入:      
//          double* I						灰度图的灰度值矩阵   
//          int     width					灰度图的宽度   
//          int     height					灰度图的高度   
//          int     gausswidth				高斯滤波宽度参数   
//          double  sigma					高斯滤波的参数   
//          int     size					角点区域   
//          int     thresh					R的阈值      
//          vector<StrCornerPoint> &point	提取的角点坐标
////////////////////////////////////////////////////////////////////////// 
int GeoHarris::Harris(double* I,int width, int height, int gw, double sigma,
					int size, int thresh, vector<MatchPoint> &point)
{   
	// 清空角点
	point.clear();
	point.reserve(5000);
    int i,j;   
    //创建I、Ix、Ix2、Iy、Iy2、Ixy、cim、mx、corner数组   
	long wh = width*height;
    double *Ix  = new double[wh];   
    double *Ix2 = new double[wh];   
    double *Iy  = new double[wh];   
    double *Iy2 = new double[wh];   
    double *Ixy = new double[wh];   
    double *cim = new double[wh];   
    double *mx=new double[wh];               
    //定义宏以方便访问元素   
    #define I(ROW,COL) I[width*(ROW)+(COL)]   
    #define Ix(ROW,COL) Ix[width*(ROW)+(COL)]   
    #define Ix2(ROW,COL) Ix2[width*(ROW)+(COL)]   
    #define Iy(ROW,COL) Iy[width*(ROW)+(COL)]   
    #define Iy2(ROW,COL) Iy2[width*(ROW)+(COL)]   
    #define Ixy(ROW,COL) Ixy[width*(ROW)+(COL)]   
    #define cim(ROW,COL) cim[width*(ROW)+(COL)]   
    #define mx(ROW,COL) mx[width*(ROW)+(COL)]   
    #define corner(ROW,COL) corner[width*(ROW)+(COL)]   
   
    ///////////////////////////////////////////////// 
    //  第一步：利用差分算子对图像进行滤波
	//  (确定图像的边界，确定Ix^2,Iy^2,Ix*Iy)   
    /////////////////////////////////////////////////    
    //定义水平方向差分算子并求Ix   
    double dx[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};   
    Mbys(I, Ix, width, height, dx, 3, 3);    
    //定义垂直方向差分算子并求Iy   
    double dy[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};    
    Mbys(I, Iy, width, height, dy, 3, 3);       
    //计算Ix2、Iy2、Ixy   
    for(i = 0; i < height; i++)   
    {   
        for(j = 0; j < width; j++)   
        {      
            Ix2(i,j)=Ix(i,j)*Ix(i,j);   
            Iy2(i,j)=Iy(i,j)*Iy(i,j);   
            Ixy(i,j)=Ix(i,j)*Iy(i,j);   
        }   
    }      
    /////////////////////////////////////////////////    
    //  第二步：对Ix2/Iy2/Ixy进行高斯平滑，以去除噪声   
    /////////////////////////////////////////////////         
    //本例中使用5×5的高斯模板   
    //计算模板参数   
    double* g = new double[gw*gw];   
    for(i=0;i<gw;i++)
	{
        for(j=0;j<gw;j++) 
		{
            g[i*gw+j]=exp(-((i-int(gw/2))*(i-int(gw/2))+(j-int(gw/2))*(j-int(gw/2)))/(2*sigma));  
		}
	}      
    // 归一化：使模板参数之和为1（其实此步可以省略）   
    double total=0;   
    for(i=0;i<gw*gw;i++)
	{
        total += g[i]; 
	}
    for(i=0;i<gw;i++)  
	{
        for(j=0;j<gw;j++)  
		{
            g[i*gw+j] /= total;  
		}
	}
   
    // 进行高斯平滑          
    double* Ix21 = new double[wh];   
    double* Iy21 = new double[wh];   
    double* Ixy1 = new double[wh];    
    for(i=0; i<width; i++)   
    {   
        for(j=0; j<height; j++)   
        {   
            Ix21[i*height+j] = Ix2[i*height+j];   
            Iy21[i*height+j] = Iy2[i*height+j];   
            Ixy1[i*height+j] = Ixy[i*height+j];   
        }   
    }   
    Mbys(Ix21, Ix2, width, height, g, gw, gw);   
    Mbys(Iy21, Iy2, width, height, g, gw, gw);   
    Mbys(Ixy1, Ixy, width, height, g, gw, gw);   
    delete []Ix21;   
    delete []Iy21;   
    delete []Ixy1;         
    /////////////////////////////////////////////////   
    // 第三步：计算角点量   
    /////////////////////////////////////////////////    
    //计算cim：即cornerness of image，我们把它称做‘角点量’   
    for(i=0; i<height; i++)   
    {   
        for(j=0; j<width; j++)   
        {   
            //注意：要在分母中加入一个极小量以防止除数为零溢出   
            cim(i,j) = (Ix2(i,j)*Iy2(i,j) - Ixy(i,j)*Ixy(i,j))/(Ix2(i,j) + Iy2(i,j) + 0.000001);        
        }   
    }
    /////////////////////////////////////////////////  
    // 第四步：进行局部非极大值抑制以获得最终角点   
    /////////////////////////////////////////////////        
    //注意进行局部极大值抑制的思路      
    int m,n;     
    double max;   
    //对每个点在邻域内做极大值滤波：即将该点的值设为邻域中最大的那个值（跟中值滤波有点类似）   
    for(i = 0; i < height; i++)   
    {   
        for(j = 0; j < width; j++)   
        {   
            max=-1000000;   
            if(i>int(size/2) && i<height-int(size/2) && j>int(size/2) && j<width-int(size/2))
			{
                for(m=0;m<size;m++)   
                {   
                    for(n=0;n<size;n++) 
					{
                        {                          
                            if(cim(i+m-int(size/2),j+n-int(size/2))>max)   
                                max=cim(i+m-int(size/2),j+n-int(size/2));   
                        }   
					}
                }   
			}
            if(max>0)   
                mx(i,j)=max;   
            else   
                mx(i,j)=0;   
        }   
    }   
    //最终确定角点
	MatchPoint pointtemp;
    for(i=0; i<height; i++)   
    {   
        for(j=0; j<width; j++)   
        {      
            if(cim(i,j)==mx(i,j))  //首先取得局部极大值   
			{
				if(mx(i,j)>thresh)  //然后大于这个阈值   
				{   
					pointtemp.lx = j;
					pointtemp.ly = i;
					point.push_back(pointtemp);  
				}   
			}
        }   
    }   
    if(Ix!=NULL)	delete []Ix;	Ix = NULL;
	if(Iy!=NULL)	delete []Iy;	Iy = NULL;
	if(Ix2!=NULL)	delete []Ix2;	Ix2 = NULL;
	if(Iy2!=NULL)	delete []Iy2;	Iy2 = NULL;
	if(Ixy!=NULL)	delete []Ixy;	Ixy = NULL;
	if(cim!=NULL)	delete []cim;	cim = NULL;
	if(mx!=NULL)	delete []mx;	mx = NULL;
	if(g!=NULL)		delete []g;		g = NULL;
    return 1;   
}   
   

////////////////////////////////////////////////////////////////////////// 
//定义模板运算函数
//im：输入图像  tp：模板参数
////////////////////////////////////////////////////////////////////////// 
int GeoHarris::Mbys(double * im,double* out,int imW,int imH,double *tp,int tpW,int tpH)   
{     
    int i,j,m,n;   
    #define im(ROW,COL) im[imW*(ROW)+(COL)]   
    #define tp(ROW,COL) tp[tpW*(ROW)+(COL)]   
    #define out(ROW,COL) out[imW*(ROW)+(COL)]   
    double a;   
    for(i=0;i<imH;i++)   
	{
        for(j=0;j<imW;j++)   
        {   
            a = 0;
            //去掉靠近边界的行   
            if(i>int(tpH/2) && i<imH-int(tpH/2) && j>int(tpW/2) && j<imW-int(tpW/2))
			{
                for(m=0;m<tpH;m++)  
				{
                    for(n=0;n<tpW;n++)   
                    {   
                        a += im(i+m-int(tpH/2),j+n-int(tpW/2))*tp(m,n);   
                    }
				}
			}
            out(i,j) = a; 
        } 
	}
    return 1;   
}   
