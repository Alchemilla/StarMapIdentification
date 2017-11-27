#include"GeoHarris.h"
#include "math.h"
#include "stdio.h"

GeoHarris::GeoHarris(){}
GeoHarris::~GeoHarris(void){}


////////////////////////////////////////////////////////////////////////// 
// Harris�ǵ���ȡ
// ����:      
//          double* I						�Ҷ�ͼ�ĻҶ�ֵ����   
//          int     width					�Ҷ�ͼ�Ŀ��   
//          int     height					�Ҷ�ͼ�ĸ߶�   
//          int     gausswidth				��˹�˲���Ȳ���   
//          double  sigma					��˹�˲��Ĳ���   
//          int     size					�ǵ�����   
//          int     thresh					R����ֵ      
//          vector<StrCornerPoint> &point	��ȡ�Ľǵ�����
////////////////////////////////////////////////////////////////////////// 
int GeoHarris::Harris(double* I,int width, int height, int gw, double sigma,
					int size, int thresh, vector<MatchPoint> &point)
{   
	// ��սǵ�
	point.clear();
	point.reserve(5000);
    int i,j;   
    //����I��Ix��Ix2��Iy��Iy2��Ixy��cim��mx��corner����   
	long wh = width*height;
    double *Ix  = new double[wh];   
    double *Ix2 = new double[wh];   
    double *Iy  = new double[wh];   
    double *Iy2 = new double[wh];   
    double *Ixy = new double[wh];   
    double *cim = new double[wh];   
    double *mx=new double[wh];               
    //������Է������Ԫ��   
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
    //  ��һ�������ò�����Ӷ�ͼ������˲�
	//  (ȷ��ͼ��ı߽磬ȷ��Ix^2,Iy^2,Ix*Iy)   
    /////////////////////////////////////////////////    
    //����ˮƽ���������Ӳ���Ix   
    double dx[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};   
    Mbys(I, Ix, width, height, dx, 3, 3);    
    //���崹ֱ���������Ӳ���Iy   
    double dy[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};    
    Mbys(I, Iy, width, height, dy, 3, 3);       
    //����Ix2��Iy2��Ixy   
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
    //  �ڶ�������Ix2/Iy2/Ixy���и�˹ƽ������ȥ������   
    /////////////////////////////////////////////////         
    //������ʹ��5��5�ĸ�˹ģ��   
    //����ģ�����   
    double* g = new double[gw*gw];   
    for(i=0;i<gw;i++)
	{
        for(j=0;j<gw;j++) 
		{
            g[i*gw+j]=exp(-((i-int(gw/2))*(i-int(gw/2))+(j-int(gw/2))*(j-int(gw/2)))/(2*sigma));  
		}
	}      
    // ��һ����ʹģ�����֮��Ϊ1����ʵ�˲�����ʡ�ԣ�   
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
   
    // ���и�˹ƽ��          
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
    // ������������ǵ���   
    /////////////////////////////////////////////////    
    //����cim����cornerness of image�����ǰ����������ǵ�����   
    for(i=0; i<height; i++)   
    {   
        for(j=0; j<width; j++)   
        {   
            //ע�⣺Ҫ�ڷ�ĸ�м���һ����С���Է�ֹ����Ϊ�����   
            cim(i,j) = (Ix2(i,j)*Iy2(i,j) - Ixy(i,j)*Ixy(i,j))/(Ix2(i,j) + Iy2(i,j) + 0.000001);        
        }   
    }
    /////////////////////////////////////////////////  
    // ���Ĳ������оֲ��Ǽ���ֵ�����Ի�����սǵ�   
    /////////////////////////////////////////////////        
    //ע����оֲ�����ֵ���Ƶ�˼·      
    int m,n;     
    double max;   
    //��ÿ������������������ֵ�˲��������õ��ֵ��Ϊ�����������Ǹ�ֵ������ֵ�˲��е����ƣ�   
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
    //����ȷ���ǵ�
	MatchPoint pointtemp;
    for(i=0; i<height; i++)   
    {   
        for(j=0; j<width; j++)   
        {      
            if(cim(i,j)==mx(i,j))  //����ȡ�þֲ�����ֵ   
			{
				if(mx(i,j)>thresh)  //Ȼ����������ֵ   
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
//����ģ�����㺯��
//im������ͼ��  tp��ģ�����
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
            //ȥ�������߽����   
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
