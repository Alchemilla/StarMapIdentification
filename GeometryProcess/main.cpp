// GeometryProcess.cpp : Defines the entry point for the console application.
//
#include <time.h>
#include <vector>
#include<iostream>
#include <iomanip>
#include "WorkFlow_ZY3.h"
#include <windows.h>
#pragma comment(lib, "winmm.lib")
using namespace std;

int main(int argc, char* argv[])
{
	string argv2 = "C:\\Users\\wcsgz\\Documents\\2-CProject\\6-严密模型\\ExtDlls\\EOP00.txt";
	string strDEM = "C:\\Users\\wcsgz\\Documents\\5-工具软件\\几何精度检校v5.0\\全球DEM.tif";
	if (argc==4)
	{		
		WorkFlow_ZY3 *pflow = new WorkFlow_ZY3();
		pflow->GetEOP(argv2); pflow->GetDEM(strDEM);

		//设置小面阵相机参数
		StrCamParamInput caminput;
		caminput.f = 1.524;
		caminput.Xsize = 5.5 / 1e6;		caminput.Ysize = 5.5 / 1e6;
		caminput.Xnum = 256;			caminput.Ynum = 256;
		caminput.Xstart = -128;			caminput.Ystart = -128;
		pflow->SetCamInput(caminput);
		pflow->GetPitch(-35.2, 22);


		if (atoi(argv[4]) == 1)//采用点投影法验证交会精度
		{
			//////////////////////////////////////////////////////////////////////////
			//功能：以下为小面阵相机仿真与验证
			//日期：2017.08.14
			/////////////////////////////////////////////////////////////////////////
			//pflow->LittleArrayCamera(argv[3]);//根据模型自己生产rpc	
			//pflow->Image_registration_rpc(argv[3]);//根据siftGPU来匹配控制点
			pflow->CalcRealMatchPoint(argv[3]);//根据不带误差的模型生产一定数量控制点,目前可以在无差点上加系统差
			//pflow->CalcRealAttitude(argv[3]);//根据第一帧和匹配点递推姿态
			//pflow->CalcOmegaKalman(argv[1]);//根据匹配点采用卡尔曼滤波计算姿态
			//pflow->CalcRealAttitude_sparse(argv[1]);
			pflow->CalcModifyAttitude(argv[3]);//根据第一帧和匹配点递推姿态

			//////////////////////////////////////////////////////////////////////////
			//功能：以下为正视前后视相机仿真与验证
			//日期：2017.09.07
			/////////////////////////////////////////////////////////////////////////
			//plow->NADCamera(argv[1]);
			//pflow->ModelVerify();
			//pflow->FwdBwdModelVerify(argv[1], -22. / 180 * PI, 0);//验证模型精度
			pflow->CalcFwdBwdRealMatchPoint(argv);//根据前后视不带误差模型计算真实匹配点
			pflow->CalcFwdBwdIntersection(argv);//对前后视进行前方交会，然后解求精度
			pflow->ChangeAttPath(argv);//改变姿态文件
			pflow->CalcFwdBwdIntersection(argv);//对前后视进行前方交会，然后解求精度
		}
		else if (atoi(argv[4]) == 2)//采用立体平差程序给出结果
		{
			//对小面阵处理
			//pflow->CalcRealMatchPoint(argv[3]);//根据不带误差的模型生产一定数量控制点,目前可以在无差点上加系统差
			//pflow->CalcModifyAttitude(argv[3]);//根据第一帧和匹配点递推姿态
			//对前后视处理
			//pflow->FwdBwdModelVerify(argv[1], -22. / 180 * PI, 1);//验证模型精度
			pflow->CalcFwdBwdRealMatchPoint(argv);//根据前后视不带误差模型计算真实匹配点
			//pflow->CalcFwdBwdRPC(argv);
			pflow->Calc3DAccuracyByAdjustment(argv);//解求精度
			pflow->ChangeAttPath(argv);//改变姿态文件
			pflow->CalcFwdBwdRPC(argv);
			pflow->Calc3DAccuracyByAdjustment(argv);//解求精度
		}
		if (pflow != NULL)		delete[]pflow;		pflow = NULL;
	}
	else
	{
		//////////////////////////////////////////////////////////////////////////
		//功能：以下为资三02星定标和定位模型
		//日期：2017.04.25
		/////////////////////////////////////////////////////////////////////////
		WorkFlow_ZY3 *pflow2 = new WorkFlow_ZY3();
		//外方位元素检校
		//pflow2->CalibrationModel2(argv[1], argv2);
		//严密模型构建
		pflow2->GenerateRigorousModel(argv[1], argv2);
		pflow2->AccuracyVerify(argv[1]);
		//pflow2->ModelVerify();
		if (pflow2 != NULL)		delete[]pflow2;		pflow2 = NULL;

		//////////////////////////////////////////////////////////////////////////
		//功能：以下为资三01星定标和定位模型
		//日期：2016.12.21
		/////////////////////////////////////////////////////////////////////////
		//WorkFlow_ZY3 *pflow = new WorkFlow_ZY3();
		//外方位元素检校
		//pflow->CalibrationModel(argv[1],argv[2]);
		//严密模型构建
		//pflow->GenerateRigorousModel(argv[1],argv[2]);
		//double lat, lon, h, x, y, lat1, lon1, lat2, lon2, lat3, lon3;
		//h = 0;
		//FromXY2LatLon(x,y,h,lat,lon),x是line，y是sample
		//pflow->pModel->FromXY2LatLon( 7183, 3256,h, lat, lon);
		//lat1 = lat*180/PI;
		//lon1 = lon*180/PI;
		//pflow->pModel->FromXY2LatLon(14375, 1793,  h, lat, lon);
		//lat2 = lat*180/PI;
		//lon2 = lon*180/PI;
		//pflow->pModel->FromXY2LatLon(15279, 16913, h, lat, lon);
		//lat3 = lat*180/PI;
		//lon3 = lon*180/PI;

		//河南381轨
		//h = 326.5;
		//pflow->pModel->FromXY2LatLon(7579, 5267, h, lat, lon);
		//lat1 = lat*180/PI;
		//lon1 = lon*180/PI;
		//pflow->pModel->FromXY2LatLon(1571, 11433,  h, lat, lon);
		//lat2 = lat*180/PI;
		//lon2 = lon*180/PI;
		//pflow->pModel->FromXY2LatLon(2741, 22080, h, lat, lon);
		//lat3 = lat*180/PI;
		//lon3 = lon*180/PI;
		//pflow->pModel->FromXY2LatLon(5383, 14532, h, lat, lon);
		//lat1 = lat*180/PI;
		//lon1 = lon*180/PI;	
		//h = 413.3;
		//pflow->pModel->FromXY2LatLon(1683, 5562,  h, lat, lon);
		//lat2 = lat*180/PI;
		//lon2 = lon*180/PI;
		////pflow->pModel->FromLatLon2XY(lat, lon, h, x, y);
	}

	PlaySound(TEXT("C:\\WINDOWS\\Media\\Alarm01.wav"), NULL, NULL);
	return 0;
}

