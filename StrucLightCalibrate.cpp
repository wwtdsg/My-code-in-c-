#include <atlstr.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fstream> //文件输入输出流搜索
#include <list>
#include <vector>
#include "opencv2/core/core_c.h"
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

float f = 12.0f; //相机焦距，单位：mm
float dx;		//像元x方向长度
float dy;		//像元y方向长度
CvMat * intrinsic;
CvMat * distortion;

//————————结构光标定使用的变量——————————
vector<float* > cline; //保存单幅图像的角点线，每幅图5条
float struct_line[4];  //存储单条结构光直线
IplImage* show;
IplImage* show_test;
IplImage* show_corner;

int number_image = 32;
char filename[300];
char* adrSrcImage="SrcImage//%d.bmp";                //源图像文件夹地址
float d=1.76f;                               //棋盘格物理尺寸。 
CvSize board_size=cvSize(5,3);                       //棋盘规格width*height
int board_width=board_size.width;
int board_height=board_size.height;
int total_per_image=board_width*board_height;
int iterations=100;               //迭代次数
double accuracy=0.01;              //收敛精度
int successes = 0;


char* adrDstImage="DstImage//%d.bmp";         //畸变校正后的图像文件夹地址
char* adrTempImage="TempImage//%d.bmp";       //过程图像文件夹地址

CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];        //单张图像的角点，图像坐标

CvMat * image_points=cvCreateMat(number_image*total_per_image,2,CV_32FC1);  //图像坐标系
CvMat * object_points=cvCreateMat(number_image*total_per_image,3,CV_32FC1); //世界坐标系
CvMat * point_counts=cvCreateMat(number_image,1,CV_32SC1);         //每张图像的角点数量

FILE* fp = fopen("DataResult.txt","w+");      //保存数据的txt文件地址
FILE* fpInfo=fopen("Information.txt","w+");   //保存异常信息
FILE* fpTest=fopen("LightErrorTest.txt","w+");//保存异常信息

float planeLight[4];                             //光平面。
vector<CvPoint3D32f> point_c;									//存储所有图像的交点的摄像机坐标

vector<CvPoint3D32f> testline;

/*
结构光平面标定设计分析：

步骤：
1.为标定方便，便于提取结构光平面，采用特制标定板。
2.根据相机标定得到的畸变系数，对图像进行畸变校正。
3.求取结构光直线方程（l1）（图像像素坐标系）。
4.求取标定板角点坐标（单位：像素；坐标系：图像像素坐标系）。
5.对每行的三个角点进行直线拟合，得到角点线方程（l2）（图像像素坐标系）。
6.由角点线方程（l2）和结构光直线方程（l1）求取交点（p）的图像像素坐标坐标
7.根据相机标定得到的内参，将三个角点（a，b，c）和一个交点（p）的图像像素坐标映射为相机坐标（单位：mm）。
8.据此分别求得向量的夹角a1（角BOC）、a2（角AOC）以及a3（角COP）的值。
9.由角a1/a2/a3和A、B、C两两之间的距离d，求得|OP|的值。
10.再由向量Op的方向，即可得到P点的相机坐标。
11.分别对每一行角点线重复5~10步，可在一副标定图上求得结构光光线上的多个点的相机坐标（x, y, z），
12.对多个空间点拟合成一条相机坐标系的空间直线 / 存储所有点
13.分别对每一幅结构光标定图重复2~12步，得到结构光平面上的多个直线方程 / 存储所有点
14.对多个直线方程进行拟合，得到结构光平面方程 / 对13步得到的所有点进行平面拟合，得到结构光平面方程
15.结构光平面标定完成。

*/


/*
结构光提取中心线:

需进行中值滤波，二值化，开运算
再提取中心线，得到直线方程
*/



//求两直线端点
void twoEndsOfLine( float* pline, CvPoint2D32f &startpt, CvPoint2D32f &endpt)
{
	double k=pline[1]/pline[0];
	double d=pline[3]-k*pline[2];
	startpt.x=5;
	startpt.y=k*startpt.x+d;
	endpt.x=635;
	endpt.y=k*endpt.x+d;
}

//矫正图像
void adjustImage(int a)
{
	//矫正畸变，测试效果。
	IplImage * mapx=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * mapy=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * clone;
	//默认校准一次各图片


		//按内参校准图像
		cvInitUndistortMap(intrinsic,distortion,mapx,mapy);
		clone=cvCloneImage(show);
		cvRemap(clone,show,mapx,mapy);

		sprintf_s (filename,adrDstImage,a);  // 保存目标图像 
		cvSaveImage(filename,show);

	cvReleaseImage(&clone);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
}

void getCornerPoints(IplImage* show, IplImage* gray_image, int a)
{
	int count;
	int found=cvFindChessboardCorners(show,board_size,image_points_buf,&count,
			/*0*/CV_CALIB_CB_ADAPTIVE_THRESH/*|CV_CALIB_CB_FILTER_QUADS*//*|CV_CALIB_CB_NORMALIZE_IMAGE*/ );
	if(found==0)
	{       
		fprintf(fpInfo, "第 %d 帧图片无法找到棋盘格所有角点!count=%d\n",a,count);
		//角点亚像素矫正
	}
	else
	{
		fprintf(fpInfo, "第 %d 帧图像成功获得 %d 个角点!\n",a,count);			
		//角点亚像素矫正
		cvFindCornerSubPix(show,image_points_buf,count,board_size,cvSize(-1,-1),
			cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,iterations,accuracy));
		fprintf(fpInfo, "灰度图亚像素化过程完成...\n");
		//绘制角点
		cvDrawChessboardCorners(gray_image,board_size,image_points_buf,count,found);
		fprintf(fpInfo, "在源图像上绘制角点过程完成...\n\n");
		//保存过程图像
		sprintf_s (filename,adrTempImage,a);
		cvSaveImage(filename,gray_image);
	}
}

//拟合角点线
void fitCornerLine(CvPoint2D32f* image_points_buf, int a)
{	
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* point_seq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
	show_corner = cvCloneImage(show);
	for(int i=0; i<board_width; i++)
	{	
		float* line =  new float[4];
		for(int j=0; j<board_height; j++)
		{
			cvSeqPush(point_seq, &image_points_buf[j*board_width + i]);
			CvPoint corner = cvPoint(image_points_buf[j*board_width + i].x, image_points_buf[j*board_width + i].y);
			cvCircle(show_corner, corner, 6, Scalar(255));
		}
		cvFitLine(point_seq, CV_DIST_L2, 0, 0.01, 0.01, line);
		cline.push_back(line);//将一幅图像中的角点线以vector保存在cline中。
		CvPoint2D32f startpt;
		CvPoint2D32f endpt;
		twoEndsOfLine(line,startpt,endpt);	
		cvLine(show_corner,cvPoint(startpt.x,startpt.y),cvPoint(endpt.x,endpt.y),Scalar(0),1);
		cvClearSeq(point_seq);
	}
}

//二值化
void thresholdOTSU(IplImage* img,IplImage*dstImg,int* outThreshold)
{
	uchar* dataSrc = (uchar*)img->imageData;
	uchar* dataDst = (uchar*)dstImg->imageData;
	int digram[256];
	int height = img->height;
	int width  = img->width;
	int step   = img->widthStep;
	int i, j, pixel, count, s=0;
	uchar m=0;
	double csum=0, variance=0, m1=0, m2=0;
	int g=-1, n=0, n1=0, n2=0;

	for (i=0; i<height; i++)
		for (j=0; j<width; j++)
			digram[dataSrc[i * step + j]]++;

	for (pixel=0; pixel<256; pixel++)
	{
		s += pixel * digram[pixel];
		n += digram[pixel];
	}
	for (count=0; count<256; count++)
	{
		n1 += digram[count];
		if (n1 == 0)
			continue;
		n2 = n - n1;
		if (n2 == 0)
			break;
		csum += (double) count * digram[count];
		m1 = csum / n1;
		m2 = (s - csum) / n2;
		variance = n1 * n2 * (m1 - m2) * (m1 - m2);
		if (variance > g)
		{
			g = variance;
			m = count;
		}
	}

	//根据图像阈值对图像进行二值化  
	*outThreshold = m;
	for (i=0; i<height; i++)
	{
		for (j=0; j<width; j++)
		{
			if (dataSrc[i * img->nChannels * width+img->nChannels * j] >= /*m*/250)
			{	
				dataDst[i * img->nChannels * width+img->nChannels * j] = 255;
				if (img->nChannels == 3)
				{
					dataDst[i * img->nChannels * width+img->nChannels * j + 1] = 255;
					dataDst[i * img->nChannels * width+img->nChannels * j + 2] = 255;
				}
			}
			else
			{
				dataDst[i * img->nChannels * width+img->nChannels * j] = 0;
				if (img->nChannels == 3)
				{
					dataDst[i * img->nChannels * width + img->nChannels * j + 1] = 0;
					dataDst[i * img->nChannels * width + img->nChannels * j + 2] = 0;
				}
			}
		}
	}
}



//求取图像中的中心线
void getCenterLine(IplImage* imageTest, int a)
{

	IplImage* imageDst = cvCreateImage(cvSize(imageTest->width, imageTest->height), 8, 1);
	uchar* dataimage=(uchar*)imageTest->imageData;
	int width = imageTest->width;
	int hight = imageTest->height;
	int stepwide = imageTest->widthStep;
	int right[1000], left[1000];
	char* smooth = "DstImage//%d中值滤波.bmp";
	char* threshold = "DstImage//%d二值化后.bmp";
	char* morphy = "DstImage//%d开运算后.bmp";

	cvSmooth(imageTest,imageTest,CV_MEDIAN,3,3);
	sprintf_s (filename,smooth,a);
	cvSaveImage(filename,imageTest,0);

	int* tempThreshold=new int[1];
	thresholdOTSU(imageTest,imageDst,tempThreshold);
	sprintf_s (filename,threshold,a);
	cvSaveImage(filename,imageDst,0);

	IplImage* tempImg=cvCreateImage(cvSize(imageTest->width,imageTest->height),8,1);
	cvMorphologyEx(imageDst,imageDst,tempImg,NULL,CV_MOP_OPEN,1);
	sprintf_s (filename,morphy,a);
	cvSaveImage(filename,imageDst,0);

	//方向自左至右，列扫描	 
	for (int i=0; i<hight; i++)
	{
		for (int j=0; j<width; j++)
		{
			if ((UINT)dataimage[j + i * stepwide] == 255)   
			{
				left[i] = j;
				break;
			}
			if (j == width-1)
				left[i] = 10000;
		}
	}
	//方向自右至左，列扫描	 
	for (int i=0; i<hight; i++)
	{
		for (int j=width; j>0; j--)
		{
			if ( (UINT)dataimage[j+i*stepwide] == 255)
			{
				right[i] = j;
				break;
			}
			if (j == 0)
				right[i] = 10000;
		}
	}
	//取行扫描中的平均值作为中心线的各点坐标 
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* point_seq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
	for (int i=0; i<hight; i++)
	{
		if (abs((int)right[i]) > width | abs((int)left[i]) > width)              //TODO  2014-11-19 避免连通的，未初始化的值。
			continue;
		cvSeqPush(point_seq, &cvPoint2D32f(0.5 * (right[i] + left[i]), i));
	}
	//拟合直线
	cvFitLine(point_seq, CV_DIST_L2, 0, 0.01, 0.01, struct_line);

	//画出直线
	CvPoint2D32f startpt;
	CvPoint2D32f endpt;
	twoEndsOfLine(struct_line,startpt,endpt);
	cvLine(show_corner,cvPoint(startpt.x,startpt.y),cvPoint(endpt.x,endpt.y),Scalar(0),1);
	/*char* strNameLineImage="DstImage//直线拟合%d.bmp";*/
	//sprintf_s (filename,strNameLineImage,a);   
	//cvSaveImage(filename,show_corner,0);

	if (struct_line[0] == 0)
		struct_line[0] = 0.0001f;

	cvClearSeq(point_seq);
	cvReleaseMemStorage(&storage);

	//得到结构光直线，存储在全局变量struct_line之中
}

//画出两点之间的直线
//cvLine(imageDst,cvPoint(startpt.x,startpt.y),cvPoint(endpt.x,endpt.y),Scalar(0),1);



//求取角点线和结构光中心线的交点
void getCrossOfLines(float* line1, float* line2, CvPoint2D32f &pt)
{
	if (line1[0] == 0)
		line1[0] = 0.0001f;
	if (line2[0] == 0)
		line2[0] = 0.0001f;
	float k1 = line1[1]/line1[0];
	float b1 = line1[3]-k1*line1[2];
	float k2 = line2[1]/line2[0];
	float b2 = line2[3]-k2*line2[2];
	float deltaK = k1-k2;
	if (deltaK == 0)
		deltaK = 0.0001f;
		
	pt.x = (b2-b1) / deltaK;
	pt.y = k1*(pt.x) + b1;
}


//图像坐标系转摄像机坐标系
void imgPTtoCameraPT(CvPoint2D32f &imgPT, CvPoint3D32f &cameraPT)
{
	cameraPT.x = (imgPT.x - CV_MAT_ELEM(*intrinsic, float, 0, 2)) * dx;
	cameraPT.y = (CV_MAT_ELEM(*intrinsic, float, 1, 2) - imgPT.y) * dy; //此处有修改
	cameraPT.z = f;
}

//求取两点之间的夹角
float getAngle(CvPoint3D32f &angle1, CvPoint3D32f &angle2)
{
	float dot_product = angle1.x*angle2.x + angle1.y*angle2.y + angle1.z*angle2.z;
	float module1 = sqrt(angle1.x*angle1.x + angle1.y*angle1.y + angle1.z*angle1.z);
	float module2 = sqrt(angle2.x*angle2.x + angle2.y*angle2.y + angle2.z*angle2.z);
	float angle = acos(dot_product/(module1*module2));
	return angle;
}

//根据多个点拟合光平面
//解奇异方程组 AX+BY+CZ+D=0
void SVD_Solve(double *SA, double *SU, double *SS, double *SV,int count)
{
	CvMat A, U, S, V;
	cvInitMatHeader(&A,count,4,CV_64FC1,SA,CV_AUTOSTEP);
	cvInitMatHeader(&U,4,4,CV_64FC1,SU,CV_AUTOSTEP);       //固定分解成4X4的矩阵。
	cvInitMatHeader(&S,4,4,CV_64FC1,SS,CV_AUTOSTEP);
	cvInitMatHeader(&V,4,4,CV_64FC1,SV,CV_AUTOSTEP);
	cvSVD(&A, &U, &S, &V, CV_SVD_U_T);

}

//获取交点P的摄像机坐标
void getCrossCamera(float &a1, float &a2, float &a3, float &OP, CvPoint3D32f &cross_point_c)
{
	double ta1 = tan(a1);
	double ta2 = tan(a2);
	double ta3 = tan(a3);
	//cout<<"a3------------:"<<a3<<endl;
	if( ta1*2 == ta2)
		OP = d / (tan(a1) * cos(a3));
	else
	{
		double k = ta1 * ta2 / (2 * ta1 - ta2);
		double b = d * k * (ta1 - k) / (ta1 * sqrt(1 + k*k));
		//double abs_b = abs(b);

		//else
		//	b = -abs_b;
		//double k = -6.23;
		//double b = -298.26;
		OP = abs(b / (ta3 + k) * cos(a3));//单位是厘米 /////////此处改动
	}
	cout<<"OP-------------:"<<OP<<endl;
	CvPoint3D32f temp_point;
	float temp_module = sqrt(cross_point_c.x*cross_point_c.x + cross_point_c.y*cross_point_c.y + cross_point_c.z*cross_point_c.z);
	temp_point.x = 10 * OP * cross_point_c.x / temp_module;
	temp_point.y = 10 * OP * cross_point_c.y / temp_module;
	temp_point.z = 10 * OP * cross_point_c.z / temp_module; //单位是 mm
	point_c.push_back(temp_point);//将每个点的摄像机坐标保存到vector中
}

//求解结构光平面：根据解奇异方程组求解光平面系数 AX+ BY+ CZ+ D=0 。
void planSolve()
{
	int lenth = point_c.size();
	if(lenth < 4)
	{
		MessageBox(NULL, TEXT("光条交点数量过少!"), TEXT("光平面求取"), MB_OK);
		return;
	}
	double* A = (double*)malloc(sizeof(double)*4*lenth);
	for (int i=0; i<lenth; i++)
	{
		CvPoint3D32f temp = point_c[i];
		A[4*i] = (double)temp.x;
		A[4*i+1] = (double)temp.y;
		A[4*i+2] = (double)temp.z;
		A[4*i+3] = 1;
	}
	double U[16], S[16], V[16];
	SVD_Solve(A, U, S, V, lenth);
	planeLight[0] = V[3]/V[3];   //系数A
	planeLight[1] = V[7]/V[3];   //系数B
	planeLight[2] = V[11]/V[3];  //系数C
	planeLight[3] = V[15]/V[3];  //系数D	
	fprintf(fpTest, "A=%f ; B=%f ; C=%f ;  D=%f  \n",
		planeLight[0], planeLight[1], planeLight[2], planeLight[3]);
	CvMat* LightPlaneABCD = cvCreateMat(1, 4, CV_32FC1);

	for (int i=0; i<4; i++)
		CV_MAT_ELEM(*LightPlaneABCD, float, 0, i) = planeLight[i];
	
	cvSave("LightPlaneABCD.xml", LightPlaneABCD);    // 保存结构光平面
	cvReleaseMat(&LightPlaneABCD);
}

//平面精度测试
void planeTest(CvPoint2D32f point)
{
	CvMat* planeABCD = (CvMat *)cvLoad("LightPlaneABCD.xml");
	double a[16];
	float A = CV_MAT_ELEM(*planeABCD, float, 0, 0);
	float B = CV_MAT_ELEM(*planeABCD, float, 0, 1);
	float C = CV_MAT_ELEM(*planeABCD, float, 0, 2);
	float D = CV_MAT_ELEM(*planeABCD, float, 0, 3);
	float fx = CV_MAT_ELEM(*intrinsic, float, 0, 0);
	float fy = CV_MAT_ELEM(*intrinsic, float, 1, 1);
	float cx = CV_MAT_ELEM(*intrinsic, float, 0, 2);
	float cy = CV_MAT_ELEM(*intrinsic, float, 1, 2);
	float u = point.x;
	float v = point.y;
	a[0] = A; a[1] = B; a[2] = C; a[3] = D;
	a[4] = 0; a[5] = fy; a[6] = v-cy; a[7] = 0;
	a[8] = fx; a[9] = 0; a[10] = cx-u; a[11] = 0;
	a[12] = 0; a[13] = 0; a[14] = 0; a[15] = 0;
	//for(int i=0; i<16; i++)
	//{
	//	cout<<" "<<a[i]<<" ";
	//	if (i%4 - 3 == 0)
	//		cout<<endl;
	//}
	double U[16], S[16], V[16];
	SVD_Solve(a, U, S, V, 4);
	CvPoint3D32f temp;
	temp.x = V[3]/V[15];   //系数A
	temp.y = V[7]/V[15];   //系数B
	temp.z = V[11]/V[15];  //系数C
	//cout<<endl<<"x:"<<temp.x<<" y:"<<temp.y<<" z:"<<temp.z<<endl;
	//cout<<"test  x:"<<temp.x<<" y: "<<temp.y<<" z: "<<temp.z<<endl;
	testline.push_back(temp);
}


void main()
{
	intrinsic=(CvMat *)cvLoad("Intrinsics.xml"); //内参数矩阵
	distortion=(CvMat *)cvLoad("Distortion.xml"); //畸变矩阵
	dx = f / CV_MAT_ELEM(*intrinsic, float, 0, 0);
	dy = f / CV_MAT_ELEM(*intrinsic, float, 1, 1);
	float a1[5], a2[5], a3[5];
	int a = 1;     //从第一张图片开始读取。
	while(a <= number_image)
	{
		CvPoint3D32f cross_point_c;										//存储单线交点p摄像机坐标
		float OP[5];													//存储每幅图像的交点摄像机坐标的|OP|值
		CvPoint3D32f * image_points_buf_c = new CvPoint3D32f[3];		//存储单线角点摄像机坐标
		cout<<"loop:"<<a<<endl;
		//加载图片   
		sprintf_s (filename, adrSrcImage, a);
		show = cvLoadImage(filename, -1);

		//畸变校正
		adjustImage(a);

		IplImage * gray_image;
		gray_image = cvCloneImage(show);
		//提取角点的像素坐标
		getCornerPoints(show, gray_image, a);

		//拟合角点线方程，拟合结果保存在vector cline中，共五条
		
		fitCornerLine(image_points_buf, a);

		//提取中心线投影，像素坐标系
		getCenterLine(show, a);

		//每幅图的五张角点线进行操作
		for(int i=0; i<board_width; i++)
		{
			//分别获得中心线和五条角点线的交点投影p的像素坐标
			CvPoint2D32f cross_point;
			getCrossOfLines(cline[i], struct_line, cross_point);

			//planeTest(cross_point);

			CvPoint circle = cvPoint(cross_point.x, cross_point.y);
			cvCircle(show_corner, circle, 6, Scalar(2));
			char* strNameLineImage="DstImage//%d直线和交点提取.bmp";
			sprintf_s (filename,strNameLineImage,a);   
			cvSaveImage(filename,show_corner,0);

			//转换为交点投影p的摄像机坐标
			imgPTtoCameraPT(cross_point, cross_point_c);
			//cout<<"交点"<<"x:"<<cross_point_c.x<<" y:"<<cross_point_c.y<<" z:"<<cross_point_c.z<<endl<<endl;
			//求取角点投影a,b,c的摄像机坐标
			for(int j=0; j<3; j++)
				imgPTtoCameraPT(image_points_buf[i + board_width*j], image_points_buf_c[j]);
			//if(i == 4)
			//{
			//	for(int j=0; j<3; j++)
			//		cout<<"x:"<<image_points_buf_c[j].x<<" y:"<<image_points_buf_c[j].y<<" z:"<<image_points_buf_c[j].z<<endl;
			//	cout<<endl;
			//}
			//分别求取五条线的三个角度，a1,a2,a3
			a1[i] = getAngle(image_points_buf_c[1], image_points_buf_c[2]);
			a2[i] = getAngle(image_points_buf_c[2], image_points_buf_c[0]);
			a3[i] = getAngle(cross_point_c, image_points_buf_c[2]);
			//cout<<"a1:"<<a1[i]<<" a2:"<<a2[i]<<" a3:"<<a3[i]<<endl;
			//分别求取五个交点P的摄像机坐标
			getCrossCamera(a1[i], a2[i], a3[i], OP[i], cross_point_c);
		}
		//for(int i=0; i<5; i++)
		//{
		//	cout<<"交点P"<<i<<"的摄像机坐标："<<endl<<"	x:"<<point_c[i+(a-1)*5].x<<" y:"<<point_c[i+(a-1)*5].y<<" z:"<<point_c[i+(a-1)*5].z<<endl;
		//	
		//}
		cline.clear();
		delete[] image_points_buf_c;
		a++;
	}
	cvReleaseMat(&intrinsic);
	cvReleaseMat(&distortion);
	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);
	//求解结构光平面
	planSolve();
	MessageBox(NULL, TEXT("光平面标定完成，数据已存储到 LightPlaneABCD.xml 文件中"), TEXT("光平面求取"), MB_OK);
}