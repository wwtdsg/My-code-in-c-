#include <atlstr.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv2/core/core_c.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

/*
��Ʒ�����
1.��ȡ�ڲξ���
2.��ȡƽ��ϵ������
3.ͼ�����У��
4.ͼ��Ԥ����
	������ƽ������ֵ����
5.�ṹ����������ȡ
	Ϊ������ȡ��������ɨ���ʱ����Ҫ�����ع��
6.������ⷽ���飬�õ�X,Y,Z��ֵ


*/

//����ȫ�ֱ���
CvMat* intrinsic;		//����ڲξ���
CvMat* distortion;		//����ϵ������
CvMat* planeABCD;		//ƽ��ϵ������

vector<CvPoint2D32f> centerLine;//���������
vector<CvPoint3D32f> cline;
IplImage *show;
IplImage* showtest;
char filename[100];
int number_image = 4;	//ͼ������
char* SrcImage = "SrcImage//%d.bmp";

CvPoint2D32f mouse_location;
int mouse_count = 0;
double lenth;
//char* filename[100];

//��ȡ�ڲκ�ƽ��ϵ������
void loadXML()
{
	intrinsic = (CvMat *)cvLoad("Intrinsics.xml");
	distortion = (CvMat *)cvLoad("Distortion.xml");
	planeABCD = (CvMat *)cvLoad("LightPlaneABCD.xml");
}

//ͼ�����У��
void ajustImage(IplImage* show)
{
		//�������䣬����Ч����
	IplImage * mapx=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * mapy=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * clone;
	//Ĭ��У׼һ�θ�ͼƬ


		//���ڲ�У׼ͼ��
		cvInitUndistortMap(intrinsic,distortion,mapx,mapy);
		clone=cvCloneImage(show);
		cvRemap(clone,show,mapx,mapy);
		//sprintf_s (filename,adrDstImage,a);  // ����Ŀ��ͼ�� 
		cvSaveImage("tempImage//Undistortion.bmp",show);

	cvReleaseImage(&clone);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
}

//�򵥶�ֵ��
void thresholdOTSU(IplImage* img,IplImage*dstImg,int* outThreshold)
{
	uchar* dataSrc = (uchar*)img->imageData;
	uchar* dataDst = (uchar*)dstImg->imageData;
	int height = img->height;
	int width  = img->width;
	int step   = img->widthStep;


	//����ͼ����ֵ��ͼ����ж�ֵ��  
	*outThreshold = 250;
	for (int i=0; i<height; i++)
	{
		for (int j=0; j<width; j++)
		{
			if (dataSrc[i * img->nChannels * width+img->nChannels * j] >= 250)
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



//��ȡ�ṹ���������
void getCenterLine(IplImage* imageTest, int a)
{

	IplImage* imageDst = cvCreateImage(cvSize(imageTest->width, imageTest->height), 8, 1);
	uchar* dataimage=(uchar*)imageTest->imageData;
	int width = imageTest->width;
	int hight = imageTest->height;
	int stepwide = imageTest->widthStep;
	int up[1000], down[1000];
	char* smooth = "DstImage//%d��ֵ�˲�.bmp";
	char* threshold = "DstImage//%d��ֵ����.bmp";
	char* morphy = "DstImage//%d�������.bmp";

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

	//�����������ң���ɨ��	 
	for (int i=0; i<hight; i++)
	{
		for (int j=0; j<width; j++)
		{
			if ((UINT)dataimage[j + i * stepwide] == 255)   
			{
				up[i] = j;
				break;
			}
		}
	}
	//��������������ɨ��	 
	for (int i=0; i<hight; i++)
	{
		for (int j=width; j>0; j--)
		{
			if ( (UINT)dataimage[j+i*stepwide] == 255)
			{
				down[i] = j;
				break;
			}
		}
	}
	//ȡ��ɨ���е�ƽ��ֵ��Ϊ�����ߵĸ������� 
	for (int i=0; i<hight; i++)
	{
		if (abs(up[i]) > width && abs(down[i]) > width)              //TODO  2014-11-19 ������ͨ�ģ�δ��ʼ����ֵ��
			continue;
		CvPoint2D32f temp;
		temp.x = (float)0.5 * (up[i] + down[i]);
		temp.y = (float)i;
		CvPoint point_temp;
		centerLine.push_back(temp);

		point_temp.x = temp.x;
		point_temp.y = temp.y;
		char* draw_line = "DstImage//%d�ṹ��������.bmp";
		sprintf_s(filename, draw_line, a);
		cvCircle(show, point_temp, 0.5, Scalar(100));
		
	}
	cvSaveImage(filename,show,0);
}


//��ⷽ����
void SVD_Solve(double *SA, double *SU, double *SS, double *SV,int count)
{
	CvMat A, U, S, V;
	cvInitMatHeader(&A,count,4,CV_64FC1,SA,CV_AUTOSTEP);
	cvInitMatHeader(&U,4,4,CV_64FC1,SU,CV_AUTOSTEP);       //�̶��ֽ��4X4�ľ���
	cvInitMatHeader(&S,4,4,CV_64FC1,SS,CV_AUTOSTEP);
	cvInitMatHeader(&V,4,4,CV_64FC1,SV,CV_AUTOSTEP);
	cvSVD(&A, &U, &S, &V, CV_SVD_U_T);
}

void lineSolve(CvPoint2D32f& point)
{
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
	float m = (cy - v)/fy;
	float n = (u - cx)/fx; 
	a[0] = A; a[1] = B; a[2] = C; a[3] = D;
	float lz = -D/C;
	a[4] = 0; a[5] = fy; a[6] = v-cy; a[7] = 0;
	a[8] = fx; a[9] = 0; a[10] = cx-u; a[11] = 0;
	a[12] = 0; a[13] = 0; a[14] = 0; a[15] = 0;
	CvPoint3D32f temp;
	temp.z = -D / (A*n + B*m + C);
	temp.x = n * temp.z;
	temp.y = m * temp.z;
	//for(int i=0; i<16; i++)
	//{
	//	cout<<" "<<a[i]<<" ";
	//	if (i%4 - 3 == 0)
	//		cout<<endl;
	//}
	//double U[16], S[16], V[16];
	//SVD_Solve(a, U, S, V, 4);
	//CvPoint3D32f temp;
	//temp.x = V[3]/V[15];   //ϵ��A
	//temp.y = V[7]/V[15];   //ϵ��B
	//temp.z = V[11]/V[15];  //ϵ��C
	//cout<<endl<<"x:"<<temp.x<<" y:"<<temp.y<<" z:"<<temp.z<<endl;
	cline.push_back(temp);
}

void on_mouse( int event, int x, int y, int flags, void* param )
{
	if( !show )
		return;
	
	if(event == CV_EVENT_LBUTTONDOWN)
	{
		CvPoint pt = cvPoint(x, y);
		cvCircle( showtest, pt, 1,cvScalar(125) ,-1, CV_AA, 0 );
		cvShowImage("����ͼ", showtest);

		cout<<"��ȡ���λ�óɹ�"<<endl;
		CvScalar s = cvGet2D(showtest, y, x);
		cout<<"��������Ϊ��"<<endl;
		cout<<"("<<x<<","<<y<<")"<<endl;
		cout<<"\n\n";
		mouse_location.x = x;
		mouse_location.y = y;
		lineSolve(mouse_location);
		cout<<"x: "<<cline.back().x<<" y: "<<cline.back().y<<" z: "<<cline.back().z<<endl;
		if(mouse_count == 0)
			cout<<"�ȴ�����ڶ���"<<endl;
		//λ�Ƽ��㲢�����
		if(mouse_count > 0)
		{
			CvPoint3D32f p1 = cline.back();
			cline.pop_back();
			CvPoint3D32f p2 = cline.back();
			cline.pop_back();
			lenth = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
			cout<<"����λ��ֵΪ��"<<lenth<<"\n\n";
			mouse_count = 0;
			cout<<"�ȴ������һ��"<<endl;
		}
		else
			mouse_count++;
	}
}

void test()
{
	
	cvNamedWindow( "����ͼ", 2);//�ɸı䴰�ڵĴ�С��1-���ڵĴ�С���ɸı�
	cvSetMouseCallback( "����ͼ", on_mouse, 0 );
	cvShowImage("����ͼ", showtest);
	cvWaitKey(0); //�ȴ�����
}

void main()
{
	loadXML();
	int a = 1;
	while(a <= number_image)
	{
		mouse_count =0;
		sprintf_s(filename, SrcImage, a);
		show = cvLoadImage(filename, 0);
		showtest = cvLoadImage(filename, 0);
		ajustImage(show);
		ajustImage(showtest);
		//getCenterLine(show, a);
		//for(int i=0; i<centerLine.size(); i++)
		//{
		//	lineSolve(centerLine[i]);
		//	if(i == 40)
		//	cout<<"x:"<<cline[i].x<<" y:"<<cline[i].y<<" z:"<<cline[i].z<<endl;
		//}
		cline.clear();
		a++;
		centerLine.clear();
		test();
	}

}