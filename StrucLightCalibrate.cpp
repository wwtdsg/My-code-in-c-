#include <atlstr.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fstream> //�ļ��������������
#include <list>
#include <vector>
#include "opencv2/core/core_c.h"
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

float f = 12.0f; //������࣬��λ��mm
float dx;		//��Ԫx���򳤶�
float dy;		//��Ԫy���򳤶�
CvMat * intrinsic;
CvMat * distortion;

//�����������������ṹ��궨ʹ�õı�����������������������
vector<float* > cline; //���浥��ͼ��Ľǵ��ߣ�ÿ��ͼ5��
float struct_line[4];  //�洢�����ṹ��ֱ��
IplImage* show;
IplImage* show_test;
IplImage* show_corner;

int number_image = 32;
char filename[300];
char* adrSrcImage="SrcImage//%d.bmp";                //Դͼ���ļ��е�ַ
float d=1.76f;                               //���̸�����ߴ硣 
CvSize board_size=cvSize(5,3);                       //���̹��width*height
int board_width=board_size.width;
int board_height=board_size.height;
int total_per_image=board_width*board_height;
int iterations=100;               //��������
double accuracy=0.01;              //��������
int successes = 0;


char* adrDstImage="DstImage//%d.bmp";         //����У�����ͼ���ļ��е�ַ
char* adrTempImage="TempImage//%d.bmp";       //����ͼ���ļ��е�ַ

CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];        //����ͼ��Ľǵ㣬ͼ������

CvMat * image_points=cvCreateMat(number_image*total_per_image,2,CV_32FC1);  //ͼ������ϵ
CvMat * object_points=cvCreateMat(number_image*total_per_image,3,CV_32FC1); //��������ϵ
CvMat * point_counts=cvCreateMat(number_image,1,CV_32SC1);         //ÿ��ͼ��Ľǵ�����

FILE* fp = fopen("DataResult.txt","w+");      //�������ݵ�txt�ļ���ַ
FILE* fpInfo=fopen("Information.txt","w+");   //�����쳣��Ϣ
FILE* fpTest=fopen("LightErrorTest.txt","w+");//�����쳣��Ϣ

float planeLight[4];                             //��ƽ�档
vector<CvPoint3D32f> point_c;									//�洢����ͼ��Ľ�������������

vector<CvPoint3D32f> testline;

/*
�ṹ��ƽ��궨��Ʒ�����

���裺
1.Ϊ�궨���㣬������ȡ�ṹ��ƽ�棬�������Ʊ궨�塣
2.��������궨�õ��Ļ���ϵ������ͼ����л���У����
3.��ȡ�ṹ��ֱ�߷��̣�l1����ͼ����������ϵ����
4.��ȡ�궨��ǵ����꣨��λ�����أ�����ϵ��ͼ����������ϵ����
5.��ÿ�е������ǵ����ֱ����ϣ��õ��ǵ��߷��̣�l2����ͼ����������ϵ����
6.�ɽǵ��߷��̣�l2���ͽṹ��ֱ�߷��̣�l1����ȡ���㣨p����ͼ��������������
7.��������궨�õ����ڲΣ��������ǵ㣨a��b��c����һ�����㣨p����ͼ����������ӳ��Ϊ������꣨��λ��mm����
8.�ݴ˷ֱ���������ļн�a1����BOC����a2����AOC���Լ�a3����COP����ֵ��
9.�ɽ�a1/a2/a3��A��B��C����֮��ľ���d�����|OP|��ֵ��
10.��������Op�ķ��򣬼��ɵõ�P���������ꡣ
11.�ֱ��ÿһ�нǵ����ظ�5~10��������һ���궨ͼ����ýṹ������ϵĶ�����������꣨x, y, z����
12.�Զ���ռ����ϳ�һ���������ϵ�Ŀռ�ֱ�� / �洢���е�
13.�ֱ��ÿһ���ṹ��궨ͼ�ظ�2~12�����õ��ṹ��ƽ���ϵĶ��ֱ�߷��� / �洢���е�
14.�Զ��ֱ�߷��̽�����ϣ��õ��ṹ��ƽ�淽�� / ��13���õ������е����ƽ����ϣ��õ��ṹ��ƽ�淽��
15.�ṹ��ƽ��궨��ɡ�

*/


/*
�ṹ����ȡ������:

�������ֵ�˲�����ֵ����������
����ȡ�����ߣ��õ�ֱ�߷���
*/



//����ֱ�߶˵�
void twoEndsOfLine( float* pline, CvPoint2D32f &startpt, CvPoint2D32f &endpt)
{
	double k=pline[1]/pline[0];
	double d=pline[3]-k*pline[2];
	startpt.x=5;
	startpt.y=k*startpt.x+d;
	endpt.x=635;
	endpt.y=k*endpt.x+d;
}

//����ͼ��
void adjustImage(int a)
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

		sprintf_s (filename,adrDstImage,a);  // ����Ŀ��ͼ�� 
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
		fprintf(fpInfo, "�� %d ֡ͼƬ�޷��ҵ����̸����нǵ�!count=%d\n",a,count);
		//�ǵ������ؽ���
	}
	else
	{
		fprintf(fpInfo, "�� %d ֡ͼ��ɹ���� %d ���ǵ�!\n",a,count);			
		//�ǵ������ؽ���
		cvFindCornerSubPix(show,image_points_buf,count,board_size,cvSize(-1,-1),
			cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,iterations,accuracy));
		fprintf(fpInfo, "�Ҷ�ͼ�����ػ��������...\n");
		//���ƽǵ�
		cvDrawChessboardCorners(gray_image,board_size,image_points_buf,count,found);
		fprintf(fpInfo, "��Դͼ���ϻ��ƽǵ�������...\n\n");
		//�������ͼ��
		sprintf_s (filename,adrTempImage,a);
		cvSaveImage(filename,gray_image);
	}
}

//��Ͻǵ���
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
		cline.push_back(line);//��һ��ͼ���еĽǵ�����vector������cline�С�
		CvPoint2D32f startpt;
		CvPoint2D32f endpt;
		twoEndsOfLine(line,startpt,endpt);	
		cvLine(show_corner,cvPoint(startpt.x,startpt.y),cvPoint(endpt.x,endpt.y),Scalar(0),1);
		cvClearSeq(point_seq);
	}
}

//��ֵ��
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

	//����ͼ����ֵ��ͼ����ж�ֵ��  
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



//��ȡͼ���е�������
void getCenterLine(IplImage* imageTest, int a)
{

	IplImage* imageDst = cvCreateImage(cvSize(imageTest->width, imageTest->height), 8, 1);
	uchar* dataimage=(uchar*)imageTest->imageData;
	int width = imageTest->width;
	int hight = imageTest->height;
	int stepwide = imageTest->widthStep;
	int right[1000], left[1000];
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
				left[i] = j;
				break;
			}
			if (j == width-1)
				left[i] = 10000;
		}
	}
	//��������������ɨ��	 
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
	//ȡ��ɨ���е�ƽ��ֵ��Ϊ�����ߵĸ������� 
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* point_seq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
	for (int i=0; i<hight; i++)
	{
		if (abs((int)right[i]) > width | abs((int)left[i]) > width)              //TODO  2014-11-19 ������ͨ�ģ�δ��ʼ����ֵ��
			continue;
		cvSeqPush(point_seq, &cvPoint2D32f(0.5 * (right[i] + left[i]), i));
	}
	//���ֱ��
	cvFitLine(point_seq, CV_DIST_L2, 0, 0.01, 0.01, struct_line);

	//����ֱ��
	CvPoint2D32f startpt;
	CvPoint2D32f endpt;
	twoEndsOfLine(struct_line,startpt,endpt);
	cvLine(show_corner,cvPoint(startpt.x,startpt.y),cvPoint(endpt.x,endpt.y),Scalar(0),1);
	/*char* strNameLineImage="DstImage//ֱ�����%d.bmp";*/
	//sprintf_s (filename,strNameLineImage,a);   
	//cvSaveImage(filename,show_corner,0);

	if (struct_line[0] == 0)
		struct_line[0] = 0.0001f;

	cvClearSeq(point_seq);
	cvReleaseMemStorage(&storage);

	//�õ��ṹ��ֱ�ߣ��洢��ȫ�ֱ���struct_line֮��
}

//��������֮���ֱ��
//cvLine(imageDst,cvPoint(startpt.x,startpt.y),cvPoint(endpt.x,endpt.y),Scalar(0),1);



//��ȡ�ǵ��ߺͽṹ�������ߵĽ���
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


//ͼ������ϵת���������ϵ
void imgPTtoCameraPT(CvPoint2D32f &imgPT, CvPoint3D32f &cameraPT)
{
	cameraPT.x = (imgPT.x - CV_MAT_ELEM(*intrinsic, float, 0, 2)) * dx;
	cameraPT.y = (CV_MAT_ELEM(*intrinsic, float, 1, 2) - imgPT.y) * dy; //�˴����޸�
	cameraPT.z = f;
}

//��ȡ����֮��ļн�
float getAngle(CvPoint3D32f &angle1, CvPoint3D32f &angle2)
{
	float dot_product = angle1.x*angle2.x + angle1.y*angle2.y + angle1.z*angle2.z;
	float module1 = sqrt(angle1.x*angle1.x + angle1.y*angle1.y + angle1.z*angle1.z);
	float module2 = sqrt(angle2.x*angle2.x + angle2.y*angle2.y + angle2.z*angle2.z);
	float angle = acos(dot_product/(module1*module2));
	return angle;
}

//���ݶ������Ϲ�ƽ��
//�����췽���� AX+BY+CZ+D=0
void SVD_Solve(double *SA, double *SU, double *SS, double *SV,int count)
{
	CvMat A, U, S, V;
	cvInitMatHeader(&A,count,4,CV_64FC1,SA,CV_AUTOSTEP);
	cvInitMatHeader(&U,4,4,CV_64FC1,SU,CV_AUTOSTEP);       //�̶��ֽ��4X4�ľ���
	cvInitMatHeader(&S,4,4,CV_64FC1,SS,CV_AUTOSTEP);
	cvInitMatHeader(&V,4,4,CV_64FC1,SV,CV_AUTOSTEP);
	cvSVD(&A, &U, &S, &V, CV_SVD_U_T);

}

//��ȡ����P�����������
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
		OP = abs(b / (ta3 + k) * cos(a3));//��λ������ /////////�˴��Ķ�
	}
	cout<<"OP-------------:"<<OP<<endl;
	CvPoint3D32f temp_point;
	float temp_module = sqrt(cross_point_c.x*cross_point_c.x + cross_point_c.y*cross_point_c.y + cross_point_c.z*cross_point_c.z);
	temp_point.x = 10 * OP * cross_point_c.x / temp_module;
	temp_point.y = 10 * OP * cross_point_c.y / temp_module;
	temp_point.z = 10 * OP * cross_point_c.z / temp_module; //��λ�� mm
	point_c.push_back(temp_point);//��ÿ�������������걣�浽vector��
}

//���ṹ��ƽ�棺���ݽ����췽��������ƽ��ϵ�� AX+ BY+ CZ+ D=0 ��
void planSolve()
{
	int lenth = point_c.size();
	if(lenth < 4)
	{
		MessageBox(NULL, TEXT("����������������!"), TEXT("��ƽ����ȡ"), MB_OK);
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
	planeLight[0] = V[3]/V[3];   //ϵ��A
	planeLight[1] = V[7]/V[3];   //ϵ��B
	planeLight[2] = V[11]/V[3];  //ϵ��C
	planeLight[3] = V[15]/V[3];  //ϵ��D	
	fprintf(fpTest, "A=%f ; B=%f ; C=%f ;  D=%f  \n",
		planeLight[0], planeLight[1], planeLight[2], planeLight[3]);
	CvMat* LightPlaneABCD = cvCreateMat(1, 4, CV_32FC1);

	for (int i=0; i<4; i++)
		CV_MAT_ELEM(*LightPlaneABCD, float, 0, i) = planeLight[i];
	
	cvSave("LightPlaneABCD.xml", LightPlaneABCD);    // ����ṹ��ƽ��
	cvReleaseMat(&LightPlaneABCD);
}

//ƽ�澫�Ȳ���
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
	temp.x = V[3]/V[15];   //ϵ��A
	temp.y = V[7]/V[15];   //ϵ��B
	temp.z = V[11]/V[15];  //ϵ��C
	//cout<<endl<<"x:"<<temp.x<<" y:"<<temp.y<<" z:"<<temp.z<<endl;
	//cout<<"test  x:"<<temp.x<<" y: "<<temp.y<<" z: "<<temp.z<<endl;
	testline.push_back(temp);
}


void main()
{
	intrinsic=(CvMat *)cvLoad("Intrinsics.xml"); //�ڲ�������
	distortion=(CvMat *)cvLoad("Distortion.xml"); //�������
	dx = f / CV_MAT_ELEM(*intrinsic, float, 0, 0);
	dy = f / CV_MAT_ELEM(*intrinsic, float, 1, 1);
	float a1[5], a2[5], a3[5];
	int a = 1;     //�ӵ�һ��ͼƬ��ʼ��ȡ��
	while(a <= number_image)
	{
		CvPoint3D32f cross_point_c;										//�洢���߽���p���������
		float OP[5];													//�洢ÿ��ͼ��Ľ�������������|OP|ֵ
		CvPoint3D32f * image_points_buf_c = new CvPoint3D32f[3];		//�洢���߽ǵ����������
		cout<<"loop:"<<a<<endl;
		//����ͼƬ   
		sprintf_s (filename, adrSrcImage, a);
		show = cvLoadImage(filename, -1);

		//����У��
		adjustImage(a);

		IplImage * gray_image;
		gray_image = cvCloneImage(show);
		//��ȡ�ǵ����������
		getCornerPoints(show, gray_image, a);

		//��Ͻǵ��߷��̣���Ͻ��������vector cline�У�������
		
		fitCornerLine(image_points_buf, a);

		//��ȡ������ͶӰ����������ϵ
		getCenterLine(show, a);

		//ÿ��ͼ�����Žǵ��߽��в���
		for(int i=0; i<board_width; i++)
		{
			//�ֱ��������ߺ������ǵ��ߵĽ���ͶӰp����������
			CvPoint2D32f cross_point;
			getCrossOfLines(cline[i], struct_line, cross_point);

			//planeTest(cross_point);

			CvPoint circle = cvPoint(cross_point.x, cross_point.y);
			cvCircle(show_corner, circle, 6, Scalar(2));
			char* strNameLineImage="DstImage//%dֱ�ߺͽ�����ȡ.bmp";
			sprintf_s (filename,strNameLineImage,a);   
			cvSaveImage(filename,show_corner,0);

			//ת��Ϊ����ͶӰp�����������
			imgPTtoCameraPT(cross_point, cross_point_c);
			//cout<<"����"<<"x:"<<cross_point_c.x<<" y:"<<cross_point_c.y<<" z:"<<cross_point_c.z<<endl<<endl;
			//��ȡ�ǵ�ͶӰa,b,c�����������
			for(int j=0; j<3; j++)
				imgPTtoCameraPT(image_points_buf[i + board_width*j], image_points_buf_c[j]);
			//if(i == 4)
			//{
			//	for(int j=0; j<3; j++)
			//		cout<<"x:"<<image_points_buf_c[j].x<<" y:"<<image_points_buf_c[j].y<<" z:"<<image_points_buf_c[j].z<<endl;
			//	cout<<endl;
			//}
			//�ֱ���ȡ�����ߵ������Ƕȣ�a1,a2,a3
			a1[i] = getAngle(image_points_buf_c[1], image_points_buf_c[2]);
			a2[i] = getAngle(image_points_buf_c[2], image_points_buf_c[0]);
			a3[i] = getAngle(cross_point_c, image_points_buf_c[2]);
			//cout<<"a1:"<<a1[i]<<" a2:"<<a2[i]<<" a3:"<<a3[i]<<endl;
			//�ֱ���ȡ�������P�����������
			getCrossCamera(a1[i], a2[i], a3[i], OP[i], cross_point_c);
		}
		//for(int i=0; i<5; i++)
		//{
		//	cout<<"����P"<<i<<"����������꣺"<<endl<<"	x:"<<point_c[i+(a-1)*5].x<<" y:"<<point_c[i+(a-1)*5].y<<" z:"<<point_c[i+(a-1)*5].z<<endl;
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
	//���ṹ��ƽ��
	planSolve();
	MessageBox(NULL, TEXT("��ƽ��궨��ɣ������Ѵ洢�� LightPlaneABCD.xml �ļ���"), TEXT("��ƽ����ȡ"), MB_OK);
}