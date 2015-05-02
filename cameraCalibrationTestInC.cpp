#include "cameraCalibrationTestInC.h"


using namespace cv;
using namespace std;

//��������������������궨ʹ�ñ�����������������������������������
//���������
char* adrSrcImage="SrcImage//%d.bmp";                //Դͼ���ļ��е�ַ
float squareSize=1.76f;                               //���̸�����ߴ硣     
CvSize board_size=cvSize(5,3);                       //���̹��
int number_image=32;                                //����궨ͼ������

char filename[300]="";             //����ͼƬ�ļ��е�ַ //������Ϊ�������ݡ�
int iterations=100;               //��������
double accuracy=0.01;              //��������

//���������
char* adrDstImage="DstImage//%d.bmp";         //�궨���ͼ���ļ��е�ַ
char* adrTempImage="TempImage//%d.bmp";       //����ͼ���ļ��е�ַ
FILE* fp = fopen("DataResult.txt","w+");      //�������ݵ�txt�ļ���ַ
FILE* fpInfo=fopen("Information.txt","w+");   //�����쳣��Ϣ
FILE* fpTest=fopen("LightErrorTest.txt","w+");//�����쳣��Ϣ
int successes=0;                              //��ЧͼƬ����
IplImage * show;                              //ͼ��ָ��
IplImage * show_test;                         //��У׼���ͼƬָ��

//�м����

int board_width=board_size.width;
int board_height=board_size.height;
int total_per_image=board_width*board_height;                               //����ͼ��Ľǵ���
CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];        //����ͼ��Ľǵ�
CvMat * image_points=cvCreateMat(number_image*total_per_image,2,CV_32FC1);  //ͼ������ϵ
CvMat * object_points=cvCreateMat(number_image*total_per_image,3,CV_32FC1); //��������ϵ
CvMat * point_counts=cvCreateMat(number_image,1,CV_32SC1);         //ÿ��ͼ��Ľǵ�����
CvMat * intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);                //�ڲ�������
CvMat * distortion_coeffs=cvCreateMat(5,1,CV_32FC1);               //����ϵ������
CvMat *rotation_vectors= cvCreateMat(number_image,3,CV_32FC1);     //��ת����
CvMat *translation_vectors = cvCreateMat(number_image,3,CV_32FC1); //ƽ�ƾ���

CvMat *object_points2Test=cvCreateMat(total_per_image,3,CV_32FC1); //��ʱ�Բ��Ա���
CvMat *rotation_vectorsTest= cvCreateMat(1,3,CV_32FC1);
CvMat *translation_vectorsTest= cvCreateMat(1,3,CV_32FC1);
//������������������������������������������������
//

//��ӡ����
void printMT(CvMat* mat,int rows ,int cols,FILE* fp );


void getCornerPoints()
{
	int a=1;     //�ӵ�һ��ͼƬ��ʼ��ȡ��
	int count;   //�ҵ��Ľǵ����
	int step;    //ִ�е��ڼ���ͼƬ 
	while(a<=number_image)
	{
		cout<<"loop:"<<a<<endl;
		//����ͼƬ   
		sprintf_s (filename,adrSrcImage,a);   
	
		show=cvLoadImage(filename,0);
		IplImage * gray_image= cvCreateImage(cvGetSize(show),8,1); 

		//���̽ǵ���      //TODO 2014-11-20
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
			//cvFindCornerSubPix(show,image_points_buf,count,board_size,cvSize(-1,-1),
			//	cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,iterations,accuracy));
			fprintf(fpInfo, "�Ҷ�ͼ�����ػ��������...\n");
			//���ƽǵ�
			cvDrawChessboardCorners(show,board_size,image_points_buf,count,found);
			fprintf(fpInfo, "��Դͼ���ϻ��ƽǵ�������...\n\n");
			//�������ͼ��
			sprintf_s (filename,adrTempImage,a);  

			cvSaveImage(filename,show);
		}

		if(total_per_image==count)
		{
			step=successes*total_per_image;
			for(int i=step,j=0;j<total_per_image;++i,++j)
			{
				//˳������������������ʽ����һ�¡�
				CV_MAT_ELEM(*image_points,float,i,0)=image_points_buf[j].x;  
				CV_MAT_ELEM(*image_points,float,i,1)=image_points_buf[j].y;   
			}
			CV_MAT_ELEM(*point_counts,int,successes,0)=total_per_image;
			successes++;
		}
		else
		{
			fprintf(fpTest, " \n\n");
			fprintf(fpTest,"��%d��ͼƬ�Ƕ���ȡʧ�ܣ�",a);
		}
		a++; 
		cvReleaseImage(&gray_image);
	}

	fprintf(fpInfo, "*********************************************\n");
	fprintf(fpInfo, "�� %d ֡ͼƬ�У��궨�ɹ���ͼƬΪ %d ֡!\n",number_image,successes);
	fprintf(fpInfo, "�궨ʧ�ܵ�ͼƬΪ %d ֡!\n",number_image-successes);
	fprintf(fpInfo, "*********************************************\n");
}
//��Ŀ����궨
void calibrateCamera()
{
	fprintf(fpInfo, "��ʼ������궨!\n\n\n");

	if (object_points->rows>1)
	{
		cvCalibrateCamera2(object_points,image_points,point_counts,cvGetSize(show),
			intrinsic_matrix,distortion_coeffs,/*NULL*/rotation_vectors,/*NULL*/translation_vectors,
			0/*CV_CALIB_USE_INTRINSIC_GUESS/*CV_CALIB_FIX_PRINCIPAL_POINT|*/|CV_CALIB_FIX_K3  
			/*CV_CALIB_FIX_ASPECT_RATIO*//*|CV_CALIB_ZERO_TANGENT_DIST*/,cvTermCriteria(CV_TERMCRIT_EPS,iterations,accuracy));
	}
	cout<<"�ڲ�����"<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,0,0)<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,1,1)<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,0,2)<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,1,2)<<endl;

	cout<<endl;
	cout<<"���䣺"<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,0,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,1,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,2,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,3,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,4,0)<<endl;

	cout<<endl;
	cout<<"z����ƽ�ƣ�"<<endl;
	cout<<endl;
	for(int i=0; i<number_image; i++)
	{
		cout<<"��"<<i+1<<"���궨ͼ��"<<endl;
		cout<<CV_MAT_ELEM(*translation_vectors,float,i,2)<<endl;
	}

	//���ݼ�¼���ļ�
	fprintf(fp,"\n");
	fprintf(fp,"������ڲ�������Ϊ��(3��3��)\n");
	for(int i=0;i<3;i++)
	{			
		fprintf(fp,"%.2f      %.2f     %.2f\t\n",CV_MAT_ELEM(*intrinsic_matrix,float,i,0),CV_MAT_ELEM(*intrinsic_matrix,float,i,1),
			CV_MAT_ELEM(*intrinsic_matrix,float,i,2));
	}
	fprintf(fp,"\n\n");

	fprintf(fp,"������������Ϊ��(K1��K2��P1��P2��K3)\n");
	for (int i=0;i<5;i++)
	{
		fprintf(fp,"%.2f   ",CV_MAT_ELEM(*distortion_coeffs,float,i,0));
	}
	fprintf(fp,"\n\n");
	
	//saveParamXML();
	fprintf(fpInfo, "��������󡢻���ϵ�����������ϵ�������Ѿ��ֱ�洢����ΪIntrinsics.xml��Distortion.xml��rotation_vectors.xml��translation_vectors.xml�ĵ���\n\n");
}

//�趨������������
void InitCorners3D_temp(CvMat *Corners3D, CvSize ChessBoardSize, int NImages, float SquareSize)
{
	int CurrentImage = 0;
	int CurrentRow = 0;
	int CurrentColumn = 0;
	int NPoints = ChessBoardSize.height*ChessBoardSize.width;
	
	for (CurrentImage = 0 ; CurrentImage < NImages ; CurrentImage++)
	{
		for (CurrentRow = 0; CurrentRow < ChessBoardSize.height; CurrentRow++)
		{
			for (CurrentColumn = 0; CurrentColumn < ChessBoardSize.width; CurrentColumn++)
			{
				CV_MAT_ELEM(*Corners3D,float, (CurrentImage*NPoints)+(CurrentRow*ChessBoardSize.width + CurrentColumn),0)=(float)(CurrentColumn*SquareSize);
				CV_MAT_ELEM(*Corners3D,float, (CurrentImage*NPoints)+(CurrentRow*ChessBoardSize.width + CurrentColumn),1)=(float)(CurrentRow*SquareSize);
				CV_MAT_ELEM(*Corners3D,float, (CurrentImage*NPoints)+(CurrentRow*ChessBoardSize.width + CurrentColumn),2)=0.0f;
			}
		}
	}
}



//����궨���
void calculateErrors()
{
	//��ӡͳһ������������Ϣ
	fprintf(fp,"ͼƬ����Ӧ��������������(%d��%d��)��\n",board_size.height,board_size.width);
	for(int i=0;i<board_size.height;i++)
	{
		for (int j=0;j<board_size.width;j++)
		{
			fprintf(fp,"%.1f  %.1f \t",CV_MAT_ELEM(*object_points,float,i*board_size.width+j,0),CV_MAT_ELEM(*object_points,float,i*board_size.width+j,1));
		}		
		fprintf(fp,"\n");
	}

	fprintf(fp,"\n\n");
	//����ÿһ��ͼƬ����
	CvMat * image_points_TEST=cvCreateMat(total_per_image,2,CV_32FC1);
	CvMat * rotTemp=cvCreateMat(1,3,CV_32FC1);
	CvMat * RtTemp33=cvCreateMat(3,3,CV_32FC1); //��ת����ת��������3X3���� 

	for (int nImage=1;nImage<successes+1;nImage++)
	{
		//ԭʼͼ���������Ϣ��¼���ļ�
		fprintf(fp,"��%d��ͼƬ����(%d��%d��)��\n",nImage,board_size.height,board_size.width);
		for(int i=0;i<board_size.height;i++)
		{
			for (int j=0;j<board_size.width;j++)
			{
				fprintf(fp,"%.1f  %.1f \t",CV_MAT_ELEM(*image_points,float,total_per_image*(nImage-1)+i*board_size.width+j,0),CV_MAT_ELEM(*image_points,float,total_per_image*(nImage-1)+i*board_size.width+j,1));
			}		
			fprintf(fp,"\n");
		}
		fprintf(fp,"\n");

		for (int i=0;i<3;i++)
		{
			CV_MAT_ELEM(*rotation_vectorsTest,float,0,i)=CV_MAT_ELEM(*rotation_vectors,float,nImage-1,i);
			CV_MAT_ELEM(*translation_vectorsTest,float,0,i)=CV_MAT_ELEM(*translation_vectors,float,nImage-1,i);		
		}

		//�����������ꡣ
		InitCorners3D_temp(object_points2Test,board_size,1,squareSize);

		//����������ת��������������ӳ���ͼ������
		cvProjectPoints2(object_points2Test,rotation_vectorsTest,translation_vectorsTest,intrinsic_matrix,
			distortion_coeffs,image_points_TEST,NULL,NULL,NULL,NULL,NULL);
		//�õ�ͼ������image_points_TEST

		//�������굽ͼ������ӳ�������
		fprintf(fp,"\n");
		fprintf(fp,"��%d��ͼƬ�������굽ͼ������ӳ�������(%d��%d��)��\n",nImage,board_size.height,board_size.width);
		for(int i=0;i<board_size.height;i++)
		{
			for (int j=0;j<board_size.width;j++)
			{
				fprintf(fp,"%.1f  %.1f \t",CV_MAT_ELEM(*image_points_TEST,float,i*board_size.width+j,0),CV_MAT_ELEM(*image_points_TEST,float,i*board_size.width+j,1));
			}
			fprintf(fp,"\n");
		}

		//����궨���
		fprintf(fp,"\n");
		fprintf(fp,"��%d��ͼƬ����궨���(%d��%d��)��\n",nImage,board_size.height,board_size.width);
		float max=0;
		float total=0;
		float average=0;
		for(int i=0;i<board_size.height;i++)
		{
			for (int j=0;j<board_size.width;j++)
			{
				float objectX=CV_MAT_ELEM(*image_points_TEST,float,i*board_size.width+j,1);
				float objectY=CV_MAT_ELEM(*image_points_TEST,float,i*board_size.width+j,0);
				float imageX=CV_MAT_ELEM(*image_points,float,(nImage-1)*total_per_image+i*board_size.width+j,1);
				float imageY=CV_MAT_ELEM(*image_points,float,(nImage-1)*total_per_image+i*board_size.width+j,0);
				float d=sqrt((objectX-imageX)*(objectX-imageX)+(objectY-imageY)*(objectY-imageY));
				fprintf(fp,"%.1f   ",d);
				max=max>d?max:d;
				total+=d;
			}
			fprintf(fp,"\n");
		}
		average=total/(board_size.width*board_size.height);
		fprintf(fp,"\nƽ�����: %.2f  �����%.2f \n",average,max);
		fprintf(fp,"\n");

		//��ӡ�����   todo test2014-12-3
		//for (int i=0;i<3;i++)
		//{
		//	fprintf(fp,"%f\t",CV_MAT_ELEM(*rotation_vectors,float,nImage-1,i));
		//}
		rotTemp=cvCreateMat(1,3,CV_32FC1);
		CV_MAT_ELEM(*rotTemp,float,0,0)=CV_MAT_ELEM(*rotation_vectors,float,nImage-1,0);
		CV_MAT_ELEM(*rotTemp,float,0,1)=CV_MAT_ELEM(*rotation_vectors,float,nImage-1,1);
		CV_MAT_ELEM(*rotTemp,float,0,2)=CV_MAT_ELEM(*rotation_vectors,float,nImage-1,2);

		RtTemp33=cvCreateMat(3,3,CV_32FC1); //��ת����ת��������3X3���� 
		cvRodrigues2(rotTemp,RtTemp33,0);

		//fprintf(fp,"�������ת����Ϊ��(��%d��ͼƬ)\n",nImage);   //TODO test 12-19
		//printMT(rotTemp,1,3,fp);
		fprintf(fp,"��%d��ͼƬ�������ת����3X3Ϊ��\n",nImage);
		printMT(RtTemp33,3,3,fp);
		
		fprintf(fp,"��%d��ͼƬ�����ƽ�ƾ���+��ת����Ϊ��\n",nImage);
		for (int i=0;i<3;i++)
		{
			fprintf(fp,"%.2f\t",CV_MAT_ELEM(*translation_vectors,float,nImage-1,i));
		}
		for (int i=0;i<3;i++)
		{
			fprintf(fp,"%.2f\t",CV_MAT_ELEM(*rotation_vectors,float,nImage-1,i));
		}
		fprintf(fp,"\n\n\n\n");
	}		
	cvReleaseMat(&image_points_TEST);
	cvReleaseMat(&rotTemp);
	cvReleaseMat(&RtTemp33);

}

//��ӡ����
void printMT(CvMat* mat,int rows ,int cols,FILE* fp )
{	
	for(int row=0;row<rows;row++)
	{
		for(int col=0;col<cols;col++)
		{
			fprintf(fp, "  %f  ",CV_MAT_ELEM(*mat,float,row,col));
		}
		fprintf(fp, " \n");
	}
	fprintf(fp, "\n\n");
}

void saveParamXML()
{
	//�����ڲ�
	cvSave("Intrinsics.xml",intrinsic_matrix);
	cvSave("Distortion.xml",distortion_coeffs);
}

//����ͼ��
void adjustImage()
{
	//�������䣬����Ч����
	CvMat * intrinsic=(CvMat *)cvLoad("Intrinsics.xml");
	CvMat * distortion=(CvMat *)cvLoad("Distortion.xml");
	IplImage * mapx=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * mapy=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * clone;
	//Ĭ��У׼һ�θ�ͼƬ
	for(int i=1;i<number_image+1;i++)
	{
		sprintf_s (filename,adrSrcImage,i);     
		show_test=cvLoadImage(filename,-1);

		//���ڲ�У׼ͼ��
		cvInitUndistortMap(intrinsic,distortion,mapx,mapy);
		clone=cvCloneImage(show_test);
		cvRemap(clone,show_test,mapx,mapy);

		sprintf_s (filename,adrDstImage,i);  // ����Ŀ��ͼ�� 
		cvSaveImage(filename,show_test);
	}

	cvReleaseImage(&clone);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
	cvReleaseMat(&intrinsic);
	cvReleaseMat(&distortion);
}

void main()
{
	getCornerPoints();


	//��ʼ����������
	InitCorners3D_temp(object_points,board_size,successes,squareSize);

	//����궨
	calibrateCamera();

	//����궨���
	calculateErrors();

	saveParamXML();

	//����ͼ��
	adjustImage();
}