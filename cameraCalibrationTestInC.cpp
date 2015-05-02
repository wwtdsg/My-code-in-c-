#include "cameraCalibrationTestInC.h"


using namespace cv;
using namespace std;

//————————相机标定使用变量————————————————
//输入参数：
char* adrSrcImage="SrcImage//%d.bmp";                //源图像文件夹地址
float squareSize=1.76f;                               //棋盘格物理尺寸。     
CvSize board_size=cvSize(5,3);                       //棋盘规格
int number_image=32;                                //相机标定图像总数

char filename[300]="";             //离线图片文件夹地址 //后续作为参数传递。
int iterations=100;               //迭代次数
double accuracy=0.01;              //收敛精度

//输出参数：
char* adrDstImage="DstImage//%d.bmp";         //标定后的图像文件夹地址
char* adrTempImage="TempImage//%d.bmp";       //过程图像文件夹地址
FILE* fp = fopen("DataResult.txt","w+");      //保存数据的txt文件地址
FILE* fpInfo=fopen("Information.txt","w+");   //保存异常信息
FILE* fpTest=fopen("LightErrorTest.txt","w+");//保存异常信息
int successes=0;                              //有效图片张数
IplImage * show;                              //图像指针
IplImage * show_test;                         //被校准后的图片指针

//中间变量

int board_width=board_size.width;
int board_height=board_size.height;
int total_per_image=board_width*board_height;                               //单张图像的角点数
CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];        //单张图像的角点
CvMat * image_points=cvCreateMat(number_image*total_per_image,2,CV_32FC1);  //图像坐标系
CvMat * object_points=cvCreateMat(number_image*total_per_image,3,CV_32FC1); //世界坐标系
CvMat * point_counts=cvCreateMat(number_image,1,CV_32SC1);         //每张图像的角点数量
CvMat * intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);                //内参数矩阵
CvMat * distortion_coeffs=cvCreateMat(5,1,CV_32FC1);               //畸变系数矩阵
CvMat *rotation_vectors= cvCreateMat(number_image,3,CV_32FC1);     //旋转矩阵
CvMat *translation_vectors = cvCreateMat(number_image,3,CV_32FC1); //平移矩阵

CvMat *object_points2Test=cvCreateMat(total_per_image,3,CV_32FC1); //临时性测试变量
CvMat *rotation_vectorsTest= cvCreateMat(1,3,CV_32FC1);
CvMat *translation_vectorsTest= cvCreateMat(1,3,CV_32FC1);
//————————————————————————
//

//打印数据
void printMT(CvMat* mat,int rows ,int cols,FILE* fp );


void getCornerPoints()
{
	int a=1;     //从第一张图片开始读取。
	int count;   //找到的角点个数
	int step;    //执行到第几张图片 
	while(a<=number_image)
	{
		cout<<"loop:"<<a<<endl;
		//加载图片   
		sprintf_s (filename,adrSrcImage,a);   
	
		show=cvLoadImage(filename,0);
		IplImage * gray_image= cvCreateImage(cvGetSize(show),8,1); 

		//棋盘角点检测      //TODO 2014-11-20
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
			//cvFindCornerSubPix(show,image_points_buf,count,board_size,cvSize(-1,-1),
			//	cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,iterations,accuracy));
			fprintf(fpInfo, "灰度图亚像素化过程完成...\n");
			//绘制角点
			cvDrawChessboardCorners(show,board_size,image_points_buf,count,found);
			fprintf(fpInfo, "在源图像上绘制角点过程完成...\n\n");
			//保存过程图像
			sprintf_s (filename,adrTempImage,a);  

			cvSaveImage(filename,show);
		}

		if(total_per_image==count)
		{
			step=successes*total_per_image;
			for(int i=step,j=0;j<total_per_image;++i,++j)
			{
				//顺序与世界坐标点的行列式保持一致。
				CV_MAT_ELEM(*image_points,float,i,0)=image_points_buf[j].x;  
				CV_MAT_ELEM(*image_points,float,i,1)=image_points_buf[j].y;   
			}
			CV_MAT_ELEM(*point_counts,int,successes,0)=total_per_image;
			successes++;
		}
		else
		{
			fprintf(fpTest, " \n\n");
			fprintf(fpTest,"第%d张图片角度提取失败！",a);
		}
		a++; 
		cvReleaseImage(&gray_image);
	}

	fprintf(fpInfo, "*********************************************\n");
	fprintf(fpInfo, "共 %d 帧图片中，标定成功的图片为 %d 帧!\n",number_image,successes);
	fprintf(fpInfo, "标定失败的图片为 %d 帧!\n",number_image-successes);
	fprintf(fpInfo, "*********************************************\n");
}
//单目相机标定
void calibrateCamera()
{
	fprintf(fpInfo, "开始摄像机标定!\n\n\n");

	if (object_points->rows>1)
	{
		cvCalibrateCamera2(object_points,image_points,point_counts,cvGetSize(show),
			intrinsic_matrix,distortion_coeffs,/*NULL*/rotation_vectors,/*NULL*/translation_vectors,
			0/*CV_CALIB_USE_INTRINSIC_GUESS/*CV_CALIB_FIX_PRINCIPAL_POINT|*/|CV_CALIB_FIX_K3  
			/*CV_CALIB_FIX_ASPECT_RATIO*//*|CV_CALIB_ZERO_TANGENT_DIST*/,cvTermCriteria(CV_TERMCRIT_EPS,iterations,accuracy));
	}
	cout<<"内参数："<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,0,0)<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,1,1)<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,0,2)<<endl;
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,1,2)<<endl;

	cout<<endl;
	cout<<"畸变："<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,0,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,1,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,2,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,3,0)<<endl;
	cout<<CV_MAT_ELEM(*distortion_coeffs,float,4,0)<<endl;

	cout<<endl;
	cout<<"z方向平移："<<endl;
	cout<<endl;
	for(int i=0; i<number_image; i++)
	{
		cout<<"第"<<i+1<<"幅标定图："<<endl;
		cout<<CV_MAT_ELEM(*translation_vectors,float,i,2)<<endl;
	}

	//数据记录到文件
	fprintf(fp,"\n");
	fprintf(fp,"摄像机内参数矩阵为：(3行3列)\n");
	for(int i=0;i<3;i++)
	{			
		fprintf(fp,"%.2f      %.2f     %.2f\t\n",CV_MAT_ELEM(*intrinsic_matrix,float,i,0),CV_MAT_ELEM(*intrinsic_matrix,float,i,1),
			CV_MAT_ELEM(*intrinsic_matrix,float,i,2));
	}
	fprintf(fp,"\n\n");

	fprintf(fp,"摄像机畸变矩阵为：(K1，K2，P1，P2，K3)\n");
	for (int i=0;i<5;i++)
	{
		fprintf(fp,"%.2f   ",CV_MAT_ELEM(*distortion_coeffs,float,i,0));
	}
	fprintf(fp,"\n\n");
	
	//saveParamXML();
	fprintf(fpInfo, "摄像机矩阵、畸变系数向量、外参系数矩阵已经分别存储在名为Intrinsics.xml、Distortion.xml、rotation_vectors.xml、translation_vectors.xml文档中\n\n");
}

//设定世界坐标数据
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



//计算标定误差
void calculateErrors()
{
	//打印统一的世界坐标信息
	fprintf(fp,"图片所对应的世界坐标数据(%d行%d列)：\n",board_size.height,board_size.width);
	for(int i=0;i<board_size.height;i++)
	{
		for (int j=0;j<board_size.width;j++)
		{
			fprintf(fp,"%.1f  %.1f \t",CV_MAT_ELEM(*object_points,float,i*board_size.width+j,0),CV_MAT_ELEM(*object_points,float,i*board_size.width+j,1));
		}		
		fprintf(fp,"\n");
	}

	fprintf(fp,"\n\n");
	//计算每一张图片的误差。
	CvMat * image_points_TEST=cvCreateMat(total_per_image,2,CV_32FC1);
	CvMat * rotTemp=cvCreateMat(1,3,CV_32FC1);
	CvMat * RtTemp33=cvCreateMat(3,3,CV_32FC1); //旋转向量转化过来的3X3矩阵。 

	for (int nImage=1;nImage<successes+1;nImage++)
	{
		//原始图像的坐标信息记录到文件
		fprintf(fp,"第%d张图片数据(%d行%d列)：\n",nImage,board_size.height,board_size.width);
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

		//生成世界坐标。
		InitCorners3D_temp(object_points2Test,board_size,1,squareSize);

		//将世界坐标转按计算的内外参数映射成图像坐标
		cvProjectPoints2(object_points2Test,rotation_vectorsTest,translation_vectorsTest,intrinsic_matrix,
			distortion_coeffs,image_points_TEST,NULL,NULL,NULL,NULL,NULL);
		//得到图像坐标image_points_TEST

		//世界坐标到图像坐标映射的数据
		fprintf(fp,"\n");
		fprintf(fp,"第%d张图片世界坐标到图像坐标映射的数据(%d行%d列)：\n",nImage,board_size.height,board_size.width);
		for(int i=0;i<board_size.height;i++)
		{
			for (int j=0;j<board_size.width;j++)
			{
				fprintf(fp,"%.1f  %.1f \t",CV_MAT_ELEM(*image_points_TEST,float,i*board_size.width+j,0),CV_MAT_ELEM(*image_points_TEST,float,i*board_size.width+j,1));
			}
			fprintf(fp,"\n");
		}

		//计算标定误差
		fprintf(fp,"\n");
		fprintf(fp,"第%d张图片计算标定误差(%d行%d列)：\n",nImage,board_size.height,board_size.width);
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
		fprintf(fp,"\n平均误差: %.2f  最大误差：%.2f \n",average,max);
		fprintf(fp,"\n");

		//打印外参数   todo test2014-12-3
		//for (int i=0;i<3;i++)
		//{
		//	fprintf(fp,"%f\t",CV_MAT_ELEM(*rotation_vectors,float,nImage-1,i));
		//}
		rotTemp=cvCreateMat(1,3,CV_32FC1);
		CV_MAT_ELEM(*rotTemp,float,0,0)=CV_MAT_ELEM(*rotation_vectors,float,nImage-1,0);
		CV_MAT_ELEM(*rotTemp,float,0,1)=CV_MAT_ELEM(*rotation_vectors,float,nImage-1,1);
		CV_MAT_ELEM(*rotTemp,float,0,2)=CV_MAT_ELEM(*rotation_vectors,float,nImage-1,2);

		RtTemp33=cvCreateMat(3,3,CV_32FC1); //旋转向量转化过来的3X3矩阵。 
		cvRodrigues2(rotTemp,RtTemp33,0);

		//fprintf(fp,"摄像机旋转矩阵为：(第%d张图片)\n",nImage);   //TODO test 12-19
		//printMT(rotTemp,1,3,fp);
		fprintf(fp,"第%d张图片摄像机旋转矩阵3X3为：\n",nImage);
		printMT(RtTemp33,3,3,fp);
		
		fprintf(fp,"第%d张图片摄像机平移矩阵+旋转矩阵为：\n",nImage);
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

//打印矩阵
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
	//保存内参
	cvSave("Intrinsics.xml",intrinsic_matrix);
	cvSave("Distortion.xml",distortion_coeffs);
}

//矫正图像
void adjustImage()
{
	//矫正畸变，测试效果。
	CvMat * intrinsic=(CvMat *)cvLoad("Intrinsics.xml");
	CvMat * distortion=(CvMat *)cvLoad("Distortion.xml");
	IplImage * mapx=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * mapy=cvCreateImage(cvGetSize(show),IPL_DEPTH_32F,1);
	IplImage * clone;
	//默认校准一次各图片
	for(int i=1;i<number_image+1;i++)
	{
		sprintf_s (filename,adrSrcImage,i);     
		show_test=cvLoadImage(filename,-1);

		//按内参校准图像
		cvInitUndistortMap(intrinsic,distortion,mapx,mapy);
		clone=cvCloneImage(show_test);
		cvRemap(clone,show_test,mapx,mapy);

		sprintf_s (filename,adrDstImage,i);  // 保存目标图像 
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


	//初始化世界坐标
	InitCorners3D_temp(object_points,board_size,successes,squareSize);

	//相机标定
	calibrateCamera();

	//计算标定误差
	calculateErrors();

	saveParamXML();

	//矫正图像
	adjustImage();
}