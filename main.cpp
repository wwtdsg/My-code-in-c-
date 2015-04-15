#include <cv.h>
#include <highgui.h>
using namespace cv;
using namespace std;

//定义全局变量
IplImage* pImg,*Img;
Mat m_img;
int flags = 0;
CvPoint pt;
CvScalar s;
void Calibration(float *A);
double mouse_location[2][2];
double lenth;
int mouse_count = 0;
float A[2];

void on_mouse( int event, int x, int y, int flags, void* param )
{
	if( !Img )
		return;
	
	if(event == CV_EVENT_LBUTTONDOWN)
		{

			pt = cvPoint(x, y);
			circle( m_img, pt, 1,cvScalar(200,255,0) ,-1, CV_AA, 0 );
			imshow("待测图", m_img);

			cout<<"获取鼠标位置成功"<<endl;
			s=cvGet2D(Img,y,x);
			cout<<"像素坐标为："<<endl;
			cout<<"("<<x<<","<<y<<")"<<"="<<"("<< s.val[0]<<","<<s.val[1]<<","<<s.val[2]<<")"<<endl;
			cout<<"\n\n";
			mouse_location[mouse_count][0] = x;
			mouse_location[mouse_count][1] = y;
			if(mouse_count == 0)
				cout<<"等待点击第二点"<<endl;
			//位移计算并输出：
			if(mouse_count > 0)
			{
				lenth = sqrt((mouse_location[0][0]-mouse_location[1][0])*(mouse_location[0][0]-mouse_location[1][0])*A[0]*A[0]
					+ (mouse_location[0][1]-mouse_location[1][1])*(mouse_location[0][1]-mouse_location[1][1])*A[1]*A[1]);
				cout<<"两点位移值为："<<lenth<<"\n\n";
				mouse_count = 0;
				cout<<"等待点击第一点"<<endl;
				
			}
			else
				mouse_count++;
		}
}

//float caculate_lenth(

int main()
{
	
	Calibration(A);
	cout<<"A[0]="<<A[0]<<"---A[1]="<<A[1]<<endl;
	Img = cvLoadImage( "daice.jpg", 1);
	m_img = imread("daice.jpg", 1);
	cvNamedWindow( "待测图", 2);//可改变窗口的大小。1-窗口的大小不可改变
	cvSetMouseCallback( "待测图", on_mouse, 0 );
	cvShowImage( "待测图", Img ); //显示图像
	cout<<"等待点击第一点"<<endl;
	cvWaitKey(0); //等待按键

	cvDestroyWindow( "待测图" );//销毁窗口
	cvReleaseImage( &Img ); //释放图像
	return 0;
}