#include <cv.h>
#include <highgui.h>
using namespace cv;
using namespace std;

//����ȫ�ֱ���
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
			imshow("����ͼ", m_img);

			cout<<"��ȡ���λ�óɹ�"<<endl;
			s=cvGet2D(Img,y,x);
			cout<<"��������Ϊ��"<<endl;
			cout<<"("<<x<<","<<y<<")"<<"="<<"("<< s.val[0]<<","<<s.val[1]<<","<<s.val[2]<<")"<<endl;
			cout<<"\n\n";
			mouse_location[mouse_count][0] = x;
			mouse_location[mouse_count][1] = y;
			if(mouse_count == 0)
				cout<<"�ȴ�����ڶ���"<<endl;
			//λ�Ƽ��㲢�����
			if(mouse_count > 0)
			{
				lenth = sqrt((mouse_location[0][0]-mouse_location[1][0])*(mouse_location[0][0]-mouse_location[1][0])*A[0]*A[0]
					+ (mouse_location[0][1]-mouse_location[1][1])*(mouse_location[0][1]-mouse_location[1][1])*A[1]*A[1]);
				cout<<"����λ��ֵΪ��"<<lenth<<"\n\n";
				mouse_count = 0;
				cout<<"�ȴ������һ��"<<endl;
				
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
	cvNamedWindow( "����ͼ", 2);//�ɸı䴰�ڵĴ�С��1-���ڵĴ�С���ɸı�
	cvSetMouseCallback( "����ͼ", on_mouse, 0 );
	cvShowImage( "����ͼ", Img ); //��ʾͼ��
	cout<<"�ȴ������һ��"<<endl;
	cvWaitKey(0); //�ȴ�����

	cvDestroyWindow( "����ͼ" );//���ٴ���
	cvReleaseImage( &Img ); //�ͷ�ͼ��
	return 0;
}