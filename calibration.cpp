#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

void Calibration(float* A)
{
	Size patternsize(14, 14);// ���ñ궨��ǵ���Ϣ
	Mat gray = imread("biaoding.jpg", 0);
	Mat img = imread("biaoding.jpg");
	vector<Point2f> corners;
	bool patternfound = findChessboardCorners(gray, patternsize, corners, 
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		 + CALIB_CB_FAST_CHECK);

	drawChessboardCorners(img, patternsize, Mat(corners), patternfound);
	namedWindow("hh",2);
	imshow("hh", img);
	waitKey(0);
	//����궨����x�����y����ǵ����Ϊ5����
	float x = 2;
	float y = 2;
	float lx;
	float ly;
	float tempx = 0.0;
	float tempy = 0.0;
	//������궨�壬�����²�����Ҫ�Ķ�
	for(int i=0; i<14; i=i++)
	{
		lx = x*13 / (corners[14*i+13].x - corners[14*i].x); //ÿһ��һ�����ص��Ӧ��x���򳤶�
		tempx = tempx + lx;

		ly = x*13 / (corners[i+182].y - corners[i].y);
		tempy = tempy + ly;
	}
	lx = tempx / 14;
	ly = tempy / 14;
	//������궨�壬�����ϲ�����Ҫ�Ķ�
	/*cout<<"lx="<<lx<<endl;
	cout<<"ly="<<ly<<endl;*/
	A[0] = lx;
	A[1] = ly;
	

	/*for(int i=0; i<49; i++)
	{
		cout<<"x="<<corners[i].x;
		cout<<",y="<<corners[i].y<<endl;
	}
	imshow("d", img);
	waitKey(0);*/

}