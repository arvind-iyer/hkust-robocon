
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2\opencv.hpp>

using namespace cv;
using namespace std;
int test()
{
	VideoCapture cap(0);
	/*
	Mat imgOriginal, img1;
	int flag = 0;
	while (cap.read(imgOriginal))
	{
		GaussianBlur(imgOriginal, img1, Size(9, 9));

		imshow("grayscale", img1);

		imshow("The Real OG", imgOriginal);
		if(waitKey(10)==30)
			break;

	}
	*/
	return 0;
}