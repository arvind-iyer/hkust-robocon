/*
Simple object tracking based entirely on colour of pixels. To calibrate, simply click with Left Mouse Button at point on
Original Video Capture footage

*/



#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace cv;
using namespace std;


static Mat imgOriginal;
static Mat imgThresholded;
static Mat imgPrev;

static int iLowB = 0;
static int iHighB = 255;

static int iLowG = 0;
static int iHighG = 255;

static int iLowR = 0;
static int iHighR = 255;

//"delta" represents the sensitivity allowed.
#define delta 40

void CallBack(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		int b = imgOriginal.data[imgOriginal.channels()*(imgOriginal.cols*y + x) + 0];
		int g = imgOriginal.data[imgOriginal.channels()*(imgOriginal.cols*y + x) + 1];
		int r = imgOriginal.data[imgOriginal.channels()*(imgOriginal.cols*y + x) + 2];

		iLowB = b - delta;
		iHighB = b + delta;

		iLowG = g - delta;
		iHighG = g + delta;

		iLowR = r - delta;
		iHighR = r + delta;


		cout << "RGB: (" << r << ", " << g << ", " << b << " ) " << endl;
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}



int lpmain(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video


	int iLastX = -1;
	int iLastY = -1;

	//Capture a temporary image from the camera
	cap.read(imgPrev);
	imgPrev -= imgPrev;
	//Create a black image with the size as the camera output
	Mat imgLines = Mat::zeros(imgPrev.size(), CV_8UC3);;

	while (true)
	{

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video



		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		Mat imgHSV;
		imgOriginal.copyTo(imgHSV);
		//cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		//Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowB, iLowG, iLowR), Scalar(iHighB, iHighG, iHighR), imgThresholded); //Threshold the image


		//imgThresholded.copyTo(imgOriginal, );
		Mat out;
		imgOriginal.copyTo(out, imgThresholded);
		imshow("Thresholded Image", imgThresholded); //show the thresholded image

		//imgOriginal = imgOriginal - imgPrev;
		imshow("Original", imgOriginal); //show the original image

		imshow("Masked", out);
		//set the callback function for any mouse event
		setMouseCallback("Original", CallBack, NULL);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}

		/*imgPrev=imgOriginal.clone();
		waitKey(1);*/
	}

	return 0;
}