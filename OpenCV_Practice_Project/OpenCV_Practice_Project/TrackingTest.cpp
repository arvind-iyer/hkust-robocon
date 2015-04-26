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

static int iLowB = 0;
static int iHighB = 255;

static int iLowG = 0;
static int iHighG = 255;

static int iLowR = 0;
static int iHighR = 255;

//"delta" represents the sensitivity allowed.
#define delta 40

void CallBackFunc1(int event, int x, int y, int flags, void* userdata)
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

void callBack2(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		int b = imgThresholded.data[imgThresholded.channels()*(imgThresholded.cols*y + x) + 0];
		int g = imgThresholded.data[imgThresholded.channels()*(imgThresholded.cols*y + x) + 1];
		int r = imgThresholded.data[imgThresholded.channels()*(imgThresholded.cols*y + x) + 2];

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

int lpllppmain(int argc, char** argv)
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
	Mat imgTmp;
	cap.read(imgTmp);

	//Create a black image with the size as the camera output
	Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);;

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

		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (removes small holes from the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000)
		{
			//calculate the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;

			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
			{
				//Draw a red line from the previous point to the current point
				line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 255, 255), 2);
			}

			iLastX = posX;
			iLastY = posY;
		}
		
		//imgThresholded.copyTo(imgOriginal, );

		imshow("Thresholded Image", imgThresholded); //show the thresholded image

		imgOriginal = imgOriginal + imgLines;
		imshow("Original", imgOriginal); //show the original image

		//set the callback function for any mouse event
		setMouseCallback("Original", CallBackFunc1, NULL);
		setMouseCallback("Thresholded Image", callBack2, NULL);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;
}