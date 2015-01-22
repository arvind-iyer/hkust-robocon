/*
Simple object tracking based entirely on colour of pixels. To calibrate, simply click with Left Mouse Button at point on
Original Video Capture footage

*/



#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2\opencv.hpp>



using namespace cv;
using namespace std;


static Mat imgOriginal;

BackgroundSubtractorMOG2 bg(50, 10, false);//reduced to 50 cause my computer is slowwwwww

static int iLowB = 0;
static int iHighB = 255;

static int iLowG = 0;
static int iHighG = 255;

static int iLowR = 0;
static int iHighR = 255;

//"delta" represents the sensitivity allowed.
#define delta 40

void CallBack2(int event, int x, int y, int flags, void* userdata)
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



int main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	Mat imgThresholded;
	Mat imgPrev;
	Mat imgFG;
	Mat mogMasked;
	//Capture a temporary image from the camera
	cap.read(imgPrev);


	while (true)
	{

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video
		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		bg.operator()(imgOriginal, imgFG);
		//imshow("MOG", imgFG);//only moving pixels
		inRange(imgOriginal, Scalar(iLowB, iLowG, iLowR), Scalar(iHighB, iHighG, iHighR), imgThresholded); //Threshold the image

		Mat out;
		//Apply imgThresholded as mask on imgOriginal to out
		imgOriginal.copyTo(out, imgThresholded);

		//imshow("Thresholded Image", imgThresholded); //only selected colour shown as B/W
		imshow("Original", imgOriginal); //show the original image
		//imshow("Masked", out); //only selected colour with retained RGB values


		//Apply imgFg as mask on out to mogMasked
		out.copyTo(mogMasked, imgFG);
		imshow("MOG'd", mogMasked); //only the selected colour and only if it is moving, i.e. not background
		bg.operator()(mogMasked.clone(), mogMasked);
		//set the callback function for any mouse event
		setMouseCallback("Original", CallBack2, NULL);

		if (waitKey(1) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}

	}

	return 0;
}