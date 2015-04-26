#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace std;
using namespace cv;

static Mat img;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		int b = img.data[img.channels()*(img.cols*y + x) + 0];
		int g = img.data[img.channels()*(img.cols*y + x) + 1];
		int r = img.data[img.channels()*(img.cols*y + x) + 2];
		cout << "RGB: (" << r << ", " << g << ", " << b << " ) " << endl;
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}

int nkmmain(int argc, char** argv)
{
	VideoCapture cap(0);
	// Read image from file 

	cap.read(img);

	//if fail to read the image
	if (img.empty())
	{
		cout << "Error loading the image" << endl;
		return -1;
	}

	//Create a window
	namedWindow("My Window", 1);

	//set the callback function for any mouse event
	setMouseCallback("My Window", CallBackFunc, NULL);

	//show the image
	imshow("My Window", img);

	// Wait until user press some key
	waitKey(0);

	return 0;

}