// Please change it if there are more the one camera available (Start from 0)
#define CAMERA_NO 0
// The window name used in OpenCV for showing the frame
#define WINDOW_NAME "OpenCV Camera Test"

#include <cstdio>
#include "opencv2_include.h"

using namespace std;

int main(int argc, char* argv[]) {
	string cvWindowName = WINDOW_NAME;

	cv::VideoCapture cap(CAMERA_NO);	// Open the video camera
	if (!cap.isOpened()) {
		printf("Cannot open the video camera.\n");
		return -1;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);	//get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	
	printf("Frame size: %.0f x %.0f\n", dWidth, dHeight);

	cv::namedWindow(cvWindowName, CV_WINDOW_AUTOSIZE);

	for(;;) {
        cv::Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

		if (!bSuccess) { // If not success, break loop
			printf("Cannot read a frame from video stream.\n");
			break;
		}

		/* for(int i=0 ; i<dHeight ; i++) {
			for(int j=0 ; j<dWidth ; j++) {
				int p = (i * dWidth + j) * 3;
				frame.data[p] = 0;	// Set Each Pixel.B to 0
				frame.data[p+1] = 0;	// Set Each Pixel.G to 0
				uchar avg = (frame.data[p] + frame.data[p+1] + frame.data[p+2]) / 3;
				frame.data[p] = frame.data[p+1] = frame.data[p+2] = avg;
			}
		} */

		cv::imshow(cvWindowName, frame);	// show the frame

        if (cv::waitKey(30) == 27) { // wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			printf("Esc key is pressed by user.\n");
			break;
		}

	}
    return 0;
}