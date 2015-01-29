// Please change it if there are more the one camera available (Start from 0)
#define CAMERA_NO 0
// The window name used in OpenCV for showing the frame
#define WINDOW_NAME "libCMT"

#include <cstdio>
#include "CMT.h"

cv::Point2f* current_pos;
cv::Point2f* tl;
cv::Point2f* br;
bool released_once = false;

void onMouse(int event, int x, int y, int flags, void* param) {
	// This function header is required by cv::setMouseCallback;
	if(current_pos == NULL) {
		current_pos = new cv::Point2f();
	}
	current_pos->x = x;
	current_pos->y = y;

	if(tl != NULL && !(flags && cv::EVENT_FLAG_LBUTTON)) {
		released_once = true;
	}

	if(flags && cv::EVENT_FLAG_LBUTTON) {
		if (tl == NULL) {
			tl = new cv::Point2f();
			tl->x = current_pos->x;
			tl->y = current_pos->y;
		} else if (released_once) {
			if(br == NULL) {
				br = new cv::Point2f();
			}
			br->x = current_pos->x;
			br->y = current_pos->y;
		}
	}
}

void getRect(cv::Mat &im, cv::Point2f &returnTl, cv::Point2f &returnBr, const std::string windowTitle = "getRect") {
	if(current_pos != NULL) {
		delete current_pos;
		current_pos = NULL;
	}
	if(tl != NULL) {
		delete tl;
		tl = NULL;
	}
	if(br != NULL) {
		delete br;
		br = NULL;
	}
	released_once = false;

	cv::namedWindow(windowTitle);
	cv::moveWindow(windowTitle, 100, 100);
	
	cv::setMouseCallback(windowTitle, onMouse);
	cv::imshow(windowTitle, im);

	while(br==NULL) {
		auto im_draw = im.clone();
		if(tl != NULL) {
			cv::Scalar borderColor(255,0,0);
			cv::rectangle(im_draw, *tl, *current_pos, borderColor);
		}
		cv::imshow(windowTitle, im_draw);
		cv::waitKey(10);
	}

	returnTl = *tl;
	returnBr = *br;
}

void drawKeypoints(std::vector<std::pair<cv::KeyPoint,int> > &keypoints, cv::Mat &im, cv::Scalar &color) {
	for(auto k : keypoints) {
		int radius = 3; // int(k.size / 2)
		auto center = k.first.pt;
		cv::circle(im, center, radius, color);
	}
}

void drawKeypoints(std::vector<cv::Point2f> &keypoints, cv::Mat &im, cv::Scalar &color) {
	for(auto k : keypoints) {
		int radius = 3;
		auto center = k;
		cv::circle(im, center, radius, color);
	}
}

int main(int argc, char* argv[]) {
	int k;
	unsigned char key;

	// cv::initModule_nonfree();

	std::string cvWindowName = WINDOW_NAME;
	
	// CMT
	CMT cmt;

	// Open Camera
	cv::VideoCapture cap(CAMERA_NO);
	if(!cap.isOpened()) {
		printf("Unable to open video input.\n");
		return 1;
	}

	// Preview
	const std::string previewWindowTitle = "Preview";
	cv::Mat im;
	bool status;
	for(;;) {
		status = cap.read(im);
		cv::imshow(previewWindowTitle, im);
		if(cv::waitKey(10)!=-1) {
			break;
		}
	}

	// Read first frame
	cv::Mat im0, im_gray0;
	status = cap.read(im0);
	cv::cvtColor(im0, im_gray0, cv::COLOR_BGR2GRAY);
	auto im_draw = im0.clone();

	cv::Point2f tl, br;
	getRect(im_draw, tl, br);
	printf("using (%.0f, %.0f), (%.0f, %.0f) as init bb\n", tl.x, tl.y, br.x, br.y );

	cmt.initialise(im_gray0, tl, br);
	auto frame = 1;

	for(;;) {
		auto status = cap.read(im);
		cv::Mat im_prev;
		if(!status) {
			break;
		}
		cv::Mat im_gray;
		cv::cvtColor(im, im_gray, cv::COLOR_BGR2GRAY);
		im_draw = im.clone();

		// tic = ?
		cmt.processFrame(im_gray);
		// toc = ?

		
		cv::Scalar redColor(0,0,255), blueColor(255,0,0), whiteColor(255,255,255);

		if(cmt.hasResult) {
			cv::line(im_draw, cmt.topLeft, cmt.topRight, blueColor, 4);
			cv::line(im_draw, cmt.topRight, cmt.bottomRight, blueColor, 4);
			cv::line(im_draw, cmt.bottomRight, cmt.bottomLeft, blueColor, 4);
			cv::line(im_draw, cmt.bottomLeft, cmt.topLeft, blueColor, 4);
		}

		drawKeypoints(cmt.trackedKeypoints, im_draw, whiteColor);
		// This is from simplescale
		drawKeypoints(cmt.votes, im_draw, blueColor);
		drawKeypoints(cmt.outliers, im_draw, blueColor);

		cv::imshow("main", im_draw);

		k = cv::waitKey(10);
		key = (char)(k & 255);
		if(key=='q') {
			break;
		} else if(key=='d') {
			//import ipdb; ipdb.set_trace()
		}

		im_prev = im_gray;
		frame += 1;

		//printf("center: (%.2f, %.2f), scale: %.2f, active: %d", cmt.
		
	}

	return 0;
}