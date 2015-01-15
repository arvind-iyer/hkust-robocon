// Please change it if there are more the one camera available (Start from 0)
#define CAMERA_NO 2
// The window name used in OpenCV for showing the frame
#define WINDOW_NAME "libCMT"

#include <cstdio>
#include "CMT.h"

int main(int argc, char* argv[]) {
	std::string cvWindowName = WINDOW_NAME;
	
	// CMT
	CMT cmt;

	// Open Camera
	cv::VideoCapture cap(CAMERA_NO);
	if(!cap.isOpened()) {
		printf("Unable to open video input");
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
	/*

	# Read first frame
	status, im0 = cap.read()
	im_gray0 = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
	im_draw = np.copy(im0)

	if args.bbox is not None:
		# Try to disassemble user specified bounding box
		values = args.bbox.split(',')
		try:
			values = [int(v) for v in values]
		except:
			raise Exception('Unable to parse bounding box')
		if len(values) != 4:
			raise Exception('Bounding box must have exactly 4 elements')
		bbox = np.array(values)

		# Convert to point representation, adding singleton dimension
		bbox = util.bb2pts(bbox[None, :])

		# Squeeze
		bbox = bbox[0, :]

		tl = bbox[:2]
		br = bbox[2:4]
	else:
		# Get rectangle input from user
		(tl, br) = util.get_rect(im_draw)

	print 'using', tl, br, 'as init bb'


	CMT.initialise(im_gray0, tl, br)

	frame = 1
	while True:
		# Read image
		status, im = cap.read()
		if not status:
			break
		im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
		im_draw = np.copy(im)

		tic = time.time()
		CMT.process_frame(im_gray)
		toc = time.time()

		# Display results

		# Draw updated estimate
		if CMT.has_result:

			cv2.line(im_draw, CMT.tl, CMT.tr, (255, 0, 0), 4)
			cv2.line(im_draw, CMT.tr, CMT.br, (255, 0, 0), 4)
			cv2.line(im_draw, CMT.br, CMT.bl, (255, 0, 0), 4)
			cv2.line(im_draw, CMT.bl, CMT.tl, (255, 0, 0), 4)

		util.draw_keypoints(CMT.tracked_keypoints, im_draw, (255, 255, 255))
		# this is from simplescale
		util.draw_keypoints(CMT.votes[:, :2], im_draw)  # blue
		util.draw_keypoints(CMT.outliers[:, :2], im_draw, (0, 0, 255))

		if args.output is not None:
			# Original image
			cv2.imwrite('{0}/input_{1:08d}.png'.format(args.output, frame), im)
			# Output image
			cv2.imwrite('{0}/output_{1:08d}.png'.format(args.output, frame), im_draw)

			# Keypoints
			with open('{0}/keypoints_{1:08d}.csv'.format(args.output, frame), 'w') as f:
				f.write('x y\n')
				np.savetxt(f, CMT.tracked_keypoints[:, :2], fmt='%.2f')

			# Outlier
			with open('{0}/outliers_{1:08d}.csv'.format(args.output, frame), 'w') as f:
				f.write('x y\n')
				np.savetxt(f, CMT.outliers, fmt='%.2f')

			# Votes
			with open('{0}/votes_{1:08d}.csv'.format(args.output, frame), 'w') as f:
				f.write('x y\n')
				np.savetxt(f, CMT.votes, fmt='%.2f')

			# Bounding box
			with open('{0}/bbox_{1:08d}.csv'.format(args.output, frame), 'w') as f:
				f.write('x y\n')
				# Duplicate entry tl is not a mistake, as it is used as a drawing instruction
				np.savetxt(f, np.array((CMT.tl, CMT.tr, CMT.br, CMT.bl, CMT.tl)), fmt='%.2f') 

		if not args.quiet:
			cv2.imshow('main', im_draw)

			# Check key input
			k = cv2.waitKey(pause_time)
			key = chr(k & 255)
			if key == 'q':
				break
			if key == 'd':
				import ipdb; ipdb.set_trace()

		# Remember image
		im_prev = im_gray

		# Advance frame number
		frame += 1

		print '{5:04d}: center: {0:.2f},{1:.2f} scale: {2:.2f}, active: {3:03d}, {4:04.0f}ms'.format(CMT.center[0], CMT.center[1], CMT.scale_estimate, CMT.active_keypoints.shape[0], 1000 * (toc - tic), frame)
		*/


	return 0;
}