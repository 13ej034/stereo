#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char **argv){

	VideoCapture cap(0);
	if (!cap.isOpened()) return -1;
	Size cap_size(1344, 376);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

	int i = 1;

	while (true){

		Mat frame;
		cap >> frame;

		Mat frame_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		Mat frame_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		imshow("left", frame_l);
		imshow("right", frame_r);

		if (waitKey(15) == 13){
			string leftName = "left" + to_string(i) + ".png";
			string rightName = "right" + to_string(i) + ".png";

			imwrite(leftName, frame_l);
			imwrite(rightName, frame_r);

			i++;
		}

	}
	return 0;
}