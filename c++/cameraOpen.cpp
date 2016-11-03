#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]){
	
	Size cap_size(640, 480);
	const double fps = 30.0;

	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
	cap.set(CV_CAP_PROP_FPS, fps);
	if (!cap.isOpened()){
		cout << "Camera can not open." << endl;
		return -1; 
	}
	else{
		cout << "Camera has been opened." << endl;
	}

	Mat frame;

	string windowName = "frame";
	namedWindow(windowName,CV_WINDOW_AUTOSIZE);

	while (true) {
		do{
			cap >> frame;
		} while (frame.empty());

		imshow(windowName, frame);
		
		if (waitKey(30) > 0){
			break;
		}
	}

	return 0;
}