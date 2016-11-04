#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

int
main(int argc, char *argv[])
{
	int i = 1;
	double fps = 60.0;

	VideoCapture cap(0);	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CV_CAP_PROP_FPS, fps);
	
	if (!cap.isOpened()){
		cout << "Camera can not open." << endl;
		return -1;
	}
	else{
		cout << "Camera has been opened." << endl;
	}
	
	string strLeftWindow = "Left Image";
	string strRightWindow = "Right Image";

	namedWindow(strLeftWindow, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
	namedWindow(strRightWindow, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
	resizeWindow(strLeftWindow, 720, 404);
	resizeWindow(strRightWindow, 720, 404);


	while (true) {
		Mat frame;
		cap >> frame;

		Mat leftFrame = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		Mat rightFrame = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		imshow(strLeftWindow, leftFrame);
		imshow(strRightWindow, rightFrame);
		
		if (waitKey(20) >= 0)
		{
			string str = "Data/stereoImages/stereoImage" + to_string(i) + ".png";
			string str2 = "Data/leftImages/image_l_" + to_string(i) + ".png";
			string str3 = "Data/rightImages/image_r_" + to_string(i) + ".png";

			imwrite(str, frame);
			imwrite(str2, leftFrame);
			imwrite(str3, rightFrame);

			cout << "Saved " << endl << str << endl << str2 << endl << str3 << endl << endl;
			
			i++;
			if (i > 21){
				break;
			}
			
		}
	
	}

	VideoCapture release(cap);
	destroyAllWindows();

	return 0;
}