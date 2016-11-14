/**
* @file takeStereoImages.cpp
* @brief ステレオカメラで画像を取得する OpenCVは2.4.10を使用
* @author 13EJ034
* @date 最終更新日 : 2016/11/14
*/

#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
	int i = 1;
	double fps = 60.0;
	cv::Size cap_size(1280 * 2, 720);

	cv::VideoCapture cap;
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
	cap.set(CV_CAP_PROP_FPS, fps);
	
	if (!cap.isOpened()){
		std::cout << "Camera not found." << std::endl;
		return -1;
	}
	else{
		std::cout << "Camera has been opened." << std::endl;
	}
	
	const std::string windowName_l = "left";
	const std::string windowName_r = "right";

	cv::namedWindow(windowName_l, CV_WINDOW_NORMAL);
	cv::namedWindow(windowName_r, CV_WINDOW_NORMAL);

	cv::resizeWindow(windowName_l, cap_size.width * (5 / 8), cap_size.height * (5 / 8));
	cv::resizeWindow(windowName_r, cap_size.width * (5 / 8), cap_size.height * (5 / 8));

	cv::moveWindow(windowName_l, 100, 100);
	cv::moveWindow(windowName_r, 100 + cap_size.width * (5 / 8), 100);


	while (true) {

		cv::Mat frame;
		cap >> frame;

		cv::Mat frame_l = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
		cv::Mat frame_r = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		cv::imshow(windowName_l, frame_l);
		cv::imshow(windowName_r, frame_r);
		
		if (waitKey(5) >= 0)
		{
			std::string str = "Data/stereoImages/stereoImage" + to_string(i) + ".png";
			std::string str_l = "Data/leftImages/image_l_" + to_string(i) + ".png";
			std::string str_r = "Data/rightImages/image_r_" + to_string(i) + ".png";

			cv::imwrite(str, frame);
			cv::imwrite(str_l, frame_l);
			cv::imwrite(str_r, frame_r);

			std::cout << "Saved " << str << "," << str_l << "," << str_r << std::endl;
			
			i++;
			if (i > 21){
				break;
			}
			
		}
	
	}

	return 0;
}