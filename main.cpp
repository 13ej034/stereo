/**
* @file main.cpp
* @brief �[�x��񂩂�ʉ߉\�̈�𔻕ʂ���
* @author 13ej034
* @date 2016.12.24
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <chrono>

using namespace cv;
using namespace std;

void detectSquare(Mat &InputOutputArray, vector<vector<Point>> InputContours,
	chrono::time_point<chrono::system_clock, chrono::system_clock::duration> &InputStarttime);
void estimateGravityCenter(Mat &InputOutputArray, vector<Point> &InputApprox);
void changeToHueColor(Mat Imput,Mat &Output);

// �l�p�`�����o
void detectSquare(Mat &InputOutputArray, vector<vector<Point>> InputContours,
	chrono::time_point<chrono::system_clock, chrono::system_clock::duration> &InputStarttime){
	for (auto i = InputContours.begin(); i != InputContours.end(); i++){

		// �֊s�𒼐��ߎ�
		vector<Point> approx;
		approxPolyDP(Mat(*i), approx, 0.01 * arcLength(*i, true), true);

		// �ߎ������������̖ʐς��Z�o
		double area = contourArea(approx);

		// ���_���W��4�� ���� �ʐς�1000�ȏ�Ŏl�p�`�ł���Ɣ��肷��
		if (approx.size() == 4 && area > 1000.0){

			estimateGravityCenter(InputOutputArray, approx);

			auto checkTime = chrono::system_clock::now();	// �l�p�`�����o��������
			double elapsedTime = chrono::duration_cast<std::chrono::milliseconds>(checkTime - InputStarttime).count();	// �����J�n���Ԃ���l�p�`���o�܂ł̌o�ߎ���

			// �摜��ۑ�
			ostringstream streamOutputFileName;
			streamOutputFileName << elapsedTime;
			string outputFileName = "data/detectSquare_" + streamOutputFileName.str() + "msec.png";
			polylines(InputOutputArray, approx, true, Scalar(255, 0, 0), 2);
			imwrite(outputFileName, InputOutputArray);
		}
	}
}


// �d�S����
void estimateGravityCenter(Mat &InputOutputArray,vector<Point> &InputApprox){

	Moments mu_approx = moments(InputApprox);
	Point2f mc_approx = Point2f(mu_approx.m10 / mu_approx.m00, mu_approx.m01 / mu_approx.m00);
	circle(InputOutputArray, mc_approx, 4, Scalar(0, 0, 100), 2, 4);
	ostringstream Gravity;
	Gravity << mc_approx;
	string OutputGravity = Gravity.str();
	putText(InputOutputArray, OutputGravity, mc_approx, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(200, 200, 0), 1, CV_AA);
}

// �J���[��
void changeToHueColor(Mat InputArray, Mat &OutputArray){
	Mat channel[3];
	channel[0] = Mat(InputArray.size(), CV_8UC1);
	channel[1] = Mat(InputArray.size(), CV_8UC1, 255);
	channel[2] = Mat(InputArray.size(), CV_8UC1, 255);
	Mat hsv;
	int d;
	for (int i = 0; i < InputArray.cols; i++){
		for (int j = 0; j < InputArray.rows; j++){
			d = InputArray.at<uchar>(j, i);
			channel[0].at<uchar>(j, i) = (255 - d) / 2;
		}
		merge(channel, 3, hsv);
		cvtColor(hsv, OutputArray, CV_HSV2BGR);
	}
}



int main(int argc, const char* argv[])
{
	//�����p�����[�^
	const double fku_l = 717.720635659324;
	const double fkv_l = 716.012167656560;
	const double cx_l = 669.090942511285;
	const double cy_l = 352.652835236136;

	const double fku_r = 718.249831068048;
	const double fkv_r = 716.243173393304;
	const double cx_r = 675.677025980489;
	const double cy_r = 353.446662559247;

	Mat cameraParameter_l = (Mat_<double>(3, 3) << fku_l, 0., cx_l, 0., fkv_l, cy_l, 0., 0., 1.);
	Mat cameraParameter_r = (Mat_<double>(3, 3) << fku_r, 0., cx_r, 0., fkv_r, cy_r, 0., 0., 1.);

	// �c�݌W��
	const double k1_l = -0.159780304278012;
	const double k2_l = 0.000684899487477897;
	const double p1_l = 0.0;
	const double p2_l = 0.0;

	const double k1_r = -0.161702334521399;
	const double k2_r = 0.00604078284205945;
	const double p1_r = 0.0;
	const double p2_r = 0.0;

	Mat distCoeffs_l = (Mat_<double>(1, 4) << k1_l, k2_l, p1_l, p2_l);
	Mat distCoeffs_r = (Mat_<double>(1, 4) << k1_r, k2_r, p1_r, p2_r);

	// 
	VideoCapture cap(0);
	if (!cap.isOpened()) return -1;
	Size cap_size(2560, 720);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

	namedWindow("Left", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	namedWindow("Right", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	namedWindow("depth", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	namedWindow("depth clone", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

	int window_size = 3;
	int minDisparity = 0;
	int numDisparities = 48;
	int blockSize = 3;
	int P1 = 8 * 3 * window_size * window_size;
	int P2 = 32 * 3 * window_size * window_size;
	int disp12MaxDiff = 1;
	int preFilterCap = 0;
	int uniquenessRatio = 10;
	int speckleWindowSize = 100;
	int speckleRange = 32;

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(
		minDisparity,
		numDisparities,
		blockSize,
		P1,
		P2, 
		disp12MaxDiff,
		preFilterCap,
		uniquenessRatio,
		speckleWindowSize,
		speckleRange,
		cv::StereoSGBM::MODE_SGBM_3WAY);

	double baseline = 120.0;

	auto startTime = chrono::system_clock::now();	// �����J�n����
	double processingTime = 0;
	double previousTime = 0;

	while (true){

		// �摜�擾
		Mat frame;
		cap >> frame;

		// ����
		Mat frame_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		Mat frame_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		Size frameSize(frame_l.cols, frame_l.rows);

		// �␳
		
		Mat undistort_l, undistort_r;
		Mat mapx_l, mapy_l,mapx_r,mapy_r;
		initUndistortRectifyMap(cameraParameter_l, distCoeffs_l, Mat(), cameraParameter_l, frameSize, CV_32FC1, mapx_l, mapy_l);
		initUndistortRectifyMap(cameraParameter_r, distCoeffs_r, Mat(), cameraParameter_r, frameSize, CV_32FC1, mapx_r, mapy_r);
		remap(frame_l, undistort_l, mapx_l, mapy_l, INTER_LINEAR);
		remap(frame_r, undistort_r, mapx_r, mapy_r, INTER_LINEAR);
		

		// �O���[�X�P�[����
		Mat gray_l,gray_r;
		cvtColor(undistort_l, gray_l, CV_BGR2GRAY);
		cvtColor(undistort_r, gray_r, CV_BGR2GRAY);
		
		// ��l��
		//Mat binaly_l,binaly_r;
		//threshold(gray_l, binaly_l, 0.0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);
		//threshold(gray_r, binaly_r, 0.0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);

		// �֊s���o
		//vector<vector<Point>> contours_l, contours_r;
		//findContours(binaly_l, contours_l, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		//findContours(binaly_r, contours_r, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		// �l�p�`���o
		//detectSquare(undistort_l, contours_l, startTime);
		//detectSquare(undistort_r, contours_r, startTime);

		// SGBM
		
		Mat disparity;
		sgbm->compute(gray_l, gray_r, disparity);

		Mat disparity_float;
		disparity.convertTo(disparity_float, CV_32F);

		// 3D
		Mat depth = fku_l * baseline / disparity_float;
		
		// depth�̐؂蔲��
		Mat depth_clone = depth.clone();
		for (int y = 0; y < depth_clone.rows; y++){
			uchar *ptr = depth_clone.ptr<uchar>(y);
			for (int x = 0; x < depth_clone.cols; x++){
				uchar dep = ptr[x];
				double Z = dep;
				//double X = (x - cx_l) * Z / fku_l;
				//double Y = (y - cy_l) * Z / fkv_l;
				if (Z <= 150 && Z >= 100 ){
				}
				else{
					ptr[x] = uchar(0);
				}
			}
		}
		
		auto checkTime = chrono::system_clock::now();
		double elapsedTime = chrono::duration_cast<std::chrono::milliseconds>(checkTime - startTime).count();
		processingTime = elapsedTime - previousTime;
		previousTime = elapsedTime;
		ostringstream elapsed,processing;
		elapsed << elapsedTime;
		processing << processingTime;
		string elapsedTimeStr = "elapsed time : " + elapsed.str() + "msec";
		string processingTimeStr = "processing time : " + processing.str() + "msec";
		cout << elapsedTimeStr << endl
			<< processingTimeStr << endl;

		Mat depth_show;
		depth.convertTo(depth_show, CV_32S);

		Mat depth_clone_show;
		depth_clone.convertTo(depth_clone_show, CV_32S);

		imshow("Left", undistort_l);
		imshow("Right", undistort_r);
		imshow("depth", depth_show);
		imshow("depth clone", depth_clone_show);

		//string saveRgbFileName = "data/calib_left/frame" + elapsed.str() + ".png";
		//string saveDepthFileName = "data/calib_right/frame" + elapsed.str() + ".png";
		//imwrite(saveRgbFileName, frame_l);
		//imwrite(saveDepthFileName, frame_r);

		if (waitKey(30) >= 0){
			break;
		}
	}
	return 0;
}