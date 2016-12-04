/**
* @file visualServo.cpp
* @brief ステレオカメラを用いたロボットの駆動制御
*/

#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <runCtrl.h>
#include <vutils.h>

#pragma comment(lib, "vxv2.lib")

using namespace std;
using namespace cv;

// main

int main(int argc, char *argv[]){

	// カメラ起動設定

	const double cap_fps = 30.0;
	Size cap_size(1280 * 2, 720);
	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
	cap.set(CV_CAP_PROP_FPS, cap_fps);
	if (!cap.isOpened()){	// カメラ起動確認
		cout << "Camera could not found." << endl;
		return false;
	}
	else{
		cout << "Camera has been opened." << endl;
	}

	// 録画設定

	const string saveMovieName_l = "./Data/movie/result_l.avi";
	const string saveMovieName_r = "./Data/movie/result_r.avi";
	Size rec_size(cap_size.width / 2, cap_size.height);
	const int codec = CV_FOURCC('D', 'I', 'V', '3');
	const double rec_fps = 15.0;
	VideoWriter writer_l(saveMovieName_l, codec, rec_fps, rec_size, true);
	VideoWriter writer_r(saveMovieName_r, codec, rec_fps, rec_size, true);

	// ウィンドウ設定

	const string windowName_l = "left";
	const string windowName_r = "right";
	namedWindow(windowName_l, CV_WINDOW_NORMAL);
	namedWindow(windowName_r, CV_WINDOW_NORMAL);
	resizeWindow(windowName_l, 800, 450);
	resizeWindow(windowName_r, 800, 450);
	moveWindow(windowName_l, 100, 100);
	moveWindow(windowName_r, 100 + 820, 100);

	// ロボット構成

	static RunCtrl run;
	const int motor_r = 0;
	const int motor_l = 1;
	const string robot_COM = "COM6";
	int PWM = 200;	
	if (run.connect(robot_COM) < 0){
		cout << "beego not found." << endl;
		return false;
	}
	else{
		cout << "beego has been connected." << endl;
		run.setWheelVel(motor_r, 5);
		run.setWheelVel(motor_l, 5);
	}

	// マーカー検出用変数

	Point2d d11, d12, d21, d22, rmin1, rmax1, rmin2, rmax2;

	// 時間測定用変数 

	TickMeter one_roop_timer;
	double total_time = 0;
	ofstream run_time("run_time.txt");

	// fps測定用変数

	int cnt = 0;
	int oldcnt = 0;
	int64 nowTime = 0;
	int64 diffTime = 0;
	int fps = 0;
	const double f = (1000 / getTickFrequency());
	Point point(2, 28);
	int roop_count = 0;

	//内部パラメータ

	const double fku_l = 758.963535485534;
	const double fkv_l = 753.717143388798;
	const double cx_l = 663.977396599560;
	const double cy_l = 348.145988560347;

	const double fku_r = 762.693976978763;
	const double fkv_r = 757.502496252634;
	const double cx_r = 661.685897420813;
	const double cy_r = 348.579796095923;

	Mat cameraParameter_l = (Mat_<double>(3, 3) << fku_l, 0., cx_l, 0., fkv_l, cy_l, 0., 0., 1.);
	Mat cameraParameter_r = (Mat_<double>(3, 3) << fku_r, 0., cx_r, 0., fkv_r, cy_r, 0., 0., 1.);

	const double k1_l = -0.105077838090415;
	const double k2_l = -0.0370249961738757;
	const double p1_l = 0.0;
	const double p2_l = 0.0;

	const double k1_r = -0.115246237256405;
	const double k2_r = -0.0215251205172864;
	const double p1_r = 0.0;
	const double p2_r = 0.0;

	Mat distCoeffs_l = (Mat_<double>(1, 4) << k1_l, k2_l, p1_l, p2_l);
	Mat distCoeffs_r = (Mat_<double>(1, 4) << k1_r, k2_r, p1_r, p2_r);

	Mat frame;
	Mat frame_l;
	Mat frame_r;
	Mat undistort_l;
	Mat undistort_r;
	Mat gray_l;
	Mat gray_r;
	Mat edge_l;
	Mat edge_r;

	vector<vector<Point>> contours_l;
	vector<vector<Point>> contours_r;
	vector<Vec4i> hierarchy_l;
	vector<Vec4i> hierarchy_r;

	// 処理開始

	cout << "Start!" << endl;

	int64 startTime = getTickCount();

	while (true){

		one_roop_timer.reset();
		one_roop_timer.start();	

		cap >> frame;
		
		// ステレオイメージを左右に分割

		frame_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		frame_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		// 歪み補正

		undistort(frame_l, undistort_l, cameraParameter_l, distCoeffs_l);
		undistort(frame_r, undistort_r, cameraParameter_r, distCoeffs_r);

		// グレースケール化

		cvtColor(undistort_l, gray_l, CV_BGR2GRAY);
		cvtColor(undistort_r, gray_r, CV_BGR2GRAY);

		// エッジを抽出

		Canny(undistort_l, edge_l, 50, 150, 3);
		Canny(undistort_r, edge_r, 50, 150, 3);

	// マーカー認識処理

		// 輪郭を抽出

		findContours(edge_l, contours_l, hierarchy_l, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0));
		findContours(edge_r, contours_r, hierarchy_r, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0));

		// 輪郭をフレームに描画する

		// 追加する予定

		// fpsをframe上に表示する
		
		nowTime = getTickCount();
		diffTime = (int)((nowTime - startTime)*f);
		if (diffTime >= 1000) {
			startTime = nowTime;
			fps = cnt - oldcnt;
			oldcnt = cnt;
		}
		ostringstream os;
		os << fps;
		string number = os.str();
		string put_fps = "fps:" + number;
		cnt++;
		putText(undistort_l, put_fps, point, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 200), 2, CV_AA);
		putText(undistort_r, put_fps, point, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 200), 2, CV_AA);

		// 録画

		writer_l << undistort_l;
		writer_r << undistort_r;

		// キャプチャを表示

		imshow(windowName_l, undistort_l);
		imshow(windowName_r, undistort_r);

		// モータ駆動

		run.setMotorPwm(motor_r, PWM);
		run.setMotorPwm(motor_l, PWM);

		// ループ回数を測定

		one_roop_timer.stop();
		run_time << one_roop_timer.getTimeMilli() << std::endl;
		total_time += one_roop_timer.getTimeMilli();
		roop_count++;

		if (waitKey(10) > 0){
			break;
		}
	}
	
	double ave_time = total_time / roop_count;
	cout << "total time : " << total_time << "[ms]" << endl
		<< "roop : " << roop_count << endl
		<< "average time : " << ave_time << "[ms]" << endl;
	
	for (int i = PWM; i >= 0; i=i-PWM/8){
		run.setMotorPwm(motor_r, i);
		run.setMotorPwm(motor_l, i);
	}

	return 0;
}