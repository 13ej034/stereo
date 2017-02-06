/**
* @file main.cpp
* @brief 深度情報から通過可能領域を判別する
* @author 13ej034
* @date 2017.1.27
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <chrono>
#include <runCtrl.h>
#include <vutils.h>

using namespace cv;
using namespace std;

int main(int argc, const char* argv[])
{

	const double fku_l = 720.662839213821;
	const double fkv_l = 718.406797571986;
	const double cx_l = 292.267044761211;
	const double cy_l = 242.558077956841;

	const double fku_r = 723.920825845424;
	const double fkv_r = 722.036833934553;
	const double cx_r = 314.804058188331;
	const double cy_r = 219.792366985127;

	Mat cameraParameter_l = (Mat_<double>(3, 3) << fku_l, 0., cx_l, 0., fkv_l, cy_l, 0., 0., 1.);
	Mat cameraParameter_r = (Mat_<double>(3, 3) << fku_r, 0., cx_r, 0., fkv_r, cy_r, 0., 0., 1.);

	const double k1_l = -0.106670002338667;
	const double k2_l = -0.0442327810840701;
	const double p1_l = 0.0;
	const double p2_l = 0.0;

	const double k1_r = -0.130118213894710;
	const double k2_r = 0.0802158917652363;
	const double p1_r = 0.0;
	const double p2_r = 0.0;

	Mat distCoeffs_l = (Mat_<double>(1, 4) << k1_l, k2_l, p1_l, p2_l);
	Mat distCoeffs_r = (Mat_<double>(1, 4) << k1_r, k2_r, p1_r, p2_r);

	double width_robot = 46.0;	// [cm]

	double r = 0;

	VideoCapture cap(0);
	if (!cap.isOpened()) return -1;
	Size cap_size(1280, 480);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

	VideoWriter sceneVideo("scene.avi", VideoWriter::fourcc('M', '4', 'S', '2'), 10, Size(640,480),true);
	if (!sceneVideo.isOpened()){ return -1; }

	int minDisparity = 16 * 0;
	int numDisparities = 16 * 4;
	int blockSize = 3;
	int P1 = 0;
	int P2 = 200;
	int disp12MaxDiff = 0;
	int preFilterCap = 0;
	int uniquenessRatio = 0;
	int speckleWindowSize = 0;
	int speckleRange = 1;

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
		StereoSGBM::MODE_SGBM_3WAY);

	Ptr<StereoBM> bm = StereoBM::create(0, 15);

	double baseline = 120.0;

	double error = 0;
	double pre_error = 0;
	double Kp = 0.10;
	double Ki = 0.05;
	double Kd = 0.02;
	double P = 0;
	double I = 0;
	double D = 0;
	double U = 0;
	double integral = 0;
	int lr = 0;

	int robot_switch = 0;

	static RunCtrl run;
	run.connect("COM6");

	const int motor_r = 0;
	const int motor_l = 1;

	if (robot_switch == 0){
		run.setWheelVel(motor_r, 0);
		run.setWheelVel(motor_l, 0);
	} 
	else{
		run.setWheelVel(motor_r, 5);
		run.setWheelVel(motor_l, 5);
	}

	auto startTime = chrono::system_clock::now();
	double processingTime = 0;
	double previousTime = 0;

	while (true){

		Mat frame;
		cap >> frame;

		Mat frame_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		Mat frame_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		Size frameSize(frame_l.cols, frame_l.rows);
		
		Mat undistort_l, undistort_r;
		Mat mapx_l, mapy_l,mapx_r,mapy_r;
		initUndistortRectifyMap(cameraParameter_l, distCoeffs_l, Mat(), cameraParameter_l, frameSize, CV_32FC1, mapx_l, mapy_l);
		initUndistortRectifyMap(cameraParameter_r, distCoeffs_r, Mat(), cameraParameter_r, frameSize, CV_32FC1, mapx_r, mapy_r);
		remap(frame_l, undistort_l, mapx_l, mapy_l, INTER_LINEAR);
		remap(frame_r, undistort_r, mapx_r, mapy_r, INTER_LINEAR);
		
		Mat gray_l,gray_r;
		cvtColor(undistort_l, gray_l, CV_BGR2GRAY);
		cvtColor(undistort_r, gray_r, CV_BGR2GRAY);
		
		Mat disparity;
		sgbm->compute(gray_l, gray_r, disparity);
		//bm->compute(gray_l, gray_r, disparity);

		// CV_16S -> CV_64F
		Mat disparity_64f;
		disparity.convertTo(disparity_64f, CV_64F);

		// 深度情報に変換
		Mat depth = fku_l * baseline / disparity_64f;
		
		// 領域認識
		
		Mat depth_clone = depth.clone();
		for (int y = 0; y < depth_clone.rows; y++){
			double *dep = depth_clone.ptr<double>(y);
			double *dis = depth_clone.ptr<double>(y);
			for (int x = 0; x < depth_clone.cols; x++){
				if (dep[x] > 250 || dep[x] < 30) dep[x] = double(0);
			}
		}

		Mat cut = depth_clone(Rect(70, depth_clone.rows - (depth_clone.rows / 2), depth_clone.cols - 70, depth_clone.rows / 2));

		double ave[57] = { 0 };
		double ave_sum = 0;
		double sum = 0;
		double element_count = 0;

		for (int ave_num = 0; ave_num < 57; ave_num++){
			for (int y = 0; y < cut.rows; y++){
				double *cc = cut.ptr<double>(y);
				for (int x = (cut.cols / 57) * ave_num; x < (cut.cols / 57)*(ave_num + 1); x++){
					sum = sum + cc[x];
					element_count++;
				}
			}
			ave[ave_num] = sum / element_count;
			sum = 0;
			element_count = 0;
			ave_sum = ave_sum + ave[ave_num];
		}

		double ave_ave = ave_sum / 57;

		for (int ave_num = 0; ave_num < 57; ave_num++){
			if (ave[ave_num] > 300) ave[ave_num] = 0;
		}

		double J = 0;
		double cco = 1;
		double cco_max = 0;
		double cco_num = 0;

		for (int ave_num = 0; ave_num < 57; ave_num++){
			J = ave[ave_num + 1] - ave[ave_num];
			if (ave[ave_num] == 0){
				cco++;

				if (cco_max < cco){ 
					cco_max = cco;
					cco_num = ave_num;
				}

			}
			else{
				cco = 0;
			}
		}

		double start = cco_num - cco_max;

		double x_s = (start * 10) * cut.ptr<double>(cut.rows / 2)[(int)start * 10] / fku_l;
		double x_e = (((cco_num + 1) * 10) - 1) * cut.ptr<double>(cut.rows / 2)[(((int)cco_num + 1) * 10) - 1] / fku_l;

		double width_x = abs(x_e - x_s);



		if (width_x > width_robot){
			r = (((((cco_num + 1) * 10) - 1) + (start * 10)) / 2) ;
			Point run_reference(r, undistort_l.rows / 2);
			circle(undistort_l, run_reference, 15, Scalar(0, 0, 200), 5, CV_AA);

			auto sampling = chrono::system_clock::now();
			double sampling_time = chrono::duration_cast<std::chrono::milliseconds>(sampling - startTime).count();

			pre_error = error;
			error = cx_l - r;
			integral += ((error + pre_error) * sampling_time * pow(10, -3)) / 2;

			P = Kp * error;
			I = Ki * integral;
			D = Kd * ((pre_error - error) / sampling_time);

			U = P + I + D;

			double D_l = 200 - U;
			double D_r = 200 + U;

			if (robot_switch == 0){
				cout << "error sum : " << error << endl
					<< "error now : " << error - pre_error << endl
					<<"Left Moter Output : " << D_l << endl
					<< "Right Motor Output : " << D_r << endl;
			}
			else{
				run.setMotorPwm(motor_r, D_r);
				run.setMotorPwm(motor_l, D_l);
			}

		}

		sceneVideo << undistort_l;

		auto checkTime = chrono::system_clock::now();
		double elapsedTime = chrono::duration_cast<std::chrono::milliseconds>(checkTime - startTime).count();
		processingTime = elapsedTime - previousTime;
		previousTime = elapsedTime;
		ostringstream elapsed,processing;
		elapsed << elapsedTime;
		processing << processingTime;
		string elapsedTimeStr = "elapsed time : " + elapsed.str() + "msec";
		string processingTimeStr = "processing time : " + processing.str() + "msec";
		cout << elapsedTimeStr << " " << processingTimeStr << endl;

		imshow("left", undistort_l);
		imshow("right", undistort_r);

		if (waitKey(15) == 13){
			break;
		}
	}

	run.setMotorPwm(motor_r, 0);
	run.setMotorPwm(motor_l, 0);

	return 0;
}