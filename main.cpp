/**
* @author 13ej034
* @date 2017.2.18
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
	// Setting camera intrinsic parameter and distortion coefficient
	// Left camera intrinsic parameter
	const double fku_l = 353.600559219653;
	const double fkv_l = 352.562464480179;
	const double cx_l = 320.306982522657;
	const double cy_l = 191.383465238258;

	// Right camera intrinsic parameter
	const double fku_r = 355.659530311593;
	const double fkv_r = 354.734600040007;
	const double cx_r = 335.004584045585;
	const double cy_r = 180.558275004874;

	// Create intrinsic parameter matrix
	Mat cameraParameter_l = (Mat_<double>(3, 3) << fku_l, 0., cx_l, 0., fkv_l, cy_l, 0., 0., 1.);
	Mat cameraParameter_r = (Mat_<double>(3, 3) << fku_r, 0., cx_r, 0., fkv_r, cy_r, 0., 0., 1.);

	// Left camera distortion coefficient
	const double k1_l = -0.173747838157089;
	const double k2_l = 0.0272481881774572;
	const double p1_l = 0.0;
	const double p2_l = 0.0;

	// Right camera distortion coefficient
	const double k1_r = -0.176327723277872;
	const double k2_r = 0.0286008197857787;
	const double p1_r = 0.0;
	const double p2_r = 0.0;

	// Create distortion coefficient matrix
	Mat distCoeffs_l = (Mat_<double>(1, 4) << k1_l, k2_l, p1_l, p2_l);
	Mat distCoeffs_r = (Mat_<double>(1, 4) << k1_r, k2_r, p1_r, p2_r);

	// Setting camera resolution
	VideoCapture cap(0);
	if (!cap.isOpened()) return -1;
	Size cap_size(1344, 376);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

	// Setting recording
	double rec_fps = 10.0;
	Size rec_size(cap_size.width / 2, cap_size.height);
	VideoWriter rec("rec.avi", VideoWriter::fourcc('M', '4', 'S', '2'), rec_fps, rec_size, true);
	if (!rec.isOpened()) return -1;

	// Setting Semi-Global block matching parameter
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

	// Setting block matching parameter
	Ptr<StereoBM> bm = StereoBM::create(0, 15);

	// Setting variable
	double baseline = 120.0;	// [mm]
	double width_robot = 31.0;	// [cm]
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
	double r = 0;
	double D_r = 0;
	double D_l = 0;

	// Setting robot
	// 1 : turn on , 0 : turn off
	int robot_switch = 1;
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

	// Start time measuremen
	auto startTime = chrono::system_clock::now();
	double processingTime = 0;
	double previousTime = 0;

	// main roop
	while (true){

		// 1.Get frame
		Mat frame;
		cap >> frame;

		// 2.Split left images and right image from stereo image
		Mat frame_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		Mat frame_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));	
		Size frameSize(frame_l.cols, frame_l.rows);
		
		// 3.Correct distortion
		Mat undistorted_l, undistorted_r;
		Mat mapx_l, mapy_l,mapx_r,mapy_r;
		initUndistortRectifyMap(cameraParameter_l, distCoeffs_l, Mat(), cameraParameter_l, frameSize, CV_32FC1, mapx_l, mapy_l);
		initUndistortRectifyMap(cameraParameter_r, distCoeffs_r, Mat(), cameraParameter_r, frameSize, CV_32FC1, mapx_r, mapy_r);
		remap(frame_l, undistorted_l, mapx_l, mapy_l, INTER_LINEAR);
		remap(frame_r, undistorted_r, mapx_r, mapy_r, INTER_LINEAR);
		
		// 4.Change grayscale
		Mat gray_l,gray_r;
		cvtColor(undistorted_l, gray_l, CV_BGR2GRAY);
		cvtColor(undistorted_r, gray_r, CV_BGR2GRAY);
		
		// 5.Compute disparity
		Mat disparity;
		sgbm->compute(gray_l, gray_r, disparity);
		//bm->compute(gray_l, gray_r, disparity);

		// CV_16S -> CV_64F
		Mat disparity_64f;
		disparity.convertTo(disparity_64f, CV_64F);

		// 6.Compute depth
		Mat depth = fku_l * baseline / disparity_64f;
		
		// Find route process
		
		// 7.Cut the lower half of the depth image
		Mat depth_clone = depth.clone();
		Mat cut = depth_clone(Rect(64, depth_clone.rows - (depth_clone.rows / 2), depth_clone.cols - 64, depth_clone.rows / 2));

		// 8.If cut elements value < 30, the elements change to 0
		for (int y = 0; y < cut.rows; y++){
			double *cutp = cut.ptr<double>(y);
			for (int x = 0; x < depth_clone.cols; x++){
				if (cutp[x] < 30) cutp[x] = double(0);
			}
		}

		// 9.Compute element average
		double ave[608] = { 0 };
		double ave_sum = 0;
		double sum = 0;
		double element_count = 0;

		for (int ave_num = 0; ave_num < 608; ave_num++){
			for (int y = 0; y < cut.rows; y++){
				double *cut_elem = cut.ptr<double>(y);
				for (int x = ave_num; x < ave_num + 1; x++){
					sum += cut_elem[x];	// sum total element value
					element_count++;	// count element number
				}
			}
			ave[ave_num] = sum / element_count; // compute sum average
			sum = 0;							// reset
			element_count = 0;					// reset
			ave_sum += ave[ave_num];			// sum total average value
		}

		double ave_ave = ave_sum / 608;			// compute ave[ave_num] average

		// 10.Search for places where the largest width exists
		// If ave array elements > ave_ave, the elements change to 0
		// 0 is judged as a passable area
		for (int ave_num = 0; ave_num < 608; ave_num++){
			if (ave[ave_num] > ave_ave) ave[ave_num] = 0;
		}

		double J = 0;
		int zero_count = 1;
		int zero_count_max = 0;
		int zero_count_num = 0;

		for (int ave_num = 0; ave_num < 608; ave_num++){
			// compute the difference from adjacent pixels
			J = ave[ave_num + 1] - ave[ave_num];
			if (J == 0){
				zero_count++;
				// extract the maximum value of zero_count
				if (zero_count_max < zero_count){
					zero_count_max = zero_count;	// maximum value
					zero_count_num = ave_num;		// maximum value pixel's end index
				}

			}
			else{
				zero_count = 1;						//reset
			}
		}

		double start = zero_count_num - zero_count_max;	// maximum value pixel's start index

		// 11.compute 3D width
		double x_s = start * depth.ptr<double>(depth.rows * 3 / 4)[start] / fku_l;
		double x_e = zero_count_num * cut.ptr<double>(cut.rows *3 / 4)[zero_count_num] / fku_l;
		double width_x = abs(x_e - x_s);

		// 12.compute reference 
		if (width_x > width_robot){
			r = ( (cco_num + start) / 2 ) - 30;
			Point run_reference(r, undistorted_l.rows * 3 / 4);
			circle(undistort_l, run_reference, 15, Scalar(0, 0, 200), 5, CV_AA);

			auto sampling = chrono::system_clock::now();
			double sampling_time = chrono::duration_cast<std::chrono::milliseconds>(sampling - startTime).count();

			// PID controller
			pre_error = error;
			error = cx_l - r;
			integral += ((error + pre_error) * sampling_time * pow(10, -3)) / 2;

			P = Kp * error;
			I = Ki * integral;
			D = Kd * ((pre_error - error) / sampling_time);

			U = P + I + D;

			// Convert U to PWM
			D_l = 180 - U;
			D_r = 180 + U;

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
		else{	// If r not detected, lower the robot speed 
			double D_r_red = D_r - 20;
			double D_l_red = D_l - 20;
			run.setMotorPwm(motor_r, D_r_red);
			run.setMotorPwm(motor_l, D_l_red);
        }

		// recording left frame
		rec << undistort_l;

		// compute process time and elapsed time
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

		// preview 
		imshow("left", undistorted_l);
		imshow("right", undistorted_r);
		imshow("dep", cut);
		
		// Loop break when the enter key is pressed
		if (waitKey(15) == 13){
			break;
		}
	}

	// robot stop
	run.setMotorPwm(motor_r, 0);
	run.setMotorPwm(motor_l, 0);

	return 0;
}