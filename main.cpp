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

void detectSquare(Mat &InputOutputArray, vector<vector<Point>> InputContours,
	chrono::time_point<chrono::system_clock, chrono::system_clock::duration> &InputStarttime);
void estimateGravityCenter(Mat &InputOutputArray, vector<Point> &InputApprox);
void changeToHueColor(Mat Input, Mat &Output);

// 四角形を検出
void detectSquare(Mat &InputOutputArray, vector<vector<Point>> InputContours,
	chrono::time_point<chrono::system_clock, chrono::system_clock::duration> &InputStarttime){
	for (auto i = InputContours.begin(); i != InputContours.end(); i++){

		// 輪郭を直線近似
		vector<Point> approx;
		approxPolyDP(Mat(*i), approx, 0.01 * arcLength(*i, true), true);

		// 近似した直線内の面積を算出
		double area = contourArea(approx);

		// 頂点座標が4つ かつ 面積が1000以上で四角形であると判定する
		if (approx.size() == 4 && area > 1000.0){

			estimateGravityCenter(InputOutputArray, approx);

			auto checkTime = chrono::system_clock::now();	// 四角形を検出した時間
			double elapsedTime = chrono::duration_cast<std::chrono::milliseconds>(checkTime - InputStarttime).count();	// 処理開始時間から四角形検出までの経過時間

			// 画像を保存
			ostringstream streamOutputFileName;
			streamOutputFileName << elapsedTime;
			string outputFileName = "data/detectSquare_" + streamOutputFileName.str() + "msec.png";
			polylines(InputOutputArray, approx, true, Scalar(255, 0, 0), 2);
			imwrite(outputFileName, InputOutputArray);
		}
	}
}


// 重心推定
void estimateGravityCenter(Mat &InputOutputArray,vector<Point> &InputApprox){

	Moments mu_approx = moments(InputApprox);
	Point2f mc_approx = Point2f(mu_approx.m10 / mu_approx.m00, mu_approx.m01 / mu_approx.m00);
	circle(InputOutputArray, mc_approx, 4, Scalar(0, 0, 100), 2, 4);
	ostringstream Gravity;
	Gravity << mc_approx;
	string OutputGravity = Gravity.str();
	putText(InputOutputArray, OutputGravity, mc_approx, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(200, 200, 0), 1, CV_AA);
}

// 視差情報と深度情報を色で表現する
void changeToHueColor(Mat Input,Mat &Output){
	
	Mat channels[3];
	channels[0] = Mat(Input.size(), CV_8UC1);
	channels[1] = Mat(Input.size(), CV_8UC1, 255);
	channels[2] = Mat(Input.size(), CV_8UC1, 255);
	Mat hsv_image;
	for (int i = 0; i < Input.cols; i++){
		for (int j = 0; j < Input.rows; j++){
			int d = Input.at<uchar>(j, i);
			channels[0].at<uchar>(j, i) = (255 - d) / 2;
		}
	}
	merge(channels, 3, hsv_image);
	cvtColor(hsv_image, Output, CV_HSV2BGR);
}

int main(int argc, const char* argv[])
{

	const double fku_l = 352.982274953091;
	const double fkv_l = 351.250756508819;
	const double cx_l = 345.906100335532;
	const double cy_l = 184.892250145259;

	const double fku_r = 352.546641417207;
	const double fkv_r = 350.889872835968;
	const double cx_r = 347.909011932106;
	const double cy_r = 184.782844969847;

	Mat cameraParameter_l = (Mat_<double>(3, 3) << fku_l, 0., cx_l, 0., fkv_l, cy_l, 0., 0., 1.);
	Mat cameraParameter_r = (Mat_<double>(3, 3) << fku_r, 0., cx_r, 0., fkv_r, cy_r, 0., 0., 1.);

	const double k1_l = -0.153606435507657;
	const double k2_l = 0.00714696451127945;
	const double p1_l = 0.0;
	const double p2_l = 0.0;

	const double k1_r = -0.150252804108977;
	const double k2_r = 0.00311575109211558;
	const double p1_r = 0.0;
	const double p2_r = 0.0;

	Mat distCoeffs_l = (Mat_<double>(1, 4) << k1_l, k2_l, p1_l, p2_l);
	Mat distCoeffs_r = (Mat_<double>(1, 4) << k1_r, k2_r, p1_r, p2_r);

	double width_robot = 31.20;	// [cm]

	double r = 0;

	VideoCapture cap(0);
	if (!cap.isOpened()) return -1;
	Size cap_size(1344, 376);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

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
	double Kp = 0.2;
	double Ki = 0.15;
	double Kd = 1;
	double P = 0;
	double I = 0;
	double U = 0;

	static RunCtrl run;
	run.connect("COM6");

	const int motor_r = 0;
	const int motor_l = 1;

	run.setWheelVel(motor_r, 5);
	run.setWheelVel(motor_l, 5);

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
		
		// 二値化
		//Mat binaly_l,binaly_r;
		//threshold(gray_l, binaly_l, 0.0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);
		//threshold(gray_r, binaly_r, 0.0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);

		// 輪郭抽出
		//vector<vector<Point>> contours_l, contours_r;
		//findContours(binaly_l, contours_l, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		//findContours(binaly_r, contours_r, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		// 四角形検出
		//detectSquare(undistort_l, contours_l, startTime);
		//detectSquare(undistort_r, contours_r, startTime);

		Mat disparity;
		sgbm->compute(gray_l, gray_r, disparity);
		//bm->compute(gray_l, gray_r, disparity);

		// 確認用 不要時はコメントアウト
		// ここから

		//double max, min;
		//minMaxLoc(disparity, &min, &max);
		//Mat disparity_map;
		//disparity.convertTo(disparity_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
		//Mat disparity_map_hist;
		//equalizeHist(disparity_map, disparity_map_hist);
		//Mat disparity_map_Hue;
		//changeToHueColor(disparity_map_hist, disparity_map_Hue);

		//　ここまで

		// CV_16S -> CV_64F
		Mat disparity_64f;
		disparity.convertTo(disparity_64f, CV_64F);

		// 深度情報に変換
		Mat depth = fku_l * baseline / disparity_64f;
		
		// 確認用 不要時はコメントアウト
		// ここから

		//minMaxLoc(depth, &min, &max);
		//Mat depth_map;
		//depth.convertTo(depth_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
		//Mat depth_map_hist;
		//equalizeHist(depth_map, depth_map_hist);
		//Mat depth_map_Hue;
		//changeToHueColor(depth_map_hist, depth_map_Hue);

		// ここまで

		// X,Yの情報を取得
		
		Mat depth_clone = depth.clone();
		for (int y = 0; y < depth_clone.rows; y++){
			double *dep = depth_clone.ptr<double>(y);
			double *dis = depth_clone.ptr<double>(y);
			for (int x = 0; x < depth_clone.cols; x++){
				if (dep[x] < 0) dep[x] = double(0);
			}
		}

		Mat cut = depth_clone(Rect(0, depth_clone.rows - (depth_clone.rows / 2), depth_clone.cols, depth_clone.rows / 2));
		

		double ave[56] = { 0 };
		double ave_sum = 0;
		double sum = 0;
		double element_count = 0;

		for (int ave_num = 0; ave_num < 56; ave_num++){
			for (int y = 0; y < cut.rows; y++){
				double *cc = cut.ptr<double>(y);
				for (int x = (cut.cols / 56) * ave_num; x < (cut.cols / 56)*(ave_num + 1); x++){
					sum = sum + cc[x];
					element_count++;
				}
			}
			ave[ave_num] = sum / element_count;
			sum = 0;
			element_count = 0;
			ave_sum = ave_sum + ave[ave_num];
		}

		double ave_ave = ave_sum / 56;

		for (int ave_num = 0; ave_num < 56; ave_num++){
			if (ave[ave_num] > ave_ave) ave[ave_num] = 0;
		}

		double J = 0;
		double cco = 1;
		double cco_max = 0;
		double cco_num = 0;

		for (int ave_num = 0; ave_num < 55; ave_num++){
			J = ave[ave_num + 1] - ave[ave_num];
			if (J == 0){
				cco++;

				if (cco_max < cco){ 
					cco_max = cco;
					cco_num = ave_num + 1;
				}

			}
			else{
				cco = 0;
			}
		}

		double start = cco_num - cco_max;

		double x_s = (start * 12) * cut.ptr<double>(cut.rows / 2)[(int)start * 12] / fku_l;
		double x_e = (((cco_num + 1) * 12) - 1) * cut.ptr<double>(cut.rows / 2)[(((int)cco_num + 1) * 12) - 1] / fku_l;

		double width_x = x_e - x_s;

		if (width_x > width_robot){
			r = (((((cco_num + 1) * 12) - 1) - (start * 12)) / 2 ) +(start * 12);
			Point run_reference(r, cut.rows / 2);
			circle(cut, run_reference, 60, Scalar(0, 0, 200), 5, CV_AA);
		}

		auto sampling = chrono::system_clock::now();
		double sampling_time = chrono::duration_cast<std::chrono::milliseconds>(sampling - startTime).count();

		pre_error = error;
		error = cx_l - r;

		P = Kp * error;
		I = Ki * (((error + pre_error) * sampling_time * pow(10,-3)) / 2);

		U = P + I;

		double D_l = 200 + U;
		double D_r = 200 - U;

		run.setMotorPwm(motor_r, D_l);
		run.setMotorPwm(motor_l, D_r);

		//cout << "error sum : " << error << endl
		//	<< "error now : " << error - pre_error << endl
		//	<<"Left Moter Output : " << D_l << endl
		//	<< "Right Motor Output : " << D_r << endl;

		//minMaxLoc(depth_clone, &min, &max);
		//Mat depth_clone_map;
		//depth_clone.convertTo(depth_clone_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
		//Mat depth_clone_map_hist;
		//equalizeHist(depth_clone_map, depth_clone_map_hist);
		//Mat depth_clone_map_Hue;
		//changeToHueColor(depth_clone_map_hist, depth_clone_map_Hue);


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

		imshow("left.png", undistort_l);
		imshow("right.png", undistort_r);
		imshow("cut", cut);

		if (waitKey(15) == 13){
			break;
		}
	}

	run.setMotorPwm(motor_r, 0);
	run.setMotorPwm(motor_l, 0);

	return 0;
}