/**
* @file main.cpp
* @brief 深度情報から通過可能領域を判別する
* @author 13ej034
* @date 2017.1.19
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

	//内部パラメータ
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

	// 歪み係数
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

	// ロボットの大きさ
	double W_r = 312.0;	// [mm]

	// ロボットの目標点
	double r = 0;

	// 
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

	Ptr<StereoBM> bm = StereoBM::create(0, 11);

	double baseline = 120.0;

	auto startTime = chrono::system_clock::now();	// 処理開始時間
	double processingTime = 0;
	double previousTime = 0;

	while (true){

		// 画像取得
		Mat frame;
		cap >> frame;

		// 分割
		Mat frame_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		Mat frame_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		Size frameSize(frame_l.cols, frame_l.rows);

		// 補正
		
		Mat undistort_l, undistort_r;
		Mat mapx_l, mapy_l,mapx_r,mapy_r;
		initUndistortRectifyMap(cameraParameter_l, distCoeffs_l, Mat(), cameraParameter_l, frameSize, CV_32FC1, mapx_l, mapy_l);
		initUndistortRectifyMap(cameraParameter_r, distCoeffs_r, Mat(), cameraParameter_r, frameSize, CV_32FC1, mapx_r, mapy_r);
		remap(frame_l, undistort_l, mapx_l, mapy_l, INTER_LINEAR);
		remap(frame_r, undistort_r, mapx_r, mapy_r, INTER_LINEAR);
		

		// グレースケール化
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

		// 視差情報の取得
		Mat disparity;
		sgbm->compute(gray_l, gray_r, disparity);
		//bm->compute(gray_l, gray_r, disparity);
		// 確認用 不要時はコメントアウト
		// ここから

		double max, min;
		minMaxLoc(disparity, &min, &max);
		Mat disparity_map;
		disparity.convertTo(disparity_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
		Mat disparity_map_hist;
		equalizeHist(disparity_map, disparity_map_hist);
		Mat disparity_map_Hue;
		changeToHueColor(disparity_map_hist, disparity_map_Hue);

		//　ここまで

		// CV_16S -> CV_64F
		Mat disparity_64f;
		disparity.convertTo(disparity_64f, CV_64F);

		// 深度情報に変換
		Mat depth = fku_l * baseline / disparity_64f;
		
		// 確認用 不要時はコメントアウト
		// ここから

		minMaxLoc(depth, &min, &max);
		Mat depth_map;
		depth.convertTo(depth_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
		Mat depth_map_hist;
		equalizeHist(depth_map, depth_map_hist);
		Mat depth_map_Hue;
		changeToHueColor(depth_map_hist, depth_map_Hue);

		// ここまで

		Mat X(frameSize, CV_64F);
		Mat Y(frameSize, CV_64F);

		// X,Yの情報を取得
		
		int zero_check = 0;
		int count = 0;
		int index = 0;
		Mat depth_clone = depth.clone();
		for (int y = 0; y < depth_clone.rows; y++){
			double *dep = depth_clone.ptr<double>(y);
			double *xx = X.ptr<double>(y);
			double *yy = Y.ptr<double>(y);
			for (int x = 0; x < depth_clone.cols; x++){
				double Z = dep[x];
				xx[x] = x * Z / fku_l;
				yy[x] = y * Z / fkv_l;

				if (Z > 200 || Z < 0){
					dep[x] = double(0);
				}

			}
		}
		
		double J = 0;		// 通過領域の判定式
		double x_end = 0;	// 通過領域の終端
		double x_start = 0;	// 通過領域の始端
		double W = 0;

		for (int y = 0; y < depth_clone.rows; y++){
			double *dep = depth_clone.ptr<double>(y);
			double *dis = disparity_64f.ptr<double>(y);
			for (int x = 0; x < depth_clone.cols - 1; x++){
				J = dep[x + 1] - dep[x];
				if (J > 0){
					x_end = x;
				}
				else if (J < 0){
					x_start = x;
				}
				W = (x_end - x_start)  *baseline / dis[x];
				if (W > W_r){
					r = (x_end - x_start) / 2;
					//cout << "r : " << r << endl;
				}
			}
		}

		minMaxLoc(depth_clone, &min, &max);
		Mat depth_clone_map;
		depth_clone.convertTo(depth_clone_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
		Mat depth_clone_map_hist;
		equalizeHist(depth_clone_map, depth_clone_map_hist);
		Mat depth_clone_map_Hue;
		changeToHueColor(depth_clone_map_hist, depth_clone_map_Hue);


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
		imshow("dep.png", depth_map_Hue);
		imshow("dis.png", disparity_map_Hue);
		imshow("dep_Raw.png", disparity);
		imshow("dis_Raw.png", depth);

		//string saveRgbFileName = "data/left/frame" + elapsed.str() + ".png";
		//string saveDepthFileName = "data/calib_right/frame" + elapsed.str() + ".png";
		//imwrite(saveRgbFileName, frame_l);
		//imwrite(saveDepthFileName, frame_r);

		if (waitKey(15) == 13){
			string imageName_l = "data/left.png";
			string imageName_r = "data/right.png";
			string imageName_dis = "data/dis.png";
			string imageName_dep = "data/dep.png";
			string imageName_dis_Raw = "data/dis_raw.png";
			string imageName_dep_Raw = "data/dep_raw.png";
			imwrite(imageName_l, undistort_l);
			imwrite(imageName_r, undistort_r);
			imwrite(imageName_dis, disparity_map_Hue);
			imwrite(imageName_dep, depth_map_Hue);
			imwrite(imageName_dis_Raw, disparity);
			imwrite(imageName_dep_Raw, depth);
			break;
		}
	}
	return 0;
}