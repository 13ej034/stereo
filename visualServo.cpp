/**
* @file visualServo.cpp
* @brief ステレオカメラを用いた2駆動輪1キャスタ(2DW1C)方式ロボットの駆動制御
* @author 13EJ034
* @date 最終更新日 : 2016/11/15
*/

#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <runCtrl.h>
#include <vutils.h>

#pragma comment(lib, "vxv2.lib")

/**
* @fn
* PWM制御信号 障害物回避 画像ベース
* @brief 要約説明
* @param (double con) 引数の説明
* @param (引数名) 引数の説明
* @return 戻り値の説明
* @sa 参照すべき関数を書けばリンクが貼れる
* @detail 詳細な説明
*/
static double pwm1(double con, double I2, double Ta2)
{
	double	Pc = 8 * (con);
	I2 = 6 * I2;
	return (Pc + I2 / Ta2);
}

/**
* @fn
* ここに関数の説明を書く
* @brief 要約説明
* @param (引数名) 引数の説明
* @param (引数名) 引数の説明
* @return 戻り値の説明
* @sa 参照すべき関数を書けばリンクが貼れる
* @detail 詳細な説明
*/
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
* @fn
* ここに関数の説明を書く
* @brief 要約説明
* @param (引数名) 引数の説明
* @param (引数名) 引数の説明
* @return 戻り値の説明
* @sa 参照すべき関数を書けばリンクが貼れる
* @detail 詳細な説明
*/
static void drawSquares11(cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const cv::Point* p = &squares[i][0];
		int n = (int)squares[i].size();
		polylines(image, &p, &n, 1, true, cv::Scalar(255, 255, 255), 1, CV_AA);

	}
	//	namedWindow( "Squares01" );
	//	 imshow( "Squares01", image );
}

/**
* @fn
* ここに関数の説明を書く
* @brief 要約説明
* @param (引数名) 引数の説明
* @param (引数名) 引数の説明
* @return 戻り値の説明
* @sa 参照すべき関数を書けばリンクが貼れる
* @detail 詳細な説明
*/
static void drawSquares12(cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const cv::Point* p = &squares[i][0];
		int n = (int)squares[i].size();
		polylines(image, &p, &n, 1, true, cv::Scalar(255, 255, 255), 1, CV_AA);
	}
	//	namedWindow( "Squares02" );
	//	 imshow( "Squares02", image );
}

/********************************************//**
 *  main
 ***********************************************/

int main(int argc, char *argv[]){


	/********************************************//**
	 *  カメラ起動構成
	 ***********************************************/

	const double realTime_fps = 60.0;
	cv::Size cap_size(1280 * 2, 720);

	cv::VideoCapture cap;
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
	cap.set(CV_CAP_PROP_FPS, realTime_fps);

	/*!
	 * カメラ起動確認
	 * 起動失敗時はエラー終了する
	 */
	if (!cap.isOpened()){
		std::cout << "Camera could not found." << std::endl;
		return false;
	}
	else{
		std::cout << "Camera has been opened." << std::endl;
	}

	/********************************************//**
	 *  録画構成
	 ***********************************************/

	const std::string saveMovieName_l = "./Data/movie/result_l.avi";
	const std::string saveMovieName_r = "./Data/movie/result_r.avi";

	cv::Size video_size(1280, 720);

	const int codec = CV_FOURCC('X', 'V', 'I', 'D');
	const double record_fps = 30.0;

	cv::VideoWriter writer_l(saveMovieName_l, codec, record_fps, video_size, true);
	cv::VideoWriter writer_r(saveMovieName_r, codec, record_fps, video_size, true);

	/********************************************//**
	 *  ウィンドウ構成
	 ***********************************************/

	const std::string windowName_l = "left";
	const std::string windowName_r = "right";

	cv::namedWindow(windowName_l, CV_WINDOW_NORMAL);
	cv::namedWindow(windowName_r, CV_WINDOW_NORMAL);

	cv::resizeWindow(windowName_l, cap_size.width * (5 / 8), cap_size.height * (5 / 8));
	cv::resizeWindow(windowName_r, cap_size.width * (5 / 8), cap_size.height * (5 / 8));

	cv::moveWindow(windowName_l, 100, 100);
	cv::moveWindow(windowName_r, 100 + cap_size.width * (5 / 8), 100);

	/********************************************//**
	 *  ロボット構成
	 ***********************************************/

	
	static RunCtrl run;			//! ロボットの制御構造体

	const int MotorID_r = 0;	//! 右モータID
	const int MotorID_l = 1;	//! 左モータID

	int PWM = 200;				//! 基準PWM信号

	/*!
	 * ロボット起動確認
	 */
	if (run.connect("COM6") < 0){
		std::cout << "beego not found." << std::endl;
		return -1;
	}
	else{
		std::cout << "beego has been connected." << std::endl;
	}

	/*!
	 * 走行方向を指定
	 */
	run.setWheelVel(MotorID_r, 5);
	run.setWheelVel(MotorID_l, 5);


	/********************************************//**
	 *  変数
	 ***********************************************/

	//! マーカー検出用変数
	cv::Point2d d11, d12, d21, d22, rmin1, rmax1, rmin2, rmax2;

	/********************************************//**
	 *  処理開始
	 ***********************************************/

	std::cout << "Start!" << std::endl;

	while (true){

		/********************************************//**
		 *  画像処理
		 ***********************************************/

		/*!
		 * キャプチャ開始
		 */

		cv::Mat frame;
		cap >> frame;
		
		/*!
		 * ステレオイメージを左右に分割
		 */

		cv::Mat frame_l = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
		cv::Mat frame_r = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		/*!
		 * 録画開始
		 */

		writer_l << frame_l;
		writer_r << frame_r;

		/*!
		 * キャプチャを表示
		 */

		cv::imshow(windowName_l, frame_l);
		cv::imshow(windowName_r, frame_r);

		/*!
		 * モータにPWM信号を送信
		 */

		run.setMotorPwm(MotorID_r, PWM);
		run.setMotorPwm(MotorID_l, PWM);

		/*!
		 * 5[msec]キーボードから入力待機
		 * 入力がある場合,処理終了
		 */

		if (cv::waitKey(5) > 0){
			break;
		}
	}

	/*!
	 * 急停止しないようPWM信号を段階的に減少
	 * 8段階で減速
	 */

	for (int i = PWM; i >= 0; i=i-PWM/8){
		run.setMotorPwm(MotorID_r, i);
		run.setMotorPwm(MotorID_l, i);
	}

	return 0;
}