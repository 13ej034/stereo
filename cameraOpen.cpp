/** 
* @file cameraOpen.cpp
* @brief カメラ起動テスト OpenCVを用いてカメラを起動する OpenCVは2.4.10を使用
* @author 13EJ034
* @date 最終更新日 : 2016/11/14
*/

#include <opencv2/opencv.hpp>
#include <iostream>

/********************************************//**
 *  メイン関数
 ***********************************************/
int main(int argc, char *argv[]){

	/********************************************//**
	 *  カメラ起動構成
	 ***********************************************/
	
	cv::Size cap_size(1280 * 2, 720);                    //! キャプチャサイズ
	const double fps = 60.0;                             //! キャプチャfps

	cv::VideoCapture cap;                                //! キャプチャ構造体
	cap.open(0);                                         //! カメラ起動
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);    //! キャプチャの横幅
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);  //! キャプチャの縦幅
	cap.set(CV_CAP_PROP_FPS, fps);                       //! キャプチャのfps

	/*!
	 * カメラ起動確認
	 * 起動失敗時はエラー終了する
	 */

	if (!cap.isOpened()){
		std::cout << "Camera not found." << std::endl;
		return -1; 
	}
	else{
		std::cout << "Camera has been opened." << std::endl;
	}

	/********************************************//**
	 * ウィンドウ構成
	 ***********************************************/

	const std::string windowName_l = "left";             //! 左画像ウィンドウ名
	const std::string windowName_r = "right";            //! 右画像ウィンドウ名

	cv::namedWindow(windowName_l, CV_WINDOW_NORMAL);     //! 左画像ウィンドウを生成
	cv::namedWindow(windowName_r, CV_WINDOW_NORMAL);     //! 右画像ウィンドウを生成

	cv::resizeWindow(windowName_l, cap_size.width * (5 / 8), cap_size.height * (5 / 8)); //! 左画像ウィンドウサイズを元のサイズの(5/8)倍に変更
	cv::resizeWindow(windowName_r, cap_size.width * (5 / 8), cap_size.height * (5 / 8)); //! 右画像ウィンドウサイズを元のサイズの(5/8)倍に変更

	cv::moveWindow(windowName_l, 100, 100);                               //! 左画像ウィンドウをディスプレイの左上から (x,y)=(100,100) の位置に配置
	cv::moveWindow(windowName_r, 100 + cap_size.width * (5 / 8), 100);    //! 右画像ウィンドウをディスプレイの左上から (x,y)=(100 + 左画像ウィンドウの幅,100) の位置に配置

	/********************************************//**
	 * 処理開始
	 ***********************************************/

	while (true) {
		
		/********************************************//**
		 *  画像処理
		 ***********************************************/

		/*!
		 * キャプチャ開始
		 */

		cv::Mat frame;		//! キャプチャを格納するMat
		cap >> frame;		//! キャプチャを格納

		/*!
		 * ステレオイメージを左右に分割
		 */

		cv::Mat frame_l = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));				//! 左画像
		cv::Mat frame_r = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));	//! 右画像

		/*!
		 * キャプチャを表示
		 */

		cv::imshow(windowName_l, frame_l);	//! 左画像を表示
		cv::imshow(windowName_r, frame_r);	//! 右画像を表示
		
		/*!
		 * 5[msec]キーボードから入力待機
		 * 入力がある場合,処理終了
		 */

		if (cv::waitKey(5) > 0){
			break;
		}
	}

	return 0;
}