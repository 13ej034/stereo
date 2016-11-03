/*

	開発環境
	OS:		Windows7/10
	IDE:	Visual studio 2013 Community
	OpenCV:	2.4.10(NuGet)

	カメラの精度性能を検証する

	最終更新日:	2016.11.4

*/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int
main(int argc, char *argv[]){

	// 読み込む画像の名前
	const string image_l_name = "image/left.png";
	const string image_r_name = "image/right.png";

	// 画像の読み込み処理
	Mat image_l = imread(image_l_name,1);
	Mat image_r = imread(image_r_name,1);

	// カメラの内部パラメータ
	Mat cameraMatrix_l = (Mat_<double>(3, 3) << 672.991236639696, 0., 648.654435166223, 0., 673.205591701543, 345.371324805689, 0., 0., 1);
	Mat cameraMatrix_r = (Mat_<double>(3, 3) << 671.948583247860, 0., 660.303578974912, 0., 672.341599149669, 345.406480909995, 0., 0., 1);

	// カメラの歪み係数
	Mat distCoeffs_l = (Mat_<double>(1, 5) << -0.152527769962967, 0.0133861422874566, 0., 0., 0.00372815854695288);
	Mat distCoeffs_r = (Mat_<double>(1, 5) << -0.152015516334721, 0.0125715563805952, 0., 0., 0.00424905488907074);

	// カメラの内部パラメータ行列から1行1列目の要素を"fku"に格納
	Vec3d *src_l = cameraMatrix_l.ptr<Vec3d>(0);
	double fku = src_l[0][0];

	// 歪み補正後の画像データを格納するMat配列
	Mat undistort_l;
	Mat undistort_r;

	// 画像を内部パラメータと歪み係数によって補正
	// undistort(input ,output ,input, input)
	undistort(image_l, undistort_l, cameraMatrix_l, distCoeffs_l);
	undistort(image_r, undistort_r, cameraMatrix_r, distCoeffs_r);
	
	// エッジ抽出画像を格納するMat
	Mat edge_l;
	Mat edge_r;

	// エッジ抽出処理
	Canny(undistort_l, edge_l, 50, 200);
	Canny(undistort_r, edge_r, 50, 200);



	// 処理時間測定
	TickMeter time;
	// 測定開始
	time.start();

	// 画像の各ピクセルの色素情報(RGB)を取得
	for (int v = 0; v < undistort_l.rows; v++){

		// 配列のポインタを取得
		Vec3b *bgr_ptr = undistort_l.ptr<Vec3b>(v);

		for (int u = 0; u < undistort_l.cols; u++){
			
			//double b = correct_image_l.at<Vec3b>(v, u)[0];
			//double g = correct_image_l.at<Vec3b>(v, u)[1];
			//double r = correct_image_l.at<Vec3b>(v, u)[2];

			// [u,v]番目の色素情報を取得
			Vec3b bgr = bgr_ptr[u];

			double B = bgr[0];
			double G = bgr[1];
			double R = bgr[2];

		}
	}
	// 測定終了
	time.stop();

	// 処理時間を表示
	cout << time << endl;

	// 入力待ち
	waitKey(0);
	return 0;
}