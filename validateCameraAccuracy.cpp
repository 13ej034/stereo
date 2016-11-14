/**
* @file validateCameraAccuracy.cpp
* @brief ステレオカメラの精度検証 OpenCVは2.4.10を使用
* @author 13EJ034
* @date 最終更新日 : 2016/11/14
*/

#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char *argv[]){

	const std::string imageName_l = "image/left.png";
	const std::string imageName_r = "image/right.png";

	cv::Mat image_l = cv::imread(imageName_l,1);
	cv::Mat image_r = cv::imread(imageName_r,1);

	cv::Mat cameraMatrix_l = (cv::Mat_<double>(3, 3) << 672.991236639696, 0., 648.654435166223, 0., 673.205591701543, 345.371324805689, 0., 0., 1);
	cv::Mat cameraMatrix_r = (cv::Mat_<double>(3, 3) << 671.948583247860, 0., 660.303578974912, 0., 672.341599149669, 345.406480909995, 0., 0., 1);

	cv::Mat distCoeffs_l = (cv::Mat_<double>(1, 5) << -0.152527769962967, 0.0133861422874566, 0., 0., 0.00372815854695288);
	cv::Mat distCoeffs_r = (cv::Mat_<double>(1, 5) << -0.152015516334721, 0.0125715563805952, 0., 0., 0.00424905488907074);

	cv::Vec3d *src_l = cameraMatrix_l.ptr<cv::Vec3d>(0);
	double fku = src_l[0][0];

	cv::Mat undistort_l;
	cv::Mat undistort_r;

	cv::undistort(image_l, undistort_l, cameraMatrix_l, distCoeffs_l);
	cv::undistort(image_r, undistort_r, cameraMatrix_r, distCoeffs_r);
	
	cv::Mat edge_l;
	cv::Mat edge_r;

	cv::Canny(undistort_l, edge_l, 50, 200);
	cv::Canny(undistort_r, edge_r, 50, 200);

	cv::TickMeter time;
	time.start();

	for (int v = 0; v < undistort_l.rows; v++){

		cv::Vec3b *bgr_ptr = undistort_l.ptr<cv::Vec3b>(v);

		for (int u = 0; u < undistort_l.cols; u++){
			
			cv::Vec3b bgr = bgr_ptr[u];

			double B = bgr[0];
			double G = bgr[1];
			double R = bgr[2];

		}
	}
	
	time.stop();

	std::cout << time << std::endl;

	waitKey(0);

	return 0;
}