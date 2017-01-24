#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv){

	double B = 120.0;	// [mm]
	double fx = 352.982274953091;

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

	Mat undistort_l = imread("data/left.png", CV_LOAD_IMAGE_COLOR);
	Mat undistort_r = imread("data/right.png", CV_LOAD_IMAGE_COLOR);

	Mat gray_l, gray_r;
	cvtColor(undistort_l, gray_l, CV_BGR2GRAY);
	cvtColor(undistort_r, gray_r, CV_BGR2GRAY);

	Mat disparity;
	sgbm->compute(gray_l, gray_r, disparity);

	Mat disparity_64f;
	disparity.convertTo(disparity_64f, CV_64F);

	Mat depth = fx * B / disparity_64f;

	double J = 0;
	double x_start = 0;
	double x_end = 0;
	double route_width = 0;			// [cm]
	double vehicle_width = 31.20;	// [cm]
	double reference = 0;

	auto start = chrono::system_clock::now();

	for (int y = 0; y < depth.rows; y++){

		double *dis = disparity_64f.ptr<double>(y);
		double *dep = depth.ptr<double>(y);

		for (int x = 0; x < depth.cols; x++){

			if (dep[x] > 200 || dep[x] < 70){
				dep[x] = double(0);
			}

			if (y >= 188 && y % 8 == 0){

					J = dep[x + 1] - dep[x];

					if (J > 0){
						x_start = x;
					}
					else if (J < 0){
						x_end = x;
					}
					
					route_width = (x_end - x_start) * B / dis[x];

					if (route_width > vehicle_width){
						reference = (x_end - x_start) / 2;
						Point vehicle_reference((int)reference, y);
						circle(depth, vehicle_reference, 4, Scalar(0, 0, 200), 2, 4);
					}

				}
			}
		}
	

	auto end = chrono::system_clock::now();
	double elapsed = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	cout << "elapsed time : " << elapsed << "msec" << endl;

	//imshow("depth", disparity);

	//imwrite("data/depth_data.png", depth);

	waitKey(0);

	return 0;
}