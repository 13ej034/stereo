#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv){

	auto start = chrono::system_clock::now();

	Mat depth = imread("data/depth/depth26510.png", -1);
	
	for (int y = 0; y < depth.rows; y++){
		uchar *ptr = depth.ptr<uchar>(y);
		for (int x = 0; x < depth.cols; x++){
			uchar Z = ptr[x];
			if (Z >= 200 || Z <= 20){
				ptr[x] = uchar(0);
			}
			if (y >= 500){
				if (y % 20 == 0 && Z == 0){
					// —Ìˆæ‚ğŒŸo‚·‚éˆ—
				}
			}
		}
	}

	auto end = chrono::system_clock::now();
	double elapsed = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	cout << "elapsed time : " << elapsed << "msec" << endl;

	namedWindow("depth", CV_WINDOW_AUTOSIZE);
	imshow("depth", depth);
	imwrite("data/depth_data.png", depth);
	waitKey(0);

	return 0;
}