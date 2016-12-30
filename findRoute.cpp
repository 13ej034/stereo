#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv){


	Mat depth_original = imread("data/depth/depth26510.png", 0);
	Mat depth_copy = depth_original.clone();
	int zero_count = 0;	// 連続した0のカウンタ
	int index_now = 0;	// 0が途切れた最新の要素位置
	int index_pre = 0;	// 0が途切れた前の要素位置

	auto start = chrono::system_clock::now();

	for (int y = 0; y < depth_copy.rows; y++){
		uchar *ptr = depth_copy.ptr<uchar>(y);
		for (int x = 0; x < depth_copy.cols; x++){
			uchar Z = ptr[x];
			if (Z >= 200 || Z <= 70){
				ptr[x] = 0;
			}
			if (y >= 500){
				if (y % 20 == 0 && Z == 0){

					// 領域を検出する処理
					uchar check = ptr[x + 1] - ptr[x];	// 隣の要素との差
					//cout << +check << endl;	// ucharを数値としてcoutしたい場合は check -> +check にすれば可能
					if (check == 0){	// 差が0 つまり0が連続している
						zero_count++;	// カウント
					}
					else{				// 差が0以外 つまり0が連続ではない
						index_now = x;								// 途切れた要素の位置を取得
						int width_pixel = index_now - index_pre;	// 0の連続している要素数を計算 zero_countだけでもいい? 要検証	
						double X = width_pixel ;					// 三次元復元(X軸)の式 気が乗ったら書く

						zero_count = 0;	// カウントリセット
						index_pre = index_now;	// 値の保存
					}
				}
			}
		}
	}

	auto end = chrono::system_clock::now();
	uint64_t elapsed = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	cout << "elapsed time : " << elapsed << "msec" << endl;

	namedWindow("depth", CV_WINDOW_AUTOSIZE);
	imshow("depth", depth_copy);
	imwrite("data/depth_data.png", depth_copy);
	waitKey(0);

	return 0;
}