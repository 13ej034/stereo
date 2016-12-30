#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv){


	Mat depth_original = imread("data/depth/depth26510.png", 0);
	Mat depth_copy = depth_original.clone();
	int zero_count = 0;	// �A������0�̃J�E���^
	int index_now = 0;	// 0���r�؂ꂽ�ŐV�̗v�f�ʒu
	int index_pre = 0;	// 0���r�؂ꂽ�O�̗v�f�ʒu

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

					// �̈�����o���鏈��
					uchar check = ptr[x + 1] - ptr[x];	// �ׂ̗v�f�Ƃ̍�
					//cout << +check << endl;	// uchar�𐔒l�Ƃ���cout�������ꍇ�� check -> +check �ɂ���Ή\
					if (check == 0){	// ����0 �܂�0���A�����Ă���
						zero_count++;	// �J�E���g
					}
					else{				// ����0�ȊO �܂�0���A���ł͂Ȃ�
						index_now = x;								// �r�؂ꂽ�v�f�̈ʒu���擾
						int width_pixel = index_now - index_pre;	// 0�̘A�����Ă���v�f�����v�Z zero_count�����ł�����? �v����	
						double X = width_pixel ;					// �O��������(X��)�̎� �C��������珑��

						zero_count = 0;	// �J�E���g���Z�b�g
						index_pre = index_now;	// �l�̕ۑ�
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