/*

	�J����
	OS:		Windows7/10
	IDE:	Visual studio 2013 Community
	OpenCV:	2.4.10(NuGet)

	�J�����̐��x���\�����؂���

	�ŏI�X�V��:	2016.11.4

*/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int
main(int argc, char *argv[]){

	// �ǂݍ��މ摜�̖��O
	const string image_l_name = "image/left.png";
	const string image_r_name = "image/right.png";

	// �摜�̓ǂݍ��ݏ���
	Mat image_l = imread(image_l_name,1);
	Mat image_r = imread(image_r_name,1);

	// �J�����̓����p�����[�^
	Mat cameraMatrix_l = (Mat_<double>(3, 3) << 672.991236639696, 0., 648.654435166223, 0., 673.205591701543, 345.371324805689, 0., 0., 1);
	Mat cameraMatrix_r = (Mat_<double>(3, 3) << 671.948583247860, 0., 660.303578974912, 0., 672.341599149669, 345.406480909995, 0., 0., 1);

	// �J�����̘c�݌W��
	Mat distCoeffs_l = (Mat_<double>(1, 5) << -0.152527769962967, 0.0133861422874566, 0., 0., 0.00372815854695288);
	Mat distCoeffs_r = (Mat_<double>(1, 5) << -0.152015516334721, 0.0125715563805952, 0., 0., 0.00424905488907074);

	// �J�����̓����p�����[�^�s�񂩂�1�s1��ڂ̗v�f��"fku"�Ɋi�[
	Vec3d *src_l = cameraMatrix_l.ptr<Vec3d>(0);
	double fku = src_l[0][0];

	// �c�ݕ␳��̉摜�f�[�^���i�[����Mat�z��
	Mat undistort_l;
	Mat undistort_r;

	// �摜������p�����[�^�Ƙc�݌W���ɂ���ĕ␳
	// undistort(input ,output ,input, input)
	undistort(image_l, undistort_l, cameraMatrix_l, distCoeffs_l);
	undistort(image_r, undistort_r, cameraMatrix_r, distCoeffs_r);
	
	// �G�b�W���o�摜���i�[����Mat
	Mat edge_l;
	Mat edge_r;

	// �G�b�W���o����
	Canny(undistort_l, edge_l, 50, 200);
	Canny(undistort_r, edge_r, 50, 200);



	// �������ԑ���
	TickMeter time;
	// ����J�n
	time.start();

	// �摜�̊e�s�N�Z���̐F�f���(RGB)���擾
	for (int v = 0; v < undistort_l.rows; v++){

		// �z��̃|�C���^���擾
		Vec3b *bgr_ptr = undistort_l.ptr<Vec3b>(v);

		for (int u = 0; u < undistort_l.cols; u++){
			
			//double b = correct_image_l.at<Vec3b>(v, u)[0];
			//double g = correct_image_l.at<Vec3b>(v, u)[1];
			//double r = correct_image_l.at<Vec3b>(v, u)[2];

			// [u,v]�Ԗڂ̐F�f�����擾
			Vec3b bgr = bgr_ptr[u];

			double B = bgr[0];
			double G = bgr[1];
			double R = bgr[2];

		}
	}
	// ����I��
	time.stop();

	// �������Ԃ�\��
	cout << time << endl;

	// ���͑҂�
	waitKey(0);
	return 0;
}