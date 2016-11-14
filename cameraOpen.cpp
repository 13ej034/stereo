/** 
* @file cameraOpen.cpp
* @brief �J�����N���e�X�g OpenCV��p���ăJ�������N������ OpenCV��2.4.10���g�p
* @author 13EJ034
* @date �ŏI�X�V�� : 2016/11/14
*/

#include <opencv2/opencv.hpp>
#include <iostream>

/********************************************//**
 *  ���C���֐�
 ***********************************************/
int main(int argc, char *argv[]){

	/********************************************//**
	 *  �J�����N���\��
	 ***********************************************/
	
	cv::Size cap_size(1280 * 2, 720);                    //! �L���v�`���T�C�Y
	const double fps = 60.0;                             //! �L���v�`��fps

	cv::VideoCapture cap;                                //! �L���v�`���\����
	cap.open(0);                                         //! �J�����N��
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);    //! �L���v�`���̉���
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);  //! �L���v�`���̏c��
	cap.set(CV_CAP_PROP_FPS, fps);                       //! �L���v�`����fps

	/*!
	 * �J�����N���m�F
	 * �N�����s���̓G���[�I������
	 */

	if (!cap.isOpened()){
		std::cout << "Camera not found." << std::endl;
		return -1; 
	}
	else{
		std::cout << "Camera has been opened." << std::endl;
	}

	/********************************************//**
	 * �E�B���h�E�\��
	 ***********************************************/

	const std::string windowName_l = "left";             //! ���摜�E�B���h�E��
	const std::string windowName_r = "right";            //! �E�摜�E�B���h�E��

	cv::namedWindow(windowName_l, CV_WINDOW_NORMAL);     //! ���摜�E�B���h�E�𐶐�
	cv::namedWindow(windowName_r, CV_WINDOW_NORMAL);     //! �E�摜�E�B���h�E�𐶐�

	cv::resizeWindow(windowName_l, cap_size.width * (5 / 8), cap_size.height * (5 / 8)); //! ���摜�E�B���h�E�T�C�Y�����̃T�C�Y��(5/8)�{�ɕύX
	cv::resizeWindow(windowName_r, cap_size.width * (5 / 8), cap_size.height * (5 / 8)); //! �E�摜�E�B���h�E�T�C�Y�����̃T�C�Y��(5/8)�{�ɕύX

	cv::moveWindow(windowName_l, 100, 100);                               //! ���摜�E�B���h�E���f�B�X�v���C�̍��ォ�� (x,y)=(100,100) �̈ʒu�ɔz�u
	cv::moveWindow(windowName_r, 100 + cap_size.width * (5 / 8), 100);    //! �E�摜�E�B���h�E���f�B�X�v���C�̍��ォ�� (x,y)=(100 + ���摜�E�B���h�E�̕�,100) �̈ʒu�ɔz�u

	/********************************************//**
	 * �����J�n
	 ***********************************************/

	while (true) {
		
		/********************************************//**
		 *  �摜����
		 ***********************************************/

		/*!
		 * �L���v�`���J�n
		 */

		cv::Mat frame;		//! �L���v�`�����i�[����Mat
		cap >> frame;		//! �L���v�`�����i�[

		/*!
		 * �X�e���I�C���[�W�����E�ɕ���
		 */

		cv::Mat frame_l = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));				//! ���摜
		cv::Mat frame_r = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));	//! �E�摜

		/*!
		 * �L���v�`����\��
		 */

		cv::imshow(windowName_l, frame_l);	//! ���摜��\��
		cv::imshow(windowName_r, frame_r);	//! �E�摜��\��
		
		/*!
		 * 5[msec]�L�[�{�[�h������͑ҋ@
		 * ���͂�����ꍇ,�����I��
		 */

		if (cv::waitKey(5) > 0){
			break;
		}
	}

	return 0;
}