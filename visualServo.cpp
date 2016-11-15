/**
* @file visualServo.cpp
* @brief �X�e���I�J������p����2�쓮��1�L���X�^(2DW1C)�������{�b�g�̋쓮����
* @author 13EJ034
* @date �ŏI�X�V�� : 2016/11/15
*/

#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <runCtrl.h>
#include <vutils.h>

#pragma comment(lib, "vxv2.lib")

/**
* @fn
* PWM����M�� ��Q����� �摜�x�[�X
* @brief �v�����
* @param (double con) �����̐���
* @param (������) �����̐���
* @return �߂�l�̐���
* @sa �Q�Ƃ��ׂ��֐��������΃����N���\���
* @detail �ڍׂȐ���
*/
static double pwm1(double con, double I2, double Ta2)
{
	double	Pc = 8 * (con);
	I2 = 6 * I2;
	return (Pc + I2 / Ta2);
}

/**
* @fn
* �����Ɋ֐��̐���������
* @brief �v�����
* @param (������) �����̐���
* @param (������) �����̐���
* @return �߂�l�̐���
* @sa �Q�Ƃ��ׂ��֐��������΃����N���\���
* @detail �ڍׂȐ���
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
* �����Ɋ֐��̐���������
* @brief �v�����
* @param (������) �����̐���
* @param (������) �����̐���
* @return �߂�l�̐���
* @sa �Q�Ƃ��ׂ��֐��������΃����N���\���
* @detail �ڍׂȐ���
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
* �����Ɋ֐��̐���������
* @brief �v�����
* @param (������) �����̐���
* @param (������) �����̐���
* @return �߂�l�̐���
* @sa �Q�Ƃ��ׂ��֐��������΃����N���\���
* @detail �ڍׂȐ���
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
	 *  �J�����N���\��
	 ***********************************************/

	const double realTime_fps = 60.0;
	cv::Size cap_size(1280 * 2, 720);

	cv::VideoCapture cap;
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
	cap.set(CV_CAP_PROP_FPS, realTime_fps);

	/*!
	 * �J�����N���m�F
	 * �N�����s���̓G���[�I������
	 */
	if (!cap.isOpened()){
		std::cout << "Camera could not found." << std::endl;
		return false;
	}
	else{
		std::cout << "Camera has been opened." << std::endl;
	}

	/********************************************//**
	 *  �^��\��
	 ***********************************************/

	const std::string saveMovieName_l = "./Data/movie/result_l.avi";
	const std::string saveMovieName_r = "./Data/movie/result_r.avi";

	cv::Size video_size(1280, 720);

	const int codec = CV_FOURCC('X', 'V', 'I', 'D');
	const double record_fps = 30.0;

	cv::VideoWriter writer_l(saveMovieName_l, codec, record_fps, video_size, true);
	cv::VideoWriter writer_r(saveMovieName_r, codec, record_fps, video_size, true);

	/********************************************//**
	 *  �E�B���h�E�\��
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
	 *  ���{�b�g�\��
	 ***********************************************/

	
	static RunCtrl run;			//! ���{�b�g�̐���\����

	const int MotorID_r = 0;	//! �E���[�^ID
	const int MotorID_l = 1;	//! �����[�^ID

	int PWM = 200;				//! �PWM�M��

	/*!
	 * ���{�b�g�N���m�F
	 */
	if (run.connect("COM6") < 0){
		std::cout << "beego not found." << std::endl;
		return -1;
	}
	else{
		std::cout << "beego has been connected." << std::endl;
	}

	/*!
	 * ���s�������w��
	 */
	run.setWheelVel(MotorID_r, 5);
	run.setWheelVel(MotorID_l, 5);


	/********************************************//**
	 *  �ϐ�
	 ***********************************************/

	//! �}�[�J�[���o�p�ϐ�
	cv::Point2d d11, d12, d21, d22, rmin1, rmax1, rmin2, rmax2;

	/********************************************//**
	 *  �����J�n
	 ***********************************************/

	std::cout << "Start!" << std::endl;

	while (true){

		/********************************************//**
		 *  �摜����
		 ***********************************************/

		/*!
		 * �L���v�`���J�n
		 */

		cv::Mat frame;
		cap >> frame;
		
		/*!
		 * �X�e���I�C���[�W�����E�ɕ���
		 */

		cv::Mat frame_l = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
		cv::Mat frame_r = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		/*!
		 * �^��J�n
		 */

		writer_l << frame_l;
		writer_r << frame_r;

		/*!
		 * �L���v�`����\��
		 */

		cv::imshow(windowName_l, frame_l);
		cv::imshow(windowName_r, frame_r);

		/*!
		 * ���[�^��PWM�M���𑗐M
		 */

		run.setMotorPwm(MotorID_r, PWM);
		run.setMotorPwm(MotorID_l, PWM);

		/*!
		 * 5[msec]�L�[�{�[�h������͑ҋ@
		 * ���͂�����ꍇ,�����I��
		 */

		if (cv::waitKey(5) > 0){
			break;
		}
	}

	/*!
	 * �}��~���Ȃ��悤PWM�M����i�K�I�Ɍ���
	 * 8�i�K�Ō���
	 */

	for (int i = PWM; i >= 0; i=i-PWM/8){
		run.setMotorPwm(MotorID_r, i);
		run.setMotorPwm(MotorID_l, i);
	}

	return 0;
}