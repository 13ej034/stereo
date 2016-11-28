/**
* @file visualServo.cpp
* @brief �X�e���I�J������p����2�쓮��1�L���X�^(2DW1C)�������{�b�g�̋쓮����
* @author 13EJ034
* @date �ŏI�X�V�� : 2016/11/28
*/

#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <runCtrl.h>
#include <vutils.h>

#pragma comment(lib, "vxv2.lib")

using namespace std;
using namespace cv;


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
static double angle(Point pt1, Point pt2, Point pt0){
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
static void drawSquares11(Mat& image, const vector<vector<Point> >& squares)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const Point* p = &squares[i][0];
		int n = (int)squares[i].size();
		polylines(image, &p, &n, 1, true, Scalar(255, 255, 255), 1, CV_AA);

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
static void drawSquares12(Mat& image, const vector<vector<Point> >& squares)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const Point* p = &squares[i][0];
		int n = (int)squares[i].size();
		polylines(image, &p, &n, 1, true, Scalar(255, 255, 255), 1, CV_AA);
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

	const double cap_fps = 30.0;
	Size cap_size(1280 * 2, 720);

	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
	cap.set(CV_CAP_PROP_FPS, cap_fps);

	/*!
	 * �J�����N���m�F
	 * �N�����s���̓G���[�I������
	 */
	if (!cap.isOpened()){
		cout << "Camera could not found." << endl;
		return false;
	}
	else{
		cout << "Camera has been opened." << endl;
	}

	/********************************************//**
	 *  �^��\��
	 ***********************************************/

	const string saveMovieName_l = "./Data/movie/result_l.avi";
	const string saveMovieName_r = "./Data/movie/result_r.avi";

	Size video_size(cap_size.width / 2, cap_size.height);

	const int codec = CV_FOURCC('D', 'I', 'V', '3');
	const double rec_fps = 15.0;

	VideoWriter writer_l(saveMovieName_l, codec, rec_fps, video_size, true);
	VideoWriter writer_r(saveMovieName_r, codec, rec_fps, video_size, true);

	/********************************************//**
	 *  �E�B���h�E�\��
	 ***********************************************/

	const string windowName_l = "left";
	const string windowName_r = "right";

	namedWindow(windowName_l, CV_WINDOW_NORMAL);
	namedWindow(windowName_r, CV_WINDOW_NORMAL);

	resizeWindow(windowName_l, 800, 450);
	resizeWindow(windowName_r, 800, 450);

	moveWindow(windowName_l, 100, 100);
	moveWindow(windowName_r, 100 + 820, 100);

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
		cout << "beego not found." << endl;
		return -1;
	}
	else{
		cout << "beego has been connected." << endl;
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
	Point2d d11, d12, d21, d22, rmin1, rmax1, rmin2, rmax2;

	/*!
	 * ���ԑ���p�ϐ�
	 */ 

	TickMeter one_roop_timer;	//! ���̃��[�v�����ɂ����鎞�Ԃ𑪒�
	double total_time = 0;			//! ����������
	ofstream run_time("run_time.txt");

	/*!
	 * fps����p�ϐ�
	 */

	int cnt = 0;		//! frame��
	int oldcnt = 0;		//! �O�t���[����
	int64 nowTime = 0;  //! ������
	int64 diffTime = 0; //! �o�ߎ���

	int fps = 0;		//! 1�b�̃t���[����
	const double f = (1000 / cv::getTickFrequency());

	Point point(2, 28);	//! frame���fps�\���ʒu

	int roop_count = 0;		//! ���[�v��


	/*!
	 * �����p�����[�^
	 */

	const double fku_l = 758.963535485534;
	const double fkv_l = 753.717143388798;
	const double cx_l = 663.977396599560;
	const double cy_l = 348.145988560347;

	const double fku_r = 762.693976978763;
	const double fkv_r = 757.502496252634;
	const double cx_r = 661.685897420813;
	const double cy_r = 348.579796095923;

	Mat cameraParameter_l = (Mat_<double>(3, 3) << fku_l, 0., cx_l, 0., fkv_l, cy_l, 0., 0., 1.);
	Mat cameraParameter_r = (Mat_<double>(3, 3) << fku_r, 0., cx_r, 0., fkv_r, cy_r, 0., 0., 1.);

	const double k1_l = -0.105077838090415;
	const double k2_l = -0.0370249961738757;
	const double p1_l = 0.0;
	const double p2_l = 0.0;

	const double k1_r = -0.115246237256405;
	const double k2_r = -0.0215251205172864;
	const double p1_r = 0.0;
	const double p2_r = 0.0;

	Mat distCoeffs_l = (Mat_<double>(1, 4) << k1_l, k2_l, p1_l, p2_l);
	Mat distCoeffs_r = (Mat_<double>(1, 4) << k1_r, k2_r, p1_r, p2_r);

	Mat frame;
	Mat frame_l;
	Mat frame_r;
	Mat undistort_l;
	Mat undistort_r;


	/********************************************//**
	 *  �����J�n
	 ***********************************************/

	cout << "Start!" << endl;

	int64 startTime = getTickCount();	//! fps�\���Ɏg�p
	
	while (true){

		one_roop_timer.reset();
		one_roop_timer.start();	//! ���ԑ���J�n

		/********************************************//**
		 *  �摜����
		 ***********************************************/

		/*!
		 * �L���v�`���J�n
		 */

		cap >> frame;
		
		/*!
		 * �X�e���I�C���[�W�����E�ɕ���
		 */

		frame_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		frame_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		/*!
		 * �c�ݕ␳
		 */

		undistort(frame_l, undistort_l, cameraParameter_l, distCoeffs_l);
		undistort(frame_r, undistort_r, cameraParameter_r, distCoeffs_r);

		/*!
		* fps��frame��ɕ\������
		*/
		
		nowTime = getTickCount();
		diffTime = (int)((nowTime - startTime)*f);

		if (diffTime >= 1000) {
			startTime = nowTime;
			fps = cnt - oldcnt;
			oldcnt = cnt;
		}

		ostringstream os;
		os << fps;
		string number = os.str();

		string put_fps = "fps:" + number;
		cnt++;

		putText(undistort_l, put_fps, point, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 200), 2, CV_AA);
		putText(undistort_r, put_fps, point, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 200), 2, CV_AA);


		/*!
		 * �^��J�n
		 */

		writer_l << undistort_l;
		writer_r << undistort_r;

		/*!
		 * �L���v�`����\��
		 */

		imshow(windowName_l, undistort_l);
		imshow(windowName_r, undistort_r);

		/*!
		 * ���[�^��PWM�M���𑗐M
		 */
		
		run.setMotorPwm(MotorID_r, PWM);
		run.setMotorPwm(MotorID_l, PWM);
		
		/*!
		 * 5[msec]�L�[�{�[�h������͑ҋ@
		 * ���͂�����ꍇ,�����I��
		 */

		one_roop_timer.stop();

		run_time << one_roop_timer.getTimeMilli() << std::endl;
		
		total_time += one_roop_timer.getTimeMilli();
		roop_count++;

		if (waitKey(10) > 0){
			break;
		}

	}
	
	double ave_time = total_time / roop_count;

	cout << "total time : " << total_time << "[ms]" << endl
		<< "roop : " << roop_count << endl
		<< "average time : " << ave_time << "[ms]" << endl;


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