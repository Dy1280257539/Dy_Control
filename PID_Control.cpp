#include"PID_Control.h"

void Dy::PID_Control::Normal_force_Control_base_on_PID(double Fd, double Ts, double Vx, double tForServo) {

	//��ȡ��ǰλ����Ϣ������Ϣ
	vector<double> pose = rtde_r->getActualTCPPose();
	vector<double> force = rtde_r->getActualTCPForce();

	//��ʼ����λ��λ���Լ���Ӧ��
	//double Xe = pose[2];//------------- ���ƻ�����ϵZ�᷽�����
	//double Fe = force[2];
	double Xe = pose[0]; //------------- ���ƻ�����ϵX�᷽�����
	double Fe = force[0];


	//------------------��Fd���Ҳ����---------------------//

	double A = 1.0; // ���Ҳ������
	double f = 1.0 / 5; // ���Ҳ���Ƶ�ʣ���λHz
	double phi = 0; // ���Ҳ�����λ����λ����
	double C = Fd; // ���Ҳ���ֱ��ƫ��

	//---------------------------------------------------//

	//pid�����㷨��������ֵS
	double value_p = 0.0;
	double value_i = 0.0;
	double value_d = 0.0;

	//��һʱ�̵Ļ����� (����΢�ֻ�����ֵ���)
	double Fe_last = Fe;

	//����λ���ƶ�
	//double Xd_x = pose[0];//------------- ���ƻ�����ϵX�᷽���ƶ��ٶ�
	double Xd_x = pose[1];//------------- ���ƻ�����ϵY�᷽���ƶ��ٶ�
	Xd_x += Vx * Ts;

	double Xc;//����������ָ���λ��
	startFlag = true;

	//----------------��Fd���Ҳ��Ĳ���--------------------//

	timeb tb;
	ftime(&tb);//��ȡ����

	long long startTime = tb.time * 1000 + tb.millitm;

	//---------------------------------------------------//

	while (startFlag)
	{
		timeb start;
		ftime(&start);//��ȡ����


		//--------------��Fd���Ҳ��Ĳ���----------------------//

		long long currentTime = start.time * 1000 + start.millitm;
		double t = (currentTime - startTime) / 1000.0; // ת��Ϊ��

		// ����ʱ�䶯̬����Fd��ֵ
		Fd = A * sin(2 * M_PI * f * t) + C;

		//---------------------------------------------------//

		//PID�����㷨����
		value_p = Fe - Fd; //Ŀ��ƫ��
		value_i += (Fe - Fd) * Ts; //Ŀ��ƫ��Ļ���
		value_d = (Fe - Fe_last) / Ts; //Ŀ��ƫ���΢��
		//����λ���Ǵ���������λ���ŷ��ջ���ͬʱ����һʱ�̺�pid��ֵ���
		Xc = Xe + Kp * value_p + Ki * value_i + Kd * value_d;

		//�����ƶ�
		Xd_x += Vx * Ts;
		
		//pose[0] = Xd_x; //------------- ���ƻ�����ϵX�᷽���ƶ��ٶ�
		//pose[2] = Xc; //------------- ���ƻ�����ϵZ��λ�� ��ʵ��Z�᷽��������
		pose[1] = Xd_x; //------------- ���ƻ�����ϵY�᷽���ƶ��ٶ�
		pose[0] = Xc; //------------- ���ƻ�����ϵX��λ�� ��ʵ��X�᷽��������

		//��ȫ�ŷ��ƶ�
		Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);

		//���²���,ֻ���·���
		Fe_last = Fe;
		//Xe = rtde_r->getActualTCPPose()[2]; //---------��ȡZ�᷽����Ϣ
		//Fe = rtde_r->getActualTCPForce()[2];
		Xe = rtde_r->getActualTCPPose()[0]; //---------��ȡX�᷽����Ϣ
		Fe = rtde_r->getActualTCPForce()[0];

		timeb end;
		ftime(&end);//��ȡ����
		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		if (deta_t < Ts * 1000) {
			Sleep(Ts * 1000 - deta_t);
		}
	}
	rtde_c->servoStop();
}

void Dy::PID_Control::Safe_servoL(const std::vector<double>& pose, double speed, double acceleration, double time, double lookahead_time, double gain) {

	std::vector<double> pose_now = rtde_r->getActualTCPPose();
	double pose_count = pow((pow(pose_now[0] - pose[0], 2) + pow(pose_now[1] - pose[1], 2) + pow(pose_now[2] - pose[2], 2)), 0.5) * 1000;
	cv::Mat _pose_nowk�� = (cv::Mat_<float>(1, 3) << pose_now[3], pose_now[4], pose_now[5]);
	cv::Mat pose_nowk��;
	Rodrigues(_pose_nowk��, pose_nowk��);
	cv::Mat _pose_ck�� = (cv::Mat_<float>(1, 3) << pose[3], pose[4], pose[5]);
	cv::Mat pose_ck��;
	Rodrigues(_pose_ck��, pose_ck��);

	cv::Mat axis_z = (cv::Mat_<float>(1, 3) << 0, 0, 1);
	cv::Mat axis_z_now = axis_z * pose_nowk��;
	cv::Mat axis_z_c = axis_z * pose_ck��;
	double axis_z_now_size = pow(pow(axis_z_now.at<float>(0, 0), 2) + pow(axis_z_now.at<float>(0, 1), 2) + pow(axis_z_now.at<float>(0, 2), 2), 0.5);
	double axis_z_c_size = pow(pow(axis_z_c.at<float>(0, 0), 2) + pow(axis_z_c.at<float>(0, 1), 2) + pow(axis_z_c.at<float>(0, 2), 2), 0.5);
	double �� = acos(((double)axis_z_now.at<float>(0, 0) * axis_z_c.at<float>(0, 0) + (double)axis_z_now.at<float>(0, 1) * axis_z_c.at<float>(0, 1) + (double)axis_z_now.at<float>(0, 2) * axis_z_c.at<float>(0, 2)) / (axis_z_now_size * axis_z_c_size)) / M_PI * 180;
	qDebug() << "��:" << �� << '\t' << "pose_count:" << pose_count << endl;
	//�Ȳ����ǽǶ� ���ڿ��ǽǶ�ʱ��Ҫ����abs(��)<30 ��ʾת�ǲ��ܳ���30��
	if (pose_count < 15) {
		rtde_c->servoL(pose, speed, acceleration, time, lookahead_time, gain);
	}
	else {
		abort();
	}
}

void Dy::testForPID() {

	double Kp = 1;
	double Ki = 1;
	double Kd = 1;
	double Fd = 10;
	double Ts = 0.002;
	double Vx = 0.01;
	double tForservo = 0.002;

	// ����ָ����QT����Ĵ�������У���Dy_Control
	RTDEControlInterface* rtde_c = new RTDEControlInterface("192.168.55.101");
	RTDEReceiveInterface* rtde_r = new RTDEReceiveInterface("192.168.55.101");

	//����ʵ��������ʹ�ö�Ӧ�ĺ���
	PID_Control pc(Kp, Ki, Kd, rtde_c, rtde_r);
	pc.Normal_force_Control_base_on_PID(Fd, Ts, Vx, tForservo);

}