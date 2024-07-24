#include"PID_Control.h"

void Dy::PID_Control::Normal_force_Control_base_on_PID(double Fd, double Ts, double Vx, double tForServo) {

	//获取当前位置信息与力信息
	vector<double> pose = rtde_r->getActualTCPPose();
	vector<double> force = rtde_r->getActualTCPForce();

	//初始法向位置位置以及对应力
	//double Xe = pose[2];//------------- 控制基座标系Z轴方向的力
	//double Fe = force[2];
	double Xe = pose[0]; //------------- 控制基座标系X轴方向的力
	double Fe = force[0];


	//------------------变Fd正弦波相关---------------------//

	double A = 1.0; // 正弦波的振幅
	double f = 1.0 / 5; // 正弦波的频率，单位Hz
	double phi = 0; // 正弦波的相位，单位弧度
	double C = Fd; // 正弦波的直流偏置

	//---------------------------------------------------//

	//pid控制算法的三个数值S
	double value_p = 0.0;
	double value_i = 0.0;
	double value_d = 0.0;

	//上一时刻的环境力 (用于微分环节数值求解)
	double Fe_last = Fe;

	//切向位置移动
	//double Xd_x = pose[0];//------------- 控制基座标系X轴方向移动速度
	double Xd_x = pose[1];//------------- 控制基座标系Y轴方向移动速度
	Xd_x += Vx * Ts;

	double Xc;//传给机器人指令的位置
	startFlag = true;

	//----------------变Fd正弦波的参数--------------------//

	timeb tb;
	ftime(&tb);//获取毫秒

	long long startTime = tb.time * 1000 + tb.millitm;

	//---------------------------------------------------//

	while (startFlag)
	{
		timeb start;
		ftime(&start);//获取毫秒


		//--------------变Fd正弦波的参数----------------------//

		long long currentTime = start.time * 1000 + start.millitm;
		double t = (currentTime - startTime) / 1000.0; // 转换为秒

		// 根据时间动态计算Fd的值
		Fd = A * sin(2 * M_PI * f * t) + C;

		//---------------------------------------------------//

		//PID控制算法本体
		value_p = Fe - Fd; //目标偏差
		value_i += (Fe - Fd) * Ts; //目标偏差的积分
		value_d = (Fe - Fe_last) / Ts; //目标偏差的微分
		//法向位置是传给机器人位置伺服闭环，同时与上一时刻和pid数值相加
		Xc = Xe + Kp * value_p + Ki * value_i + Kd * value_d;

		//切向移动
		Xd_x += Vx * Ts;
		
		//pose[0] = Xd_x; //------------- 控制基座标系X轴方向移动速度
		//pose[2] = Xc; //------------- 控制基座标系Z轴位置 以实现Z轴方向力控制
		pose[1] = Xd_x; //------------- 控制基座标系Y轴方向移动速度
		pose[0] = Xc; //------------- 控制基座标系X轴位置 以实现X轴方向力控制

		//安全伺服移动
		Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);

		//更新参数,只更新法向
		Fe_last = Fe;
		//Xe = rtde_r->getActualTCPPose()[2]; //---------获取Z轴方向信息
		//Fe = rtde_r->getActualTCPForce()[2];
		Xe = rtde_r->getActualTCPPose()[0]; //---------获取X轴方向信息
		Fe = rtde_r->getActualTCPForce()[0];

		timeb end;
		ftime(&end);//获取毫秒
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
	cv::Mat _pose_nowkθ = (cv::Mat_<float>(1, 3) << pose_now[3], pose_now[4], pose_now[5]);
	cv::Mat pose_nowkθ;
	Rodrigues(_pose_nowkθ, pose_nowkθ);
	cv::Mat _pose_ckθ = (cv::Mat_<float>(1, 3) << pose[3], pose[4], pose[5]);
	cv::Mat pose_ckθ;
	Rodrigues(_pose_ckθ, pose_ckθ);

	cv::Mat axis_z = (cv::Mat_<float>(1, 3) << 0, 0, 1);
	cv::Mat axis_z_now = axis_z * pose_nowkθ;
	cv::Mat axis_z_c = axis_z * pose_ckθ;
	double axis_z_now_size = pow(pow(axis_z_now.at<float>(0, 0), 2) + pow(axis_z_now.at<float>(0, 1), 2) + pow(axis_z_now.at<float>(0, 2), 2), 0.5);
	double axis_z_c_size = pow(pow(axis_z_c.at<float>(0, 0), 2) + pow(axis_z_c.at<float>(0, 1), 2) + pow(axis_z_c.at<float>(0, 2), 2), 0.5);
	double θ = acos(((double)axis_z_now.at<float>(0, 0) * axis_z_c.at<float>(0, 0) + (double)axis_z_now.at<float>(0, 1) * axis_z_c.at<float>(0, 1) + (double)axis_z_now.at<float>(0, 2) * axis_z_c.at<float>(0, 2)) / (axis_z_now_size * axis_z_c_size)) / M_PI * 180;
	qDebug() << "θ:" << θ << '\t' << "pose_count:" << pose_count << endl;
	//先不考虑角度 后期考虑角度时候要加上abs(θ)<30 表示转角不能超过30度
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

	// 两个指针在QT里面的窗口类会有，即Dy_Control
	RTDEControlInterface* rtde_c = new RTDEControlInterface("192.168.55.101");
	RTDEReceiveInterface* rtde_r = new RTDEReceiveInterface("192.168.55.101");

	//创建实例化对象并使用对应的函数
	PID_Control pc(Kp, Ki, Kd, rtde_c, rtde_r);
	pc.Normal_force_Control_base_on_PID(Fd, Ts, Vx, tForservo);

}