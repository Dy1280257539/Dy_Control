#pragma once
#include<iostream>
#include <ur_rtde/rtde_control_interface.h>
#include<ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include<thread>
#include"Genetic_algorithms.h"
#include<memory>
#include<opencv.hpp>
#include<math.h>
#include<time.h>
#include <sys/timeb.h>
#include<qdebug.h>
#include<qmath.h>
#include"Impedance_control.h"

using namespace ur_rtde;
using namespace std;


//以下是一些基本的调整步骤：
//	首先将所有的PID参数都设为零。
//	增加比例参数（Kp）直到系统开始震荡。
//	减小比例参数（Kp）直到系统停止震荡并保持稳定。
//	增加积分参数（Ki）以消除系统的稳态误差。
//	如果增加积分参数（Ki）导致系统开始震荡，则减小比例参数（Kp）或增加微分参数（Kd）。
//	逐步增加微分参数（Kd）以提高系统的响应速度和稳定性。


namespace Dy {

	class PID_Control
	{
	public :
		PID_Control(double Kp, double Ki, double Kd, RTDEControlInterface* rtde_c, RTDEReceiveInterface* rtde_r)
			:Kp(Kp), Ki(Ki), Kd(Kd), rtde_r(rtde_r), rtde_c(rtde_c) {};

		void Normal_force_Control_base_on_PID(double Fd, double Ts, double Vx, double tForServo);
		void Safe_servoL(const std::vector<double>& pose, double speed, double acceleration, double time, double lookahead_time, double gain);
	
	public:
		//开启标签
		bool startFlag;
	
	private:
		//PID控制三参数
		double Kp;
		double Ki;
		double Kd;

		//控制机器人的指针
		RTDEControlInterface* rtde_c;
		RTDEReceiveInterface* rtde_r;

	};

	void testForPID();
}


