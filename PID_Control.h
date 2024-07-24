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


//������һЩ�����ĵ������裺
//	���Ƚ����е�PID��������Ϊ�㡣
//	���ӱ���������Kp��ֱ��ϵͳ��ʼ�𵴡�
//	��С����������Kp��ֱ��ϵͳֹͣ�𵴲������ȶ���
//	���ӻ��ֲ�����Ki��������ϵͳ����̬��
//	������ӻ��ֲ�����Ki������ϵͳ��ʼ�𵴣����С����������Kp��������΢�ֲ�����Kd����
//	������΢�ֲ�����Kd�������ϵͳ����Ӧ�ٶȺ��ȶ��ԡ�


namespace Dy {

	class PID_Control
	{
	public :
		PID_Control(double Kp, double Ki, double Kd, RTDEControlInterface* rtde_c, RTDEReceiveInterface* rtde_r)
			:Kp(Kp), Ki(Ki), Kd(Kd), rtde_r(rtde_r), rtde_c(rtde_c) {};

		void Normal_force_Control_base_on_PID(double Fd, double Ts, double Vx, double tForServo);
		void Safe_servoL(const std::vector<double>& pose, double speed, double acceleration, double time, double lookahead_time, double gain);
	
	public:
		//������ǩ
		bool startFlag;
	
	private:
		//PID����������
		double Kp;
		double Ki;
		double Kd;

		//���ƻ����˵�ָ��
		RTDEControlInterface* rtde_c;
		RTDEReceiveInterface* rtde_r;

	};

	void testForPID();
}


