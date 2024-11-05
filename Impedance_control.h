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
#include<qmutex.h>
#include <QReadWriteLock>
#include"functionSet.h"
#include"myScience.h"
#include <chrono>

#define PI acos(-1)

using namespace ur_rtde;


//�������迹����
/*
	ע�����֪ʶС�ǣ���
	1.�����getXr��������������ֻ��һ���������Ҳֻ��һ�������ѹ��ģ��ʹ�õġ�
	2.�ҷ��֣�������z�᷽����ռ�������һ������ ����õ���kxita ��ͨ���޵����ʽ���õ�����ת���󣬾����ƶ�������ϵ����ת����
	���ң���̬��rxryrz��ŷ���ǣ�����ת�������ƹ̶�����ת����ת������ôkxita ����rxryrz��ʸ���ϳ�

	���ɣ�
	�ʣ� �����ռ��е����������������ת�̶�����ϵ��ʹ������һ��������һ���ķ�����ͬ��
	GPT�������ռ��е���������v1��v2�����ǿ���ͨ�����²�������ת�̶�����ϵ��ʹ��v1�ķ�����v2��ͬ��
		����v1��v2�ļн�theta��
		����v1��v2�Ĳ��n���������������ת�ᡣ
		������n����תtheta�ǵ���ת����R��
		����������ϵ������˵�ǰ�������v1��v2��ƽ�棩������ת����R���õ���ת�������ϵ��
	
	3.�ƹ̶������ת˳���ǲ����Ըı�ģ��ı����ǲ�һ����  rxryrz���Ⱥ�˳���ƹ̶���������ת��ŷ����
	4.����servoj�����ϣ�youtube���ӣ����ú���������䱾��������pd����https://www.youtube.com/watch?v=K-43FTU8Yp4&t=495s&ab_channel=SimpleStudent
	5.getActualTcpForce���ǰѻ�����ϵƽ�Ƶ������������Ƕ���Ȼ���ڸ�����ϵ�µ�����ʾ��Ѫ�Ľ�ѵ��
	6.getActualTcpPose��ԭtcp����ϵ�ڻ�����ϵ�µı�ʾ
*/


namespace Dy {

	//�迹����API
	class Impedance_control
	{
	public:
		//��������̬�ĳ�ʼ��
		Impedance_control(double M, double B, double K,BpNet* bpnet,RTDEControlInterface* rtde_c, RTDEReceiveInterface* rtde_r)
			:M(M), B(B), K(K), tau_0(0), gamma(0), E_star(0),k(0), delta_alpha(0), delta_beta(0),bpnet(bpnet),rtde_c(rtde_c),rtde_r(rtde_r),startFlag(false){};

		//������̬�ĳ�ʼ��
		Impedance_control(double M, double B, double K, double tau_0,double gamma,double E_star,int k,double delta_alpha,double delta_beta,BpNet* bpnet,RTDEControlInterface* rtde_c, RTDEReceiveInterface* rtde_r)
			:M(M), B(B), K(K), tau_0(tau_0), gamma(gamma), E_star(E_star),k(k), delta_alpha(delta_alpha), delta_beta(delta_beta),bpnet(bpnet), rtde_c(rtde_c), rtde_r(rtde_r), startFlag(false){};

		//��һ�������� �������������ڶ�������������λ�ã������������ǿ������ڣ����ĸ�������x��������ƶ��ٶȣ������������servoL��t����
		void Normal_force_control( double Fd,double Xr,double Ts,double Vx,double tForServo);

		//��ȡ��ʼѹ�۶�Ӧ��GABP��ע��λ��һ��Ҫ����Ƥ����ʹ��
		void getInit(double threshold, int mostTimes);

		//������������ȡ��Ӧ������λ��Xr
		double getXr(double Fd);

		//�迹����ʱ������ʵʱѵ��,������Xr
		void doTrain_IC(double &Xr,double Fd);

		//�������迹���ƣ����ڵ�ǰλ�õ��迹����
		//����KĿǰûɶ��
		void Normal_force_control_base_on_now(double Fd, double Ts, vector<double>_moveUnitDirInWorld, double V, double tForServo,double sigma,int mode,double downLimit,double R,double LengthOfSensor2MassageHeadCentre,double K);

	public://������һЩ�������ܺ���

		//��ȫservoL,��ֹ�ƶ������´���
		void Safe_servoL(const std::vector<double>& pose, double speed, double acceleration, double time, double lookahead_time, double gain);

		//���ڻ�ȡTCP�����µ���
		vector<double>world_2tcp_force(vector<double> world_force, vector<double>in_pose);

		//����r xita  ��TCP����ϵ�µ����� ת��Ϊ�ڻ�����ϵ�±�ʾ
		vector<double> tran_P_from_tcp_2world(vector<double> P, vector<double> rxryrz);

		//�Ӹ���������������ϵ���ƶ���������������������λ�ÿ�����ָ��ķ�������������ϵ��xoy���ͶӰ��Z��ͨ��Ϊ0����Ϊ�Ǹ������ظ���һ����ŵ��ƶ�������kxita��������ط��������ķ�������
		vector<double> get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(vector<double> _moveUnitDirInWorld, vector<double> rxryrz);

		//�����ƶ������У����������ߺ�TCPZ��ͬһƽ���Ҵ�ֱ�����ߡ���TCPZ��н�С��90��ĵ�λ���� ,����tangential������������P��TCPZ���������ڻ�����ϵ�£�
		vector<double> getTarget(const vector<double>& tangential, const vector<double>& P);

		//������Ҫ���TCP���ת�ǵõ� ��Ӧ��ŷ����
		vector<double> get_RxRyRz_from_xitaX_and_xitaY(double ��x, double ��y);

		//������Ҫ���TCP���ת�ǵõ� ��Ӧ��ŷ����
		vector<double> get_RxRyRz_from_xitaX_and_xitaY_and_xitaZ(double ��x, double ��y, double ��z);

		////�ؼ��㴴�º���������Ƥ��ģ�ͺ���ά�����Լ�kֵ��ȡδ֪���淨����,����ԭ�����ȸ,kȡֵ0��1,RΪ��Ħͷ�뾶��������ά������TCP����ϵ�µ���ά����������ԭ��Ҫ�ڰ�Ħͷ���ģ�����ԭ�����ȸ������ϵ��ϵ��������
		////��������ִ�����Ը���������ʱ���᷵��0����
		//cv::Vec3d calculateSurfaceNormalVector(const std::vector<double>& force, double & k,double R);

		//����Ƥ��ģ�ͺ���ά������ȡδ֪���淨����,����ԭ�����ȸ,,RΪ��Ħͷ�뾶��lΪ���������ĵ���Ħͷ�ľ��룬isCompensationΪ�Ƿ�ʹ��K�ٽ��������㷨���ֲ������㷨��ȱ�ݣ����ۼ��ҵ���ȸ���������ҵ����̬�������ۣ�����ԭ�����ȸ������ϵ���ڴ���������
		//��������ִ�����Ը���������ʱ���᷵��0���� 
		static cv::Vec3d calculateSurfaceNormalVector(const std::vector<double>& force, double R,double l,bool isCompensation = false,double tau_0 = 0,double gamma = 0,double E_star = 0,int _k = 0,double _delta_alpha = 0,double _delta_beta = 0);

		
	public:
		bool startFlag;//������ǩ
		QReadWriteLock* rotateLock = nullptr;
		int rotateTCPX = 0;
		int rotateTCPY = 0;

	private:
		//�迹������ϵ��
		double M;
		double B;
		double K;
		//Ƥ����ѧ������ϵ�� tau_0 gammaѹ��ϵ�� E_star ��Ч����ģ��
		double tau_0;
		double gamma;
		double E_star;
		//�ƶ��Ĺ���������ѵ��ʹ�õ�
		vector<double> vFz;
		vector<double> vXz;
		std::shared_ptr<BpNet> bpnet;//Bp������

		// ˮƽƽ��ʱ����������ķ��������ƶ�����ϵ�µı�ʾ �ⲿ����������̬������
		static cv::Vec3d vCompensation;
		static bool hasCompensation; // �Ƿ��Ѿ������˲���

		//k�ٽ��������㷨����ز���
		int k; //kֵ
		double delta_alpha;
		double delta_beta;

		RTDEControlInterface* rtde_c;
		RTDEReceiveInterface* rtde_r;
		Impedance_control();//ֻ�ܵ����вι��죡
	};

	//��ȡ������ľ���
	double getDistance(const vector<double>& pose1, const vector<double>& pose2);
    
	//��������
	void testForIC();
}
