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


//机器人阻抗控制
/*
	注意事项（知识小记）：
	1.这里的getXr都是针对输入层结点只有一个！输出层也只有一个，针对压痕模型使用的。
	2.我发现，基座标z轴方向与空间中任意一个向量 叉积得到的kxita 再通过罗德里格公式所得到的旋转矩阵，就是绕定轴坐标系的旋转矩阵。
	而且，姿态的rxryrz是欧拉角，当旋转矩阵是绕固定轴旋转的旋转矩阵，那么kxita 就是rxryrz的矢量合成

	理由：
	问： 给定空间中的两个向量，如何旋转固定坐标系，使得其中一个与另外一个的方向相同？
	GPT：给定空间中的两个向量v1和v2，我们可以通过以下步骤来旋转固定坐标系，使得v1的方向与v2相同：
		计算v1和v2的夹角theta。
		计算v1和v2的叉积n，这个向量就是旋转轴。
		构造绕n轴旋转theta角的旋转矩阵R。
		将整个坐标系（或者说是包含向量v1和v2的平面）乘以旋转矩阵R，得到旋转后的坐标系。
	
	3.绕固定轴的旋转顺序是不可以改变的，改变了是不一样的  rxryrz是先后顺序绕固定坐标轴旋转的欧拉角
	4.关于servoj的资料，youtube链接（讲得很清楚）（其本质类似于pd）：https://www.youtube.com/watch?v=K-43FTU8Yp4&t=495s&ab_channel=SimpleStudent
	5.getActualTcpForce，是把基座标系平移到传感器中心那儿，然后在该坐标系下的力显示（血的教训）
	6.getActualTcpPose，原tcp坐标系在基座标系下的表示
*/


namespace Dy {

	//阻抗控制API
	class Impedance_control
	{
	public:
		//不控制姿态的初始化
		Impedance_control(double M, double B, double K,BpNet* bpnet,RTDEControlInterface* rtde_c, RTDEReceiveInterface* rtde_r)
			:M(M), B(B), K(K), tau_0(0), gamma(0), E_star(0),k(0), delta_alpha(0), delta_beta(0),bpnet(bpnet),rtde_c(rtde_c),rtde_r(rtde_r),startFlag(false){};

		//控制姿态的初始化
		Impedance_control(double M, double B, double K, double tau_0,double gamma,double E_star,int k,double delta_alpha,double delta_beta,BpNet* bpnet,RTDEControlInterface* rtde_c, RTDEReceiveInterface* rtde_r)
			:M(M), B(B), K(K), tau_0(tau_0), gamma(gamma), E_star(E_star),k(k), delta_alpha(delta_alpha), delta_beta(delta_beta),bpnet(bpnet), rtde_c(rtde_c), rtde_r(rtde_r), startFlag(false){};

		//第一个参数是 期望控制力，第二个参数是期望位置，第三个参数是控制周期，第四个参数是x轴的期望移动速度，第五个参数是servoL的t参数
		void Normal_force_control( double Fd,double Xr,double Ts,double Vx,double tForServo);

		//获取初始压痕对应的GABP，注意位置一定要贴近皮肤再使用
		void getInit(double threshold, int mostTimes);

		//根据期望力获取对应的期望位置Xr
		double getXr(double Fd);

		//阻抗控制时，进行实时训练,并返回Xr
		void doTrain_IC(double &Xr,double Fd);

		//法向力阻抗控制，基于当前位置的阻抗控制
		//参数K目前没啥用
		void Normal_force_control_base_on_now(double Fd, double Ts, vector<double>_moveUnitDirInWorld, double V, double tForServo,double sigma,int mode,double downLimit,double R,double LengthOfSensor2MassageHeadCentre,double K);

	public://下面是一些辅助功能函数

		//安全servoL,防止移动过大导致错误
		void Safe_servoL(const std::vector<double>& pose, double speed, double acceleration, double time, double lookahead_time, double gain);

		//用于获取TCP坐标下的力
		vector<double>world_2tcp_force(vector<double> world_force, vector<double>in_pose);

		//根据r xita  将TCP坐标系下的向量 转化为在基座标系下表示
		vector<double> tran_P_from_tcp_2world(vector<double> P, vector<double> rxryrz);

		//从给定的在世界坐标系中移动的向量（与力控正交的位置控制所指向的方向在世界坐标系上xoy面的投影，Z轴通常为0，因为是根据力控给的一个大概的移动方向）与kxita获得与力控方向正交的方向向量
		vector<double> get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(vector<double> _moveUnitDirInWorld, vector<double> rxryrz);

		//计算移动过程中，计算与切线和TCPZ轴同一平面且垂直于切线、与TCPZ轴夹角小于90°的单位向量 ,其中tangential是切向向量，P是TCPZ轴向量（在基座标系下）
		vector<double> getTarget(const vector<double>& tangential, const vector<double>& P);

		//根据所要求的TCP轴的转角得到 对应的欧拉角
		vector<double> get_RxRyRz_from_xitaX_and_xitaY(double θx, double θy);

		//根据所要求的TCP轴的转角得到 对应的欧拉角
		vector<double> get_RxRyRz_from_xitaX_and_xitaY_and_xitaZ(double θx, double θy, double θz);

		////关键点创新函数：根据皮肤模型和六维度力以及k值获取未知曲面法向量,具体原理见语雀,k取值0到1,R为按摩头半径，其中六维度力是TCP坐标系下的六维度力，而且原点要在按摩头球心，具体原理见语雀将坐标系将系建于球心
		////当计算出现错误（如对负数开方）时，会返回0向量
		//cv::Vec3d calculateSurfaceNormalVector(const std::vector<double>& force, double & k,double R);

		//根据皮肤模型和六维度力获取未知曲面法向量,具体原理见语雀,,R为按摩头半径，l为传感器中心到按摩头的距离，isCompensation为是否使用K临近搜索点算法来弥补刚体算法的缺陷，理论见我的语雀针对柔性作业的姿态调整理论，具体原理见语雀将坐标系建于传感器中心
		//当计算出现错误（如对负数开方）时，会返回0向量 
		static cv::Vec3d calculateSurfaceNormalVector(const std::vector<double>& force, double R,double l,bool isCompensation = false,double tau_0 = 0,double gamma = 0,double E_star = 0,int _k = 0,double _delta_alpha = 0,double _delta_beta = 0);

		
	public:
		bool startFlag;//启动标签
		QReadWriteLock* rotateLock = nullptr;
		int rotateTCPX = 0;
		int rotateTCPY = 0;

	private:
		//阻抗控制三系数
		double M;
		double B;
		double K;
		//皮肤力学性能三系数 tau_0 gamma压力系数 E_star 等效杨氏模量
		double tau_0;
		double gamma;
		double E_star;
		//移动的过程中用于训练使用的
		vector<double> vFz;
		vector<double> vXz;
		std::shared_ptr<BpNet> bpnet;//Bp神经网络

		// 水平平移时，计算出来的法向量在移动坐标系下的表示 这部分是用来姿态补偿的
		static cv::Vec3d vCompensation;
		static bool hasCompensation; // 是否已经进行了补偿

		//k临近搜索点算法的相关参数
		int k; //k值
		double delta_alpha;
		double delta_beta;

		RTDEControlInterface* rtde_c;
		RTDEReceiveInterface* rtde_r;
		Impedance_control();//只能调用有参构造！
	};

	//获取两个点的距离
	double getDistance(const vector<double>& pose1, const vector<double>& pose2);
    
	//用例代码
	void testForIC();
}
