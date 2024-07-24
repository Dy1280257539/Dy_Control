#include "Impedance_control.h"

cv::Vec3d Dy::Impedance_control::vCompensation = cv::Vec3d(0.0, 0.0, 0.0);
bool Dy::Impedance_control:: hasCompensation = false; // 是否已经进行了补偿

void Dy::Impedance_control::Normal_force_control(double Fd, double Xr, double Ts, double Vx,double tForServo) {
	
	//初始化三个参数
	double ΔX_v = 0;//位置偏差的一阶导数
	double ΔX_a = 0;//位置偏差的二阶导数
	double ΔX = 0;  //位置偏差

	vector<double> pose = rtde_r->getActualTCPPose();
	vector<double> force = rtde_r->getActualTCPForce();

	double Fe = force[2];//法向环境力

	//阻抗控制本体
	ΔX_a = (Fe - Fd - B * ΔX_v - K * ΔX) / M;
	ΔX_v = ΔX_v + ΔX_a * Ts;
	ΔX = ΔX + ΔX_v * Ts;
	double Xd;
	Xd = Xr + ΔX;//期望位置

	pose[2] = Xd;

	//x轴方向上的期望位置
	double Xd_x = pose[0] + Vx * Ts;//加上在x轴方向上的移动距离在Ts上Vx的理应移动距离

	pose[0] = Xd_x;

	startFlag = true;//开启！

	//100次进行一次神经网络训练更新,count用于计数
	int count = 0;


	while (startFlag)
	{
		timeb start;
		ftime(&start);//获取毫秒
		if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
			break;
		Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);//Safe_servoL是不会阻塞进程的
		//更新数据
		force = rtde_r->getActualTCPForce();
		Fe = force[2];//法向环境力
		//阻抗控制算法本体
		ΔX_a = (Fe - Fd - B * ΔX_v - K * ΔX) / M;
		ΔX_v = ΔX_v + ΔX_a * Ts;
		ΔX = ΔX + ΔX_v * Ts;
		//qDebug() << ΔX<<endl;

		Xd = Xr + ΔX;//期望位置
		pose[2] = Xd;
		Xd_x += Vx * Ts;//x轴方向上的期望位置
		pose[0] = Xd_x;
		timeb end;
		ftime(&end);//获取毫秒
		if (count == 0)
		{
			vFz.clear();
			vXz.clear();
		}
		count++;
		vFz.push_back(rtde_r->getActualTCPForce()[2]);
		vXz.push_back(rtde_r->getActualTCPPose()[2]);
		if (count==100)//
		{
			count = 0;
			std::thread doTrain([&]() {
				doTrain_IC(Xr,Fd);
				});
			//摆脱主程序的束缚，不然会报错
			doTrain.detach();
		}
		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		if (deta_t < Ts*1000) {
			Sleep(Ts*1000 - deta_t);
		}
	}
	rtde_c->servoStop();
}

void Dy::Impedance_control::doTrain_IC(double & Xr, double Fd) {
	qDebug() << vFz.size()<<"   "<<vXz.size();
	auto rawTrainGroup = getTrainData(vector<double>(this->vFz), vector<double>(this->vXz));
	auto trainGroup = bpnet->normalisation(rawTrainGroup, true);
	this->bpnet->doTraining(trainGroup, 1, 10);//平方误差和先取个1，实验证明也达不到，训练次数是100
	qDebug() << " train finish " <<"  " <<Xr<< endl;
	Xr = getXr(Fd);//根据Fd得到期望力
	qDebug() << " getXr finish " <<"  "<<Xr << endl;
}

void Dy::Impedance_control::getInit(double threshold,int mostTimes){
	vector<double> normalForce;//法向力数据组 对应vOutput
	vector<double> base_normal_location;//基座标下的法向位移数据组 对应vInput
	vector<double> initPose = rtde_r->getActualTCPPose(); //初始点
	vector<double> finalPose = initPose;//压入的终止点
	finalPose[2] -= 0.01;//向下按压10mm
	startFlag = true;
	//获取法向力数据，并压入到normalForce
	std::thread getData([&]() mutable {
		while (startFlag)
		{
			normalForce.push_back((rtde_r->getActualTCPForce())[2]);
			base_normal_location.push_back(((rtde_r->getActualTCPPose())[2]));
			Sleep(20);//20ms读一次
		}
		 });
	std::thread moveNormal([&]()mutable {
		//一来一回
		rtde_c->moveL(finalPose, 0.001, 1.2, false);
		rtde_c->moveL(initPose, 0.001, 1.2, false);
		startFlag = false;
		});
	moveNormal.join();
	getData.join();
	//初始化数据
	vector<sample> rawTrainGroup = getTrainData(normalForce,base_normal_location);
	vector<sample> trainGroup = bpnet->normalisation(rawTrainGroup, true);//对训练集进行归一化
	getInitPop(pop, trainGroup);
	gen = 1;//重设一下代数，默认初始代数为1；
	while (gen<generation)
	{
		evolution(*bpnet, trainGroup);
	}
	int fitTag = findBest(pop);
	chromoDecode(pop[fitTag], *bpnet);
	bpnet->doTraining(trainGroup, threshold, mostTimes);
}

double Dy::Impedance_control::getXr( double Fd ) {

	//下面是将Fd做成一个vector<sample>，方便进行归一化
	vector<double> vdouble;
	vdouble.push_back(Fd);
	vector<sample> vsample;
	sample s;
	s.in = vdouble;
	vsample.push_back(s);
	vector<sample> predGroup = bpnet->normalisation(vsample, false); //预测集
	// ------------------------------------------------

	bpnet->doTesting(predGroup);
	vector<sample> rawPredGroup = bpnet->denormalisation(predGroup);//进行反归一化得到对应的期望位移
	double Xr = rawPredGroup.front().out.front();//获取预测Xr
	return Xr;
}

double Dy::getDistance(const vector<double>& pose1, const vector<double>& pose2) {

	double distance = pow(pow(pose1[0] - pose2[0], 2) + pow(pose1[1] - pose2[1], 2) + pow(pose1[2] - pose2[2], 2), 0.5);
	return distance;
}

void Dy::Impedance_control::Normal_force_control_base_on_now(double Fd, double Ts, vector<double>_moveUnitDirInWorld,double V,double tForServo,double sigma,int mode,double downLimit,double R,double LengthOfSensor2MassageHeadCentre,double K) {
	
	if (mode == -1) //test
	{
		if (rotateLock == nullptr) { //此功能需要有读写锁才能用
			qDebug() << "need a rotatelock" << endl;
			return;
		}

		//我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		int tmp1 = _moveUnitDirInWorld[0];
		int tmp2 = _moveUnitDirInWorld[1];

		Fd = -1 * Fd;//因为这里要转换为在TCP坐标系下，所以期望力10N要变成-10N

		vector<double> normalTCP_move{ 0,0,1 };//表示一个在TCP坐标系下法向的移动增量

		//Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double φt = 0;

		double theta = LengthOfSensor2MassageHeadCentre * 1000 *  PI / 180; //圆形轨迹
		double Δθ = K * PI / 180;

		//如果按下按钮，则希望1s 转动10°，则一个周期转动的角度分别为Δθx和Δθy；(注意这里要是弧度制，所以要转为弧度)
		double Δθx = 10 * R * 1000 * Ts * PI / 180;
		double Δθy = 10 * R * 1000 * Ts * PI / 180;

		qDebug() << "Δθx : " << Δθx << endl;
		qDebug() << "Δθy : " << Δθy << endl;

		double curRotateX = 0;//当前需要转动的flag
		double curRotateY = 0;

		//获取当前位置信息
		vector<double> pose = rtde_r->getActualTCPPose();

		//初始化参数
		//皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前TCP位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//初始化期望位置加速度
		Xc_ddt = 0;

		//阻抗控制本体
		auto temp_Xc = Xc;//上一时刻的Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下
		if (rotateLock->tryLockForRead()) { //尝试去读取rotateX与Y，并更新当前需要转动的角度
			curRotateX = rotateTCPX * Δθx;
			curRotateY = rotateTCPY * Δθy;
			rotateLock->unlock();
		}
		auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

		//期望位置与期望姿态
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newAngle[0];
		pose[4] = newAngle[1];
		pose[5] = newAngle[2];

		//下一次期望加速度 这里Fd与Fe都是在TCP方向上的
		vector<double> force = rtde_r->getActualTCPForce();
		force = world_2tcp_force(force, pose);//转化为TCP坐标下的力
		double Fe = force[2];//法向环境力
		φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		direction_vector[0] = direction_vector[0] * cos(theta) - direction_vector[1] * sin(theta);
		direction_vector[1] = direction_vector[0] * sin(theta) + direction_vector[1] * cos(theta);

		theta += Δθ;

		startFlag = true;//开启！

		// --------------------临时增加----------------------------
		timeb t1;
		ftime(&t1);//获取毫秒

		long long t = t1.time * 1000 + t1.millitm;

		// --------------------临时增加----------------------------

		timeb start;
		ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		while (startFlag)
		{
			if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
			//另外下降过程不用考虑姿态
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

				//期望位置
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 2000);
				ftime(&start);//获取毫秒

				continue;
			}
			else
			{
				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//获取毫秒
			}

			//更新数据
			pose = rtde_r->getActualTCPPose();

			//阻抗控制算法本体
			auto temp_Xc = Xc;//上一时刻的Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出要移动的距离
			if (rotateLock->tryLockForRead()) {
				curRotateX = rotateTCPX * Δθx;
				curRotateY = rotateTCPY * Δθy;
				rotateLock->unlock();
			}
			auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

			//更新这次期望位置与期望姿态
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			long long cur = start.time * 1000 + start.millitm;

			if (cur - t > 1000) { //下降后停止一秒再移动
				// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
				vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
				pose[0] += V * direction_vector[0] * Ts;
				pose[1] += V * direction_vector[1] * Ts;
				pose[2] += V * direction_vector[2] * Ts;
			}

			pose[3] = newAngle[0];
			pose[4] = newAngle[1];
			pose[5] = newAngle[2];

			//更新下一次的补偿位置加速度
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//转化为TCP坐标下的力
			Fe = force[2];//法向环境力
			φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

			_moveUnitDirInWorld[0] = tmp1 * cos(theta) - tmp2 * sin(theta);
			_moveUnitDirInWorld[1] = tmp1 * sin(theta) + tmp2 * cos(theta);

			theta += Δθ;
			if (theta > 2 * PI)
				theta -= 2 * PI;

			qDebug() << "theta :" << theta << endl;

		}
		rtde_c->servoStop();
	}
	else if (mode == -2) {  //姿态变换的基础：自适应阻抗控制
		
		//我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		Fd = -1 * Fd;//因为这里要转换为在TCP坐标系下，所以期望力10N要变成-10N

		vector<double> normalTCP_move{ 0,0,1 };//表示一个在TCP坐标系下法向的移动增量

		//Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double φt = 0;

		//获取当前位置信息
		vector<double> pose = rtde_r->getActualTCPPose();

		//初始化参数
		//皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前TCP位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//初始化期望加速度
		Xc_ddt = 0;

		//阻抗控制本体
		auto temp_Xc = Xc;//上一时刻的Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

		//期望位置
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// 根据控制周期计算出x轴方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误差
		pose[0] += V * Ts;

		//下一次期望加速度 这里Fd与Fe都是在TCP方向上的
		vector<double> force = rtde_r->getActualTCPForce();
		force = world_2tcp_force(force, pose);//转化为TCP坐标下的力
		double Fe = force[2];//法向环境力
		φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//开启！

		timeb start;
		ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		while (startFlag)
		{
			if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
				break;
			qDebug() << "Xc_ddt: " << Xc_ddt;
			qDebug() << "Xc_dt" << Xc_dt;
			qDebug() << "Xc" << Xc;
			qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];
			
			//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

				//期望位置
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				// 根据控制周期计算出x轴方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误差
				pose[0] += V * Ts;

				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//获取毫秒

				continue;
			}
			else
			{
				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//获取毫秒
			}

			//更新数据
			pose = rtde_r->getActualTCPPose();

			//阻抗控制算法本体
			auto temp_Xc = Xc;//上一时刻的Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出要移动的距离

			//更新这次期望位置
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];
			pose[0] += V * Ts;

			//更新下一次的补偿位置加速度
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//转化为TCP坐标下的力
			Fe = force[2];//法向环境力
			φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();
	}
	else if (mode == -3)//自适应变阻尼(旧版本自适应变阻尼，只能垂直与基座标系z轴)
	{
		//Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double φt = 0;

		//获取当前位置信息
		vector<double> pose = rtde_r->getActualTCPPose();
		//初始化参数
		//皮肤平面，则Xe_ddt=0;Xe_dt=0;Xc=Xe;以当前位置作为Xc
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = pose[2];
		Xc = Xe;

		//初始化期望加速度
		Xc_ddt = 0;

		//阻抗控制本体
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;

		//期望位置
		pose[2] = Xc;

		// 根据控制周期计算出x轴方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误差
		double Xd_x = pose[0] + V * Ts;
		pose[0] = Xd_x;

		//下一次期望加速度
		vector<double> force = rtde_r->getActualTCPForce();
		double Fe = force[2];//法向环境力
		φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//开启！

		while (startFlag)
		{
			timeb start;
			ftime(&start);//获取毫秒

			if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
				break;
			qDebug() << "Xc_ddt: " << Xc_ddt;
			qDebug() << "Xc_dt" << Xc_dt;
			qDebug() << "Xc" << Xc;
			qDebug() << "Fe" << rtde_r->getActualTCPForce()[2];
			//防止阻抗控制在高空下降得过快，添加一个阈值0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt，直到接触为止（给了0.2N的接触阈值）
			if (Xc_dt< - downLimit && rtde_r->getActualTCPForce()[2]< Fd/2) 
			{
				qDebug() << " protect";
				Xc -= Ts*downLimit; //按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002；

				pose[2] = Xc;//移动到期望位置
				Xd_x += V * Ts;
				pose[0] = Xd_x;
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);

				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				continue;
			}
			else
			{
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
			}

			//更新数据
			pose = rtde_r->getActualTCPPose();

			//阻抗控制算法本体
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;

			pose[2] = Xc;//期望位置
			Xd_x += V * Ts;
			pose[0] = Xd_x;

			//更新下一次的补偿位置加速度
			force = rtde_r->getActualTCPForce();
			Fe = force[2];//法向环境力
			φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

			timeb end;
			ftime(&end);//获取毫秒
			int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
			if (deta_t < Ts * 1000) {
				Sleep(Ts * 1000 - deta_t);
			}
		}
		rtde_c->servoStop();
	}
	else if (mode == 0)  //内窥镜与阻抗控制同时作用所使用，目的是探索与转角有关的影响因素 （这个mode 0 就对应这个内容，如果要改则Dy_Control cpp部分也要改）
	{
		if (rotateLock == nullptr) { //此功能需要有读写锁才能用
			qDebug() << "need a rotatelock" << endl;
			return;
		}

		//我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		Fd = -1 * Fd;//因为这里要转换为在TCP坐标系下，所以期望力10N要变成-10N

		vector<double> normalTCP_move{ 0,0,1 };//表示一个在TCP坐标系下法向的移动增量

		//Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double φt = 0;

		//如果按下按钮，则希望1s 转动10°，则一个周期转动的角度分别为Δθx和Δθy；(注意这里要是弧度制，所以要转为弧度)
		double Δθx = 10 * Ts * PI / 180;
		double Δθy = 10 * Ts * PI / 180;

		double curRotateX = 0;//当前需要转动的flag
		double curRotateY = 0;

		//获取当前位置信息
		vector<double> pose = rtde_r->getActualTCPPose();

		//初始化参数
		//皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前TCP位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//初始化期望位置加速度
		Xc_ddt = 0;

		//阻抗控制本体
		auto temp_Xc = Xc;//上一时刻的Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下
		if (rotateLock->tryLockForRead()) { //尝试去读取rotateX与Y，并更新当前需要转动的角度
			curRotateX = rotateTCPX * Δθx;
			curRotateY = rotateTCPY * Δθy;
			rotateLock->unlock();
		}
		auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

		//期望位置与期望姿态
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];
		
		// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newAngle[0];
		pose[4] = newAngle[1];
		pose[5] = newAngle[2];

		//下一次期望加速度 这里Fd与Fe都是在TCP方向上的
		vector<double> force = rtde_r->getActualTCPForce();
		force = world_2tcp_force(force, pose);//转化为TCP坐标下的力
		double Fe = force[2];//法向环境力
		φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//开启！

		// --------------------临时增加----------------------------
		timeb t1;
		ftime(&t1);//获取毫秒

		long long t = t1.time * 1000 + t1.millitm;

		// --------------------临时增加----------------------------

		timeb start;
		ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		while (startFlag)
		{
			if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
			//另外下降过程不用考虑姿态
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

				//期望位置
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 2000);
				ftime(&start);//获取毫秒

				continue;
			}
			else
			{
				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//获取毫秒
			}

			//更新数据
			pose = rtde_r->getActualTCPPose();

			//阻抗控制算法本体
			auto temp_Xc = Xc;//上一时刻的Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出要移动的距离
			if (rotateLock->tryLockForRead()) {
				curRotateX = rotateTCPX * Δθx;
				curRotateY = rotateTCPY * Δθy;
				rotateLock->unlock();
			}
			auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

			//更新这次期望位置与期望姿态
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			long long cur = start.time * 1000 + start.millitm;

			if (cur - t > 1000) { //下降后停止一秒再移动
				// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
				vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
				pose[0] += V * direction_vector[0] * Ts;
				pose[1] += V * direction_vector[1] * Ts;
				pose[2] += V * direction_vector[2] * Ts;
			}

			pose[3] = newAngle[0];
			pose[4] = newAngle[1];
			pose[5] = newAngle[2];

			//更新下一次的补偿位置加速度
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//转化为TCP坐标下的力
			Fe = force[2];//法向环境力
			φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();
	}
	else if(mode == 1) //匀速旋转 + 法向自适应阻抗控制
	{
		//我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		vector<double> normalMotion_move{ 0,0,1 };//表示一个在移动坐标系下法向的移动增量

		//Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double φt = 0;

		int count = 5; //收集多少个Ts周期对应的旋转矢量进行一次合算

		int norm_count = 1; //收集多少个法向量进行一次合算法向量

		vector<cv::Vec3d> normalVector_array;

		cv::Vec3d expectRotationVector{ 0,0,0 };//合算出来的期望旋转向量 基座标系下的表示

		// 每个控制周期的轴角旋转速度 （绕某个固定轴旋转的速度）
		double Δθ = 5 * Ts * PI / 180 * count; //最前面的数字是每秒的度数（粗略）

		cv::Vec3d curRotationVector{ 0,0,0 }; //旋转矢量
		double curRotationVector_size = 0;//旋转矢量的大小

		vector<cv::Vec3d> curRotationVector_array;

		vector<double> pose = rtde_r->getActualTCPPose();

		//-------------------获取移动坐标系下的力------------------------

		vector<double> cur_pose = rtde_r->getActualTCPPose();

		auto sixth_axis = getSixthAxisFromBasePose(cur_pose);

		Vec3d end_effector_direction{ _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		auto force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, {0,0,1}, rtde_r);

		//------------------------------------------------------------

		//----------------计算基座标系到移动坐标系的旋转矩阵----------------

		Vec3d x_axis, y_axis, z_axis;

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis,rtde_r);

		cv::Mat RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		cv::Mat RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Vec3d RotVecBase2Motion;

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//------------------------------------------------------------

		//初始化参数
		//皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前移动坐标系位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//初始化期望位置加速度
		Xc_ddt = 0;

		//阻抗控制本体
		auto temp_Xc = Xc;//上一时刻的Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalMotion_move[2] = Xc - temp_Xc;
		cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下


		//根据目标向量计算移动角度
		if (force[2] < Fd / 2) { // 下降状态且力控尚未稳定,则不转动 

			curRotationVector = cv::Vec3d{ 0,0,0 };
			curRotationVector_size = 0;
		}
		else {
			cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());

			cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //目标向量在移动坐标系下的表示

			normalVector_array.push_back(_targetVec);

			if (normalVector_array.size() == norm_count) {

				cv::Vec3d sum{0.0,0.0,0.0};

				for (const auto& vec : normalVector_array)
					sum += vec;

				sum /= static_cast<double>(normalVector_array.size());

				_targetVec = sum;

				cv::Vec3d targetVec = rotateVecToTarget(_targetVec, RotMatMotion2Base);//在基座标系下的目标向量表示

				//------------------------------------------补偿模块----------------------------------------------

				cv::Vec3d vCompensation_in_base = rotateVecToTarget(vCompensation, RotMatMotion2Base); //计算补偿

				//--------------------------------------------------------------------------------------------

				if (cv::norm(targetVec) != 0) {

					curRotationVector = CalculateRotationVector(-sixthAxis, targetVec);

					//------------------------------------------补偿模块----------------------------------------------

					if (hasCompensation) {
						cv::Vec3d vCompensation_RotationVector = CalculateRotationVector(-sixthAxis, vCompensation_in_base); //计算补偿
						curRotationVector = combineRotationVectors(curRotationVector, -vCompensation_RotationVector);
					}

					//--------------------------------------------------------------------------------------------


					//curRotationVector_size = 5 * Δθ < cv::norm(curRotationVector) ? Δθ : 0;
					//ResizeVector(curRotationVector, curRotationVector_size);
				}
				else {
					curRotationVector = cv::Vec3d{ 0,0,0 };
					curRotationVector_size = 0;
				}
			}

		}

		if (normalVector_array.size() == norm_count) {

			curRotationVector_array.push_back(curRotationVector);

			normalVector_array.clear();
		}


		std::vector<double> newPose = pose;

		if (curRotationVector_array.size() == count) { //收集十组矢量就可以调整姿态了

			expectRotationVector = averageRotationVector(curRotationVector_array);
			ResizeVector(expectRotationVector, 5 * Δθ < cv::norm(expectRotationVector) ? Δθ : 0);

			curRotationVector_array.clear();

			qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
			qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
		}

		newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] ,expectRotationVector[1] ,expectRotationVector[2] });

		//期望位置与期望姿态
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newPose[3];
		pose[4] = newPose[4];
		pose[5] = newPose[5];

		//-------------------获取移动坐标系下的力------------------------

		cur_pose = rtde_r->getActualTCPPose(); 

		sixth_axis = getSixthAxisFromBasePose(cur_pose);

		end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

		//-----------------------------------------------------------

		//----------------计算基座标系到移动坐标系的旋转矩阵----------------

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis,rtde_r);

		RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//-------------------------------------------------------------

		double Fe = force[2];//法向环境力
		φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//开启！


		// --------------------初始时间点----------------------------

		timeb t1;
		ftime(&t1);//获取毫秒

		long long t = t1.time * 1000 + t1.millitm;

		// ---------------------------------------------------------

		timeb start;
		ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		while (startFlag)
		{
			if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
			//另外下降过程不用考虑姿态
			if (Xc_dt < -downLimit && getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r)[2] < Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc -= downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

				normalMotion_move[2] = Xc - temp_Xc;
				cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //根据阻抗控制得出在移动坐标系下要移动的距离，并将该距离增量转化为世界坐标系下

				//期望位置
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				//Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//获取毫秒

				continue;
			}
			else
			{
				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				//qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}

				//Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//获取毫秒
			}

			//更新数据
			pose = rtde_r->getActualTCPPose();

			//阻抗控制算法本体
			auto temp_Xc = Xc;//上一时刻的Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalMotion_move[2] = Xc - temp_Xc;
			cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //根据阻抗控制得出要移动的距离

			//qDebug() << "Force :" << force[2] << endl;
			//qDebug() << "moveDistance :" << moveDistance[0] << moveDistance[1] << moveDistance[2] << endl;
			//qDebug() << "Xc :" << Xc << " Xc_dt:" << Xc_dt << " Xc_ddt:" << Xc_ddt << endl;

			//根据目标向量计算移动角度
			if (force[2] < Fd / 2) { // 下降状态且力控尚未稳定,则不转动 

				curRotationVector = cv::Vec3d{ 0,0,0 };
				curRotationVector_size = 0;
			}
			else {
				cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
				
				//qDebug() << "base_force" << " : " << _force[0] << " " << _force[1] << " " << _force[2] << " " << _force[3] << " " << _force[4] << " " << _force[5];
				qDebug() << "force" << " : " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5];

				//force[0] = force[1] = force[3] = force[4] = force[5] = 0;
				cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //目标向量在移动坐标系下的表示

				normalVector_array.push_back(_targetVec);

				if (normalVector_array.size() == norm_count) {

					cv::Vec3d sum{ 0.0,0.0,0.0 };

					for (const auto& vec : normalVector_array)
						sum += vec;

					sum /= static_cast<double>(normalVector_array.size());

					_targetVec = sum;

					if(cv::norm(_targetVec) > 1e-8)
						cv::normalize(_targetVec, _targetVec);
					
					cv::Vec3d targetVec = rotateVecToTarget(_targetVec, RotMatMotion2Base);//在基座标系下的目标向量表示

					//------------------------------------------补偿模块----------------------------------------------

					cv::Vec3d vCompensation_in_base = rotateVecToTarget(vCompensation, RotMatMotion2Base); //计算补偿

					//----------------------------------------------------------------------------------------------

					if (cv::norm(targetVec) != 0) {

						curRotationVector = CalculateRotationVector(-sixthAxis, targetVec);

						//------------------------------------------补偿模块----------------------------------------------

						if (hasCompensation) {
							cv::Vec3d vCompensation_RotationVector = CalculateRotationVector(-sixthAxis, vCompensation_in_base); //计算补偿
							curRotationVector = combineRotationVectors(curRotationVector, -vCompensation_RotationVector);
						}

						//--------------------------------------------------------------------------------------------

						qDebug() << "RotMatMotion2Base:" << endl << 
						RotMatMotion2Base.at<double>(0, 0) << RotMatMotion2Base.at<double>(0, 1) << RotMatMotion2Base.at<double>(0, 2) << endl <<
						RotMatMotion2Base.at<double>(1, 0) << RotMatMotion2Base.at<double>(1, 1) << RotMatMotion2Base.at<double>(1, 2) << endl <<
						RotMatMotion2Base.at<double>(2, 0) << RotMatMotion2Base.at<double>(2, 1) << RotMatMotion2Base.at<double>(2, 2) << endl;

						qDebug() << "_targetVec :" << _targetVec[0] << _targetVec[1] << _targetVec[2] << endl;
						qDebug() << "_targetVec_size" << RAD_TO_DEG(cv::norm(_targetVec)) << endl;

						qDebug() << "vCompensation :" << vCompensation[0] << vCompensation[1] << vCompensation[2] << endl;
						qDebug() << "vCompensation_size" << RAD_TO_DEG(cv::norm(vCompensation)) << endl;

						qDebug() << "targetVec :" << targetVec[0] << targetVec[1] << targetVec[2] << endl;
						qDebug() << "targetVec_size" << RAD_TO_DEG(cv::norm(targetVec)) << endl;

						//-------------------------------------------- 用来调试用的这个---------------------------------------------------

						//与 0 0 1 的夹角
						auto rrot = CalculateRotationVector(targetVec, { 0,0,1 });
						qDebug() << "targetVec and {0,0,1} 's rotVec_size :" << RAD_TO_DEG(cv::norm(rrot)) << endl;;

						//------------------------------------------------------------------------------------------------------------

						qDebug() << "curRotationVector :" << curRotationVector[0] << curRotationVector[1] << curRotationVector[2] << endl;
						qDebug() << "curRotationVector_size" << RAD_TO_DEG(cv::norm(curRotationVector)) << endl;

						qDebug() << "vCompensation_in_base :" << vCompensation_in_base[0] << vCompensation_in_base[1] << vCompensation_in_base[2] << endl;
						qDebug() << "vCompensation_in_base_size" << RAD_TO_DEG(cv::norm(vCompensation_in_base)) << endl;

						//curRotationVector_size = 5 * Δθ < cv::norm(curRotationVector) ? Δθ : 0;
						//ResizeVector(curRotationVector, curRotationVector_size);
					}
					else {
						curRotationVector = cv::Vec3d{ 0,0,0 };
						curRotationVector_size = 0;
					}
				}

			}

			if (normalVector_array.size() == norm_count) {

				curRotationVector_array.push_back(curRotationVector);

				normalVector_array.clear();
			}

			if (curRotationVector_array.size() == count) { //收集十组矢量就可以调整姿态了

				expectRotationVector = averageRotationVector(curRotationVector_array);

				qDebug() << "expectRotationVector before resize :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
				qDebug() << "expectRotationVector_size before resize" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;

				ResizeVector(expectRotationVector, 5 * Δθ < cv::norm(expectRotationVector) ? Δθ : 0); // ？

				curRotationVector_array.clear();

				qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
				qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
			}


			//-----------------------------保证开始到此处的时间间隔为多少才开始调整姿态(目的是为了越过粘滞阶段)----------------------------

			long long cur = start.time * 1000 + start.millitm;

			if (cur - t > 6000) { 
				newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] ,expectRotationVector[1] ,expectRotationVector[2] });
			}
			else newPose = pose;
			

			//更新这次期望位置与期望姿态
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
			vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
			pose[0] += V * direction_vector[0] * Ts;
			pose[1] += V * direction_vector[1] * Ts;
			pose[2] += V * direction_vector[2] * Ts;

			pose[3] = newPose[3];
			pose[4] = newPose[4];
			pose[5] = newPose[5];

			//-------------------获取移动坐标系下的力------------------------

			auto cur_pose = rtde_r->getActualTCPPose();

			sixth_axis = getSixthAxisFromBasePose(cur_pose);

			end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

			force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);


			//-----------------------------------------------------------

			//----------------计算基座标系到移动坐标系的旋转矩阵----------------

			computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis,rtde_r);

			RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

			RotMatMotion2Base = RotMatBase2Motion.t();

			cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

			//-------------------------------------------------------------

			Fe = force[2];//法向环境力
			φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();



		////我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		//Fd = -1 * Fd;//因为这里要转换为在TCP坐标系下，所以期望力10N要变成-10N

		//vector<double> normalTCP_move{ 0,0,1 };//表示一个在TCP坐标系下法向的移动增量

		////Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		//double Xc_ddt = 0;
		//double Xc_dt = 0;
		//double Xc = 0;
		//double Xe_ddt = 0;
		//double Xe_dt = 0;
		//double Xe = 0;

		//double φt = 0;

		//int count = 10; //收集多少个Ts周期对应的旋转矢量进行一次合算

		//cv::Vec3d expectRotationVector{ 0,0,0 };//合算出来的期望旋转向量

		//// 每个控制周期的轴角旋转速度 （绕某个固定轴旋转的速度）
		//double Δθ = 40 * Ts * PI / 180 * count; //最前面的数字是每秒的度数（粗略）

		//cv::Vec3d curRotationVector{ 0,0,0 }; //旋转矢量
		//double curRotationVector_size = 0;//旋转矢量的大小

		//vector<cv::Vec3d> curRotationVector_array;

		////获取当前位置信息
		//vector<double> pose = rtde_r->getActualTCPPose();

		////初始化力信息 
		//vector<double> force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);
		////获取我的公式下的传感器坐标系的力信息以及 基座标到该坐标系的旋转矩阵
		//auto myFormulaForce_and_Base2Formula = getMyFormulaCDSForceAndBase2Formula(rtde_r);

		//auto myFormulaForce = myFormulaForce_and_Base2Formula.first;

		//auto Base2Formula = myFormulaForce_and_Base2Formula.second;

		////初始化参数
		////皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前TCP位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		//Xe_ddt = 0;
		//Xe_dt = 0;
		//Xe = 0;
		//Xc = Xe;

		////初始化期望位置加速度
		//Xc_ddt = 0;

		////阻抗控制本体
		//auto temp_Xc = Xc;//上一时刻的Xc
		//Xc_dt = Xc_dt + Xc_ddt * Ts;
		//Xc = Xc + Xc_dt * Ts;
		//normalTCP_move[2] = Xc - temp_Xc;
		//auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下
		//
		////根据目标向量计算移动角度
		//if (force[2] > Fd / 2) { // 下降状态且力控尚未稳定,则不转动

		//	curRotationVector = cv::Vec3d{ 0,0,0 };
		//	curRotationVector_size = 0;
		//}
		//else {
		//	cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//	cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, R, LengthOfSensor2MassageHeadCentre);
		//	cv::Mat Formula2Base = Base2Formula.inv();
		//	cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//在基座标系下的目标向量表示
		//	if (cv::norm(targetVec) != 0) {
		//		curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//		//curRotationVector_size = 5 * Δθ < cv::norm(curRotationVector) ? Δθ : 0;
		//		//ResizeVector(curRotationVector, curRotationVector_size);
		//	}
		//	else {
		//		curRotationVector = cv::Vec3d{0,0,0};
		//		curRotationVector_size = 0;
		//	}
		//}
		//curRotationVector_array.push_back(curRotationVector);

		//std::vector<double> newPose = pose;

		//if (curRotationVector_array.size() == count) { //收集十组矢量就可以调整姿态了
		//	
		//	expectRotationVector = {0,0,0};
		//	for (int i = 0; i < curRotationVector_array.size(); i++) {
		//		
		//		expectRotationVector[0] += curRotationVector_array[i][0];
		//		expectRotationVector[1] += curRotationVector_array[i][1];
		//		expectRotationVector[2] += curRotationVector_array[i][2];
		//	}
		//	expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//	ResizeVector(expectRotationVector, 5 * Δθ < cv::norm(expectRotationVector) ? Δθ : 0);

		//	curRotationVector_array.clear();

		//	qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//	qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
		//}

		//newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		////期望位置与期望姿态
		//pose[0] += moveDistance[0];
		//pose[1] += moveDistance[1];
		//pose[2] += moveDistance[2];

		//// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		//vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//pose[0] += V * direction_vector[0] * Ts;
		//pose[1] += V * direction_vector[1] * Ts;
		//pose[2] += V * direction_vector[2] * Ts;

		//pose[3] = newPose[3];
		//pose[4] = newPose[4];
		//pose[5] = newPose[5];

		////下一次期望加速度 这里Fd与Fe都是在TCP方向上的
		//force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);//转化为TCP坐标下的力

		////获取我的公式下的传感器坐标系的力信息以及 基座标到该坐标系的旋转矩阵
		//auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//myFormulaForce = temp.first;
		//Base2Formula = temp.second;

		//double Fe = force[2];//法向环境力
		//φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		//Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//startFlag = true;//开启！

		//timeb start;
		//ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		//while (startFlag)
		//{
		//	if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
		//		break;
		//	//qDebug() << "Xc_ddt: " << Xc_ddt;
		//	//qDebug() << "Xc_dt" << Xc_dt;
		//	//qDebug() << "Xc" << Xc;
		//	//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

		//	//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
		//	//另外下降过程不用考虑姿态
		//	if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
		//	{
		//		qDebug() << " protect";
		//		auto temp_Xc = Xc;
		//		Xc += downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

		//		normalTCP_move[2] = Xc - temp_Xc;
		//		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

		//		//期望位置
		//		pose[0] += moveDistance[0];
		//		pose[1] += moveDistance[1];
		//		pose[2] += moveDistance[2];

		//		//严格控制每个servoL指令之间的间隔为Ts！
		//		timeb end;
		//		ftime(&end);//获取毫秒
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.03, 1000);
		//		ftime(&start);//获取毫秒

		//		continue;
		//	}
		//	else
		//	{
		//		//严格控制每个servoL指令之间的间隔为Ts！
		//		timeb end;
		//		ftime(&end);//获取毫秒
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
		//		ftime(&start);//获取毫秒
		//	}

		//	//更新数据
		//	pose = rtde_r->getActualTCPPose();

		//	//阻抗控制算法本体
		//	auto temp_Xc = Xc;//上一时刻的Xc
		//	Xc_dt = Xc_dt + Xc_ddt * Ts;
		//	Xc = Xc + Xc_dt * Ts;
		//	normalTCP_move[2] = Xc - temp_Xc;
		//	auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出要移动的距离
		//	
		//	//根据目标向量计算移动角度
		//	if (force[2] > Fd / 2) { // 下降状态且力控尚未稳定,则不转动

		//		curRotationVector = cv::Vec3d{ 0,0,0 };
		//		curRotationVector_size = 0;
		//	}
		//	else {
		//		cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//		cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, R, LengthOfSensor2MassageHeadCentre);
		//		cv::Mat Formula2Base = Base2Formula.inv();
		//		cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//在基座标系下的目标向量表示
		//		if (cv::norm(targetVec) != 0) {
		//			curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//			//curRotationVector_size = 5 * Δθ < cv::norm(curRotationVector) ? Δθ : 0;
		//			//ResizeVector(curRotationVector, curRotationVector_size);
		//		}
		//		else {
		//			curRotationVector = cv::Vec3d{ 0,0,0 };
		//			curRotationVector_size = 0;
		//		}
		//	}

		//	curRotationVector_array.push_back(curRotationVector);

		//	if (curRotationVector_array.size() == count) { //收集十组矢量就可以调整姿态了

		//		expectRotationVector = { 0,0,0 };
		//		for (int i = 0; i < curRotationVector_array.size(); i++) {

		//			expectRotationVector[0] += curRotationVector_array[i][0];
		//			expectRotationVector[1] += curRotationVector_array[i][1];
		//			expectRotationVector[2] += curRotationVector_array[i][2];
		//		}
		//		expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//		ResizeVector(expectRotationVector, 5 * Δθ < cv::norm(expectRotationVector) ? Δθ : 0);
		//		
		//		curRotationVector_array.clear();

		//		qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//		qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;

		//	}

		//	newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		//	//更新这次期望位置与期望姿态
		//	pose[0] += moveDistance[0];
		//	pose[1] += moveDistance[1];
		//	pose[2] += moveDistance[2];

		//	// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		//	vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//	pose[0] += V * direction_vector[0] * Ts;
		//	pose[1] += V * direction_vector[1] * Ts;
		//	pose[2] += V * direction_vector[2] * Ts;

		//	pose[3] = newPose[3];
		//	pose[4] = newPose[4];
		//	pose[5] = newPose[5];

		//	//更新下一次的补偿位置加速度
		//	force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//转化为TCP坐标下的力
		//	Fe = force[2];//法向环境力

		//	//获取我的公式下的传感器坐标系的力信息以及 基座标到该坐标系的旋转矩阵
		//	auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//	myFormulaForce = temp.first;
		//	Base2Formula = temp.second;

		//	φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		//	Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//}
		//rtde_c->servoStop();



		////我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		//Fd = -1 * Fd;//因为这里要转换为在TCP坐标系下，所以期望力10N要变成-10N

		//vector<double> normalTCP_move{ 0,0,1 };//表示一个在TCP坐标系下法向的移动增量

		////Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		//double Xc_ddt = 0;
		//double Xc_dt = 0;
		//double Xc = 0;
		//double Xe_ddt = 0;
		//double Xe_dt = 0;
		//double Xe = 0;

		//double φt = 0;

		//int count = 10; //收集多少个Ts周期对应的旋转矢量进行一次合算

		//cv::Vec3d expectRotationVector{ 0,0,0 };//合算出来的期望旋转向量

		//// 每个控制周期的轴角旋转速度 （绕某个固定轴旋转的速度）
		//double Δθ = 40 * Ts * PI / 180 * count; //最前面的数字是每秒的度数（粗略）

		//cv::Vec3d curRotationVector{ 0,0,0 }; //旋转矢量
		//double curRotationVector_size = 0;//旋转矢量的大小

		//vector<cv::Vec3d> curRotationVector_array;

		////获取当前位置信息
		//vector<double> pose = rtde_r->getActualTCPPose();

		////初始化力信息 
		//vector<double> force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);
		////获取我的公式下的传感器坐标系的力信息以及 基座标到该坐标系的旋转矩阵
		//auto [myFormulaForce, Base2Formula] = getMyFormulaCDSForceAndBase2Formula(rtde_r);

		////初始化参数
		////皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前TCP位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		//Xe_ddt = 0;
		//Xe_dt = 0;
		//Xe = 0;
		//Xc = Xe;

		////初始化期望位置加速度
		//Xc_ddt = 0;

		////阻抗控制本体
		//auto temp_Xc = Xc;//上一时刻的Xc
		//Xc_dt = Xc_dt + Xc_ddt * Ts;
		//Xc = Xc + Xc_dt * Ts;
		//normalTCP_move[2] = Xc - temp_Xc;
		//auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下
		//
		////根据目标向量计算移动角度
		//if (force[2] > Fd / 2) { // 下降状态且力控尚未稳定,则不转动

		//	curRotationVector = cv::Vec3d{ 0,0,0 };
		//	curRotationVector_size = 0;
		//}
		//else {
		//	cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//	cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, K, R, LengthOfSensor2MassageHeadCentre);
		//	cv::Mat Formula2Base = Base2Formula.inv();
		//	cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//在基座标系下的目标向量表示
		//	if (cv::norm(targetVec) != 0) {
		//		curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//		//curRotationVector_size = 5 * Δθ < cv::norm(curRotationVector) ? Δθ : 0;
		//		//ResizeVector(curRotationVector, curRotationVector_size);
		//	}
		//	else {
		//		curRotationVector = cv::Vec3d{0,0,0};
		//		curRotationVector_size = 0;
		//	}
		//}
		//curRotationVector_array.push_back(curRotationVector);

		//std::vector<double> newPose = pose;

		//if (curRotationVector_array.size() == count) { //收集十组矢量就可以调整姿态了
		//	
		//	expectRotationVector = {0,0,0};
		//	for (int i = 0; i < curRotationVector_array.size(); i++) {
		//		
		//		expectRotationVector[0] += curRotationVector_array[i][0];
		//		expectRotationVector[1] += curRotationVector_array[i][1];
		//		expectRotationVector[2] += curRotationVector_array[i][2];
		//	}
		//	expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//	ResizeVector(expectRotationVector, 5 * Δθ < cv::norm(expectRotationVector) ? Δθ : 0);

		//	curRotationVector_array.clear();

		//	qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//	qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
		//}

		//newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		////期望位置与期望姿态
		//pose[0] += moveDistance[0];
		//pose[1] += moveDistance[1];
		//pose[2] += moveDistance[2];

		//// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		//vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//pose[0] += V * direction_vector[0] * Ts;
		//pose[1] += V * direction_vector[1] * Ts;
		//pose[2] += V * direction_vector[2] * Ts;

		//pose[3] = newPose[3];
		//pose[4] = newPose[4];
		//pose[5] = newPose[5];

		////下一次期望加速度 这里Fd与Fe都是在TCP方向上的
		//force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);//转化为TCP坐标下的力

		////获取我的公式下的传感器坐标系的力信息以及 基座标到该坐标系的旋转矩阵
		//auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//myFormulaForce = temp.first;
		//Base2Formula = temp.second;

		//double Fe = force[2];//法向环境力
		//φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		//Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//startFlag = true;//开启！

		//timeb start;
		//ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		//while (startFlag)
		//{
		//	if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
		//		break;
		//	//qDebug() << "Xc_ddt: " << Xc_ddt;
		//	//qDebug() << "Xc_dt" << Xc_dt;
		//	//qDebug() << "Xc" << Xc;
		//	//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

		//	//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
		//	//另外下降过程不用考虑姿态
		//	if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
		//	{
		//		qDebug() << " protect";
		//		auto temp_Xc = Xc;
		//		Xc += downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

		//		normalTCP_move[2] = Xc - temp_Xc;
		//		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

		//		//期望位置
		//		pose[0] += moveDistance[0];
		//		pose[1] += moveDistance[1];
		//		pose[2] += moveDistance[2];

		//		//严格控制每个servoL指令之间的间隔为Ts！
		//		timeb end;
		//		ftime(&end);//获取毫秒
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.03, 1000);
		//		ftime(&start);//获取毫秒

		//		continue;
		//	}
		//	else
		//	{
		//		//严格控制每个servoL指令之间的间隔为Ts！
		//		timeb end;
		//		ftime(&end);//获取毫秒
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
		//		ftime(&start);//获取毫秒
		//	}

		//	//更新数据
		//	pose = rtde_r->getActualTCPPose();

		//	//阻抗控制算法本体
		//	auto temp_Xc = Xc;//上一时刻的Xc
		//	Xc_dt = Xc_dt + Xc_ddt * Ts;
		//	Xc = Xc + Xc_dt * Ts;
		//	normalTCP_move[2] = Xc - temp_Xc;
		//	auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出要移动的距离
		//	
		//	//根据目标向量计算移动角度
		//	if (force[2] > Fd / 2) { // 下降状态且力控尚未稳定,则不转动

		//		curRotationVector = cv::Vec3d{ 0,0,0 };
		//		curRotationVector_size = 0;
		//	}
		//	else {
		//		cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//		cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, K, R, LengthOfSensor2MassageHeadCentre);
		//		cv::Mat Formula2Base = Base2Formula.inv();
		//		cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//在基座标系下的目标向量表示
		//		if (cv::norm(targetVec) != 0) {
		//			curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//			//curRotationVector_size = 5 * Δθ < cv::norm(curRotationVector) ? Δθ : 0;
		//			//ResizeVector(curRotationVector, curRotationVector_size);
		//		}
		//		else {
		//			curRotationVector = cv::Vec3d{ 0,0,0 };
		//			curRotationVector_size = 0;
		//		}
		//	}

		//	curRotationVector_array.push_back(curRotationVector);

		//	if (curRotationVector_array.size() == count) { //收集十组矢量就可以调整姿态了

		//		expectRotationVector = { 0,0,0 };
		//		for (int i = 0; i < curRotationVector_array.size(); i++) {

		//			expectRotationVector[0] += curRotationVector_array[i][0];
		//			expectRotationVector[1] += curRotationVector_array[i][1];
		//			expectRotationVector[2] += curRotationVector_array[i][2];
		//		}
		//		expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//		ResizeVector(expectRotationVector, 5 * Δθ < cv::norm(expectRotationVector) ? Δθ : 0);
		//		
		//		curRotationVector_array.clear();

		//		qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//		qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;

		//	}

		//	newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		//	//更新这次期望位置与期望姿态
		//	pose[0] += moveDistance[0];
		//	pose[1] += moveDistance[1];
		//	pose[2] += moveDistance[2];

		//	// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		//	vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//	pose[0] += V * direction_vector[0] * Ts;
		//	pose[1] += V * direction_vector[1] * Ts;
		//	pose[2] += V * direction_vector[2] * Ts;

		//	pose[3] = newPose[3];
		//	pose[4] = newPose[4];
		//	pose[5] = newPose[5];

		//	//更新下一次的补偿位置加速度
		//	force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//转化为TCP坐标下的力
		//	Fe = force[2];//法向环境力

		//	//获取我的公式下的传感器坐标系的力信息以及 基座标到该坐标系的旋转矩阵
		//	auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//	myFormulaForce = temp.first;
		//	Base2Formula = temp.second;

		//	φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		//	Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//}
		//rtde_c->servoStop();



		/*
		//我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		Fd = -1 * Fd;//因为这里要转换为在TCP坐标系下，所以期望力10N要变成-10N

		vector<double> normalTCP_move{ 0,0,1 };//表示一个在TCP坐标系下法向的移动增量

		//Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		//对tcp坐标系下 x轴和y轴的调整转角,以初始的TCP坐标的副本的xitaX 和 xitaY为0（即将一开始的TCP坐标系求一个副本，姿态不随时间改变，把该TCP的初始调整转角X和调整转角Y设为0 ）
		double θx_ddt = 0;
		double θx_dt = 0;
		double θx = 0;
		double θy_ddt = 0;
		double θy_dt = 0;
		double θy = 0;

		double φt = 0;
		double φt_θx = 0; //为角度也提供一个类似位置的阻抗控制一样的控制
		double φt_θy = 0;

		//获取当前位置信息
		vector<double> pose = rtde_r->getActualTCPPose();

		//初始化参数
		//皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前TCP位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;
		//初始化转角
		θx_dt = 0;
		θx = 0;
		θy_dt = 0;
		θy = 0;

		//初始化期望位置加速度与姿态加速度
		Xc_ddt = 0;
		θy_ddt = 0;
		θx_ddt = 0;

		//阻抗控制本体
		auto temp_Xc = Xc;//上一时刻的Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, {pose[3],pose[4],pose[5]}); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下
		θx_dt = θx_dt + θx_ddt * Ts;
		auto temp_θx = θx;//上一时刻的θx
		θx = θx + θx_dt * Ts;
		θy_dt = θy_dt + θy_ddt * Ts;
		auto temp_θy = θy;//上一时刻的θy
		θy = θy + θy_dt * Ts;
		auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(θx - temp_θx, θy - temp_θy);

		//期望位置与期望姿态
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newAngle[0];
		pose[4] = newAngle[1];
		pose[5] = newAngle[2];

		//下一次期望加速度 这里Fd与Fe都是在TCP方向上的
		vector<double> force = rtde_r->getActualTCPForce();
		vector<double> motion_coordinate_force = getMotionCoordinateSystemForce(rtde_r);//获取移动坐标系下的力信息

		force = world_2tcp_force(force, pose);//转化为TCP坐标下的力
		double Fe = force[2];//法向环境力
		φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		double Fe_axisX = force[0];
		double Fe_axisY = force[1];
		φt_θx = φt_θx + sigma * (-Fe_axisY) / B_θ;
		φt_θy = φt_θy + sigma * (Fe_axisX) / B_θ;
		θx_ddt = 1 / M_θ * (-1 * Fe_axisY - ((B_θ * θx_dt) + (B_θ * φt_θx - sigma * -1 * Fe_axisY)) - K_θ * θx);
		θy_ddt = 1 / M_θ * (Fe_axisX - ((B_θ * θy_dt) + (B_θ * φt_θy - sigma * Fe_axisX)) - K_θ * θy);
		
		startFlag = true;//开启！

		timeb start;
		ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		while (startFlag)
		{
			if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];
			
			//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
			//另外下降过程不用考虑姿态
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

				//期望位置
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//获取毫秒

				continue;
			}
			else
			{
				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//获取毫秒
			}

			//更新数据
			pose = rtde_r->getActualTCPPose();

			//阻抗控制算法本体
			auto temp_Xc = Xc;//上一时刻的Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //根据阻抗控制得出要移动的距离
			θx_dt = θx_dt + θx_ddt * Ts;
			auto temp_θx = θx;//上一时刻的θx
			θx = θx + θx_dt * Ts;
			θy_dt = θy_dt + θy_ddt * Ts;
			auto temp_θy = θy;//上一时刻的θy
			θy = θy + θy_dt * Ts;
			auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(θx - temp_θx, θy - temp_θy);

			//更新这次期望位置与期望姿态
			pose[0] += moveDistance[0]; 
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
			vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
			pose[0] += V * direction_vector[0] * Ts;
			pose[1] += V * direction_vector[1] * Ts;
			pose[2] += V * direction_vector[2] * Ts;

			pose[3] = newAngle[0];
			pose[4] = newAngle[1];
			pose[5] = newAngle[2];

			//更新下一次的补偿位置加速度
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//转化为TCP坐标下的力
			Fe = force[2];//法向环境力
			φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));
			Fe_axisX = force[0];
			Fe_axisY = force[1];
			φt_θx = φt_θx + sigma * (-Fe_axisY) / B_θ;
			φt_θy = φt_θy + sigma * (Fe_axisX) / B_θ;
			θx_ddt = 1 / M_θ * (-1 * Fe_axisY - ((B_θ * θx_dt) + (B_θ * φt_θx - sigma * -1 * Fe_axisY)) - K_θ * θx);
			θy_ddt = 1 / M_θ * (Fe_axisX - ((B_θ * θy_dt) + (B_θ * φt_θy - sigma * Fe_axisX)) - K_θ * θy);
			qDebug() << "  θx_ddt :" << θx_ddt<< "  θx_dt :" << θx_dt << "  θx :" << θx << "  Fe_axisY :" << Fe_axisY <<"  θx - temp_θx" << θx - temp_θx;
			qDebug() << "  θy_ddt :" << θy_ddt << "  θy_dt :" << θy_dt << "  θy :" << θy << "  Fe_axisX :" << Fe_axisX << "  θy - temp_θy" << θy - temp_θy;

		}
		rtde_c->servoStop();
		*/
	}
	else if (mode == 2) {
		
		// 获取补偿向量专用 

		//我的离散阻抗思路是：开启阻抗控制器，开启的前一时刻Xc_ddt = 0，然后根据Xc_ddt更新 Xc_dt与Xc，然后再更新下一次Xc_ddt，然后伺服移动当前次的Xc

		vector<double> normalMotion_move{ 0,0,1 };//表示一个在移动坐标系下法向的移动增量

		//Xc是输给机器人的指令位置，Xe是期望环境的位置，ddt，dt，分别对应加速度速度
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double φt = 0;

		int count = 50; //收集多少个Ts周期对应的法向量进行一次合算

		vector<cv::Vec3d> vCompensation_array;

		vector<double> pose = rtde_r->getActualTCPPose();

		//-------------------获取移动坐标系下的力------------------------

		vector<double> cur_pose = rtde_r->getActualTCPPose();

		auto sixth_axis = getSixthAxisFromBasePose(cur_pose);

		Vec3d end_effector_direction{ _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		auto force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

		//------------------------------------------------------------

		//----------------计算基座标系到移动坐标系的旋转矩阵----------------

		Vec3d x_axis, y_axis, z_axis;

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis, rtde_r);

		cv::Mat RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		cv::Mat RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Vec3d RotVecBase2Motion;

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//------------------------------------------------------------

		//初始化参数
		//皮肤平面，则Xe_ddt=0;Xe_dt=0;  以当前移动坐标系位置 建立一个固定坐标系，Xe为原点0  Xe作为Xc的初试点 Xc为在该固定坐标系下的输给机器人指令的位置
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//初始化期望位置加速度
		Xc_ddt = 0;

		//阻抗控制本体
		auto temp_Xc = Xc;//上一时刻的Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalMotion_move[2] = Xc - temp_Xc;
		cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //根据阻抗控制得出在TCP坐标下要移动的距离，并将该距离增量转化为世界坐标系下

		//根据目标向量计算移动角度
		if (force[2] >= Fd / 2) { // 下降状态且力控尚未稳定,则不计算补偿法向量

			cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
			cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //目标向量在移动坐标系下的表示
			vCompensation_array.push_back(_targetVec);

		}

		if (vCompensation_array.size() == count) { // 收集50个补偿法向量就求个平均

			hasCompensation = true;

			cv::Vec3d sum(0.0, 0.0, 0.0);

			for (const auto& vec : vCompensation_array)
				sum += vec;

			vCompensation = sum / static_cast<double>(vCompensation_array.size());

			vCompensation_array.clear();

			cv::Vec3d vCompensation_in_base = rotateVecToTarget(vCompensation, RotMatMotion2Base);

			qDebug() << "vCompensation :" << vCompensation[0] << vCompensation[1] << vCompensation[2] << endl;
			qDebug() << "vCompensation_size : " << RAD_TO_DEG(cv::norm(vCompensation)) << endl;

			qDebug() << "vCompensation_in_base :" << vCompensation_in_base[0] << vCompensation_in_base[1] << vCompensation_in_base[2] << endl;
			qDebug() << "vCompensation_in_base_size : " << RAD_TO_DEG(cv::norm(vCompensation_in_base)) << endl;

		}

		//期望位置
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;


		//-------------------获取移动坐标系下的力------------------------

		cur_pose = rtde_r->getActualTCPPose();

		sixth_axis = getSixthAxisFromBasePose(cur_pose);

		end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

		//-----------------------------------------------------------


		//----------------计算基座标系到移动坐标系的旋转矩阵----------------

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis, rtde_r);

		RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//-------------------------------------------------------------

		double Fe = force[2];//法向环境力
		φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//开启！

		timeb start;
		ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
		while (startFlag)
		{
			if (!startFlag)//再三确认，防止其他线程关闭了阻抗控制
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//防止阻抗控制在下降过程不断累计Xc_dt 导致最后力控超调量过高，这里添加一个阈值如0.1m/s ，一旦速度超过0.1m/s 则以固定速度0.1m/s来更新位置，并不再更新Xc_ddt与Xc_dt，直到接触为止（给了0.2N的接触阈值）
			//另外下降过程不用考虑姿态
			if (Xc_dt < -downLimit && getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r)[2] < Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc -= downLimit * Ts; //例如按照0.1m/s 来下降计算，0.002s内走的距离就是0.0002;

				normalMotion_move[2] = Xc - temp_Xc;
				cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //根据阻抗控制得出在移动坐标系下要移动的距离，并将该距离增量转化为世界坐标系下

				//期望位置
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				//qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//获取毫秒

				continue;
			}
			else
			{
				//严格控制每个servoL指令之间的间隔为Ts！
				timeb end;
				ftime(&end);//获取毫秒
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				//qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//获取毫秒
			}

			//更新数据
			pose = rtde_r->getActualTCPPose();

			//阻抗控制算法本体
			auto temp_Xc = Xc;//上一时刻的Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalMotion_move[2] = Xc - temp_Xc;
			cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //根据阻抗控制得出要移动的距离

			//qDebug() << "Force :" << force[2] << endl;
			//qDebug() << "moveDistance :" << moveDistance[0] << moveDistance[1] << moveDistance[2] << endl;
			//qDebug() << "Xc :" << Xc << " Xc_dt:" << Xc_dt << " Xc_ddt:" << Xc_ddt << endl;
		
			if (force[2] >= Fd / 2) { // 下降状态且力控尚未稳定,则不计算补偿法向量

				cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
				cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //目标向量在移动坐标系下的表示
				vCompensation_array.push_back(_targetVec);
			}

			if (vCompensation_array.size() == count) { // 收集50个补偿法向量就求个平均

				hasCompensation = true;

				cv::Vec3d sum(0.0, 0.0, 0.0);

				for (const auto& vec : vCompensation_array)
					sum += vec;

				vCompensation = sum / static_cast<double>(vCompensation_array.size());

				vCompensation_array.clear();

				cv::Vec3d vCompensation_in_base = rotateVecToTarget(vCompensation, RotMatMotion2Base);

				qDebug() << "vCompensation :" << vCompensation[0] << vCompensation[1] << vCompensation[2] << endl;
				qDebug() << "vCompensation_size : " << RAD_TO_DEG(cv::norm(vCompensation)) << endl;

				qDebug() << "vCompensation_in_base :" << vCompensation_in_base[0] << vCompensation_in_base[1] << vCompensation_in_base[2] << endl;
				qDebug() << "vCompensation_in_base_size : " << RAD_TO_DEG(cv::norm(vCompensation_in_base)) << endl;
			}

			//qDebug() << "vCompensation_in_base_size : " << RAD_TO_DEG(cv::norm(vCompensation_in_base)) << endl;

			//更新这次期望位置
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			// 根据控制周期计算出移动方向应该移动的距离，事实上并不太准确，机器人的位置闭环不能在固定时间内到达指定位置，还是有误
			vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
			pose[0] += V * direction_vector[0] * Ts;
			pose[1] += V * direction_vector[1] * Ts;
			pose[2] += V * direction_vector[2] * Ts;

			//-------------------获取移动坐标系下的力------------------------

			auto cur_pose = rtde_r->getActualTCPPose();

			sixth_axis = getSixthAxisFromBasePose(cur_pose);

			end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

			force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

			//-----------------------------------------------------------


			//----------------计算基座标系到移动坐标系的旋转矩阵----------------

			computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis, rtde_r);

			RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

			RotMatMotion2Base = RotMatBase2Motion.t();

			cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

			//-------------------------------------------------------------

			Fe = force[2];//法向环境力
			φt = φt + sigma * (Fd - Fe) / B;//自适应控制B 变阻尼
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * φt + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();

	}

}


//------------------------------------------------辅助功能函数-------------------------------------------------

void Dy::Impedance_control::Safe_servoL(const std::vector<double>& pose, double speed, double acceleration, double time, double lookahead_time, double gain) {

	std::vector<double> pose_now = rtde_r->getActualTCPPose();
	double pose_count = pow((pow(pose_now[0] - pose[0], 2) + pow(pose_now[1] - pose[1], 2) + pow(pose_now[2] - pose[2], 2)), 0.5)*  1000;
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
	//qDebug() << "θ:" << θ << '\t' << "pose_count:" << pose_count << endl;
	if (abs(θ) < 10 && pose_count <15 ) {
		rtde_c->servoL(pose, speed, acceleration, time, lookahead_time, gain);
	}
	else {
		abort();
	}
}

vector<double> Dy::Impedance_control::world_2tcp_force(vector<double> world_force, vector<double>in_pose)
{
	cv::Mat_<double> force = (cv::Mat_<double>(1, 3) << world_force[0], world_force[1], world_force[2]);//
	cv::Mat_<double> m_force = (cv::Mat_<double>(1, 3) << world_force[3], world_force[4], world_force[5]);//

	cv::Mat_<double> r_l = (cv::Mat_<double>(3, 1) << in_pose[3], in_pose[4], in_pose[5]);//旋转向量
	cv::Mat  R_M;
	cv::Rodrigues(r_l, R_M);

	cv::Mat tcp_force_xyz_mat = force * R_M;
	cv::Mat tcp_force_mxyz_mat = m_force * R_M;

	vector<double> tcp_force;
	tcp_force.push_back(tcp_force_xyz_mat.at<double>(0, 0));
	tcp_force.push_back(tcp_force_xyz_mat.at<double>(0, 1));
	tcp_force.push_back(tcp_force_xyz_mat.at<double>(0, 2));
	tcp_force.push_back(tcp_force_mxyz_mat.at<double>(0, 0));
	tcp_force.push_back(tcp_force_mxyz_mat.at<double>(0, 1));
	tcp_force.push_back(tcp_force_mxyz_mat.at<double>(0, 2));

	return tcp_force;
}

vector<double> Dy::Impedance_control::tran_P_from_tcp_2world(vector<double> P, vector<double> rxryrz) {
	
	cv::Vec3d moveP{P[0],P[1],P[2]};
	cv::Vec3d kxita{ rxryrz[0],rxryrz[1],rxryrz[2] };
	cv::Mat rotation_matrix;
	cv::Rodrigues(kxita, rotation_matrix);
	cv::Mat _end_direction_vector = rotation_matrix * moveP;
	cv::Vec3d end_direction_vector{ _end_direction_vector.at<double>(0,0),_end_direction_vector.at<double>(1,0) ,_end_direction_vector.at<double>(2,0) }; //末端在世界坐标系表示的列向量

	//double xita = pow(pow(rxryrz[0], 2)+pow(rxryrz[1], 2)+ pow(rxryrz[2], 2), 0.5);
	//cv::Mat r = (cv::Mat_<double>(3, 1) << rxryrz[0] / xita, rxryrz[1] / xita, rxryrz[2] / xita); //单位向量
	//cv::Mat p = (cv::Mat_<double>(3, 1) << P[0], P[1], P[2]);//初始p在tcp下的坐标 
	//cv::Mat new_P = cos(xita) * p + (1 - cos(xita)) * p.dot(r) + sin(xita) * r.cross(p);

	return { end_direction_vector[0],end_direction_vector[1],end_direction_vector[2]};
}

vector<double> Dy::Impedance_control::get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(vector<double> _moveUnitDirInWorld, vector<double> rxryrz) {

	cv::Vec3d worldZdir{ 0,0,1 };
	cv::Vec3d kxita{rxryrz[0],rxryrz[1],rxryrz[2]};
	cv::Mat rotation_matrix;
	cv::Rodrigues(kxita,rotation_matrix);
	cv::Mat _end_direction_vector = rotation_matrix * worldZdir;
	cv::Vec3d end_direction_vector{_end_direction_vector.at<double>(0,0),_end_direction_vector.at<double>(1,0) ,_end_direction_vector.at<double>(2,0)}; //末端在世界坐标系表示的列向量

	//在世界坐标系中移动的单位向量（与力控正交的位置控制所指向的方向在世界坐标系上xoy面的投影，Z轴通常为0，因为是根据力控给的一个大概的移动方向）
	cv::Vec3d moveUnitDirInWorld{_moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2]};

	if (cv::norm(moveUnitDirInWorld) < 1e-8) return { 0,0,0 }; //特判：如果_moveUnitDirInWorld为0向量则直接返回0即可

	double altered_mould_length = cv::norm(moveUnitDirInWorld);//单位化
	moveUnitDirInWorld[0] /= altered_mould_length;
	moveUnitDirInWorld[1] /= altered_mould_length;
	moveUnitDirInWorld[2] /= altered_mould_length;

	//末端与给定方向的夹角，如果共线则求叉积会报错,因此如果共线则要修改一下移动单位向量
	double angle = acos(end_direction_vector.dot(moveUnitDirInWorld)); //弧度
	if (fabs(angle - PI) < 1e-7 || angle < 1e-7) { //angle 为 0或者 为 PI，说明共线

		//这时候将moveUnitDirInWorld调整一下，直接在其z轴+1就行，然后将其变回单位向量
		moveUnitDirInWorld[2] += 1;
		double altered_mould_length = cv::norm(moveUnitDirInWorld);
		moveUnitDirInWorld[0] /= altered_mould_length;
		moveUnitDirInWorld[1] /= altered_mould_length;
		moveUnitDirInWorld[2] /= altered_mould_length;
	}

	cv::Vec3d temp = moveUnitDirInWorld.cross(end_direction_vector);
	cv::Vec3d direction_vector = end_direction_vector.cross(temp);

	return { direction_vector[0],direction_vector[1] ,direction_vector[2] };
}

vector<double> Dy::Impedance_control::getTarget(const vector<double>& tangential, const vector<double>& P) {
	
	cv::Vec3d tangential_vec{ tangential[0],tangential[1],tangential[2] };
	cv::Vec3d nowPose_vec{ P[0],P[1],P[2] };
	cv::Vec3d Vertical_to_tangential_and_nowPose = tangential_vec.cross(nowPose_vec);
	cv::Vec3d target = Vertical_to_tangential_and_nowPose.cross(tangential_vec);
	return { target[0],target[1],target[2] };
}

vector<double> Dy::Impedance_control::get_RxRyRz_from_xitaX_and_xitaY(double θx, double θy) {
	
	vector<double> nowPose = rtde_r->getActualTCPPose();
	vector<double> rxryrz = { nowPose[3],nowPose[4],nowPose[5] };
	cv::Mat kxita = (cv::Mat_<double>(3, 1) << nowPose[3] , nowPose[4] , nowPose[5]);
	cv::Mat rotation_matrix; 
	cv::Rodrigues(kxita, rotation_matrix);
	cv::Mat Rx_θx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(θx), -sin(θx), 0, sin(θx), cos(θx));
	cv::Mat Ry_θy = (cv::Mat_<double>(3, 3) << cos(θy), 0, sin(θy), 0, 1, 0, -sin(θy), 0, cos(θy));
	rotation_matrix = rotation_matrix * Rx_θx * Ry_θy;
	cv::Vec3d new_rxryrz;
	cv::Rodrigues(rotation_matrix, new_rxryrz); //得到绕固定坐标系下的kxita（由于这样的相对坐标系下得到的绕固定坐标系下的旋转矩阵是等价的，所以这个矩阵也是绕固定坐标系下的矩阵）
	return { new_rxryrz[0],new_rxryrz[1],new_rxryrz[2] }; 
}

vector<double> Dy::Impedance_control::get_RxRyRz_from_xitaX_and_xitaY_and_xitaZ(double θx, double θy,double θz) {
	
	vector<double> nowPose = rtde_r->getActualTCPPose();
	vector<double> rxryrz = { nowPose[3],nowPose[4],nowPose[5] };
	cv::Mat kxita = (cv::Mat_<double>(3, 1) << nowPose[3], nowPose[4], nowPose[5]);
	cv::Mat rotation_matrix;
	cv::Rodrigues(kxita, rotation_matrix);
	cv::Mat Rx_θx = getRotationMatrix(RotationAxis::X_AXIS, θx);
	cv::Mat Ry_θy = getRotationMatrix(RotationAxis::Y_AXIS, θy);
	cv::Mat Rz_θz = getRotationMatrix(RotationAxis::Z_AXIS, θz);
	rotation_matrix = rotation_matrix * Rx_θx * Ry_θy * Rz_θz;
	cv::Vec3d new_rxryrz;
	cv::Rodrigues(rotation_matrix, new_rxryrz); //得到绕固定坐标系下的kxita（由于这样的相对坐标系下得到的绕固定坐标系下的旋转矩阵是等价的，所以这个矩阵也是绕固定坐标系下的矩阵）
	return { new_rxryrz[0],new_rxryrz[1],new_rxryrz[2] };
}

//cv::Vec3d Dy::Impedance_control::calculateSurfaceNormalVector(const std::vector<double>& force, double& k,double R) {
//	
//	double Fx = force[0], Fy = force[1], Fz = force[2];
//	double Mx = force[3], My = force[4], Mz = force[5];
//
//	double A = Fx * Fx + Fy * Fy + Fz * Fz;
//	double B = My * Fx - Mx * Fy;
//	double C = Mx * Mx + My * My - Fz * Fz * k * k * R * R;
//
//	double discriminant = B * B - A * C;
//	qDebug() << "A:" << A << endl;
//	qDebug() << "B:" << B << endl;
//	qDebug() << "C:" << C << endl;
//	qDebug() << "discriminant:" << discriminant << endl;
//	if (discriminant < 0) {
//		// 当要开方的底数为负数，则返回0向量
//		return cv::Vec3d(0, 0, 0);
//	}
//
//	double root_term = std::sqrt(discriminant);
//
//	double nx = (-My * A + Fx * (B - root_term)) / (A * Fz * k);
//	double ny = (Mx * A + Fy * (B - root_term)) / (A * Fz * k);
//	double nz = (B - root_term) / (A * k);
//
//	return cv::Vec3d(nx, ny, nz);
//}

cv::Vec3d Dy::Impedance_control::calculateSurfaceNormalVector(const std::vector<double>& force, double R, double l,bool isCompensation) {

	double Fx = force[0], Fy = force[1], Fz = force[2];
	double Mx = force[3], My = force[4], Mz = force[5];

	double A = Fx * Fx + Fy * Fy + Fz * Fz;
	double B = My * Fx - Mx * Fy + Fx * Fx * l + Fy * Fy * l;
	double C = Mx * Mx + My * My - Fz * Fz * R * R + Fx * Fx * l * l + Fy * Fy * l * l - 2 * Mx * Fy * l + 2 * My * Fx * l;

	double discriminant = B * B - A * C;

	qDebug() << " R : " << R << " " << "L : " << l << endl;
	qDebug() << "force : " << " " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5] << endl;
	qDebug() << "A:" << A << endl;
	qDebug() << "B:" << B << endl;
	qDebug() << "C:" << C << endl;
	qDebug() << "discriminant:" << discriminant << endl;

	if (discriminant < 0) {
		// 当要开方的底数为负数，则返回0向量
		return cv::Vec3d(0, 0, 0);
	}

	double root_term = std::sqrt(discriminant);

	double cos_alpha = (-B + root_term) / A * R;
	double sin_alpha = std::sqrt(1 - cos_alpha * cos_alpha);
	double cos_beta = (-My - Fx * R * cos_alpha - Fx * l) / (Fz * R * sin_alpha);
	double sin_beta = (Mx - Fy * R * cos_alpha - Fy * l) / (Fz * R * sin_alpha);

	//double nx = -(-My * A + Fx * B - Fx * root_term - Fx * l * A) / (A * Fz);
	//double ny = -(Mx * A + Fy * B - Fy * root_term - Fy * l * A) / (A * Fz);
	//double nz = -(B - root_term) / (A);

	//qDebug() << "row_v :" << v[0] << " " << v[1] << " " << v[2] << " " << endl;

	double d1 = -(My + Fx * l + Fx * R * cos_alpha) / Fz * R;
	double d2 = (Mx - Fy * l + Fy * R * cos_alpha) / Fz * R;
	double d3 = cos_alpha;

	cv::Vec3d v(d1, d2, d3);

	cv::Vec3d normalized_v;
	cv::normalize(v, normalized_v);



	if (isCompensation) {

		qDebug() << "start compensation" << endl;
		// k临近点搜索
		
		//【】这里继续增加！

		double Mx_p = 0, My_p = 0, Mz_p = 0; 

		double cosAlpha = (-B + std::sqrt(discriminant)) / (A * R);

		double alpha = acos(cosAlpha);

		double cosBeta = (-My - Fx * R * cosAlpha - Fx * l) / (Fz * R * sin(alpha));

		double beta = acos(cosBeta);

		int k = 5; // 假设 k = 5, 可以根据需要修改

		double delta_alpha = 0.01; // α的步长，根据具体情况进行调整

		double delta_beta = 0.01; // β的步长，根据具体情况进行调整

		std::vector<double> alphas(k), betas(k);
		double min_result = std::numeric_limits<double>::max(); // 用于存储最小结果

		// 计算所有可能的alpha_i和beta_j
		for (int i = 0; i < k; ++i) {
			alphas[i] = alpha + (i - (k + 1) / 2.0) * delta_alpha;
			betas[i] = beta + (i - (k + 1) / 2.0) * delta_beta;
		}

		// 计算r(alpha_i, beta_j)的最小值
		for (int i = 0; i < k; ++i) {
			for (int j = 0; j < k; ++j) {
				// 计算坐标转换后的坐标
				double x = Fy * R * cos(alphas[i]) + Fz * R * sin(alphas[i]) * sin(betas[j]) + Fy * l;
				double y = -Fz * R * sin(alphas[i]) * cos(betas[j]) - Fx * R * cos(alphas[i]) - Fx * l;
				double z = -Fx * R * sin(alphas[i]) * sin(betas[j]) + Fy * R * sin(alphas[i]) * cos(betas[j]);

				// 计算r(alpha_i, beta_j)
				double r = std::pow(Mx - x - Mx_p, 2) + std::pow(My - y - My_p, 2) + std::pow(Mz - z - Mz_p, 2);

				// 检查是否是最小值
				if (r < min_result) {
					min_result = r;
					alpha = alphas[i];
					beta = betas[j];
				}
			}
		}

		v = { -R * sin(alpha) * cos(beta) , -R * sin(alpha) * sin(beta) ,R * cos(alpha) };
		cv::normalize(v, normalized_v);

	}
	return normalized_v;

}

void Dy::testForIC() {

	//设置二阶系统的初值以及期望力和一些参数
	double M = 1;
	double B = 1;
	double K = 1;
	double Fd = 10;
	double Ts = 0.002;
	double Vx = 0.01;
	double tForservo = 0.002;
	int mode = 1;//模式选择
	double downLimit = 0.035;//下降速度限制，如果速度过快，则使用保护程序
	//--------------------------

	double threshold = 0.1;//训练结束的误差最小值
	double mostTimes = 10000;//训练的最大次数


	// 两个指针在QT里面的窗口类会有，即Dy_Control
	RTDEControlInterface* rtde_c = new RTDEControlInterface("192.168.55.101");
	RTDEReceiveInterface* rtde_r = new RTDEReceiveInterface("192.168.55.101");

	//创建实例化对象并使用对应成员函数
	Impedance_control ic(M,B,K,new BpNet(),rtde_c,rtde_r);
	ic.getInit(threshold, mostTimes);//根据数据初始化神经网络
	double Xr = ic.getXr(Fd);

	//开启阻抗控制
	ic.Normal_force_control(Fd, Xr, Ts,Vx,tForservo);	

	//开启基于当前位置的阻抗控制
	//ic.Normal_force_control_base_on_now(Fd, Ts, Vx, tForservo,mode,downLimit);
}