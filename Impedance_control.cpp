#include "Impedance_control.h"

cv::Vec3d Dy::Impedance_control::vCompensation = cv::Vec3d(0.0, 0.0, 0.0);
bool Dy::Impedance_control:: hasCompensation = false; // �Ƿ��Ѿ������˲���

void Dy::Impedance_control::Normal_force_control(double Fd, double Xr, double Ts, double Vx,double tForServo) {
	
	//��ʼ����������
	double ��X_v = 0;//λ��ƫ���һ�׵���
	double ��X_a = 0;//λ��ƫ��Ķ��׵���
	double ��X = 0;  //λ��ƫ��

	vector<double> pose = rtde_r->getActualTCPPose();
	vector<double> force = rtde_r->getActualTCPForce();

	double Fe = force[2];//���򻷾���

	//�迹���Ʊ���
	��X_a = (Fe - Fd - B * ��X_v - K * ��X) / M;
	��X_v = ��X_v + ��X_a * Ts;
	��X = ��X + ��X_v * Ts;
	double Xd;
	Xd = Xr + ��X;//����λ��

	pose[2] = Xd;

	//x�᷽���ϵ�����λ��
	double Xd_x = pose[0] + Vx * Ts;//������x�᷽���ϵ��ƶ�������Ts��Vx����Ӧ�ƶ�����

	pose[0] = Xd_x;

	startFlag = true;//������

	//100�ν���һ��������ѵ������,count���ڼ���
	int count = 0;


	while (startFlag)
	{
		timeb start;
		ftime(&start);//��ȡ����
		if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
			break;
		Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);//Safe_servoL�ǲ����������̵�
		//��������
		force = rtde_r->getActualTCPForce();
		Fe = force[2];//���򻷾���
		//�迹�����㷨����
		��X_a = (Fe - Fd - B * ��X_v - K * ��X) / M;
		��X_v = ��X_v + ��X_a * Ts;
		��X = ��X + ��X_v * Ts;
		//qDebug() << ��X<<endl;

		Xd = Xr + ��X;//����λ��
		pose[2] = Xd;
		Xd_x += Vx * Ts;//x�᷽���ϵ�����λ��
		pose[0] = Xd_x;
		timeb end;
		ftime(&end);//��ȡ����
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
			//�������������������Ȼ�ᱨ��
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
	this->bpnet->doTraining(trainGroup, 1, 10);//ƽ��������ȡ��1��ʵ��֤��Ҳ�ﲻ����ѵ��������100
	qDebug() << " train finish " <<"  " <<Xr<< endl;
	Xr = getXr(Fd);//����Fd�õ�������
	qDebug() << " getXr finish " <<"  "<<Xr << endl;
}

void Dy::Impedance_control::getInit(double threshold,int mostTimes){
	vector<double> normalForce;//������������ ��ӦvOutput
	vector<double> base_normal_location;//�������µķ���λ�������� ��ӦvInput
	vector<double> initPose = rtde_r->getActualTCPPose(); //��ʼ��
	vector<double> finalPose = initPose;//ѹ�����ֹ��
	finalPose[2] -= 0.01;//���°�ѹ10mm
	startFlag = true;
	//��ȡ���������ݣ���ѹ�뵽normalForce
	std::thread getData([&]() mutable {
		while (startFlag)
		{
			normalForce.push_back((rtde_r->getActualTCPForce())[2]);
			base_normal_location.push_back(((rtde_r->getActualTCPPose())[2]));
			Sleep(20);//20ms��һ��
		}
		 });
	std::thread moveNormal([&]()mutable {
		//һ��һ��
		rtde_c->moveL(finalPose, 0.001, 1.2, false);
		rtde_c->moveL(initPose, 0.001, 1.2, false);
		startFlag = false;
		});
	moveNormal.join();
	getData.join();
	//��ʼ������
	vector<sample> rawTrainGroup = getTrainData(normalForce,base_normal_location);
	vector<sample> trainGroup = bpnet->normalisation(rawTrainGroup, true);//��ѵ�������й�һ��
	getInitPop(pop, trainGroup);
	gen = 1;//����һ�´�����Ĭ�ϳ�ʼ����Ϊ1��
	while (gen<generation)
	{
		evolution(*bpnet, trainGroup);
	}
	int fitTag = findBest(pop);
	chromoDecode(pop[fitTag], *bpnet);
	bpnet->doTraining(trainGroup, threshold, mostTimes);
}

double Dy::Impedance_control::getXr( double Fd ) {

	//�����ǽ�Fd����һ��vector<sample>��������й�һ��
	vector<double> vdouble;
	vdouble.push_back(Fd);
	vector<sample> vsample;
	sample s;
	s.in = vdouble;
	vsample.push_back(s);
	vector<sample> predGroup = bpnet->normalisation(vsample, false); //Ԥ�⼯
	// ------------------------------------------------

	bpnet->doTesting(predGroup);
	vector<sample> rawPredGroup = bpnet->denormalisation(predGroup);//���з���һ���õ���Ӧ������λ��
	double Xr = rawPredGroup.front().out.front();//��ȡԤ��Xr
	return Xr;
}

double Dy::getDistance(const vector<double>& pose1, const vector<double>& pose2) {

	double distance = pow(pow(pose1[0] - pose2[0], 2) + pow(pose1[1] - pose2[1], 2) + pow(pose1[2] - pose2[2], 2), 0.5);
	return distance;
}

void Dy::Impedance_control::Normal_force_control_base_on_now(double Fd, double Ts, vector<double>_moveUnitDirInWorld,double V,double tForServo,double sigma,int mode,double downLimit,double R,double LengthOfSensor2MassageHeadCentre,double K) {
	
	if (mode == -1) //test
	{
		if (rotateLock == nullptr) { //�˹�����Ҫ�ж�д��������
			qDebug() << "need a rotatelock" << endl;
			return;
		}

		//�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		int tmp1 = _moveUnitDirInWorld[0];
		int tmp2 = _moveUnitDirInWorld[1];

		Fd = -1 * Fd;//��Ϊ����Ҫת��Ϊ��TCP����ϵ�£�����������10NҪ���-10N

		vector<double> normalTCP_move{ 0,0,1 };//��ʾһ����TCP����ϵ�·�����ƶ�����

		//Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double ��t = 0;

		double theta = LengthOfSensor2MassageHeadCentre * 1000 *  PI / 180; //Բ�ι켣
		double ���� = K * PI / 180;

		//������°�ť����ϣ��1s ת��10�㣬��һ������ת���ĽǶȷֱ�Ϊ����x�ͦ���y��(ע������Ҫ�ǻ����ƣ�����ҪתΪ����)
		double ����x = 10 * R * 1000 * Ts * PI / 180;
		double ����y = 10 * R * 1000 * Ts * PI / 180;

		qDebug() << "����x : " << ����x << endl;
		qDebug() << "����y : " << ����y << endl;

		double curRotateX = 0;//��ǰ��Ҫת����flag
		double curRotateY = 0;

		//��ȡ��ǰλ����Ϣ
		vector<double> pose = rtde_r->getActualTCPPose();

		//��ʼ������
		//Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰTCPλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//��ʼ������λ�ü��ٶ�
		Xc_ddt = 0;

		//�迹���Ʊ���
		auto temp_Xc = Xc;//��һʱ�̵�Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��
		if (rotateLock->tryLockForRead()) { //����ȥ��ȡrotateX��Y�������µ�ǰ��Ҫת���ĽǶ�
			curRotateX = rotateTCPX * ����x;
			curRotateY = rotateTCPY * ����y;
			rotateLock->unlock();
		}
		auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

		//����λ����������̬
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newAngle[0];
		pose[4] = newAngle[1];
		pose[5] = newAngle[2];

		//��һ���������ٶ� ����Fd��Fe������TCP�����ϵ�
		vector<double> force = rtde_r->getActualTCPForce();
		force = world_2tcp_force(force, pose);//ת��ΪTCP�����µ���
		double Fe = force[2];//���򻷾���
		��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		direction_vector[0] = direction_vector[0] * cos(theta) - direction_vector[1] * sin(theta);
		direction_vector[1] = direction_vector[0] * sin(theta) + direction_vector[1] * cos(theta);

		theta += ����;

		startFlag = true;//������

		// --------------------��ʱ����----------------------------
		timeb t1;
		ftime(&t1);//��ȡ����

		long long t = t1.time * 1000 + t1.millitm;

		// --------------------��ʱ����----------------------------

		timeb start;
		ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		while (startFlag)
		{
			if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
			//�����½����̲��ÿ�����̬
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

				//����λ��
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 2000);
				ftime(&start);//��ȡ����

				continue;
			}
			else
			{
				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//��ȡ����
			}

			//��������
			pose = rtde_r->getActualTCPPose();

			//�迹�����㷨����
			auto temp_Xc = Xc;//��һʱ�̵�Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó�Ҫ�ƶ��ľ���
			if (rotateLock->tryLockForRead()) {
				curRotateX = rotateTCPX * ����x;
				curRotateY = rotateTCPY * ����y;
				rotateLock->unlock();
			}
			auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

			//�����������λ����������̬
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			long long cur = start.time * 1000 + start.millitm;

			if (cur - t > 1000) { //�½���ֹͣһ�����ƶ�
				// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
				vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
				pose[0] += V * direction_vector[0] * Ts;
				pose[1] += V * direction_vector[1] * Ts;
				pose[2] += V * direction_vector[2] * Ts;
			}

			pose[3] = newAngle[0];
			pose[4] = newAngle[1];
			pose[5] = newAngle[2];

			//������һ�εĲ���λ�ü��ٶ�
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//ת��ΪTCP�����µ���
			Fe = force[2];//���򻷾���
			��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

			_moveUnitDirInWorld[0] = tmp1 * cos(theta) - tmp2 * sin(theta);
			_moveUnitDirInWorld[1] = tmp1 * sin(theta) + tmp2 * cos(theta);

			theta += ����;
			if (theta > 2 * PI)
				theta -= 2 * PI;

			qDebug() << "theta :" << theta << endl;

		}
		rtde_c->servoStop();
	}
	else if (mode == -2) {  //��̬�任�Ļ���������Ӧ�迹����
		
		//�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		Fd = -1 * Fd;//��Ϊ����Ҫת��Ϊ��TCP����ϵ�£�����������10NҪ���-10N

		vector<double> normalTCP_move{ 0,0,1 };//��ʾһ����TCP����ϵ�·�����ƶ�����

		//Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double ��t = 0;

		//��ȡ��ǰλ����Ϣ
		vector<double> pose = rtde_r->getActualTCPPose();

		//��ʼ������
		//Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰTCPλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//��ʼ���������ٶ�
		Xc_ddt = 0;

		//�迹���Ʊ���
		auto temp_Xc = Xc;//��һʱ�̵�Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

		//����λ��
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// ���ݿ������ڼ����x�᷽��Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã����������
		pose[0] += V * Ts;

		//��һ���������ٶ� ����Fd��Fe������TCP�����ϵ�
		vector<double> force = rtde_r->getActualTCPForce();
		force = world_2tcp_force(force, pose);//ת��ΪTCP�����µ���
		double Fe = force[2];//���򻷾���
		��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//������

		timeb start;
		ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		while (startFlag)
		{
			if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
				break;
			qDebug() << "Xc_ddt: " << Xc_ddt;
			qDebug() << "Xc_dt" << Xc_dt;
			qDebug() << "Xc" << Xc;
			qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];
			
			//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

				//����λ��
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				// ���ݿ������ڼ����x�᷽��Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã����������
				pose[0] += V * Ts;

				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//��ȡ����

				continue;
			}
			else
			{
				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//��ȡ����
			}

			//��������
			pose = rtde_r->getActualTCPPose();

			//�迹�����㷨����
			auto temp_Xc = Xc;//��һʱ�̵�Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó�Ҫ�ƶ��ľ���

			//�����������λ��
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];
			pose[0] += V * Ts;

			//������һ�εĲ���λ�ü��ٶ�
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//ת��ΪTCP�����µ���
			Fe = force[2];//���򻷾���
			��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();
	}
	else if (mode == -3)//����Ӧ������(�ɰ汾����Ӧ�����ᣬֻ�ܴ�ֱ�������ϵz��)
	{
		//Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double ��t = 0;

		//��ȡ��ǰλ����Ϣ
		vector<double> pose = rtde_r->getActualTCPPose();
		//��ʼ������
		//Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;Xc=Xe;�Ե�ǰλ����ΪXc
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = pose[2];
		Xc = Xe;

		//��ʼ���������ٶ�
		Xc_ddt = 0;

		//�迹���Ʊ���
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;

		//����λ��
		pose[2] = Xc;

		// ���ݿ������ڼ����x�᷽��Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã����������
		double Xd_x = pose[0] + V * Ts;
		pose[0] = Xd_x;

		//��һ���������ٶ�
		vector<double> force = rtde_r->getActualTCPForce();
		double Fe = force[2];//���򻷾���
		��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//������

		while (startFlag)
		{
			timeb start;
			ftime(&start);//��ȡ����

			if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
				break;
			qDebug() << "Xc_ddt: " << Xc_ddt;
			qDebug() << "Xc_dt" << Xc_dt;
			qDebug() << "Xc" << Xc;
			qDebug() << "Fe" << rtde_r->getActualTCPForce()[2];
			//��ֹ�迹�����ڸ߿��½��ù��죬���һ����ֵ0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
			if (Xc_dt< - downLimit && rtde_r->getActualTCPForce()[2]< Fd/2) 
			{
				qDebug() << " protect";
				Xc -= Ts*downLimit; //����0.1m/s ���½����㣬0.002s���ߵľ������0.0002��

				pose[2] = Xc;//�ƶ�������λ��
				Xd_x += V * Ts;
				pose[0] = Xd_x;
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);

				timeb end;
				ftime(&end);//��ȡ����
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

			//��������
			pose = rtde_r->getActualTCPPose();

			//�迹�����㷨����
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;

			pose[2] = Xc;//����λ��
			Xd_x += V * Ts;
			pose[0] = Xd_x;

			//������һ�εĲ���λ�ü��ٶ�
			force = rtde_r->getActualTCPForce();
			Fe = force[2];//���򻷾���
			��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

			timeb end;
			ftime(&end);//��ȡ����
			int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
			if (deta_t < Ts * 1000) {
				Sleep(Ts * 1000 - deta_t);
			}
		}
		rtde_c->servoStop();
	}
	else if (mode == 0)  //�ڿ������迹����ͬʱ������ʹ�ã�Ŀ����̽����ת���йص�Ӱ������ �����mode 0 �Ͷ�Ӧ������ݣ����Ҫ����Dy_Control cpp����ҲҪ�ģ�
	{
		if (rotateLock == nullptr) { //�˹�����Ҫ�ж�д��������
			qDebug() << "need a rotatelock" << endl;
			return;
		}

		//�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		Fd = -1 * Fd;//��Ϊ����Ҫת��Ϊ��TCP����ϵ�£�����������10NҪ���-10N

		vector<double> normalTCP_move{ 0,0,1 };//��ʾһ����TCP����ϵ�·�����ƶ�����

		//Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double ��t = 0;

		//������°�ť����ϣ��1s ת��10�㣬��һ������ת���ĽǶȷֱ�Ϊ����x�ͦ���y��(ע������Ҫ�ǻ����ƣ�����ҪתΪ����)
		double ����x = 10 * Ts * PI / 180;
		double ����y = 10 * Ts * PI / 180;

		double curRotateX = 0;//��ǰ��Ҫת����flag
		double curRotateY = 0;

		//��ȡ��ǰλ����Ϣ
		vector<double> pose = rtde_r->getActualTCPPose();

		//��ʼ������
		//Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰTCPλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//��ʼ������λ�ü��ٶ�
		Xc_ddt = 0;

		//�迹���Ʊ���
		auto temp_Xc = Xc;//��һʱ�̵�Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��
		if (rotateLock->tryLockForRead()) { //����ȥ��ȡrotateX��Y�������µ�ǰ��Ҫת���ĽǶ�
			curRotateX = rotateTCPX * ����x;
			curRotateY = rotateTCPY * ����y;
			rotateLock->unlock();
		}
		auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

		//����λ����������̬
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];
		
		// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newAngle[0];
		pose[4] = newAngle[1];
		pose[5] = newAngle[2];

		//��һ���������ٶ� ����Fd��Fe������TCP�����ϵ�
		vector<double> force = rtde_r->getActualTCPForce();
		force = world_2tcp_force(force, pose);//ת��ΪTCP�����µ���
		double Fe = force[2];//���򻷾���
		��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//������

		// --------------------��ʱ����----------------------------
		timeb t1;
		ftime(&t1);//��ȡ����

		long long t = t1.time * 1000 + t1.millitm;

		// --------------------��ʱ����----------------------------

		timeb start;
		ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		while (startFlag)
		{
			if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
			//�����½����̲��ÿ�����̬
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

				//����λ��
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 2000);
				ftime(&start);//��ȡ����

				continue;
			}
			else
			{
				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//��ȡ����
			}

			//��������
			pose = rtde_r->getActualTCPPose();

			//�迹�����㷨����
			auto temp_Xc = Xc;//��һʱ�̵�Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó�Ҫ�ƶ��ľ���
			if (rotateLock->tryLockForRead()) {
				curRotateX = rotateTCPX * ����x;
				curRotateY = rotateTCPY * ����y;
				rotateLock->unlock();
			}
			auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(curRotateX, curRotateY);

			//�����������λ����������̬
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			long long cur = start.time * 1000 + start.millitm;

			if (cur - t > 1000) { //�½���ֹͣһ�����ƶ�
				// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
				vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
				pose[0] += V * direction_vector[0] * Ts;
				pose[1] += V * direction_vector[1] * Ts;
				pose[2] += V * direction_vector[2] * Ts;
			}

			pose[3] = newAngle[0];
			pose[4] = newAngle[1];
			pose[5] = newAngle[2];

			//������һ�εĲ���λ�ü��ٶ�
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//ת��ΪTCP�����µ���
			Fe = force[2];//���򻷾���
			��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();
	}
	else if(mode == 1) //������ת + ��������Ӧ�迹����
	{
		//�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		vector<double> normalMotion_move{ 0,0,1 };//��ʾһ�����ƶ�����ϵ�·�����ƶ�����

		//Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double ��t = 0;

		int count = 5; //�ռ����ٸ�Ts���ڶ�Ӧ����תʸ������һ�κ���

		int norm_count = 1; //�ռ����ٸ�����������һ�κ��㷨����

		vector<cv::Vec3d> normalVector_array;

		cv::Vec3d expectRotationVector{ 0,0,0 };//���������������ת���� ������ϵ�µı�ʾ

		// ÿ���������ڵ������ת�ٶ� ����ĳ���̶�����ת���ٶȣ�
		double ���� = 5 * Ts * PI / 180 * count; //��ǰ���������ÿ��Ķ��������ԣ�

		cv::Vec3d curRotationVector{ 0,0,0 }; //��תʸ��
		double curRotationVector_size = 0;//��תʸ���Ĵ�С

		vector<cv::Vec3d> curRotationVector_array;

		vector<double> pose = rtde_r->getActualTCPPose();

		//-------------------��ȡ�ƶ�����ϵ�µ���------------------------

		vector<double> cur_pose = rtde_r->getActualTCPPose();

		auto sixth_axis = getSixthAxisFromBasePose(cur_pose);

		Vec3d end_effector_direction{ _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		auto force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, {0,0,1}, rtde_r);

		//------------------------------------------------------------

		//----------------���������ϵ���ƶ�����ϵ����ת����----------------

		Vec3d x_axis, y_axis, z_axis;

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis,rtde_r);

		cv::Mat RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		cv::Mat RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Vec3d RotVecBase2Motion;

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//------------------------------------------------------------

		//��ʼ������
		//Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰ�ƶ�����ϵλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//��ʼ������λ�ü��ٶ�
		Xc_ddt = 0;

		//�迹���Ʊ���
		auto temp_Xc = Xc;//��һʱ�̵�Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalMotion_move[2] = Xc - temp_Xc;
		cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��


		//����Ŀ�����������ƶ��Ƕ�
		if (force[2] < Fd / 2) { // �½�״̬��������δ�ȶ�,��ת�� 

			curRotationVector = cv::Vec3d{ 0,0,0 };
			curRotationVector_size = 0;
		}
		else {
			cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());

			cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //Ŀ���������ƶ�����ϵ�µı�ʾ

			normalVector_array.push_back(_targetVec);

			if (normalVector_array.size() == norm_count) {

				cv::Vec3d sum{0.0,0.0,0.0};

				for (const auto& vec : normalVector_array)
					sum += vec;

				sum /= static_cast<double>(normalVector_array.size());

				_targetVec = sum;

				cv::Vec3d targetVec = rotateVecToTarget(_targetVec, RotMatMotion2Base);//�ڻ�����ϵ�µ�Ŀ��������ʾ

				//------------------------------------------����ģ��----------------------------------------------

				cv::Vec3d vCompensation_in_base = rotateVecToTarget(vCompensation, RotMatMotion2Base); //���㲹��

				//--------------------------------------------------------------------------------------------

				if (cv::norm(targetVec) != 0) {

					curRotationVector = CalculateRotationVector(-sixthAxis, targetVec);

					//------------------------------------------����ģ��----------------------------------------------

					if (hasCompensation) {
						cv::Vec3d vCompensation_RotationVector = CalculateRotationVector(-sixthAxis, vCompensation_in_base); //���㲹��
						curRotationVector = combineRotationVectors(curRotationVector, -vCompensation_RotationVector);
					}

					//--------------------------------------------------------------------------------------------


					//curRotationVector_size = 5 * ���� < cv::norm(curRotationVector) ? ���� : 0;
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

		if (curRotationVector_array.size() == count) { //�ռ�ʮ��ʸ���Ϳ��Ե�����̬��

			expectRotationVector = averageRotationVector(curRotationVector_array);
			ResizeVector(expectRotationVector, 5 * ���� < cv::norm(expectRotationVector) ? ���� : 0);

			curRotationVector_array.clear();

			qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
			qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
		}

		newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] ,expectRotationVector[1] ,expectRotationVector[2] });

		//����λ����������̬
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newPose[3];
		pose[4] = newPose[4];
		pose[5] = newPose[5];

		//-------------------��ȡ�ƶ�����ϵ�µ���------------------------

		cur_pose = rtde_r->getActualTCPPose(); 

		sixth_axis = getSixthAxisFromBasePose(cur_pose);

		end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

		//-----------------------------------------------------------

		//----------------���������ϵ���ƶ�����ϵ����ת����----------------

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis,rtde_r);

		RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//-------------------------------------------------------------

		double Fe = force[2];//���򻷾���
		��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//������


		// --------------------��ʼʱ���----------------------------

		timeb t1;
		ftime(&t1);//��ȡ����

		long long t = t1.time * 1000 + t1.millitm;

		// ---------------------------------------------------------

		timeb start;
		ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		while (startFlag)
		{
			if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
			//�����½����̲��ÿ�����̬
			if (Xc_dt < -downLimit && getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r)[2] < Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc -= downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

				normalMotion_move[2] = Xc - temp_Xc;
				cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //�����迹���Ƶó����ƶ�����ϵ��Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

				//����λ��
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				//Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//��ȡ����

				continue;
			}
			else
			{
				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				//qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}

				//Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//��ȡ����
			}

			//��������
			pose = rtde_r->getActualTCPPose();

			//�迹�����㷨����
			auto temp_Xc = Xc;//��һʱ�̵�Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalMotion_move[2] = Xc - temp_Xc;
			cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //�����迹���Ƶó�Ҫ�ƶ��ľ���

			//qDebug() << "Force :" << force[2] << endl;
			//qDebug() << "moveDistance :" << moveDistance[0] << moveDistance[1] << moveDistance[2] << endl;
			//qDebug() << "Xc :" << Xc << " Xc_dt:" << Xc_dt << " Xc_ddt:" << Xc_ddt << endl;

			//����Ŀ�����������ƶ��Ƕ�
			if (force[2] < Fd / 2) { // �½�״̬��������δ�ȶ�,��ת�� 

				curRotationVector = cv::Vec3d{ 0,0,0 };
				curRotationVector_size = 0;
			}
			else {
				cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
				
				//qDebug() << "base_force" << " : " << _force[0] << " " << _force[1] << " " << _force[2] << " " << _force[3] << " " << _force[4] << " " << _force[5];
				qDebug() << "force" << " : " << force[0] << " " << force[1] << " " << force[2] << " " << force[3] << " " << force[4] << " " << force[5];

				//force[0] = force[1] = force[3] = force[4] = force[5] = 0;
				cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //Ŀ���������ƶ�����ϵ�µı�ʾ

				normalVector_array.push_back(_targetVec);

				if (normalVector_array.size() == norm_count) {

					cv::Vec3d sum{ 0.0,0.0,0.0 };

					for (const auto& vec : normalVector_array)
						sum += vec;

					sum /= static_cast<double>(normalVector_array.size());

					_targetVec = sum;

					if(cv::norm(_targetVec) > 1e-8)
						cv::normalize(_targetVec, _targetVec);
					
					cv::Vec3d targetVec = rotateVecToTarget(_targetVec, RotMatMotion2Base);//�ڻ�����ϵ�µ�Ŀ��������ʾ

					//------------------------------------------����ģ��----------------------------------------------

					cv::Vec3d vCompensation_in_base = rotateVecToTarget(vCompensation, RotMatMotion2Base); //���㲹��

					//----------------------------------------------------------------------------------------------

					if (cv::norm(targetVec) != 0) {

						curRotationVector = CalculateRotationVector(-sixthAxis, targetVec);

						//------------------------------------------����ģ��----------------------------------------------

						if (hasCompensation) {
							cv::Vec3d vCompensation_RotationVector = CalculateRotationVector(-sixthAxis, vCompensation_in_base); //���㲹��
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

						//-------------------------------------------- ���������õ����---------------------------------------------------

						//�� 0 0 1 �ļн�
						auto rrot = CalculateRotationVector(targetVec, { 0,0,1 });
						qDebug() << "targetVec and {0,0,1} 's rotVec_size :" << RAD_TO_DEG(cv::norm(rrot)) << endl;;

						//------------------------------------------------------------------------------------------------------------

						qDebug() << "curRotationVector :" << curRotationVector[0] << curRotationVector[1] << curRotationVector[2] << endl;
						qDebug() << "curRotationVector_size" << RAD_TO_DEG(cv::norm(curRotationVector)) << endl;

						qDebug() << "vCompensation_in_base :" << vCompensation_in_base[0] << vCompensation_in_base[1] << vCompensation_in_base[2] << endl;
						qDebug() << "vCompensation_in_base_size" << RAD_TO_DEG(cv::norm(vCompensation_in_base)) << endl;

						//curRotationVector_size = 5 * ���� < cv::norm(curRotationVector) ? ���� : 0;
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

			if (curRotationVector_array.size() == count) { //�ռ�ʮ��ʸ���Ϳ��Ե�����̬��

				expectRotationVector = averageRotationVector(curRotationVector_array);

				qDebug() << "expectRotationVector before resize :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
				qDebug() << "expectRotationVector_size before resize" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;

				ResizeVector(expectRotationVector, 5 * ���� < cv::norm(expectRotationVector) ? ���� : 0); // ��

				curRotationVector_array.clear();

				qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
				qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
			}


			//-----------------------------��֤��ʼ���˴���ʱ����Ϊ���ٲſ�ʼ������̬(Ŀ����Ϊ��Խ��ճ�ͽ׶�)----------------------------

			long long cur = start.time * 1000 + start.millitm;

			if (cur - t > 6000) { 
				newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] ,expectRotationVector[1] ,expectRotationVector[2] });
			}
			else newPose = pose;
			

			//�����������λ����������̬
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
			vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
			pose[0] += V * direction_vector[0] * Ts;
			pose[1] += V * direction_vector[1] * Ts;
			pose[2] += V * direction_vector[2] * Ts;

			pose[3] = newPose[3];
			pose[4] = newPose[4];
			pose[5] = newPose[5];

			//-------------------��ȡ�ƶ�����ϵ�µ���------------------------

			auto cur_pose = rtde_r->getActualTCPPose();

			sixth_axis = getSixthAxisFromBasePose(cur_pose);

			end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

			force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);


			//-----------------------------------------------------------

			//----------------���������ϵ���ƶ�����ϵ����ת����----------------

			computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis,rtde_r);

			RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

			RotMatMotion2Base = RotMatBase2Motion.t();

			cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

			//-------------------------------------------------------------

			Fe = force[2];//���򻷾���
			��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();



		////�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		//Fd = -1 * Fd;//��Ϊ����Ҫת��Ϊ��TCP����ϵ�£�����������10NҪ���-10N

		//vector<double> normalTCP_move{ 0,0,1 };//��ʾһ����TCP����ϵ�·�����ƶ�����

		////Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		//double Xc_ddt = 0;
		//double Xc_dt = 0;
		//double Xc = 0;
		//double Xe_ddt = 0;
		//double Xe_dt = 0;
		//double Xe = 0;

		//double ��t = 0;

		//int count = 10; //�ռ����ٸ�Ts���ڶ�Ӧ����תʸ������һ�κ���

		//cv::Vec3d expectRotationVector{ 0,0,0 };//���������������ת����

		//// ÿ���������ڵ������ת�ٶ� ����ĳ���̶�����ת���ٶȣ�
		//double ���� = 40 * Ts * PI / 180 * count; //��ǰ���������ÿ��Ķ��������ԣ�

		//cv::Vec3d curRotationVector{ 0,0,0 }; //��תʸ��
		//double curRotationVector_size = 0;//��תʸ���Ĵ�С

		//vector<cv::Vec3d> curRotationVector_array;

		////��ȡ��ǰλ����Ϣ
		//vector<double> pose = rtde_r->getActualTCPPose();

		////��ʼ������Ϣ 
		//vector<double> force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);
		////��ȡ�ҵĹ�ʽ�µĴ���������ϵ������Ϣ�Լ� �����굽������ϵ����ת����
		//auto myFormulaForce_and_Base2Formula = getMyFormulaCDSForceAndBase2Formula(rtde_r);

		//auto myFormulaForce = myFormulaForce_and_Base2Formula.first;

		//auto Base2Formula = myFormulaForce_and_Base2Formula.second;

		////��ʼ������
		////Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰTCPλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		//Xe_ddt = 0;
		//Xe_dt = 0;
		//Xe = 0;
		//Xc = Xe;

		////��ʼ������λ�ü��ٶ�
		//Xc_ddt = 0;

		////�迹���Ʊ���
		//auto temp_Xc = Xc;//��һʱ�̵�Xc
		//Xc_dt = Xc_dt + Xc_ddt * Ts;
		//Xc = Xc + Xc_dt * Ts;
		//normalTCP_move[2] = Xc - temp_Xc;
		//auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��
		//
		////����Ŀ�����������ƶ��Ƕ�
		//if (force[2] > Fd / 2) { // �½�״̬��������δ�ȶ�,��ת��

		//	curRotationVector = cv::Vec3d{ 0,0,0 };
		//	curRotationVector_size = 0;
		//}
		//else {
		//	cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//	cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, R, LengthOfSensor2MassageHeadCentre);
		//	cv::Mat Formula2Base = Base2Formula.inv();
		//	cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//�ڻ�����ϵ�µ�Ŀ��������ʾ
		//	if (cv::norm(targetVec) != 0) {
		//		curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//		//curRotationVector_size = 5 * ���� < cv::norm(curRotationVector) ? ���� : 0;
		//		//ResizeVector(curRotationVector, curRotationVector_size);
		//	}
		//	else {
		//		curRotationVector = cv::Vec3d{0,0,0};
		//		curRotationVector_size = 0;
		//	}
		//}
		//curRotationVector_array.push_back(curRotationVector);

		//std::vector<double> newPose = pose;

		//if (curRotationVector_array.size() == count) { //�ռ�ʮ��ʸ���Ϳ��Ե�����̬��
		//	
		//	expectRotationVector = {0,0,0};
		//	for (int i = 0; i < curRotationVector_array.size(); i++) {
		//		
		//		expectRotationVector[0] += curRotationVector_array[i][0];
		//		expectRotationVector[1] += curRotationVector_array[i][1];
		//		expectRotationVector[2] += curRotationVector_array[i][2];
		//	}
		//	expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//	ResizeVector(expectRotationVector, 5 * ���� < cv::norm(expectRotationVector) ? ���� : 0);

		//	curRotationVector_array.clear();

		//	qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//	qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
		//}

		//newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		////����λ����������̬
		//pose[0] += moveDistance[0];
		//pose[1] += moveDistance[1];
		//pose[2] += moveDistance[2];

		//// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		//vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//pose[0] += V * direction_vector[0] * Ts;
		//pose[1] += V * direction_vector[1] * Ts;
		//pose[2] += V * direction_vector[2] * Ts;

		//pose[3] = newPose[3];
		//pose[4] = newPose[4];
		//pose[5] = newPose[5];

		////��һ���������ٶ� ����Fd��Fe������TCP�����ϵ�
		//force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);//ת��ΪTCP�����µ���

		////��ȡ�ҵĹ�ʽ�µĴ���������ϵ������Ϣ�Լ� �����굽������ϵ����ת����
		//auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//myFormulaForce = temp.first;
		//Base2Formula = temp.second;

		//double Fe = force[2];//���򻷾���
		//��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		//Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//startFlag = true;//������

		//timeb start;
		//ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		//while (startFlag)
		//{
		//	if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
		//		break;
		//	//qDebug() << "Xc_ddt: " << Xc_ddt;
		//	//qDebug() << "Xc_dt" << Xc_dt;
		//	//qDebug() << "Xc" << Xc;
		//	//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

		//	//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
		//	//�����½����̲��ÿ�����̬
		//	if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
		//	{
		//		qDebug() << " protect";
		//		auto temp_Xc = Xc;
		//		Xc += downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

		//		normalTCP_move[2] = Xc - temp_Xc;
		//		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

		//		//����λ��
		//		pose[0] += moveDistance[0];
		//		pose[1] += moveDistance[1];
		//		pose[2] += moveDistance[2];

		//		//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		//		timeb end;
		//		ftime(&end);//��ȡ����
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.03, 1000);
		//		ftime(&start);//��ȡ����

		//		continue;
		//	}
		//	else
		//	{
		//		//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		//		timeb end;
		//		ftime(&end);//��ȡ����
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
		//		ftime(&start);//��ȡ����
		//	}

		//	//��������
		//	pose = rtde_r->getActualTCPPose();

		//	//�迹�����㷨����
		//	auto temp_Xc = Xc;//��һʱ�̵�Xc
		//	Xc_dt = Xc_dt + Xc_ddt * Ts;
		//	Xc = Xc + Xc_dt * Ts;
		//	normalTCP_move[2] = Xc - temp_Xc;
		//	auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó�Ҫ�ƶ��ľ���
		//	
		//	//����Ŀ�����������ƶ��Ƕ�
		//	if (force[2] > Fd / 2) { // �½�״̬��������δ�ȶ�,��ת��

		//		curRotationVector = cv::Vec3d{ 0,0,0 };
		//		curRotationVector_size = 0;
		//	}
		//	else {
		//		cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//		cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, R, LengthOfSensor2MassageHeadCentre);
		//		cv::Mat Formula2Base = Base2Formula.inv();
		//		cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//�ڻ�����ϵ�µ�Ŀ��������ʾ
		//		if (cv::norm(targetVec) != 0) {
		//			curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//			//curRotationVector_size = 5 * ���� < cv::norm(curRotationVector) ? ���� : 0;
		//			//ResizeVector(curRotationVector, curRotationVector_size);
		//		}
		//		else {
		//			curRotationVector = cv::Vec3d{ 0,0,0 };
		//			curRotationVector_size = 0;
		//		}
		//	}

		//	curRotationVector_array.push_back(curRotationVector);

		//	if (curRotationVector_array.size() == count) { //�ռ�ʮ��ʸ���Ϳ��Ե�����̬��

		//		expectRotationVector = { 0,0,0 };
		//		for (int i = 0; i < curRotationVector_array.size(); i++) {

		//			expectRotationVector[0] += curRotationVector_array[i][0];
		//			expectRotationVector[1] += curRotationVector_array[i][1];
		//			expectRotationVector[2] += curRotationVector_array[i][2];
		//		}
		//		expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//		ResizeVector(expectRotationVector, 5 * ���� < cv::norm(expectRotationVector) ? ���� : 0);
		//		
		//		curRotationVector_array.clear();

		//		qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//		qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;

		//	}

		//	newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		//	//�����������λ����������̬
		//	pose[0] += moveDistance[0];
		//	pose[1] += moveDistance[1];
		//	pose[2] += moveDistance[2];

		//	// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		//	vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//	pose[0] += V * direction_vector[0] * Ts;
		//	pose[1] += V * direction_vector[1] * Ts;
		//	pose[2] += V * direction_vector[2] * Ts;

		//	pose[3] = newPose[3];
		//	pose[4] = newPose[4];
		//	pose[5] = newPose[5];

		//	//������һ�εĲ���λ�ü��ٶ�
		//	force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//ת��ΪTCP�����µ���
		//	Fe = force[2];//���򻷾���

		//	//��ȡ�ҵĹ�ʽ�µĴ���������ϵ������Ϣ�Լ� �����굽������ϵ����ת����
		//	auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//	myFormulaForce = temp.first;
		//	Base2Formula = temp.second;

		//	��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		//	Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//}
		//rtde_c->servoStop();



		////�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		//Fd = -1 * Fd;//��Ϊ����Ҫת��Ϊ��TCP����ϵ�£�����������10NҪ���-10N

		//vector<double> normalTCP_move{ 0,0,1 };//��ʾһ����TCP����ϵ�·�����ƶ�����

		////Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		//double Xc_ddt = 0;
		//double Xc_dt = 0;
		//double Xc = 0;
		//double Xe_ddt = 0;
		//double Xe_dt = 0;
		//double Xe = 0;

		//double ��t = 0;

		//int count = 10; //�ռ����ٸ�Ts���ڶ�Ӧ����תʸ������һ�κ���

		//cv::Vec3d expectRotationVector{ 0,0,0 };//���������������ת����

		//// ÿ���������ڵ������ת�ٶ� ����ĳ���̶�����ת���ٶȣ�
		//double ���� = 40 * Ts * PI / 180 * count; //��ǰ���������ÿ��Ķ��������ԣ�

		//cv::Vec3d curRotationVector{ 0,0,0 }; //��תʸ��
		//double curRotationVector_size = 0;//��תʸ���Ĵ�С

		//vector<cv::Vec3d> curRotationVector_array;

		////��ȡ��ǰλ����Ϣ
		//vector<double> pose = rtde_r->getActualTCPPose();

		////��ʼ������Ϣ 
		//vector<double> force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);
		////��ȡ�ҵĹ�ʽ�µĴ���������ϵ������Ϣ�Լ� �����굽������ϵ����ת����
		//auto [myFormulaForce, Base2Formula] = getMyFormulaCDSForceAndBase2Formula(rtde_r);

		////��ʼ������
		////Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰTCPλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		//Xe_ddt = 0;
		//Xe_dt = 0;
		//Xe = 0;
		//Xc = Xe;

		////��ʼ������λ�ü��ٶ�
		//Xc_ddt = 0;

		////�迹���Ʊ���
		//auto temp_Xc = Xc;//��һʱ�̵�Xc
		//Xc_dt = Xc_dt + Xc_ddt * Ts;
		//Xc = Xc + Xc_dt * Ts;
		//normalTCP_move[2] = Xc - temp_Xc;
		//auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��
		//
		////����Ŀ�����������ƶ��Ƕ�
		//if (force[2] > Fd / 2) { // �½�״̬��������δ�ȶ�,��ת��

		//	curRotationVector = cv::Vec3d{ 0,0,0 };
		//	curRotationVector_size = 0;
		//}
		//else {
		//	cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//	cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, K, R, LengthOfSensor2MassageHeadCentre);
		//	cv::Mat Formula2Base = Base2Formula.inv();
		//	cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//�ڻ�����ϵ�µ�Ŀ��������ʾ
		//	if (cv::norm(targetVec) != 0) {
		//		curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//		//curRotationVector_size = 5 * ���� < cv::norm(curRotationVector) ? ���� : 0;
		//		//ResizeVector(curRotationVector, curRotationVector_size);
		//	}
		//	else {
		//		curRotationVector = cv::Vec3d{0,0,0};
		//		curRotationVector_size = 0;
		//	}
		//}
		//curRotationVector_array.push_back(curRotationVector);

		//std::vector<double> newPose = pose;

		//if (curRotationVector_array.size() == count) { //�ռ�ʮ��ʸ���Ϳ��Ե�����̬��
		//	
		//	expectRotationVector = {0,0,0};
		//	for (int i = 0; i < curRotationVector_array.size(); i++) {
		//		
		//		expectRotationVector[0] += curRotationVector_array[i][0];
		//		expectRotationVector[1] += curRotationVector_array[i][1];
		//		expectRotationVector[2] += curRotationVector_array[i][2];
		//	}
		//	expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//	ResizeVector(expectRotationVector, 5 * ���� < cv::norm(expectRotationVector) ? ���� : 0);

		//	curRotationVector_array.clear();

		//	qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//	qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;
		//}

		//newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		////����λ����������̬
		//pose[0] += moveDistance[0];
		//pose[1] += moveDistance[1];
		//pose[2] += moveDistance[2];

		//// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		//vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//pose[0] += V * direction_vector[0] * Ts;
		//pose[1] += V * direction_vector[1] * Ts;
		//pose[2] += V * direction_vector[2] * Ts;

		//pose[3] = newPose[3];
		//pose[4] = newPose[4];
		//pose[5] = newPose[5];

		////��һ���������ٶ� ����Fd��Fe������TCP�����ϵ�
		//force = rtde_r->getActualTCPForce();
		//force = world_2tcp_force(force, pose);//ת��ΪTCP�����µ���

		////��ȡ�ҵĹ�ʽ�µĴ���������ϵ������Ϣ�Լ� �����굽������ϵ����ת����
		//auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//myFormulaForce = temp.first;
		//Base2Formula = temp.second;

		//double Fe = force[2];//���򻷾���
		//��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		//Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//startFlag = true;//������

		//timeb start;
		//ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		//while (startFlag)
		//{
		//	if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
		//		break;
		//	//qDebug() << "Xc_ddt: " << Xc_ddt;
		//	//qDebug() << "Xc_dt" << Xc_dt;
		//	//qDebug() << "Xc" << Xc;
		//	//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

		//	//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
		//	//�����½����̲��ÿ�����̬
		//	if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
		//	{
		//		qDebug() << " protect";
		//		auto temp_Xc = Xc;
		//		Xc += downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

		//		normalTCP_move[2] = Xc - temp_Xc;
		//		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

		//		//����λ��
		//		pose[0] += moveDistance[0];
		//		pose[1] += moveDistance[1];
		//		pose[2] += moveDistance[2];

		//		//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		//		timeb end;
		//		ftime(&end);//��ȡ����
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.03, 1000);
		//		ftime(&start);//��ȡ����

		//		continue;
		//	}
		//	else
		//	{
		//		//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		//		timeb end;
		//		ftime(&end);//��ȡ����
		//		int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
		//		//qDebug() << "deta_t:" << deta_t << endl;
		//		if (deta_t < Ts * 1000) {
		//			Sleep(Ts * 1000 - deta_t);
		//		}
		//		Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
		//		ftime(&start);//��ȡ����
		//	}

		//	//��������
		//	pose = rtde_r->getActualTCPPose();

		//	//�迹�����㷨����
		//	auto temp_Xc = Xc;//��һʱ�̵�Xc
		//	Xc_dt = Xc_dt + Xc_ddt * Ts;
		//	Xc = Xc + Xc_dt * Ts;
		//	normalTCP_move[2] = Xc - temp_Xc;
		//	auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó�Ҫ�ƶ��ľ���
		//	
		//	//����Ŀ�����������ƶ��Ƕ�
		//	if (force[2] > Fd / 2) { // �½�״̬��������δ�ȶ�,��ת��

		//		curRotationVector = cv::Vec3d{ 0,0,0 };
		//		curRotationVector_size = 0;
		//	}
		//	else {
		//		cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
		//		cv::Vec3d _targetVec = calculateSurfaceNormalVector(myFormulaForce, K, R, LengthOfSensor2MassageHeadCentre);
		//		cv::Mat Formula2Base = Base2Formula.inv();
		//		cv::Vec3d targetVec = rotateVecToTarget(_targetVec, Formula2Base);//�ڻ�����ϵ�µ�Ŀ��������ʾ
		//		if (cv::norm(targetVec) != 0) {
		//			curRotationVector = CalculateRotationVector(sixthAxis, targetVec);
		//			//curRotationVector_size = 5 * ���� < cv::norm(curRotationVector) ? ���� : 0;
		//			//ResizeVector(curRotationVector, curRotationVector_size);
		//		}
		//		else {
		//			curRotationVector = cv::Vec3d{ 0,0,0 };
		//			curRotationVector_size = 0;
		//		}
		//	}

		//	curRotationVector_array.push_back(curRotationVector);

		//	if (curRotationVector_array.size() == count) { //�ռ�ʮ��ʸ���Ϳ��Ե�����̬��

		//		expectRotationVector = { 0,0,0 };
		//		for (int i = 0; i < curRotationVector_array.size(); i++) {

		//			expectRotationVector[0] += curRotationVector_array[i][0];
		//			expectRotationVector[1] += curRotationVector_array[i][1];
		//			expectRotationVector[2] += curRotationVector_array[i][2];
		//		}
		//		expectRotationVector = expectRotationVector / (int)curRotationVector_array.size();
		//		ResizeVector(expectRotationVector, 5 * ���� < cv::norm(expectRotationVector) ? ���� : 0);
		//		
		//		curRotationVector_array.clear();

		//		qDebug() << "expectRotationVector :" << expectRotationVector[0] << expectRotationVector[1] << expectRotationVector[2] << endl;
		//		qDebug() << "expectRotationVector_size" << RAD_TO_DEG(cv::norm(expectRotationVector)) << endl;

		//	}

		//	newPose = getNewPoseFromCurPoseAndRotateVec(pose, { expectRotationVector[0] / count,expectRotationVector[1] / count,expectRotationVector[2] / count });

		//	//�����������λ����������̬
		//	pose[0] += moveDistance[0];
		//	pose[1] += moveDistance[1];
		//	pose[2] += moveDistance[2];

		//	// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		//	vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		//	pose[0] += V * direction_vector[0] * Ts;
		//	pose[1] += V * direction_vector[1] * Ts;
		//	pose[2] += V * direction_vector[2] * Ts;

		//	pose[3] = newPose[3];
		//	pose[4] = newPose[4];
		//	pose[5] = newPose[5];

		//	//������һ�εĲ���λ�ü��ٶ�
		//	force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//ת��ΪTCP�����µ���
		//	Fe = force[2];//���򻷾���

		//	//��ȡ�ҵĹ�ʽ�µĴ���������ϵ������Ϣ�Լ� �����굽������ϵ����ת����
		//	auto temp = getMyFormulaCDSForceAndBase2Formula(rtde_r);
		//	myFormulaForce = temp.first;
		//	Base2Formula = temp.second;

		//	��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		//	Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		//}
		//rtde_c->servoStop();



		/*
		//�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		Fd = -1 * Fd;//��Ϊ����Ҫת��Ϊ��TCP����ϵ�£�����������10NҪ���-10N

		vector<double> normalTCP_move{ 0,0,1 };//��ʾһ����TCP����ϵ�·�����ƶ�����

		//Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		//��tcp����ϵ�� x���y��ĵ���ת��,�Գ�ʼ��TCP����ĸ�����xitaX �� xitaYΪ0������һ��ʼ��TCP����ϵ��һ����������̬����ʱ��ı䣬�Ѹ�TCP�ĳ�ʼ����ת��X�͵���ת��Y��Ϊ0 ��
		double ��x_ddt = 0;
		double ��x_dt = 0;
		double ��x = 0;
		double ��y_ddt = 0;
		double ��y_dt = 0;
		double ��y = 0;

		double ��t = 0;
		double ��t_��x = 0; //Ϊ�Ƕ�Ҳ�ṩһ������λ�õ��迹����һ���Ŀ���
		double ��t_��y = 0;

		//��ȡ��ǰλ����Ϣ
		vector<double> pose = rtde_r->getActualTCPPose();

		//��ʼ������
		//Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰTCPλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;
		//��ʼ��ת��
		��x_dt = 0;
		��x = 0;
		��y_dt = 0;
		��y = 0;

		//��ʼ������λ�ü��ٶ�����̬���ٶ�
		Xc_ddt = 0;
		��y_ddt = 0;
		��x_ddt = 0;

		//�迹���Ʊ���
		auto temp_Xc = Xc;//��һʱ�̵�Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalTCP_move[2] = Xc - temp_Xc;
		auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, {pose[3],pose[4],pose[5]}); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��
		��x_dt = ��x_dt + ��x_ddt * Ts;
		auto temp_��x = ��x;//��һʱ�̵Ħ�x
		��x = ��x + ��x_dt * Ts;
		��y_dt = ��y_dt + ��y_ddt * Ts;
		auto temp_��y = ��y;//��һʱ�̵Ħ�y
		��y = ��y + ��y_dt * Ts;
		auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(��x - temp_��x, ��y - temp_��y);

		//����λ����������̬
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;

		pose[3] = newAngle[0];
		pose[4] = newAngle[1];
		pose[5] = newAngle[2];

		//��һ���������ٶ� ����Fd��Fe������TCP�����ϵ�
		vector<double> force = rtde_r->getActualTCPForce();
		vector<double> motion_coordinate_force = getMotionCoordinateSystemForce(rtde_r);//��ȡ�ƶ�����ϵ�µ�����Ϣ

		force = world_2tcp_force(force, pose);//ת��ΪTCP�����µ���
		double Fe = force[2];//���򻷾���
		��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		double Fe_axisX = force[0];
		double Fe_axisY = force[1];
		��t_��x = ��t_��x + sigma * (-Fe_axisY) / B_��;
		��t_��y = ��t_��y + sigma * (Fe_axisX) / B_��;
		��x_ddt = 1 / M_�� * (-1 * Fe_axisY - ((B_�� * ��x_dt) + (B_�� * ��t_��x - sigma * -1 * Fe_axisY)) - K_�� * ��x);
		��y_ddt = 1 / M_�� * (Fe_axisX - ((B_�� * ��y_dt) + (B_�� * ��t_��y - sigma * Fe_axisX)) - K_�� * ��y);
		
		startFlag = true;//������

		timeb start;
		ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		while (startFlag)
		{
			if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];
			
			//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
			//�����½����̲��ÿ�����̬
			if (Xc_dt > downLimit && world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2] > Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc += downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

				normalTCP_move[2] = Xc - temp_Xc;
				auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

				//����λ��
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//��ȡ����

				continue;
			}
			else
			{
				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);
				ftime(&start);//��ȡ����
			}

			//��������
			pose = rtde_r->getActualTCPPose();

			//�迹�����㷨����
			auto temp_Xc = Xc;//��һʱ�̵�Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalTCP_move[2] = Xc - temp_Xc;
			auto moveDistance = tran_P_from_tcp_2world(normalTCP_move, { pose[3],pose[4],pose[5] }); //�����迹���Ƶó�Ҫ�ƶ��ľ���
			��x_dt = ��x_dt + ��x_ddt * Ts;
			auto temp_��x = ��x;//��һʱ�̵Ħ�x
			��x = ��x + ��x_dt * Ts;
			��y_dt = ��y_dt + ��y_ddt * Ts;
			auto temp_��y = ��y;//��һʱ�̵Ħ�y
			��y = ��y + ��y_dt * Ts;
			auto newAngle = get_RxRyRz_from_xitaX_and_xitaY(��x - temp_��x, ��y - temp_��y);

			//�����������λ����������̬
			pose[0] += moveDistance[0]; 
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
			vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
			pose[0] += V * direction_vector[0] * Ts;
			pose[1] += V * direction_vector[1] * Ts;
			pose[2] += V * direction_vector[2] * Ts;

			pose[3] = newAngle[0];
			pose[4] = newAngle[1];
			pose[5] = newAngle[2];

			//������һ�εĲ���λ�ü��ٶ�
			force = world_2tcp_force(rtde_r->getActualTCPForce(), pose);//ת��ΪTCP�����µ���
			Fe = force[2];//���򻷾���
			��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));
			Fe_axisX = force[0];
			Fe_axisY = force[1];
			��t_��x = ��t_��x + sigma * (-Fe_axisY) / B_��;
			��t_��y = ��t_��y + sigma * (Fe_axisX) / B_��;
			��x_ddt = 1 / M_�� * (-1 * Fe_axisY - ((B_�� * ��x_dt) + (B_�� * ��t_��x - sigma * -1 * Fe_axisY)) - K_�� * ��x);
			��y_ddt = 1 / M_�� * (Fe_axisX - ((B_�� * ��y_dt) + (B_�� * ��t_��y - sigma * Fe_axisX)) - K_�� * ��y);
			qDebug() << "  ��x_ddt :" << ��x_ddt<< "  ��x_dt :" << ��x_dt << "  ��x :" << ��x << "  Fe_axisY :" << Fe_axisY <<"  ��x - temp_��x" << ��x - temp_��x;
			qDebug() << "  ��y_ddt :" << ��y_ddt << "  ��y_dt :" << ��y_dt << "  ��y :" << ��y << "  Fe_axisX :" << Fe_axisX << "  ��y - temp_��y" << ��y - temp_��y;

		}
		rtde_c->servoStop();
		*/
	}
	else if (mode == 2) {
		
		// ��ȡ��������ר�� 

		//�ҵ���ɢ�迹˼·�ǣ������迹��������������ǰһʱ��Xc_ddt = 0��Ȼ�����Xc_ddt���� Xc_dt��Xc��Ȼ���ٸ�����һ��Xc_ddt��Ȼ���ŷ��ƶ���ǰ�ε�Xc

		vector<double> normalMotion_move{ 0,0,1 };//��ʾһ�����ƶ�����ϵ�·�����ƶ�����

		//Xc����������˵�ָ��λ�ã�Xe������������λ�ã�ddt��dt���ֱ��Ӧ���ٶ��ٶ�
		double Xc_ddt = 0;
		double Xc_dt = 0;
		double Xc = 0;
		double Xe_ddt = 0;
		double Xe_dt = 0;
		double Xe = 0;

		double ��t = 0;

		int count = 50; //�ռ����ٸ�Ts���ڶ�Ӧ�ķ���������һ�κ���

		vector<cv::Vec3d> vCompensation_array;

		vector<double> pose = rtde_r->getActualTCPPose();

		//-------------------��ȡ�ƶ�����ϵ�µ���------------------------

		vector<double> cur_pose = rtde_r->getActualTCPPose();

		auto sixth_axis = getSixthAxisFromBasePose(cur_pose);

		Vec3d end_effector_direction{ _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		auto force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

		//------------------------------------------------------------

		//----------------���������ϵ���ƶ�����ϵ����ת����----------------

		Vec3d x_axis, y_axis, z_axis;

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis, rtde_r);

		cv::Mat RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		cv::Mat RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Vec3d RotVecBase2Motion;

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//------------------------------------------------------------

		//��ʼ������
		//Ƥ��ƽ�棬��Xe_ddt=0;Xe_dt=0;  �Ե�ǰ�ƶ�����ϵλ�� ����һ���̶�����ϵ��XeΪԭ��0  Xe��ΪXc�ĳ��Ե� XcΪ�ڸù̶�����ϵ�µ����������ָ���λ��
		Xe_ddt = 0;
		Xe_dt = 0;
		Xe = 0;
		Xc = Xe;

		//��ʼ������λ�ü��ٶ�
		Xc_ddt = 0;

		//�迹���Ʊ���
		auto temp_Xc = Xc;//��һʱ�̵�Xc
		Xc_dt = Xc_dt + Xc_ddt * Ts;
		Xc = Xc + Xc_dt * Ts;
		normalMotion_move[2] = Xc - temp_Xc;
		cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //�����迹���Ƶó���TCP������Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

		//����Ŀ�����������ƶ��Ƕ�
		if (force[2] >= Fd / 2) { // �½�״̬��������δ�ȶ�,�򲻼��㲹��������

			cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
			cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //Ŀ���������ƶ�����ϵ�µı�ʾ
			vCompensation_array.push_back(_targetVec);

		}

		if (vCompensation_array.size() == count) { // �ռ�50�����������������ƽ��

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

		//����λ��
		pose[0] += moveDistance[0];
		pose[1] += moveDistance[1];
		pose[2] += moveDistance[2];

		// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
		vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
		pose[0] += V * direction_vector[0] * Ts;
		pose[1] += V * direction_vector[1] * Ts;
		pose[2] += V * direction_vector[2] * Ts;


		//-------------------��ȡ�ƶ�����ϵ�µ���------------------------

		cur_pose = rtde_r->getActualTCPPose();

		sixth_axis = getSixthAxisFromBasePose(cur_pose);

		end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

		force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

		//-----------------------------------------------------------


		//----------------���������ϵ���ƶ�����ϵ����ת����----------------

		computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis, rtde_r);

		RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

		RotMatMotion2Base = RotMatBase2Motion.t();

		cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

		//-------------------------------------------------------------

		double Fe = force[2];//���򻷾���
		��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
		Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		startFlag = true;//������

		timeb start;
		ftime(&start);//��ȡ����,�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
		while (startFlag)
		{
			if (!startFlag)//����ȷ�ϣ���ֹ�����̹߳ر����迹����
				break;
			//qDebug() << "Xc_ddt: " << Xc_ddt;
			//qDebug() << "Xc_dt" << Xc_dt;
			//qDebug() << "Xc" << Xc;
			//qDebug() << "Fe" << world_2tcp_force(rtde_r->getActualTCPForce(), pose)[2];

			//��ֹ�迹�������½����̲����ۼ�Xc_dt ����������س��������ߣ��������һ����ֵ��0.1m/s ��һ���ٶȳ���0.1m/s ���Թ̶��ٶ�0.1m/s������λ�ã������ٸ���Xc_ddt��Xc_dt��ֱ���Ӵ�Ϊֹ������0.2N�ĽӴ���ֵ��
			//�����½����̲��ÿ�����̬
			if (Xc_dt < -downLimit && getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r)[2] < Fd / 2)
			{
				qDebug() << " protect";
				auto temp_Xc = Xc;
				Xc -= downLimit * Ts; //���簴��0.1m/s ���½����㣬0.002s���ߵľ������0.0002;

				normalMotion_move[2] = Xc - temp_Xc;
				cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //�����迹���Ƶó����ƶ�����ϵ��Ҫ�ƶ��ľ��룬�����þ�������ת��Ϊ��������ϵ��

				//����λ��
				pose[0] += moveDistance[0];
				pose[1] += moveDistance[1];
				pose[2] += moveDistance[2];

				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				//qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//��ȡ����

				continue;
			}
			else
			{
				//�ϸ����ÿ��servoLָ��֮��ļ��ΪTs��
				timeb end;
				ftime(&end);//��ȡ����
				int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
				//qDebug() << "deta_t:" << deta_t << endl;
				if (deta_t < Ts * 1000) {
					Sleep(Ts * 1000 - deta_t);
				}
				Safe_servoL(pose, 0, 0, tForServo, 0.03, 300);
				ftime(&start);//��ȡ����
			}

			//��������
			pose = rtde_r->getActualTCPPose();

			//�迹�����㷨����
			auto temp_Xc = Xc;//��һʱ�̵�Xc
			Xc_dt = Xc_dt + Xc_ddt * Ts;
			Xc = Xc + Xc_dt * Ts;
			normalMotion_move[2] = Xc - temp_Xc;
			cv::Vec3d moveDistance = rotateVecToTarget({ normalMotion_move[0],normalMotion_move[1],normalMotion_move[2] }, RotMatMotion2Base); //�����迹���Ƶó�Ҫ�ƶ��ľ���

			//qDebug() << "Force :" << force[2] << endl;
			//qDebug() << "moveDistance :" << moveDistance[0] << moveDistance[1] << moveDistance[2] << endl;
			//qDebug() << "Xc :" << Xc << " Xc_dt:" << Xc_dt << " Xc_ddt:" << Xc_ddt << endl;
		
			if (force[2] >= Fd / 2) { // �½�״̬��������δ�ȶ�,�򲻼��㲹��������

				cv::Vec3d sixthAxis = getSixthAxisFromBasePose(rtde_r->getActualTCPPose());
				cv::Vec3d _targetVec = calculateSurfaceNormalVector(force, R, LengthOfSensor2MassageHeadCentre); //Ŀ���������ƶ�����ϵ�µı�ʾ
				vCompensation_array.push_back(_targetVec);
			}

			if (vCompensation_array.size() == count) { // �ռ�50�����������������ƽ��

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

			//�����������λ��
			pose[0] += moveDistance[0];
			pose[1] += moveDistance[1];
			pose[2] += moveDistance[2];

			// ���ݿ������ڼ�����ƶ�����Ӧ���ƶ��ľ��룬��ʵ�ϲ���̫׼ȷ�������˵�λ�ñջ������ڹ̶�ʱ���ڵ���ָ��λ�ã���������
			vector<double> direction_vector = get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(_moveUnitDirInWorld, { pose[3],pose[4],pose[5] });
			pose[0] += V * direction_vector[0] * Ts;
			pose[1] += V * direction_vector[1] * Ts;
			pose[2] += V * direction_vector[2] * Ts;

			//-------------------��ȡ�ƶ�����ϵ�µ���------------------------

			auto cur_pose = rtde_r->getActualTCPPose();

			sixth_axis = getSixthAxisFromBasePose(cur_pose);

			end_effector_direction = { _moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2] };

			force = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

			//-----------------------------------------------------------


			//----------------���������ϵ���ƶ�����ϵ����ת����----------------

			computeMotionCoordinateSystem(sixth_axis, end_effector_direction, { 0,0,1 }, x_axis, y_axis, z_axis, rtde_r);

			RotMatBase2Motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

			RotMatMotion2Base = RotMatBase2Motion.t();

			cv::Rodrigues(RotMatBase2Motion, RotVecBase2Motion);

			//-------------------------------------------------------------

			Fe = force[2];//���򻷾���
			��t = ��t + sigma * (Fd - Fe) / B;//����Ӧ����B ������
			Xc_ddt = Xe_ddt + 1 / M * (Fe - Fd - (B * (Xc_dt - Xe_dt) + (B * ��t + sigma * (Fd - Fe))) - K * (Xc - Xe));

		}
		rtde_c->servoStop();

	}

}


//------------------------------------------------�������ܺ���-------------------------------------------------

void Dy::Impedance_control::Safe_servoL(const std::vector<double>& pose, double speed, double acceleration, double time, double lookahead_time, double gain) {

	std::vector<double> pose_now = rtde_r->getActualTCPPose();
	double pose_count = pow((pow(pose_now[0] - pose[0], 2) + pow(pose_now[1] - pose[1], 2) + pow(pose_now[2] - pose[2], 2)), 0.5)*  1000;
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
	//qDebug() << "��:" << �� << '\t' << "pose_count:" << pose_count << endl;
	if (abs(��) < 10 && pose_count <15 ) {
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

	cv::Mat_<double> r_l = (cv::Mat_<double>(3, 1) << in_pose[3], in_pose[4], in_pose[5]);//��ת����
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
	cv::Vec3d end_direction_vector{ _end_direction_vector.at<double>(0,0),_end_direction_vector.at<double>(1,0) ,_end_direction_vector.at<double>(2,0) }; //ĩ������������ϵ��ʾ��������

	//double xita = pow(pow(rxryrz[0], 2)+pow(rxryrz[1], 2)+ pow(rxryrz[2], 2), 0.5);
	//cv::Mat r = (cv::Mat_<double>(3, 1) << rxryrz[0] / xita, rxryrz[1] / xita, rxryrz[2] / xita); //��λ����
	//cv::Mat p = (cv::Mat_<double>(3, 1) << P[0], P[1], P[2]);//��ʼp��tcp�µ����� 
	//cv::Mat new_P = cos(xita) * p + (1 - cos(xita)) * p.dot(r) + sin(xita) * r.cross(p);

	return { end_direction_vector[0],end_direction_vector[1],end_direction_vector[2]};
}

vector<double> Dy::Impedance_control::get_direction_vector_from_moveUnitDirInWorld_and_rxryrz(vector<double> _moveUnitDirInWorld, vector<double> rxryrz) {

	cv::Vec3d worldZdir{ 0,0,1 };
	cv::Vec3d kxita{rxryrz[0],rxryrz[1],rxryrz[2]};
	cv::Mat rotation_matrix;
	cv::Rodrigues(kxita,rotation_matrix);
	cv::Mat _end_direction_vector = rotation_matrix * worldZdir;
	cv::Vec3d end_direction_vector{_end_direction_vector.at<double>(0,0),_end_direction_vector.at<double>(1,0) ,_end_direction_vector.at<double>(2,0)}; //ĩ������������ϵ��ʾ��������

	//����������ϵ���ƶ��ĵ�λ������������������λ�ÿ�����ָ��ķ�������������ϵ��xoy���ͶӰ��Z��ͨ��Ϊ0����Ϊ�Ǹ������ظ���һ����ŵ��ƶ�����
	cv::Vec3d moveUnitDirInWorld{_moveUnitDirInWorld[0],_moveUnitDirInWorld[1],_moveUnitDirInWorld[2]};

	if (cv::norm(moveUnitDirInWorld) < 1e-8) return { 0,0,0 }; //���У����_moveUnitDirInWorldΪ0������ֱ�ӷ���0����

	double altered_mould_length = cv::norm(moveUnitDirInWorld);//��λ��
	moveUnitDirInWorld[0] /= altered_mould_length;
	moveUnitDirInWorld[1] /= altered_mould_length;
	moveUnitDirInWorld[2] /= altered_mould_length;

	//ĩ�����������ļнǣ���������������ᱨ��,������������Ҫ�޸�һ���ƶ���λ����
	double angle = acos(end_direction_vector.dot(moveUnitDirInWorld)); //����
	if (fabs(angle - PI) < 1e-7 || angle < 1e-7) { //angle Ϊ 0���� Ϊ PI��˵������

		//��ʱ��moveUnitDirInWorld����һ�£�ֱ������z��+1���У�Ȼ�����ص�λ����
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

vector<double> Dy::Impedance_control::get_RxRyRz_from_xitaX_and_xitaY(double ��x, double ��y) {
	
	vector<double> nowPose = rtde_r->getActualTCPPose();
	vector<double> rxryrz = { nowPose[3],nowPose[4],nowPose[5] };
	cv::Mat kxita = (cv::Mat_<double>(3, 1) << nowPose[3] , nowPose[4] , nowPose[5]);
	cv::Mat rotation_matrix; 
	cv::Rodrigues(kxita, rotation_matrix);
	cv::Mat Rx_��x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(��x), -sin(��x), 0, sin(��x), cos(��x));
	cv::Mat Ry_��y = (cv::Mat_<double>(3, 3) << cos(��y), 0, sin(��y), 0, 1, 0, -sin(��y), 0, cos(��y));
	rotation_matrix = rotation_matrix * Rx_��x * Ry_��y;
	cv::Vec3d new_rxryrz;
	cv::Rodrigues(rotation_matrix, new_rxryrz); //�õ��ƹ̶�����ϵ�µ�kxita�������������������ϵ�µõ����ƹ̶�����ϵ�µ���ת�����ǵȼ۵ģ������������Ҳ���ƹ̶�����ϵ�µľ���
	return { new_rxryrz[0],new_rxryrz[1],new_rxryrz[2] }; 
}

vector<double> Dy::Impedance_control::get_RxRyRz_from_xitaX_and_xitaY_and_xitaZ(double ��x, double ��y,double ��z) {
	
	vector<double> nowPose = rtde_r->getActualTCPPose();
	vector<double> rxryrz = { nowPose[3],nowPose[4],nowPose[5] };
	cv::Mat kxita = (cv::Mat_<double>(3, 1) << nowPose[3], nowPose[4], nowPose[5]);
	cv::Mat rotation_matrix;
	cv::Rodrigues(kxita, rotation_matrix);
	cv::Mat Rx_��x = getRotationMatrix(RotationAxis::X_AXIS, ��x);
	cv::Mat Ry_��y = getRotationMatrix(RotationAxis::Y_AXIS, ��y);
	cv::Mat Rz_��z = getRotationMatrix(RotationAxis::Z_AXIS, ��z);
	rotation_matrix = rotation_matrix * Rx_��x * Ry_��y * Rz_��z;
	cv::Vec3d new_rxryrz;
	cv::Rodrigues(rotation_matrix, new_rxryrz); //�õ��ƹ̶�����ϵ�µ�kxita�������������������ϵ�µõ����ƹ̶�����ϵ�µ���ת�����ǵȼ۵ģ������������Ҳ���ƹ̶�����ϵ�µľ���
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
//		// ��Ҫ�����ĵ���Ϊ�������򷵻�0����
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
		// ��Ҫ�����ĵ���Ϊ�������򷵻�0����
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
		// k�ٽ�������
		
		//��������������ӣ�

		double Mx_p = 0, My_p = 0, Mz_p = 0; 

		double cosAlpha = (-B + std::sqrt(discriminant)) / (A * R);

		double alpha = acos(cosAlpha);

		double cosBeta = (-My - Fx * R * cosAlpha - Fx * l) / (Fz * R * sin(alpha));

		double beta = acos(cosBeta);

		int k = 5; // ���� k = 5, ���Ը�����Ҫ�޸�

		double delta_alpha = 0.01; // ���Ĳ��������ݾ���������е���

		double delta_beta = 0.01; // �µĲ��������ݾ���������е���

		std::vector<double> alphas(k), betas(k);
		double min_result = std::numeric_limits<double>::max(); // ���ڴ洢��С���

		// �������п��ܵ�alpha_i��beta_j
		for (int i = 0; i < k; ++i) {
			alphas[i] = alpha + (i - (k + 1) / 2.0) * delta_alpha;
			betas[i] = beta + (i - (k + 1) / 2.0) * delta_beta;
		}

		// ����r(alpha_i, beta_j)����Сֵ
		for (int i = 0; i < k; ++i) {
			for (int j = 0; j < k; ++j) {
				// ��������ת���������
				double x = Fy * R * cos(alphas[i]) + Fz * R * sin(alphas[i]) * sin(betas[j]) + Fy * l;
				double y = -Fz * R * sin(alphas[i]) * cos(betas[j]) - Fx * R * cos(alphas[i]) - Fx * l;
				double z = -Fx * R * sin(alphas[i]) * sin(betas[j]) + Fy * R * sin(alphas[i]) * cos(betas[j]);

				// ����r(alpha_i, beta_j)
				double r = std::pow(Mx - x - Mx_p, 2) + std::pow(My - y - My_p, 2) + std::pow(Mz - z - Mz_p, 2);

				// ����Ƿ�����Сֵ
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

	//���ö���ϵͳ�ĳ�ֵ�Լ���������һЩ����
	double M = 1;
	double B = 1;
	double K = 1;
	double Fd = 10;
	double Ts = 0.002;
	double Vx = 0.01;
	double tForservo = 0.002;
	int mode = 1;//ģʽѡ��
	double downLimit = 0.035;//�½��ٶ����ƣ�����ٶȹ��죬��ʹ�ñ�������
	//--------------------------

	double threshold = 0.1;//ѵ�������������Сֵ
	double mostTimes = 10000;//ѵ����������


	// ����ָ����QT����Ĵ�������У���Dy_Control
	RTDEControlInterface* rtde_c = new RTDEControlInterface("192.168.55.101");
	RTDEReceiveInterface* rtde_r = new RTDEReceiveInterface("192.168.55.101");

	//����ʵ��������ʹ�ö�Ӧ��Ա����
	Impedance_control ic(M,B,K,new BpNet(),rtde_c,rtde_r);
	ic.getInit(threshold, mostTimes);//�������ݳ�ʼ��������
	double Xr = ic.getXr(Fd);

	//�����迹����
	ic.Normal_force_control(Fd, Xr, Ts,Vx,tForservo);	

	//�������ڵ�ǰλ�õ��迹����
	//ic.Normal_force_control_base_on_now(Fd, Ts, Vx, tForservo,mode,downLimit);
}