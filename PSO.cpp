#include "PSO.h"



//����������������������������������������������������������������������������������������Ⱥ�㷨��������������������������������������������������������������������������������//
/*

// ���캯��
PSOOptimizer::PSOOptimizer(PSOPara* pso_para, ComputeFitness fitness_fun)
{
	particle_num_ = pso_para->particle_num_;
	max_iter_num_ = pso_para->max_iter_num_;
	dim_ = pso_para->dim_;
	curr_iter_ = 0;

	dt_ = new double[dim_];
	wstart_ = new double[dim_];
	wend_ = new double[dim_];
	C1_ = new double[dim_];
	C2_ = new double[dim_];

	for (int i = 0; i < dim_; i++)
	{
		dt_[i] = pso_para->dt_[i];
		wstart_[i] = pso_para->wstart_[i];
		wend_[i] = pso_para->wend_[i];
		C1_[i] = pso_para->C1_[i];
		C2_[i] = pso_para->C2_[i];
	}

	if (pso_para->upper_bound_ && pso_para->lower_bound_)
	{
		upper_bound_ = new double[dim_];
		lower_bound_ = new double[dim_];
		range_interval_ = new double[dim_];

		for (int i = 0; i < dim_; i++)
		{
			upper_bound_[i] = pso_para->upper_bound_[i];
			lower_bound_[i] = pso_para->lower_bound_[i];
			//range_interval_[i] = pso_para.range_interval_[i];
			range_interval_[i] = upper_bound_[i] - lower_bound_[i];
		}
	}

	particles_ = new Particle[particle_num_];
	w_ = new double[dim_];
	all_best_position_ = new double[dim_];

	results_dim_ = pso_para->results_dim_;

	if (results_dim_)
	{
		results_ = new double[results_dim_];
	}

	fitness_fun_ = fitness_fun;
}

PSOOptimizer::~PSOOptimizer()
{
	if (particles_) { delete[]particles_; }
	if (upper_bound_) { delete[]upper_bound_; }
	if (lower_bound_) { delete[]lower_bound_; }
	if (range_interval_) { delete[]range_interval_; }
	if (dt_) { delete[]dt_; }
	if (wstart_) { delete[]wstart_; }
	if (wend_) { delete[]wend_; }
	if (w_) { delete[]w_; }
	if (C1_) { delete[]C1_; }
	if (C2_) { delete[]C2_; }
	if (all_best_position_) { delete[]all_best_position_; }
	if (results_) { delete[]results_; }
}

// ��ʼ����������
void PSOOptimizer::InitialAllParticles()
{
	// ��ʼ����һ�����Ӳ�������������ֵ
	InitialParticle(0);
	all_best_fitness_ = particles_[0].best_fitness_;
	for (int j = 0; j < dim_; j++)
	{
		all_best_position_[j] = particles_[0].best_position_[j];
	}

	// ��ʼ���������ӣ�����������ֵ
	for (int i = 1; i < particle_num_; i++)
	{
		InitialParticle(i);
#ifdef MAXIMIZE_FITNESS
		if (particles_[i].best_fitness_ > all_best_fitness_)
#else
		if (particles_[i].best_fitness_ < all_best_fitness_)
#endif
		{
			all_best_fitness_ = particles_[i].best_fitness_;
			for (int j = 0; j < dim_; j++)
			{
				all_best_position_[j] = particles_[i].best_position_[j];
			}

			// �����Ҫ�����һЩ���
			if (particles_[i].results_dim_ && results_dim_ == particles_[i].results_dim_)
			{
				for (int k = 0; k < results_dim_; k++)
				{
					results_[k] = particles_[i].results_[k];
				}
			}
			else if (results_dim_)
			{
				std::cout << "WARNING: the dimension of your saved results for every particle\nis not match with the dimension you specified for PSO optimizer ant no result is saved!" << std::endl;
			}
		}
	}
}

// ��ȡ˫���������
double PSOOptimizer::GetDoubleRand(int N)
{
	double temp = rand() % (N + 1) / (double)(N + 1);
	return temp;
}

double PSOOptimizer::GetFitness(Particle & particle)
{
	return fitness_fun_(particle);
}

void PSOOptimizer::UpdateAllParticles()
{
	GetInertialWeight();
	for (int i = 0; i < particle_num_; i++)
	{
		UpdateParticle(i);
#ifdef MAXIMIZE_FITNESS
		if (particles_[i].best_fitness_ > all_best_fitness_)
#else
		if (particles_[i].best_fitness_ < all_best_fitness_)
#endif
		{
			all_best_fitness_ = particles_[i].best_fitness_;
			for (int j = 0; j < dim_; j++)
			{
				all_best_position_[j] = particles_[i].best_position_[j];
			}
			
			// �����Ҫ�����һЩ����
			if (particles_[i].results_dim_ && results_dim_ == particles_[i].results_dim_)
			{
				for (int k = 0; k < results_dim_; k++)
				{
					results_[k] = particles_[i].results_[k];
				}
			}
			else if (results_dim_)
			{
				std::cout << "WARNING: the dimension of your saved results for every particle\nis not match with the dimension you specified for PSO optimizer ant no result is saved!" << std::endl;
			}
		}
	}
	curr_iter_++;
}

void PSOOptimizer::UpdateParticle(int i)
{
	// ���㵱ǰ������Ȩ��
	for (int j = 0; j < dim_; j++)
	{
		// ������һ�ε��������position��velocity
		//double last_velocity = particles_[i].velocity_[j];
		double last_position = particles_[i].position_[j];

		particles_[i].velocity_[j] = w_[j] * particles_[i].velocity_[j] +
			C1_[j] * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
			C2_[j] * GetDoubleRand() * (all_best_position_[j] - particles_[i].position_[j]);
		particles_[i].position_[j] += dt_[j] * particles_[i].velocity_[j];

		// �����������������������
		if (upper_bound_ && lower_bound_)
		{
			if (particles_[i].position_[j] > upper_bound_[j])
			{
				double thre = GetDoubleRand(99);
				if (last_position == upper_bound_[j])
				{
					particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = upper_bound_[j] - (upper_bound_[j] - last_position) * GetDoubleRand();
				}
				else
				{
					particles_[i].position_[j] = upper_bound_[j];
				}		
			}
			if (particles_[i].position_[j] < lower_bound_[j])
			{
				double thre = GetDoubleRand(99);
				if (last_position == lower_bound_[j])
				{
					particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = lower_bound_[j] + (last_position - lower_bound_[j]) * GetDoubleRand();
				}
				else
				{
					particles_[i].position_[j] = lower_bound_[j];
				}
			}
		}
	}
	particles_[i].fitness_ = GetFitness(particles_[i]);

#ifdef MAXIMIZE_FITNESS
	if (particles_[i].fitness_ > particles_[i].best_fitness_)
#else
	if (particles_[i].fitness_ < particles_[i].best_fitness_)
#endif
	{
		particles_[i].best_fitness_ = particles_[i].fitness_;
		for (int j = 0; j < dim_; j++)
		{
			particles_[i].best_position_[j] = particles_[i].position_[j];
		}
	}
}


void PSOOptimizer::GetInertialWeight()
{
	double temp = curr_iter_ / (double)max_iter_num_;
	temp *= temp;
	for (int i = 0; i < dim_; i++)
	{
		w_[i] = wstart_[i] - (wstart_[i] - wend_[i]) * temp;
	}
}


void PSOOptimizer::InitialParticle(int i)
{
	// Ϊÿ�����Ӷ�̬�����ڴ�
	particles_[i].position_ = new double[dim_];
	particles_[i].velocity_ = new double[dim_];
	particles_[i].best_position_ = new double[dim_];

	//if (results_dim_)
	//{
	//	particles_[i].results_ = new double[results_dim_];
	//}

	// ��ʼ��position/veloctiyֵ
	for (int j = 0; j < dim_; j++)
	{
		// if defines lower bound and upper bound
		if (range_interval_)
		{
			particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
			particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
			//std::cout << particles_[i].position_[j] << std::endl;
		}
		else
		{
			particles_[i].position_[j] = GetDoubleRand() * 2;
			particles_[i].velocity_[j] = GetDoubleRand() * 0.5;
		}
	}

	// ���ó�ʼ��������Ӧ��ֵ
	particles_[i].fitness_ = GetFitness(particles_[i]);

	for (int j = 0; j < dim_; j++)
	{
		particles_[i].best_position_[j] = particles_[i].position_[j];
	}
	particles_[i].best_fitness_ = particles_[i].fitness_;
}

// �˺���δ�õ�
Particle::Particle(int dim, double * position, double * velocity, double * best_position, double best_fitness)
{
	dim_ = dim;
	//position_ = new double[dim];
	//velocity_ = new double[dim];
	//best_position_ = new double[dim];
	position_ = position;
	velocity_ = velocity;
	best_position_ = best_position;
	best_fitness_ = best_fitness;
}


// ��Ϊ������Դ�ļ��϶����ˣ������������Ϊһ���ο�
////��Ӧ�Ⱥ���
//double FitnessFunction(Particle& particle)
//{
//	double x = particle.position_[0];
//	double y = particle.position_[1];
//	double temp = sqrt(x * x + y * y);
//	double fitness = sin(temp) / temp + exp(0.5 * cos(2 * PI * x) + 0.5 * cos(2 * PI * y)) - 2.71289;
//	return fitness;
//}



//PSOʹ�ð���
int testForPSO()
{
	// ����Ⱥ�Ż���������2Ϊ����ά�ȣ�true��ʾ������������
	PSOPara psopara(2, true);
	psopara.particle_num_ = 20;		// ���Ӹ���
	psopara.max_iter_num_ = 300;	// ����������
	psopara.dt_[0] = 1.0;			// ��һά���ϵ�ʱ�䲽��
	psopara.dt_[1] = 1.0;			// �ڶ�ά���ϵ�ʱ�䲽��
	psopara.wstart_[0] = 0.9;		// ��һά���ϵ���ʼȨ��ϵ��
	psopara.wstart_[1] = 0.9;		// �ڶ�ά���ϵ���ʼȨ��ϵ��
	psopara.wend_[0] = 0.4;			// ��һά���ϵ���ֹȨ��ϵ��
	psopara.wend_[1] = 0.4;			// �ڶ�ά���ϵ���ֹȨ��ϵ��
	psopara.C1_[0] = 1.49445;		// ��һά���ϵļ��ٶ�����
	psopara.C1_[1] = 1.49445;
	psopara.C2_[0] = 1.49445;		// �ڶ�ά���ϵļ��ٶ�����
	psopara.C2_[1] = 1.49445;

	// ��������������ޣ�������������
	psopara.lower_bound_[0] = -1.0;	// ��һά����������
	psopara.lower_bound_[1] = -1.0;	// �ڶ�ά����������
	psopara.upper_bound_[0] = 1.0;	// ��һά����������
	psopara.upper_bound_[1] = 1.0;	// �ڶ�ά����������


	PSOOptimizer psooptimizer(&psopara, FitnessFunction);

	std::srand((unsigned int)time(0));
	psooptimizer.InitialAllParticles();
	double fitness = psooptimizer.all_best_fitness_;
	double* result = new double[psooptimizer.max_iter_num_];

	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
	{
		psooptimizer.UpdateAllParticles();
		result[i] = psooptimizer.all_best_fitness_;
		std::cout << "��" << i << "�ε��������";
		std::cout << "x = " << psooptimizer.all_best_position_[0] << ", " << "y = " << psooptimizer.all_best_position_[1];
		std::cout << ", fitness = " << result[i] << std::endl;
	}
	system("pause");

	return 0;
}


*/


//����������������������������������������������������������������������������������������Ӧ����Ⱥ�㷨��������������������������������������������������������������������������������//
//
//
//// �������Ӧ�Ⱥ���
//double fitnessFunction(const std::vector<double>& x) {
//	double sum = 0.0;
//	for (double xi : x) {
//		sum += xi * xi; // ʾ����ƽ����
//	}
//	return sum;
//}
//
//// ��Ҫ�� APSO ����
//std::vector<double> FunAPSO(int N, int dim, double x_max, double x_min, int iterate_max) {
//	std::vector<double> c = { 2.0, 2.0 }; // ����ϵ�� c1, c2
//	std::vector<double> gBest_result(iterate_max, 0);
//	std::vector<Particle> particles(N);
//	std::vector<double> gBest(dim);
//	double gBest_fitness = std::numeric_limits<double>::max();
//
//	// ��ʼ�������������
//	std::random_device rd;
//	std::mt19937 gen(rd());
//	std::uniform_real_distribution<> dis(x_min, x_max);
//	std::uniform_real_distribution<> dis_vel(0.2 * x_min, 0.2 * x_max);
//
//	// ���ӳ�ʼ��
//	for (Particle& p : particles) {
//		p.position.resize(dim);
//		p.velocity.resize(dim);
//		p.pBest.resize(dim);
//
//		for (int d = 0; d < dim; ++d) {
//			p.position[d] = dis(gen);
//			p.velocity[d] = dis_vel(gen);
//			p.pBest[d] = p.position[d];
//		}
//
//		p.fitness = fitnessFunction(p.position);
//		if (p.fitness < gBest_fitness) {
//			gBest_fitness = p.fitness;
//			gBest = p.position;
//		}
//	}
//
//	// ����Ⱥ�Ż���ѭ��
//	for (int iter = 0; iter < iterate_max; ++iter) {
//		for (Particle& p : particles) {
//			// �����ٶȺ�λ��
//			for (int d = 0; d < dim; ++d) {
//				double r1 = dis(gen);
//				double r2 = dis(gen);
//				p.velocity[d] = c[0] * r1 * (p.pBest[d] - p.position[d]) + c[1] * r2 * (gBest[d] - p.position[d]);
//				p.position[d] += p.velocity[d];
//
//				// ȷ������λ�ñ����ڽ�����
//				p.position[d] = std::max(p.position[d], x_min);
//				p.position[d] = std::min(p.position[d], x_max);
//			}
//
//			// ������Ӧ�Ⱥ͸�������λ��
//			p.fitness = fitnessFunction(p.position);
//			if (p.fitness < fitnessFunction(p.pBest)) {
//				p.pBest = p.position;
//			}
//
//			// ����ȫ������λ��
//			if (p.fitness < gBest_fitness) {
//				gBest_fitness = p.fitness;
//				gBest = p.position;
//			}
//		}
//
//		// ��¼ÿ�ε�����������Ӧ��
//		gBest_result[iter] = gBest_fitness;
//	}
//
//	return gBest_result;
//}
//
//
////PSOʹ�ð���
//int testForPSO()
//{
//	int N = 50; // ��Ⱥ��С
//	int dim = 2; // �����ά��
//	double x_max = 5; // ��ռ���Ͻ�
//	double x_min = -5; // ��ռ���½�
//	int iterate_max = 100; // ����������
//
//	std::vector<double> result = FunAPSO(N, dim, x_max, x_min, iterate_max);
//	// ��ӡ������
//	return 0;
//}