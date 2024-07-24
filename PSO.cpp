#include "PSO.h"



//——————————————————————————————————————————粒子群算法————————————————————————————————————————//
/*

// 构造函数
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

// 初始化所有粒子
void PSOOptimizer::InitialAllParticles()
{
	// 初始化第一个粒子参数并设置最优值
	InitialParticle(0);
	all_best_fitness_ = particles_[0].best_fitness_;
	for (int j = 0; j < dim_; j++)
	{
		all_best_position_[j] = particles_[0].best_position_[j];
	}

	// 初始化其他粒子，并更新最优值
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

			// 如果需要保存出一些结果
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

// 获取双精度随机数
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
			
			// 如果需要保存出一些参数
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
	// 计算当前迭代的权重
	for (int j = 0; j < dim_; j++)
	{
		// 保存上一次迭代结果的position和velocity
		//double last_velocity = particles_[i].velocity_[j];
		double last_position = particles_[i].position_[j];

		particles_[i].velocity_[j] = w_[j] * particles_[i].velocity_[j] +
			C1_[j] * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
			C2_[j] * GetDoubleRand() * (all_best_position_[j] - particles_[i].position_[j]);
		particles_[i].position_[j] += dt_[j] * particles_[i].velocity_[j];

		// 如果搜索区间有上下限限制
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
	// 为每个粒子动态分配内存
	particles_[i].position_ = new double[dim_];
	particles_[i].velocity_ = new double[dim_];
	particles_[i].best_position_ = new double[dim_];

	//if (results_dim_)
	//{
	//	particles_[i].results_ = new double[results_dim_];
	//}

	// 初始化position/veloctiy值
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

	// 设置初始化最优适应度值
	particles_[i].fitness_ = GetFitness(particles_[i]);

	for (int j = 0; j < dim_; j++)
	{
		particles_[i].best_position_[j] = particles_[i].position_[j];
	}
	particles_[i].best_fitness_ = particles_[i].fitness_;
}

// 此函数未用到
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


// 因为在其他源文件上定义了，所以这里仅作为一个参考
////适应度函数
//double FitnessFunction(Particle& particle)
//{
//	double x = particle.position_[0];
//	double y = particle.position_[1];
//	double temp = sqrt(x * x + y * y);
//	double fitness = sin(temp) / temp + exp(0.5 * cos(2 * PI * x) + 0.5 * cos(2 * PI * y)) - 2.71289;
//	return fitness;
//}



//PSO使用案例
int testForPSO()
{
	// 粒子群优化器参数：2为变量维度，true表示有搜索上下限
	PSOPara psopara(2, true);
	psopara.particle_num_ = 20;		// 粒子个数
	psopara.max_iter_num_ = 300;	// 最大迭代次数
	psopara.dt_[0] = 1.0;			// 第一维度上的时间步长
	psopara.dt_[1] = 1.0;			// 第二维度上的时间步长
	psopara.wstart_[0] = 0.9;		// 第一维度上的起始权重系数
	psopara.wstart_[1] = 0.9;		// 第二维度上的起始权重系数
	psopara.wend_[0] = 0.4;			// 第一维度上的终止权重系数
	psopara.wend_[1] = 0.4;			// 第二维度上的终止权重系数
	psopara.C1_[0] = 1.49445;		// 第一维度上的加速度因子
	psopara.C1_[1] = 1.49445;
	psopara.C2_[0] = 1.49445;		// 第二维度上的加速度因子
	psopara.C2_[1] = 1.49445;

	// 如果有搜索上下限，则设置上下限
	psopara.lower_bound_[0] = -1.0;	// 第一维度搜索下限
	psopara.lower_bound_[1] = -1.0;	// 第二维度搜索下限
	psopara.upper_bound_[0] = 1.0;	// 第一维度搜索上限
	psopara.upper_bound_[1] = 1.0;	// 第二维度搜索上限


	PSOOptimizer psooptimizer(&psopara, FitnessFunction);

	std::srand((unsigned int)time(0));
	psooptimizer.InitialAllParticles();
	double fitness = psooptimizer.all_best_fitness_;
	double* result = new double[psooptimizer.max_iter_num_];

	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
	{
		psooptimizer.UpdateAllParticles();
		result[i] = psooptimizer.all_best_fitness_;
		std::cout << "第" << i << "次迭代结果：";
		std::cout << "x = " << psooptimizer.all_best_position_[0] << ", " << "y = " << psooptimizer.all_best_position_[1];
		std::cout << ", fitness = " << result[i] << std::endl;
	}
	system("pause");

	return 0;
}


*/


//——————————————————————————————————————————自适应粒子群算法————————————————————————————————————————//
//
//
//// 假设的适应度函数
//double fitnessFunction(const std::vector<double>& x) {
//	double sum = 0.0;
//	for (double xi : x) {
//		sum += xi * xi; // 示例：平方和
//	}
//	return sum;
//}
//
//// 主要的 APSO 函数
//std::vector<double> FunAPSO(int N, int dim, double x_max, double x_min, int iterate_max) {
//	std::vector<double> c = { 2.0, 2.0 }; // 加速系数 c1, c2
//	std::vector<double> gBest_result(iterate_max, 0);
//	std::vector<Particle> particles(N);
//	std::vector<double> gBest(dim);
//	double gBest_fitness = std::numeric_limits<double>::max();
//
//	// 初始化随机数生成器
//	std::random_device rd;
//	std::mt19937 gen(rd());
//	std::uniform_real_distribution<> dis(x_min, x_max);
//	std::uniform_real_distribution<> dis_vel(0.2 * x_min, 0.2 * x_max);
//
//	// 粒子初始化
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
//	// 粒子群优化主循环
//	for (int iter = 0; iter < iterate_max; ++iter) {
//		for (Particle& p : particles) {
//			// 更新速度和位置
//			for (int d = 0; d < dim; ++d) {
//				double r1 = dis(gen);
//				double r2 = dis(gen);
//				p.velocity[d] = c[0] * r1 * (p.pBest[d] - p.position[d]) + c[1] * r2 * (gBest[d] - p.position[d]);
//				p.position[d] += p.velocity[d];
//
//				// 确保粒子位置保持在界限内
//				p.position[d] = std::max(p.position[d], x_min);
//				p.position[d] = std::min(p.position[d], x_max);
//			}
//
//			// 更新适应度和个体最优位置
//			p.fitness = fitnessFunction(p.position);
//			if (p.fitness < fitnessFunction(p.pBest)) {
//				p.pBest = p.position;
//			}
//
//			// 更新全局最优位置
//			if (p.fitness < gBest_fitness) {
//				gBest_fitness = p.fitness;
//				gBest = p.position;
//			}
//		}
//
//		// 记录每次迭代的最优适应度
//		gBest_result[iter] = gBest_fitness;
//	}
//
//	return gBest_result;
//}
//
//
////PSO使用案例
//int testForPSO()
//{
//	int N = 50; // 种群大小
//	int dim = 2; // 问题的维度
//	double x_max = 5; // 解空间的上界
//	double x_min = -5; // 解空间的下界
//	int iterate_max = 100; // 最大迭代次数
//
//	std::vector<double> result = FunAPSO(N, dim, x_max, x_min, iterate_max);
//	// 打印或处理结果
//	return 0;
//}