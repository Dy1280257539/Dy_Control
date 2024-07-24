#pragma once
#include <stdlib.h>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>


//——————————————————————————————————————————粒子群算法————————————————————————————————————————//

/*

#define PI 3.1415926

// 适应度是越大越好还是越小越好
#define MINIMIZE_FITNESS
//#define MAXIMIZE_FITNESS

struct PSOPara
{
	int dim_;							// 参数维度（position和velocity的维度）
	int particle_num_;					// 粒子个数
	int max_iter_num_;					// 最大迭代次数

	double *dt_ = nullptr;							// 时间步长
	double *wstart_ = nullptr;						// 初始权重
	double *wend_ = nullptr;						// 终止权重
	double *C1_ = nullptr;							// 加速度因子
	double *C2_ = nullptr;							// 加速度因子

	double *upper_bound_ = nullptr;					// position搜索范围上限
	double *lower_bound_ = nullptr;					// position搜索范围下限
	double *range_interval_ = nullptr;				// position搜索区间长度
	
	int results_dim_ = 0;								// results的维度

	PSOPara(){}

	PSOPara(int dim, bool hasBound = false)
	{
		dim_ = dim;

		dt_ = new double[dim_];
		wstart_ = new double[dim_];
		wend_ = new double[dim_];
		C1_ = new double[dim_];
		C2_ = new double[dim_];
		if (hasBound)
		{
			upper_bound_ = new double[dim_];
			lower_bound_ = new double[dim_];
			range_interval_ = new double[dim_];
		}
	}

	// 析构函数：释放堆内存
	~PSOPara()
	{
		if (upper_bound_) { delete[]upper_bound_; }
		if (lower_bound_) { delete[]lower_bound_; }
		if (range_interval_) { delete[]range_interval_; }
		if (dt_) { delete[]dt_; }
		if (wstart_) { delete[]wstart_; }
		if (wend_) { delete[]wend_; }
		if (C1_) { delete[]C1_; }
		if (C2_) { delete[]C2_; }
	}
};

struct Particle
{
	int dim_;							// 参数维度（position和velocity的维度）
	double fitness_;
	double *position_ = nullptr;
	double *velocity_ = nullptr;

	double *best_position_ = nullptr;
	double best_fitness_;
	double *results_ = nullptr;			// 一些需要保存出的结果
	int results_dim_ = 0;				// results_的维度

	Particle(){}

	~Particle()
	{
		if (position_) { delete[]position_; }
		if (velocity_) { delete[]velocity_; }
		if (best_position_) { delete[]best_position_; }
		if (results_) { delete[]results_; }
	}

	Particle(int dim, double *position, double *velocity, double *best_position, double best_fitness);
};

typedef double(*ComputeFitness)(Particle& particle);

class PSOOptimizer
{
public:
	int particle_num_;					// 粒子个数
	int max_iter_num_;					// 最大迭代次数
	int curr_iter_;						// 当前迭代次数

	int dim_;							// 参数维度（position和velocity的维度）

	Particle *particles_ = nullptr;		// 所有粒子
	
	double *upper_bound_ = nullptr;					// position搜索范围上限
	double *lower_bound_ = nullptr;					// position搜索范围下限
	double *range_interval_ = nullptr;				// position搜索区间长度

	double *dt_ = nullptr;							// 时间步长
	double *wstart_ = nullptr;						// 初始权重
	double *wend_ = nullptr;						// 终止权重
	double *w_ = nullptr;							// 当前迭代权重
	double *C1_ = nullptr;							// 加速度因子
	double *C2_ = nullptr;							// 加速度因子

	double all_best_fitness_;						// 全局最优粒子的适应度值
	double *all_best_position_ = nullptr;			// 全局最优粒子的poistion
	double *results_ = nullptr;						// 一些需要保存出的结果
	int results_dim_ = 0;							// results的维度

	ComputeFitness fitness_fun_ = nullptr;			// 适应度函数

public:
	// 默认构造函数
	PSOOptimizer() {}

	// 构造函数
	PSOOptimizer(PSOPara* pso_para, ComputeFitness fitness_fun);
	
	// 析构函数
	~PSOOptimizer();

	// 初始化所有粒子参数
	void InitialAllParticles();

	// 初始化第i个粒子参数
	void InitialParticle(int i);

	// 获取双精度随机数（默认精度为0.0001）
	double GetDoubleRand(int N = 9999);

	// 计算该粒子的适应度值
	double GetFitness(Particle& particle);

	// 更新所有粒子参数
	void UpdateAllParticles();

	// 更新第i个粒子
	void UpdateParticle(int i);

	// 获取当前迭代的权重
	void GetInertialWeight();
};

double FitnessFunction(Particle& particle);


*/

//——————————————————————————————————————————自适应粒子群算法————————————————————————————————————————//



// 定义一个粒子结构
struct Particle {
	std::vector<double> position;
	std::vector<double> velocity;
	std::vector<double> pBest;
	double fitness;
};

// 假设的适应度函数
double fitnessFunction(const std::vector<double>& x) {
	double sum = 0.0;
	for (double xi : x) {
		sum += xi * xi; // 示例：平方和
	}
	return sum;
}

// 主要的 APSO 函数
std::vector<double> FunAPSO(int N, int dim, double x_max, double x_min, int iterate_max) {
	std::vector<double> c = { 2.0, 2.0 }; // 加速系数 c1, c2
	std::vector<double> gBest_result(iterate_max, 0);
	std::vector<Particle> particles(N);
	std::vector<double> gBest(dim);
	double gBest_fitness = std::numeric_limits<double>::max();

	// 初始化随机数生成器
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(x_min, x_max);
	std::uniform_real_distribution<> dis_vel(0.2 * x_min, 0.2 * x_max);

	// 粒子初始化
	for (Particle& p : particles) {
		p.position.resize(dim);
		p.velocity.resize(dim);
		p.pBest.resize(dim);

		for (int d = 0; d < dim; ++d) {
			p.position[d] = dis(gen);
			p.velocity[d] = dis_vel(gen);
			p.pBest[d] = p.position[d];
		}

		p.fitness = fitnessFunction(p.position);
		if (p.fitness < gBest_fitness) {
			gBest_fitness = p.fitness;
			gBest = p.position;
		}
	}

	// 粒子群优化主循环
	for (int iter = 0; iter < iterate_max; ++iter) {
		for (Particle& p : particles) {
			// 更新速度和位置
			for (int d = 0; d < dim; ++d) {
				double r1 = dis(gen);
				double r2 = dis(gen);
				p.velocity[d] = c[0] * r1 * (p.pBest[d] - p.position[d]) + c[1] * r2 * (gBest[d] - p.position[d]);
				p.position[d] += p.velocity[d];

				// 确保粒子位置保持在界限内
				p.position[d] = std::max(p.position[d], x_min);
				p.position[d] = std::min(p.position[d], x_max);
			}

			// 更新适应度和个体最优位置
			p.fitness = fitnessFunction(p.position);
			if (p.fitness < fitnessFunction(p.pBest)) {
				p.pBest = p.position;
			}

			// 更新全局最优位置
			if (p.fitness < gBest_fitness) {
				gBest_fitness = p.fitness;
				gBest = p.position;
			}
		}

		// 记录每次迭代的最优适应度
		gBest_result[iter] = gBest_fitness;
	}

	return gBest_result;
}