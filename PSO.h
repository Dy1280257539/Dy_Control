#pragma once
#include <stdlib.h>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>


//����������������������������������������������������������������������������������������Ⱥ�㷨��������������������������������������������������������������������������������//

/*

#define PI 3.1415926

// ��Ӧ����Խ��Խ�û���ԽСԽ��
#define MINIMIZE_FITNESS
//#define MAXIMIZE_FITNESS

struct PSOPara
{
	int dim_;							// ����ά�ȣ�position��velocity��ά�ȣ�
	int particle_num_;					// ���Ӹ���
	int max_iter_num_;					// ����������

	double *dt_ = nullptr;							// ʱ�䲽��
	double *wstart_ = nullptr;						// ��ʼȨ��
	double *wend_ = nullptr;						// ��ֹȨ��
	double *C1_ = nullptr;							// ���ٶ�����
	double *C2_ = nullptr;							// ���ٶ�����

	double *upper_bound_ = nullptr;					// position������Χ����
	double *lower_bound_ = nullptr;					// position������Χ����
	double *range_interval_ = nullptr;				// position�������䳤��
	
	int results_dim_ = 0;								// results��ά��

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

	// �����������ͷŶ��ڴ�
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
	int dim_;							// ����ά�ȣ�position��velocity��ά�ȣ�
	double fitness_;
	double *position_ = nullptr;
	double *velocity_ = nullptr;

	double *best_position_ = nullptr;
	double best_fitness_;
	double *results_ = nullptr;			// һЩ��Ҫ������Ľ��
	int results_dim_ = 0;				// results_��ά��

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
	int particle_num_;					// ���Ӹ���
	int max_iter_num_;					// ����������
	int curr_iter_;						// ��ǰ��������

	int dim_;							// ����ά�ȣ�position��velocity��ά�ȣ�

	Particle *particles_ = nullptr;		// ��������
	
	double *upper_bound_ = nullptr;					// position������Χ����
	double *lower_bound_ = nullptr;					// position������Χ����
	double *range_interval_ = nullptr;				// position�������䳤��

	double *dt_ = nullptr;							// ʱ�䲽��
	double *wstart_ = nullptr;						// ��ʼȨ��
	double *wend_ = nullptr;						// ��ֹȨ��
	double *w_ = nullptr;							// ��ǰ����Ȩ��
	double *C1_ = nullptr;							// ���ٶ�����
	double *C2_ = nullptr;							// ���ٶ�����

	double all_best_fitness_;						// ȫ���������ӵ���Ӧ��ֵ
	double *all_best_position_ = nullptr;			// ȫ���������ӵ�poistion
	double *results_ = nullptr;						// һЩ��Ҫ������Ľ��
	int results_dim_ = 0;							// results��ά��

	ComputeFitness fitness_fun_ = nullptr;			// ��Ӧ�Ⱥ���

public:
	// Ĭ�Ϲ��캯��
	PSOOptimizer() {}

	// ���캯��
	PSOOptimizer(PSOPara* pso_para, ComputeFitness fitness_fun);
	
	// ��������
	~PSOOptimizer();

	// ��ʼ���������Ӳ���
	void InitialAllParticles();

	// ��ʼ����i�����Ӳ���
	void InitialParticle(int i);

	// ��ȡ˫�����������Ĭ�Ͼ���Ϊ0.0001��
	double GetDoubleRand(int N = 9999);

	// ��������ӵ���Ӧ��ֵ
	double GetFitness(Particle& particle);

	// �����������Ӳ���
	void UpdateAllParticles();

	// ���µ�i������
	void UpdateParticle(int i);

	// ��ȡ��ǰ������Ȩ��
	void GetInertialWeight();
};

double FitnessFunction(Particle& particle);


*/

//����������������������������������������������������������������������������������������Ӧ����Ⱥ�㷨��������������������������������������������������������������������������������//



// ����һ�����ӽṹ
struct Particle {
	std::vector<double> position;
	std::vector<double> velocity;
	std::vector<double> pBest;
	double fitness;
};

// �������Ӧ�Ⱥ���
double fitnessFunction(const std::vector<double>& x) {
	double sum = 0.0;
	for (double xi : x) {
		sum += xi * xi; // ʾ����ƽ����
	}
	return sum;
}

// ��Ҫ�� APSO ����
std::vector<double> FunAPSO(int N, int dim, double x_max, double x_min, int iterate_max) {
	std::vector<double> c = { 2.0, 2.0 }; // ����ϵ�� c1, c2
	std::vector<double> gBest_result(iterate_max, 0);
	std::vector<Particle> particles(N);
	std::vector<double> gBest(dim);
	double gBest_fitness = std::numeric_limits<double>::max();

	// ��ʼ�������������
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(x_min, x_max);
	std::uniform_real_distribution<> dis_vel(0.2 * x_min, 0.2 * x_max);

	// ���ӳ�ʼ��
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

	// ����Ⱥ�Ż���ѭ��
	for (int iter = 0; iter < iterate_max; ++iter) {
		for (Particle& p : particles) {
			// �����ٶȺ�λ��
			for (int d = 0; d < dim; ++d) {
				double r1 = dis(gen);
				double r2 = dis(gen);
				p.velocity[d] = c[0] * r1 * (p.pBest[d] - p.position[d]) + c[1] * r2 * (gBest[d] - p.position[d]);
				p.position[d] += p.velocity[d];

				// ȷ������λ�ñ����ڽ�����
				p.position[d] = std::max(p.position[d], x_min);
				p.position[d] = std::min(p.position[d], x_max);
			}

			// ������Ӧ�Ⱥ͸�������λ��
			p.fitness = fitnessFunction(p.position);
			if (p.fitness < fitnessFunction(p.pBest)) {
				p.pBest = p.position;
			}

			// ����ȫ������λ��
			if (p.fitness < gBest_fitness) {
				gBest_fitness = p.fitness;
				gBest = p.position;
			}
		}

		// ��¼ÿ�ε�����������Ӧ��
		gBest_result[iter] = gBest_fitness;
	}

	return gBest_result;
}