#pragma once
#include"Bpnetwork.h"


//遗传算法相关内容
namespace Dy{
	//建立单染色体
	class individual {
	public:
		vector<double> chromo;//染色体
		double fit;//适应度
		double prob;//选中的概率
	};

	extern const int popSize; //种群大小
	extern const int len;//编码长度
	extern const double Pc;//交叉概率
	extern const double Pm;//变异概率
	extern const int generation;//迭代次数
	extern individual pop[];//种群
	extern individual nextPop[];//下一代种群
	extern int nextPopNum;//用于记录下一代种群的个数
	extern double lineProb[];//用于轮盘赌算法的线性选择概率
	extern double totalFit; //总适应度
	extern int gen;//记录当前代数

	//引入Bpnetwork中的随机数引擎,Bpnetwork中有一个随机数引擎！
	//extern default_random_engine e;


	/*
	获得一条染色体，其上面的数据是输入层的权重（按顺序，如先第一个神经元的hidenode个权值，然后第二个，依此类推），
	隐藏层的偏置，隐藏层的权重（按顺序），输出层的偏置
	*/
	vector<double> getChromo(const BpNet& bpnet);
	
	//根据神经网络计算个体适应度，舒适度值采用bpnet中error的倒数,第三个参数是样本
	double getFitness(individual in, BpNet bpnet, const vector<sample>& vsample);
	
	//获取总适应度
	double getTotalFit(individual pop[]);
	
	//计算个体的选择概率
	double getProb(individual in);
	
	//随机获取染色体值
	vector<double> getRanChromo();
	
	//初始化种群,先根据总群数量随机分配染色体，再根据神经网络计算个体适应度，再根据总适应度计算个体选择概率
	void getInitPop(individual pop[], const vector<sample>& vsample);
	
	//判断个体是否合法，暂时不写
	//bool inlegality(individual);
	
	//轮盘赌选择个体
	void selection(individual[]);
	
	//染色体交叉
	void crossover(individual[], BpNet bpnet, const vector<sample>& vsample);
	
	//染色体变异，我的变异方式是取反且变为原来的0到2倍
	void mutataion(individual pop[], BpNet bpnet, const vector<sample>& vsample);

	//进化一代，是变异，交叉，选择的综合
	void evolution(BpNet bpnet, const vector<sample>& vsample);
	
	//寻找种群中适应度最高的一个的索引
	int findBest(individual[]);
	
	//将种群中适应度最高的个体进行解码传给神经网络
	void chromoDecode(individual in,BpNet& bpnet);

	//测试用例
	void testForGABP();

}
