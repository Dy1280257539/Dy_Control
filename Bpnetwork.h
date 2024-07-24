#pragma once
#include<iostream>
#include<cmath>
#include<vector>
#include <random>
#include<algorithm>
#include <fstream>
#include <sstream>
#include<qdebug.h>
#include <sys/timeb.h>

#define INNODE 1  //输入结点个数
#define HIDENODE 4 //隐藏层节点数
#define OUTNODE 1  //输出层节点数
#define LEARNINGRATE 0.2  //学习率

/*
	Dy:有几个注意事项：
	1.预测集中不能留有多的回车，否则会出错，我写的代码没有考虑到这一点
	2.预测集只能有输入，而且必须要有一个标题，我的代码中把标题读掉了
	3.当读取数据的文件大小发生变化，记得改变，应该在代码的112行左右改
*/

using std::vector;
using std::string;

//神经网络相关内容
namespace Dy {


	//建立随机数引擎
	extern std::default_random_engine e;

	//输入节点
	class inputNode {
	public:
		double value;//输入值
		std::vector<double> weight;//加权值，对应w
		std::vector<double> wDeltaSum;//梯度
	};

	//输出节点
	class outputNode {
	public:
		double value;//经过偏移和激活函数得到的值
		double rightValue;//实际值
		double bias;//偏移值
		double bDeltaSum;//偏移梯度
	};

	//隐藏节点
	class hiddenNode {
	public:
		double value;//经过偏移和激活函数得到的值
		double bias;//一个节点一个偏移量
		vector<double> weight;//加权值对应ww1
		vector<double> wDeltaSum; //有多个样本，就把样本们的都加起来，所以这里的梯度是总和
		double bDeltaSum;//权重和偏移梯度
	};

	//样本
	class sample {
	public:
		vector<double> in, out;//得到的样本集
	};
	//Bp神经网络
	class BpNet {
	public:
		BpNet();
		void fp();//前往传播
		void bp();//后向传播
		void doTraining(vector<sample> trainGroup, double threshold, int mostTimes);//mostTimes是训练允许的最大次数，trainGroup是传进的样本，threshold是误差阈值
		void doTesting(vector<sample>& testGroup);//测试得到的数据,
		void setInValue(vector<double> inValue); // 设置学习样本输入
		void setOutRightValue(vector<double> outRightValue); // 设置学习样本输出

		//归一化操作
		vector<sample> normalisation(const vector<sample>& vsample, bool flag);//对数据进行归一化,flag是一个表示 当其为1，说明是对训练集归一化，当其为0是对测试集归一化
		vector<sample> denormalisation(const vector<sample>& vsampleAfter);//对数据进行反归一化,注意，sample的输入输出都要有数据才行
		void setAllTag(const vector<sample>& vsample);//设置归一化的四个标签
		//void outBpPara(const char * path );

	public://为了方便遗传算法直接访问，这里访问权限设置为public
		double normalisation_formal(double source, double min, double max);//归一化公式
		double denormalisation_formal(double after, double min, double max);//反归一化公式
		inputNode* inputLayer[INNODE];//输入层
		outputNode* outputLayer[OUTNODE];//输出层
		hiddenNode* hiddenLayer[HIDENODE];//隐藏层
		double error;//误差
		double error_only_abs_denormalisation;//考虑绝对值求和求平均，而不是绝对值平方求和求平均，这是师兄采用的方法,而且这里是反归一化后的平均拟合误差

	public://为了方便遗传算法直接访问，这里访问权限设置为public

		//下面四个数组是归一化用到的对应样本列的最大值与最小值
		vector<double> maxTagIn;//输入最大值标签
		vector<double> minTagIn;//输入最小值标签
		vector<double> maxTagOut;//输出最大值标签
		vector<double> minTagOut;//输出最小值标签
	};


	//从(-1.1)生成一个数，是均匀分布的
	inline double getRandom() {
		static std::uniform_real_distribution<double> u(-10, 10);
		return (u(e));
	}
	//激活函数 sigmoid
	inline double sigmoid(double x) {
		double ans = 1 / (1 + exp(-x));
		return ans;
	}

	inline double tansig(double x) {
		double ans = 2 / (1 + exp(-2*x)) - 1;
		return ans;
	}

}

//数据获取输出等相关内容
namespace Dy
{
	//注意看情况修改
	const int NUMCOL = 12;//NUMCOL是数据文件的最大列数

	//获取某列的数据，将以一个vector<string>实现,参数分别是路径，容器和需要获取数据对应的列数
	vector<string> getdata(const char* file, int position);

	//将获取到的vector<string>,去掉标题，其他都以double形式展现，即vector<double>
	vector<double> data_to_double(const vector<string>& vdata);

	//将getdata和data_to_double 一起使用，可以直接得到对应列的vector<double>，其中没有标题
	vector<double> getOnlyDouble(const char* file, int position);


	//以下是针对神经网络写的处理数据函数--------------------------------------------------------------------------


	//获取训练样本集 一个sample可能有多个输入，vector<sample>代表多个sample的集合
	//这里我写的代码指定sample的输出只有一个,即只能适配outnode个数为1的情况，下面是对应的可变参数模板，
	template<typename...Args>
	vector<sample> getTrainData(const vector<double>& input, Args...args)
	{
		vector<sample> vsample(input.size());
		auto iinput = input.begin();
		for (auto it = vsample.begin(); it != vsample.end(); it++)
		{
			it->in.push_back(*iinput);
			iinput++;
		}
		getTrainData(vsample, args...);
		return vsample;
	}
	template<typename...Args>
	void getTrainData(vector<sample>& vsample, const vector<double>& input, Args...args)
	{
		auto iinput = input.begin();
		for (auto it = vsample.begin(); it != vsample.end(); it++)
		{
			it->in.push_back(*iinput);
			iinput++;
		}
		getTrainData(vsample, args...);
		return;
	}
	void getTrainData(vector<sample>& vsample, const vector<double>& output);


	//获取测试集也是默认的csv文件,numcol是用来记录测试集文件有多少列，默认最后一列是输出（这个csv包含标题，函数内部把标题去掉了，只保留数据）
	vector<sample> getTestData(const char* path, const int& numcol);

	//获取预测集的输入样本
	vector<sample> getPredData(const char* path);

	//将测试后的测试集输出到文件，同样只能用于sample输出值只有一个，输入可以有多个
	void outTestData(const char* path, const vector<sample>& vsample);

	//获取误差期望阈值和最大更新次数
	void getInput(double& threshold, int& mostTimes);

	//用例测试代码
	void testForBp();


	//以上是针对神经网络写的处理数据函数--------------------------------------------------------------------------
}
