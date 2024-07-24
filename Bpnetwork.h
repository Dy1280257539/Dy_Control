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

#define INNODE 1  //���������
#define HIDENODE 4 //���ز�ڵ���
#define OUTNODE 1  //�����ڵ���
#define LEARNINGRATE 0.2  //ѧϰ��

/*
	Dy:�м���ע�����
	1.Ԥ�⼯�в������ж�Ļس�������������д�Ĵ���û�п��ǵ���һ��
	2.Ԥ�⼯ֻ�������룬���ұ���Ҫ��һ�����⣬�ҵĴ����аѱ��������
	3.����ȡ���ݵ��ļ���С�����仯���ǵøı䣬Ӧ���ڴ����112�����Ҹ�
*/

using std::vector;
using std::string;

//�������������
namespace Dy {


	//�������������
	extern std::default_random_engine e;

	//����ڵ�
	class inputNode {
	public:
		double value;//����ֵ
		std::vector<double> weight;//��Ȩֵ����Ӧw
		std::vector<double> wDeltaSum;//�ݶ�
	};

	//����ڵ�
	class outputNode {
	public:
		double value;//����ƫ�ƺͼ�����õ���ֵ
		double rightValue;//ʵ��ֵ
		double bias;//ƫ��ֵ
		double bDeltaSum;//ƫ���ݶ�
	};

	//���ؽڵ�
	class hiddenNode {
	public:
		double value;//����ƫ�ƺͼ�����õ���ֵ
		double bias;//һ���ڵ�һ��ƫ����
		vector<double> weight;//��Ȩֵ��Ӧww1
		vector<double> wDeltaSum; //�ж���������Ͱ������ǵĶ�������������������ݶ����ܺ�
		double bDeltaSum;//Ȩ�غ�ƫ���ݶ�
	};

	//����
	class sample {
	public:
		vector<double> in, out;//�õ���������
	};
	//Bp������
	class BpNet {
	public:
		BpNet();
		void fp();//ǰ������
		void bp();//���򴫲�
		void doTraining(vector<sample> trainGroup, double threshold, int mostTimes);//mostTimes��ѵ���������������trainGroup�Ǵ�����������threshold�������ֵ
		void doTesting(vector<sample>& testGroup);//���Եõ�������,
		void setInValue(vector<double> inValue); // ����ѧϰ��������
		void setOutRightValue(vector<double> outRightValue); // ����ѧϰ�������

		//��һ������
		vector<sample> normalisation(const vector<sample>& vsample, bool flag);//�����ݽ��й�һ��,flag��һ����ʾ ����Ϊ1��˵���Ƕ�ѵ������һ��������Ϊ0�ǶԲ��Լ���һ��
		vector<sample> denormalisation(const vector<sample>& vsampleAfter);//�����ݽ��з���һ��,ע�⣬sample�����������Ҫ�����ݲ���
		void setAllTag(const vector<sample>& vsample);//���ù�һ�����ĸ���ǩ
		//void outBpPara(const char * path );

	public://Ϊ�˷����Ŵ��㷨ֱ�ӷ��ʣ��������Ȩ������Ϊpublic
		double normalisation_formal(double source, double min, double max);//��һ����ʽ
		double denormalisation_formal(double after, double min, double max);//����һ����ʽ
		inputNode* inputLayer[INNODE];//�����
		outputNode* outputLayer[OUTNODE];//�����
		hiddenNode* hiddenLayer[HIDENODE];//���ز�
		double error;//���
		double error_only_abs_denormalisation;//���Ǿ���ֵ�����ƽ���������Ǿ���ֵƽ�������ƽ��������ʦ�ֲ��õķ���,���������Ƿ���һ�����ƽ��������

	public://Ϊ�˷����Ŵ��㷨ֱ�ӷ��ʣ��������Ȩ������Ϊpublic

		//�����ĸ������ǹ�һ���õ��Ķ�Ӧ�����е����ֵ����Сֵ
		vector<double> maxTagIn;//�������ֵ��ǩ
		vector<double> minTagIn;//������Сֵ��ǩ
		vector<double> maxTagOut;//������ֵ��ǩ
		vector<double> minTagOut;//�����Сֵ��ǩ
	};


	//��(-1.1)����һ�������Ǿ��ȷֲ���
	inline double getRandom() {
		static std::uniform_real_distribution<double> u(-10, 10);
		return (u(e));
	}
	//����� sigmoid
	inline double sigmoid(double x) {
		double ans = 1 / (1 + exp(-x));
		return ans;
	}

	inline double tansig(double x) {
		double ans = 2 / (1 + exp(-2*x)) - 1;
		return ans;
	}

}

//���ݻ�ȡ������������
namespace Dy
{
	//ע�⿴����޸�
	const int NUMCOL = 12;//NUMCOL�������ļ����������

	//��ȡĳ�е����ݣ�����һ��vector<string>ʵ��,�����ֱ���·������������Ҫ��ȡ���ݶ�Ӧ������
	vector<string> getdata(const char* file, int position);

	//����ȡ����vector<string>,ȥ�����⣬��������double��ʽչ�֣���vector<double>
	vector<double> data_to_double(const vector<string>& vdata);

	//��getdata��data_to_double һ��ʹ�ã�����ֱ�ӵõ���Ӧ�е�vector<double>������û�б���
	vector<double> getOnlyDouble(const char* file, int position);


	//���������������д�Ĵ������ݺ���--------------------------------------------------------------------------


	//��ȡѵ�������� һ��sample�����ж�����룬vector<sample>������sample�ļ���
	//������д�Ĵ���ָ��sample�����ֻ��һ��,��ֻ������outnode����Ϊ1������������Ƕ�Ӧ�Ŀɱ����ģ�壬
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


	//��ȡ���Լ�Ҳ��Ĭ�ϵ�csv�ļ�,numcol��������¼���Լ��ļ��ж����У�Ĭ�����һ������������csv�������⣬�����ڲ��ѱ���ȥ���ˣ�ֻ�������ݣ�
	vector<sample> getTestData(const char* path, const int& numcol);

	//��ȡԤ�⼯����������
	vector<sample> getPredData(const char* path);

	//�����Ժ�Ĳ��Լ�������ļ���ͬ��ֻ������sample���ֵֻ��һ������������ж��
	void outTestData(const char* path, const vector<sample>& vsample);

	//��ȡ���������ֵ�������´���
	void getInput(double& threshold, int& mostTimes);

	//�������Դ���
	void testForBp();


	//���������������д�Ĵ������ݺ���--------------------------------------------------------------------------
}
