#pragma once
#include"Bpnetwork.h"


//�Ŵ��㷨�������
namespace Dy{
	//������Ⱦɫ��
	class individual {
	public:
		vector<double> chromo;//Ⱦɫ��
		double fit;//��Ӧ��
		double prob;//ѡ�еĸ���
	};

	extern const int popSize; //��Ⱥ��С
	extern const int len;//���볤��
	extern const double Pc;//�������
	extern const double Pm;//�������
	extern const int generation;//��������
	extern individual pop[];//��Ⱥ
	extern individual nextPop[];//��һ����Ⱥ
	extern int nextPopNum;//���ڼ�¼��һ����Ⱥ�ĸ���
	extern double lineProb[];//�������̶��㷨������ѡ�����
	extern double totalFit; //����Ӧ��
	extern int gen;//��¼��ǰ����

	//����Bpnetwork�е����������,Bpnetwork����һ����������棡
	//extern default_random_engine e;


	/*
	���һ��Ⱦɫ�壬�������������������Ȩ�أ���˳�����ȵ�һ����Ԫ��hidenode��Ȩֵ��Ȼ��ڶ������������ƣ���
	���ز��ƫ�ã����ز��Ȩ�أ���˳�򣩣�������ƫ��
	*/
	vector<double> getChromo(const BpNet& bpnet);
	
	//������������������Ӧ�ȣ����ʶ�ֵ����bpnet��error�ĵ���,����������������
	double getFitness(individual in, BpNet bpnet, const vector<sample>& vsample);
	
	//��ȡ����Ӧ��
	double getTotalFit(individual pop[]);
	
	//��������ѡ�����
	double getProb(individual in);
	
	//�����ȡȾɫ��ֵ
	vector<double> getRanChromo();
	
	//��ʼ����Ⱥ,�ȸ�����Ⱥ�����������Ⱦɫ�壬�ٸ�����������������Ӧ�ȣ��ٸ�������Ӧ�ȼ������ѡ�����
	void getInitPop(individual pop[], const vector<sample>& vsample);
	
	//�жϸ����Ƿ�Ϸ�����ʱ��д
	//bool inlegality(individual);
	
	//���̶�ѡ�����
	void selection(individual[]);
	
	//Ⱦɫ�彻��
	void crossover(individual[], BpNet bpnet, const vector<sample>& vsample);
	
	//Ⱦɫ����죬�ҵı��췽ʽ��ȡ���ұ�Ϊԭ����0��2��
	void mutataion(individual pop[], BpNet bpnet, const vector<sample>& vsample);

	//����һ�����Ǳ��죬���棬ѡ����ۺ�
	void evolution(BpNet bpnet, const vector<sample>& vsample);
	
	//Ѱ����Ⱥ����Ӧ����ߵ�һ��������
	int findBest(individual[]);
	
	//����Ⱥ����Ӧ����ߵĸ�����н��봫��������
	void chromoDecode(individual in,BpNet& bpnet);

	//��������
	void testForGABP();

}
