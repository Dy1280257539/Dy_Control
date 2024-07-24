#include"Genetic_algorithms.h"

namespace Dy {
	const int popSize = 500; //��Ⱥ��С
	const int len = INNODE * HIDENODE + HIDENODE + HIDENODE * OUTNODE + OUTNODE;//���볤��
	const double Pc = 0.8;//�������
	const double Pm = 0.2;//�������
	const int generation = 100;//��������
	individual pop[popSize];//��Ⱥ
	individual nextPop[popSize];//��һ����Ⱥ
	int nextPopNum = 0;//���ڼ�¼��һ����Ⱥ�ĸ���
	double lineProb[popSize];//�������̶��㷨������ѡ�����
	double totalFit; //����Ӧ��
	int gen = 1;//��¼��ǰ����
}


//���һ��Ⱦɫ�壬�������������������Ȩ�أ���˳�����ȵ�һ����Ԫ��hidenode��Ȩֵ��Ȼ��ڶ������������ƣ�
//�����ز��ƫ�ã����ز��Ȩ�أ���˳�򣩣�������ƫ��
vector<double> Dy::getChromo(const BpNet& bpnet)
{
	vector<double> vdouble;
	for (int i = 0; i < INNODE; i++)
	{
		for (int j = 0; j < HIDENODE; j++)
		{
			vdouble.push_back(bpnet.inputLayer[i]->weight[j]);
		}
	}
	for (int i = 0; i < HIDENODE; i++)
	{
		vdouble.push_back(bpnet.hiddenLayer[i]->bias);
	}
	for (int i = 0; i < HIDENODE; i++)
	{
		for (int j = 0; j < OUTNODE; j++)
		{
			vdouble.push_back(bpnet.hiddenLayer[i]->weight[j]);
		}
	}
	for (int i = 0; i < OUTNODE; i++)
	{
		vdouble.push_back(bpnet.outputLayer[i]->bias);
	}
	return vdouble;
}

//������������������Ӧ��
double Dy::getFitness(individual in,BpNet  bpnet,const vector<sample>& vsample)
{
	//�����������Ȩ�أ��ٽ�in�ڵ�ѹ��
	for (int i = 0; i < INNODE; i++)
	{
		for (int j = 0; j < HIDENODE; j++)
		{
			bpnet.inputLayer[i]->weight.clear();
		}
	}
	for (int i = 0; i < INNODE; i++)
	{
		for (int j = 0; j < HIDENODE; j++)
		{
			//��Ⱦɫ���һ����Աѹ��weight�������
			bpnet.inputLayer[i]->weight.push_back(in.chromo.front());
			in.chromo.erase(in.chromo.begin());
		}
	}
	//ͬ�������ز��Ȩ���������ѹ��in�ڵ����ݣ��Լ�ѹ��ƫ��
	for (int i = 0; i < HIDENODE; i++)
	{
		for (int j = 0; j < OUTNODE; j++)
		{
			bpnet.hiddenLayer[i]->weight.clear();
		}
	}
	//�޸����ز�ƫ��
	for (int i = 0; i < HIDENODE; i++)
	{
		bpnet.hiddenLayer[i]->bias = in.chromo.front();
		in.chromo.erase(in.chromo.begin());
	}
	for (int i = 0; i < HIDENODE; i++)
	{
		for (int j = 0; j < OUTNODE; j++)
		{
			bpnet.hiddenLayer[i]->weight.push_back(in.chromo.front());
			in.chromo.erase(in.chromo.begin());
		}
	}
	//��������ƫ������ΪinȾɫ������ݣ�����ͬ��
	for (int i = 0; i < OUTNODE; i++)
	{
		bpnet.outputLayer[i]->bias = in.chromo.front();
		in.chromo.erase(in.chromo.begin());
	}
	int sampleNum = vsample.size();
	//��Ӧ�ȴ��������ߵ�����һ���ĵ�����ѡ��
	bpnet.error = 0.f; 
	//bpnet.error_only_abs_denormalisation = 0.f;
	//bpnet.setAllTag(vsample);
	for (int i = 0; i < sampleNum; i++)
	{
		bpnet.setInValue(vsample[i].in);//�趨��Ӧ���������������ֵ
		bpnet.setOutRightValue(vsample[i].out);
		bpnet.fp();//�����Ӧ�����ز�����������
		//�ۼ�error
		for (int i = 0; i < OUTNODE; i++)
		{
			double tmpe = fabs(bpnet.outputLayer[i]->value - bpnet.outputLayer[i]->rightValue);//�����ֵ
			bpnet.error += tmpe * tmpe / 2;//���
			//double tmpe1 = fabs(bpnet.denormalisation_formal(bpnet.outputLayer[i]->value, bpnet.minTagOut[i], bpnet.maxTagOut[i]) - bpnet.denormalisation_formal(bpnet.outputLayer[i]->rightValue, bpnet.minTagOut[i], bpnet.maxTagOut[i]));
			//bpnet.error_only_abs_denormalisation += tmpe1;
		}
	}  
	
	return 1 / (bpnet.error);

}
//�����ȡȾɫ��
vector<double> Dy::getRanChromo()
{
	vector<double> chromo;
	for (int i = 0; i < len; i++)
	{
		chromo.push_back(getRandom());
	}
	return chromo;
}
double Dy::getTotalFit(individual pop[])
{
	double totalFit = 0;
	for (int i = 0; i < popSize; i++)
		totalFit += pop[i].fit;
	return totalFit;
}

double Dy::getProb(individual in)
{
	double prob = 0;
	prob = in.fit / totalFit;
	return prob;
}

//��ʼ����Ⱥ
void Dy::getInitPop(individual pop[],const vector<sample> & vsample)
{
	vector<double> chromo;//Ⱦɫ��
	BpNet bpnet;
	totalFit = 0;//�������ʶ�ֵ
	for (int i = 0; i < popSize; i++)
	{
		chromo = getRanChromo();
		pop[i].chromo = chromo;
		pop[i].fit = getFitness(pop[i], bpnet, vsample);
	}
	totalFit = getTotalFit(pop);
	for (int i = 0; i < popSize; i++)
	{
		pop[i].prob = getProb(pop[i]);
	}
	return;
}
void Dy::selection(individual[])
{
	lineProb[0] = pop->prob;
	for (int i = 1; i < popSize; i++)//�ۼ���Ⱥ����
	{
		lineProb[i] = lineProb[i - 1] + pop[i].prob;
	}
	std::uniform_real_distribution<double> u(0, 1);
	double r = u(e);//��ȡһ��0��1�������;
	for (int i = 0; i < popSize; i++)
	{
		if (r<=lineProb[i])
		{
			nextPop[nextPopNum] = pop[i];
			nextPopNum++;
			break;
		}
	}
}
void Dy::crossover(individual[],BpNet bpnet,const vector<sample> & vsample)
{
	std::uniform_real_distribution<double> u(0, 1);
	double r1 = u(e);//��ȡһ��0��1�������;
	//�����̶ĵķ�ʽ
	if (r1 <= Pc) //��������
	{
		//ѡ���Ҫ���н������������
		int r2 = (int)fabs((int)e()) % popSize;
		int r3 = (int)fabs((int)e()) % popSize;
		//ѡ�������Ⱦɫ�����ʼλ��
		int r4 = (int)fabs((int)e()) % len;
		if (nextPopNum + 1 > popSize - 1)//��ǰ����Ⱥֻʣ��һ��λ��,��ֻ����һ�Σ�����һ���¸���
		{
			//qDebug() << "ֻ����һ�Σ�����һ���¸���" << endl;	
			nextPop[nextPopNum] = pop[r2];
			//��ʼ����
			for (int i = r4; i < len; i++)
			{
				nextPop[nextPopNum].chromo[i] = pop[r3].chromo[i];
			}
			nextPop[nextPopNum].fit = getFitness(nextPop[nextPopNum], bpnet, vsample);
			nextPopNum++;
		}
		else //����Ⱥ�п�λ���ڵ���2������һ�β��������¸���
		{
			//qDebug() << "  ����һ�Σ����������¸���" << endl;
			nextPop[nextPopNum] = pop[r2];
			nextPopNum++;
			nextPop[nextPopNum] = pop[r3];
			for (int i = r4; i < len; i++)
			{
				auto temp1 = nextPop[nextPopNum - 1].chromo[i];
				nextPop[nextPopNum - 1].chromo[i] = nextPop[nextPopNum].chromo[i];
				nextPop[nextPopNum].chromo[i] = temp1;
			}
			nextPop[nextPopNum - 1].fit = getFitness(nextPop[nextPopNum - 1], bpnet, vsample);
			nextPop[nextPopNum].fit = getFitness(nextPop[nextPopNum], bpnet, vsample);
			nextPopNum++;//�淶��Ⱥ����
		}
	}
	else;
		//qDebug() << "  �˴�δ���н���" << endl;

}
//Ⱦɫ�����
void Dy::mutataion(individual pop[], BpNet bpnet, const vector<sample>& vsample)
{
	std::uniform_real_distribution<double> u(0, 1);
	double m1;
	int m2,m3;
	m1 = u(e);
	if (nextPopNum<popSize)//��Ⱥ����λ��ʱ
	{
		if (m1 < Pm)
		{
			//qDebug() << "  ����һ�Σ�����һ���¸���" << endl;
			m2 = (int)fabs((int)e()) % popSize;//����ĸ���
			m3 = (int)fabs((int)e()) % len;//�����Ⱦɫ��λ��
			nextPop[nextPopNum] = pop[m2];
			double tmpe1 = pop[m2].chromo[m3];
			//���췽ʽ��ȡ���ұ�Ϊԭ����0��10��
			nextPop[nextPopNum].chromo[m3] = -10 * u(e) * tmpe1;
			nextPop[nextPopNum].fit = getFitness(nextPop[nextPopNum], bpnet, vsample);
			nextPopNum++;
		}
		else;
			//qDebug() << "  �˴�δ���б���" << endl;
	}
	return;
}
//����
void Dy::evolution(BpNet bpnet,const vector<sample> & vsample)
{
	gen++;
	qDebug() << QStringLiteral("��") << gen -1<< QStringLiteral("����Ⱥ��ʼ����" )<< endl;
	while (nextPopNum < popSize)
	{
		selection(pop);
		if (nextPopNum == popSize) break;
		crossover(pop,bpnet,vsample);
		if (nextPopNum == popSize) break;
		mutataion(pop, bpnet, vsample);
		if (nextPopNum == popSize) break;
	}
	totalFit = getTotalFit(nextPop);
	qDebug() << QStringLiteral("��") << gen << QStringLiteral("����Ⱥ����") << "\t\t" << QStringLiteral("������Ӧ��Ϊ") << totalFit << endl;
	for (int i = 0; i < popSize; i++)
		nextPop[i].prob = getProb(nextPop[i]);
	for (int i = 0; i < popSize; i++)
		pop[i] = nextPop[i];
	nextPopNum = 0;
}

int Dy::findBest(individual[])
{
	//����һ����Ϊ��Ӧ�����ֵ��ǩ
	double fitMaxTag = pop[0].fit;
	int fitTag = 0;
	for (int i = 1; i < popSize; i++)
	{
		if (pop[i].fit > fitMaxTag)
		{
			fitMaxTag = pop[i].fit;
			fitTag = i;
		}
	}
	return fitTag;
}
void Dy::chromoDecode(individual in, BpNet& bpnet) {
	
	//�����֮ǰ��Ȩ��
	for (int i = 0; i < INNODE; i++)
	{
			bpnet.inputLayer[i]->weight.clear();
	}
	for (int i = 0; i < HIDENODE; i++)
	{
		bpnet.hiddenLayer[i]->weight.clear();
	}
	for (int i = 0; i < INNODE; i++)
	{
		for (int j = 0; j < HIDENODE; j++)
		{
			bpnet.inputLayer[i]->weight.push_back(in.chromo[j]);
			in.chromo.erase(in.chromo.begin());
		}
	}
	for (int i = 0; i < HIDENODE; i++)
	{
		bpnet.hiddenLayer[i]->bias = in.chromo[i];
		in.chromo.erase(in.chromo.begin());
	}
	for (int i = 0; i < HIDENODE; i++)
	{
		for (int j = 0; j < OUTNODE; j++)
		{
			bpnet.hiddenLayer[i]->weight.push_back(in.chromo[j]);
			in.chromo.erase(in.chromo.begin());
		}
	}
	for (int i = 0; i < OUTNODE; i++)
	{
		bpnet.outputLayer[i]->bias = in.chromo[i];
		in.chromo.erase(in.chromo.begin());
	}
}

//��������
void Dy::testForGABP()
{
	BpNet bpNet;
	double threshold;
	int mostTimes;
	//�����У�5�����룬14�������5��14֮�仹�����кܶ����֣�����������ֻ����һ�����������һ������
	vector<double> vInput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\JD.csv", 9);
	vector<double> vOutput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\JD.csv", 3);
	vector<sample> rawTrainGroup = getTrainData(vInput, vOutput);
	vector<sample> trainGroup = bpNet.normalisation(rawTrainGroup, true);
	vector<sample> rawPredGroup = getPredData("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\test.csv");
	vector<sample> predGroup = bpNet.normalisation(rawPredGroup, false);
	//��ʼ�Ŵ��㷨��ʼ��Ȩֵ��ƫ��
	getInitPop(pop,trainGroup);
	while (gen<generation)
		evolution(bpNet,trainGroup);
	int fitTag = findBest(pop);
	chromoDecode(pop[fitTag], bpNet);
	//getInput(threshold, mostTimes);
	//��ʼѵ��
	bpNet.doTraining(trainGroup, 1, 10000);

	//�����ǲ������迹�����н���ʵʱѵ����ʹ�õ�ʱ��--------------------------------------------------------------------------------
	//���Խ��Ϊ100�����ݣ�ʹ��ʱ��Ϊ26ms����

	//timeb start;
	//ftime(&start);//��ȡ����
	////�����ݼ���С��100��
	//vector<double> vInput_ = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\JD - ����.csv", 9);
	//vector<double> vOutput_ = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\JD - ����.csv", 3);
	//auto rawTrainGroup_ = getTrainData(vInput_, vOutput_);
	//auto trainGroup_ = bpNet.normalisation(rawTrainGroup_, true);
	//bpNet.doTraining(trainGroup_, 1, 100);//ƽ��������ȡ��1��ʵ��֤��Ҳ�ﲻ����ѵ��������100
	//timeb end;
	//ftime(&end);//��ȡ����
	//int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
	//qDebug() << deta_t;

	//------------------------------------------------------------------------------------------------------------------------------

	bpNet.doTesting(predGroup);
	vector<sample> rawTestGroup1 = bpNet.denormalisation(predGroup);
	outTestData("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\test.csv", rawTestGroup1);

	system("pause");
	return;
}