#include"Genetic_algorithms.h"

namespace Dy {
	const int popSize = 500; //种群大小
	const int len = INNODE * HIDENODE + HIDENODE + HIDENODE * OUTNODE + OUTNODE;//编码长度
	const double Pc = 0.8;//交叉概率
	const double Pm = 0.2;//变异概率
	const int generation = 100;//迭代次数
	individual pop[popSize];//种群
	individual nextPop[popSize];//下一代种群
	int nextPopNum = 0;//用于记录下一代种群的个数
	double lineProb[popSize];//用于轮盘赌算法的线性选择概率
	double totalFit; //总适应度
	int gen = 1;//记录当前代数
}


//获得一条染色体，其上面的数据是输入层的权重（按顺序，如先第一个神经元的hidenode个权值，然后第二个，依此类推）
//，隐藏层的偏置，隐藏层的权重（按顺序），输出层的偏置
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

//根据神经网络计算个体适应度
double Dy::getFitness(individual in,BpNet  bpnet,const vector<sample>& vsample)
{
	//先清空输入层的权重，再将in内的压入
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
			//将染色体第一个成员压入weight，并清除
			bpnet.inputLayer[i]->weight.push_back(in.chromo.front());
			in.chromo.erase(in.chromo.begin());
		}
	}
	//同理，将隐藏层的权重清除，并压入in内的数据，以及压入偏置
	for (int i = 0; i < HIDENODE; i++)
	{
		for (int j = 0; j < OUTNODE; j++)
		{
			bpnet.hiddenLayer[i]->weight.clear();
		}
	}
	//修改隐藏层偏置
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
	//将输出层的偏置设置为in染色体的数据，道理同上
	for (int i = 0; i < OUTNODE; i++)
	{
		bpnet.outputLayer[i]->bias = in.chromo.front();
		in.chromo.erase(in.chromo.begin());
	}
	int sampleNum = vsample.size();
	//适应度从下面两者的其中一个的倒数做选择
	bpnet.error = 0.f; 
	//bpnet.error_only_abs_denormalisation = 0.f;
	//bpnet.setAllTag(vsample);
	for (int i = 0; i < sampleNum; i++)
	{
		bpnet.setInValue(vsample[i].in);//设定对应的样本的输入输出值
		bpnet.setOutRightValue(vsample[i].out);
		bpnet.fp();//求出对应的隐藏层和输出层的输出
		//累加error
		for (int i = 0; i < OUTNODE; i++)
		{
			double tmpe = fabs(bpnet.outputLayer[i]->value - bpnet.outputLayer[i]->rightValue);//求绝对值
			bpnet.error += tmpe * tmpe / 2;//误差
			//double tmpe1 = fabs(bpnet.denormalisation_formal(bpnet.outputLayer[i]->value, bpnet.minTagOut[i], bpnet.maxTagOut[i]) - bpnet.denormalisation_formal(bpnet.outputLayer[i]->rightValue, bpnet.minTagOut[i], bpnet.maxTagOut[i]));
			//bpnet.error_only_abs_denormalisation += tmpe1;
		}
	}  
	
	return 1 / (bpnet.error);

}
//随机获取染色体
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

//初始化种群
void Dy::getInitPop(individual pop[],const vector<sample> & vsample)
{
	vector<double> chromo;//染色体
	BpNet bpnet;
	totalFit = 0;//总体舒适度值
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
	for (int i = 1; i < popSize; i++)//累计种群概率
	{
		lineProb[i] = lineProb[i - 1] + pop[i].prob;
	}
	std::uniform_real_distribution<double> u(0, 1);
	double r = u(e);//获取一个0到1的随机数;
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
	double r1 = u(e);//获取一个0到1的随机数;
	//用轮盘赌的方式
	if (r1 <= Pc) //发生交叉
	{
		//选择出要进行交叉的两个个体
		int r2 = (int)fabs((int)e()) % popSize;
		int r3 = (int)fabs((int)e()) % popSize;
		//选择出交换染色体的起始位置
		int r4 = (int)fabs((int)e()) % len;
		if (nextPopNum + 1 > popSize - 1)//当前新种群只剩下一个位置,则只交叉一次，产生一个新个体
		{
			//qDebug() << "只交叉一次，产生一个新个体" << endl;	
			nextPop[nextPopNum] = pop[r2];
			//开始交叉
			for (int i = r4; i < len; i++)
			{
				nextPop[nextPopNum].chromo[i] = pop[r3].chromo[i];
			}
			nextPop[nextPopNum].fit = getFitness(nextPop[nextPopNum], bpnet, vsample);
			nextPopNum++;
		}
		else //新种群中空位大于等于2，可以一次产生两个新个体
		{
			//qDebug() << "  交叉一次，产生二个新个体" << endl;
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
			nextPopNum++;//规范种群个数
		}
	}
	else;
		//qDebug() << "  此次未进行交叉" << endl;

}
//染色体变异
void Dy::mutataion(individual pop[], BpNet bpnet, const vector<sample>& vsample)
{
	std::uniform_real_distribution<double> u(0, 1);
	double m1;
	int m2,m3;
	m1 = u(e);
	if (nextPopNum<popSize)//种群还有位置时
	{
		if (m1 < Pm)
		{
			//qDebug() << "  变异一次，产生一个新个体" << endl;
			m2 = (int)fabs((int)e()) % popSize;//变异的个体
			m3 = (int)fabs((int)e()) % len;//变异的染色体位置
			nextPop[nextPopNum] = pop[m2];
			double tmpe1 = pop[m2].chromo[m3];
			//变异方式：取反且变为原来的0到10倍
			nextPop[nextPopNum].chromo[m3] = -10 * u(e) * tmpe1;
			nextPop[nextPopNum].fit = getFitness(nextPop[nextPopNum], bpnet, vsample);
			nextPopNum++;
		}
		else;
			//qDebug() << "  此次未进行变异" << endl;
	}
	return;
}
//进化
void Dy::evolution(BpNet bpnet,const vector<sample> & vsample)
{
	gen++;
	qDebug() << QStringLiteral("第") << gen -1<< QStringLiteral("代种群开始进化" )<< endl;
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
	qDebug() << QStringLiteral("第") << gen << QStringLiteral("代种群诞生") << "\t\t" << QStringLiteral("总体适应度为") << totalFit << endl;
	for (int i = 0; i < popSize; i++)
		nextPop[i].prob = getProb(nextPop[i]);
	for (int i = 0; i < popSize; i++)
		pop[i] = nextPop[i];
	nextPopNum = 0;
}

int Dy::findBest(individual[])
{
	//将第一个作为适应度最大值标签
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
	
	//先清空之前的权重
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

//用例代码
void Dy::testForGABP()
{
	BpNet bpNet;
	double threshold;
	int mostTimes;
	//下面中，5是输入，14是输出，5和14之间还可以有很多数字，我这里的输出只能有一个，就是最后一个数字
	vector<double> vInput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\JD.csv", 9);
	vector<double> vOutput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\JD.csv", 3);
	vector<sample> rawTrainGroup = getTrainData(vInput, vOutput);
	vector<sample> trainGroup = bpNet.normalisation(rawTrainGroup, true);
	vector<sample> rawPredGroup = getPredData("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\test.csv");
	vector<sample> predGroup = bpNet.normalisation(rawPredGroup, false);
	//开始遗传算法初始化权值和偏置
	getInitPop(pop,trainGroup);
	while (gen<generation)
		evolution(bpNet,trainGroup);
	int fitTag = findBest(pop);
	chromoDecode(pop[fitTag], bpNet);
	//getInput(threshold, mostTimes);
	//开始训练
	bpNet.doTraining(trainGroup, 1, 10000);

	//以下是测试在阻抗控制中进行实时训练所使用的时间--------------------------------------------------------------------------------
	//测试结果为100组数据，使用时间为26ms左右

	//timeb start;
	//ftime(&start);//获取毫秒
	////此数据集大小有100组
	//vector<double> vInput_ = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\JD - 副本.csv", 9);
	//vector<double> vOutput_ = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\JD - 副本.csv", 3);
	//auto rawTrainGroup_ = getTrainData(vInput_, vOutput_);
	//auto trainGroup_ = bpNet.normalisation(rawTrainGroup_, true);
	//bpNet.doTraining(trainGroup_, 1, 100);//平方误差和先取个1，实验证明也达不到，训练次数是100
	//timeb end;
	//ftime(&end);//获取毫秒
	//int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
	//qDebug() << deta_t;

	//------------------------------------------------------------------------------------------------------------------------------

	bpNet.doTesting(predGroup);
	vector<sample> rawTestGroup1 = bpNet.denormalisation(predGroup);
	outTestData("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\test.csv", rawTestGroup1);

	system("pause");
	return;
}