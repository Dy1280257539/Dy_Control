#include"Bpnetwork.h"

namespace Dy
{
	std::default_random_engine e( (unsigned)time(0));
}


//初始化
Dy::BpNet::BpNet() {
	
	error = 100.f;//设置100先，是为了后续能够进入训练
	error_only_abs_denormalisation=0.f;//第一次可以啥都不设置
	//初始化输入层
	for (int i=0; i < INNODE; i++)
	{
		inputLayer[i] = new inputNode();
		for (int j = 0; j < HIDENODE; j++)
		{
			inputLayer[i]->weight.push_back(getRandom());
			inputLayer[i]->wDeltaSum.push_back(0.f);
			
		}
	}
	//初始化隐藏层
	for (int i = 0; i < HIDENODE; i++)
	{
		hiddenLayer[i] = new hiddenNode();
		hiddenLayer[i]->bias = getRandom();
		hiddenLayer[i]->bDeltaSum = 0.f;
		for (int j = 0; j < OUTNODE; j++)
		{
			hiddenLayer[i]->weight.push_back(getRandom());
			hiddenLayer[i]->wDeltaSum.push_back(0.f);
		}
	}
	
	//初始化输出层
	for (int i = 0; i < OUTNODE; i++)
	{
		outputLayer[i] = new outputNode();
		outputLayer[i]->bias = getRandom();
		outputLayer[i]->bDeltaSum = 0.f;
	}
	
}

//前向传播
void Dy::BpNet::fp() {

	for (int i = 0; i < HIDENODE; i++)
	{	
		double sum = 0.f;
		for (int j = 0; j < INNODE; j++)
		{
			sum += inputLayer[j]->value * inputLayer[j]->weight[i];
		}
		sum += hiddenLayer[i]->bias;
		hiddenLayer[i]->value = tansig(sum);
	}
	for (int i = 0; i < OUTNODE; i++)
	{
		double sum = 0.f;
		for (int j = 0; j < HIDENODE; j++)
		{
			sum += hiddenLayer[j]->value * hiddenLayer[j]->weight[i];
		}
		sum += outputLayer[i]->bias;
		outputLayer[i]->value = tansig(sum);
	}


}

//反向传播
void Dy::BpNet::bp() {

	//求出误差
	for (int i = 0; i < OUTNODE; i++)
	{
		double tmpe = fabs(outputLayer[i]->value - outputLayer[i]->rightValue);//求绝对值
		error += tmpe * tmpe / 2;//误差
		double tmpe1 = fabs(denormalisation_formal(outputLayer[i]->value, minTagOut[i], maxTagOut[i]) - denormalisation_formal(outputLayer[i]->rightValue, minTagOut[i], maxTagOut[i]));
		error_only_abs_denormalisation += tmpe1;
		//error += tmpe1;//误差
	}
	
	//求输出层偏移的导数
	for (int i = 0; i < OUTNODE; i++)
	{
		//tansig为激活函数时的求导法则
		double bDelta = (-1) * (outputLayer[i]->rightValue - outputLayer[i]->value) * (1 - pow(outputLayer[i]->value, 2));
		outputLayer[i]->bDeltaSum += bDelta;
		
	}
	
	//求隐藏层到输出层层权重的导数
	for (int i = 0; i < HIDENODE; i++)
	{
		for (int j = 0; j < OUTNODE; j++)
		{
			//tansig为激活函数时的求导法则
			double wDelta= (-1) * (outputLayer[j]->rightValue - outputLayer[j]->value) * (1 - pow(outputLayer[j]->value, 2))
				* hiddenLayer[i]->value;
			hiddenLayer[i]->wDeltaSum[j] += wDelta;
		}
	}

	//求隐藏层的偏移的导数
	for (int i = 0; i < HIDENODE; i++)
	{
		double sum = 0;
		for (int j = 0; j < OUTNODE; j++)//根据输出层的结点个数来求sum，因为求偏导公式的隐藏层的偏移是和节点数有关系的
		{
			//tansig为激活函数时的求导法则
			sum += (-1) * (outputLayer[j]->rightValue - outputLayer[j]->value) * (1 - pow(outputLayer[j]->value, 2))
				* hiddenLayer[i]->weight[j];
		}
		//tansig为激活函数时的求导法则
		hiddenLayer[i]->bDeltaSum += (sum * (1 - pow(hiddenLayer[i]->value, 2)));
	}

	//求输入层对隐藏层权重的变化值
	for (int  i = 0; i < INNODE; i++)
	{	//这部分b1和w1很相似，是有公因式的，所以这部分的几乎代码相同
		double sum = 0;
		for (int j = 0; j < HIDENODE; j++)
		{
			for (int k = 0; k < OUTNODE; k++)
			{
				//tansig为激活函数时的求导法则
				sum += (-1) * (outputLayer[k]->rightValue - outputLayer[k]->value) * (1 - pow(outputLayer[k]->value, 2)) * hiddenLayer[j]->weight[k];
				
			}
			//tansig为激活函数时的求导法则
			inputLayer[i]->wDeltaSum[j] += (sum * (1 - pow(hiddenLayer[j]->value, 2)) * inputLayer[i]->value);
		}
	}
}
//给输入层每个节点设置输入值 每个样本进行训练时都要调用
void Dy::BpNet::setInValue(vector<double> sampleIn) {
	// 对应一次样本 输入层每个节点的输入值
	for (int i = 0; i < INNODE; i++) {
		inputLayer[i]->value = sampleIn[i];
	}
}

/**
 * 给输出层每个节点设置正确值 每个样本进行训练时都要调用
 */
void Dy::BpNet::setOutRightValue(vector<double> sampleOut) {
	// 对应一次样本 输出层层每个节点的正确值
	for (int i = 0; i < OUTNODE; i++) {
		outputLayer[i]->rightValue = sampleOut[i];
	}
}

void Dy::BpNet::doTraining(vector<sample> trainGroup, double threshold,int mostTimes) {

	int sampleNum = trainGroup.size();//有多少个样本，一个样本对应一次输入和输出
	int trainTimes = 0;
	bool isSuccess = true;
	error = 100;//重新初始化设置一个误差值为100
	while (error >= threshold) {//当误差大于阈值，则还要继续训练,一开始设置了一个100，是为了能够进入训练
		if (trainTimes>mostTimes)//训练次数超过最大训练次数
		{
			isSuccess = false;
			break;
		}

		qDebug() << QStringLiteral("训练次数:") << trainTimes++ << "\t\t" << QStringLiteral("当前误差为:")<< error << "\t\t" << QStringLiteral("平均拟合误差:") << error_only_abs_denormalisation<<endl;
		error = 0.f;
		error_only_abs_denormalisation = 0.f;

		//重新设置输入层权重的delta和
		for (int i = 0; i < INNODE; i++)
		{
			inputLayer[i]->wDeltaSum.assign(inputLayer[i]->wDeltaSum.size(), 0.f);
		}
		//重新设置隐藏层的权重和偏移的导数和
		for (int i = 0; i < HIDENODE; i++)
		{
			hiddenLayer[i]->wDeltaSum.assign(hiddenLayer[i]->wDeltaSum.size(), 0.f);
			hiddenLayer[i]->bDeltaSum = 0.f;
		}
		//重新设置输出层的偏移导数和
		for (int i = 0; i < OUTNODE; i++) {
			outputLayer[i]->bDeltaSum = 0.f;
		}
		for (int i = 0; i < sampleNum; i++)
		{
			setInValue(trainGroup[i].in);     //    设定对应的样本的输入输出值
			setOutRightValue(trainGroup[i].out);
			fp();//求出对应的隐藏层和输出层的输出
			bp();//求出对应的权重和偏移的导数和，一次样本求一次，并累加到导数和里面去,也把误差更新一下
		}
		error_only_abs_denormalisation /= (sampleNum * OUTNODE);//更新平均绝对值误差
		for (int i = 0; i < INNODE; i++) {
			for (int j = 0; j < HIDENODE; j++) {
				//更新权重
				inputLayer[i]->weight[j] -= LEARNINGRATE * inputLayer[i]->wDeltaSum[j] / sampleNum;
			}
		}
		
		//修改隐藏层的权重和偏移
		for (int i = 0; i < HIDENODE; i++) {
			// 修改每个节点的偏移 因为一个节点就一个偏移 所以不用在节点里再遍历
			hiddenLayer[i]->bias -= LEARNINGRATE * hiddenLayer[i]->bDeltaSum / sampleNum;

			// 修改每个节点的各个加权的值
			for (int j = 0; j < OUTNODE; j++) {
				hiddenLayer[i]->weight[j] -= LEARNINGRATE * hiddenLayer[i]->wDeltaSum[j] / sampleNum;
			}
		}

		//修改输出层的偏移
		for (int i = 0; i < OUTNODE; i++) {
			outputLayer[i]->bias -= LEARNINGRATE * outputLayer[i]->bDeltaSum / sampleNum;
		}

	}
	if (isSuccess) {
		qDebug() << endl << QStringLiteral("训练成功!!!" )<< "\t\t" << QStringLiteral("最终误差: " )<< error << endl << endl;
	}
	else {
		qDebug() << endl << QStringLiteral("训练失败! 超过最大次数!") << "\t\t" << QStringLiteral("最终误差: ") << error << endl << endl;
	}
	return;
}

void Dy::BpNet::doTesting(vector<sample> & testGroup) {


	int testNum = testGroup.size();
	for (int i = 0; i < testNum; i++)
	{
		testGroup[i].out.clear();//清空输出
		setInValue(testGroup[i].in);//设置输入数据
		fp();//前向传播获取测试集的输出
		for (int j = 0; j < OUTNODE; j++)
		{
			testGroup[i].out.push_back(outputLayer[j]->value);
		}
	}
	return;
}

//归一化公式
double Dy::BpNet::normalisation_formal(double source, double min, double max)
{
	double after;
	after = 2 * (source - min) / (max - min) - 1;
	return after;

}
double Dy::BpNet::denormalisation_formal(double after, double min, double max)
{
	double source;
	source = (after + 1) * (max - min) / 2 + min;
	return source;
}
vector<Dy::sample> Dy::BpNet::normalisation(const vector<sample> &vsample,bool flag)
{
	if (flag)
	{
		setAllTag(vsample);
		//拷贝构造一个vsampleAfter，用于记录归一化后的样本
		auto iter = vsample.begin();
		vector<sample> vsampleAfter(vsample);
		for (auto iter1 = vsampleAfter.begin(); iter1 < vsampleAfter.end(); iter1++)
		{
			//对输入进行归一化
			for (int num1 = 0; num1 < INNODE; num1++)
			{
				iter1->in[num1] = normalisation_formal(iter1->in[num1], minTagIn[num1], maxTagIn[num1]);
			}
			//对输出进行归一化
			for (int num2 = 0; num2 < OUTNODE; num2++)
			{
				iter1->out[num2] = normalisation_formal(iter1->out[num2], minTagOut[num2], maxTagOut[num2]);
			}
		}
		return vsampleAfter;
	}
	else
	{
		//对测试集进行归一化
		//拷贝构造一个vsampleAfter，用于记录归一化后的样本
		vector<sample> vsampleAfter(vsample);
		for (auto iter1 = vsampleAfter.begin(); iter1 < vsampleAfter.end(); iter1++)
		{
			//对输入进行归一化
			for (int num1 = 0; num1 < INNODE; num1++)
			{
				iter1->in[num1] = normalisation_formal(iter1->in[num1], minTagIn[num1], maxTagIn[num1]);
			}
		}
		return vsampleAfter;
	}
	
}

vector<Dy::sample> Dy::BpNet::denormalisation(const vector<sample>& vsampleAfter)
{

	auto iter = vsampleAfter.begin();
	//拷贝构造一个vsample，用于记录归一化前的样本
	vector<sample> vsample(vsampleAfter);
	for (auto iter1 = vsample.begin(); iter1 < vsample.end(); iter1++)
	{
		//对输入进行反归一化
		for (int num1 = 0; num1 < INNODE; num1++)
		{
			iter1->in[num1] = denormalisation_formal(iter1->in[num1], minTagIn[num1], maxTagIn[num1]);
		}
		//对输出进行反归一化
		for (int num2 = 0; num2 < OUTNODE; num2++)
		{
			iter1->out[num2] = denormalisation_formal(iter1->out[num2], minTagOut[num2], maxTagOut[num2]);
		}
	}
	return vsample;
}

void Dy::BpNet::setAllTag(const vector<sample>& vsample) {

	auto iter = vsample.begin();

	//将标签初始化，每一列输入的最大值和最小值都先把样本的第一个作为其初始化值
	for (vector<double>::const_iterator iter1 = iter->in.begin(); iter1 != iter->in.end(); iter1++)
	{
		maxTagIn.push_back(*iter1);//将标签初始化
		minTagIn.push_back(*iter1);//将标签初始化
	}
	//输出同理
	for (vector<double>::const_iterator iter1 = iter->out.begin(); iter1 != iter->out.end(); iter1++)
	{
		maxTagOut.push_back(*iter1);//将标签初始化
		minTagOut.push_back(*iter1);//将标签初始化
	}
	//得出输入和输出的最小值和最大值标签
	for (; iter < vsample.end(); iter++)
	{
		int num = 0;//用于记录到哪个输入了，范围在0到INNODE
		for (auto iter1 = iter->in.begin(); iter1 != iter->in.end(); iter1++, num++)
		{
			if (*iter1 < minTagIn[num])
			{
				minTagIn[num] = *iter1;
			}
			if (*iter1 > maxTagIn[num])
			{
				maxTagIn[num] = *iter1;
			}
		}
		num = 0;//这时用于记录到哪个输出了，范围在0到OUTNODE
		for (auto iter1 = iter->out.begin(); iter1 != iter->out.end(); iter1++, num++)
		{
			if (*iter1 < minTagOut[num])
			{
				minTagOut[num] = *iter1;
			}
			if (*iter1 > maxTagOut[num])
			{
				maxTagOut[num] = *iter1;
			}
		}
	}
	return;
}

vector<string> Dy::getdata(const char* file, int position)
{
	std::fstream f(file, std::fstream::in);
	vector<string> vdata; //用于存放要的某列的数据
	vector<string> vdata1;  //用于存放当前行的数据
	string sdata;//用于读取数据，最终转换为double
	int b = 0;//记录当前读取所在的列数
	if (!f.is_open())
	{
		qDebug() << QStringLiteral("文件不存在！请检查路径") << endl;
		return vdata;
	}
	if (f.eof()) {
		qDebug() << QStringLiteral("文件为空") << endl;
		return vdata;
	}
	while (f.peek() != EOF)//注意，这里不能使用!f.eof()作为判断条件，因为读到结束的时候，需要再进行一次循环，才能知道它到达文件尾，peek是当前位置指向的内容
	{
		while (b != NUMCOL) //当前读取行还没到尾端
		{
			if (b == NUMCOL - 1)
			{
				getline(f, sdata, '\n'); //推进到回车处
				vdata1.push_back(sdata);
			}
			else
			{
				getline(f, sdata, ',');
				vdata1.push_back(sdata);
			}
			++b;
		}
		b = 0;//复位
		auto it = vdata1.begin() + position - 1;//取所需要的数据的迭代器
		vdata.push_back(*it);
		vdata1.clear();//把vdata清空，给下一次读取提供空容器
	}
	return vdata;
	////测试代码
	//for (auto it = vdata.begin(); it != vdata.end(); ++it)
	//{
	   // qDebug() << *it << endl;
	//}
	//return;
}

vector<double> Dy::data_to_double(const vector<string>& vdata) {
	vector<double> vdoubledata;
	for (auto it = vdata.begin() + 1; it != vdata.end(); it++)//去掉标题，所以要begin+1
	{
		vdoubledata.push_back(stod(*it));
	}
	return vdoubledata;
}

vector<double> Dy::getOnlyDouble(const char* file, int position) {

	vector<string> vdata = getdata(file, position);
	vector<double> vDoubleData = data_to_double(vdata);
	return vDoubleData;
}

//获取训练数据的最后的特化版本
void Dy::getTrainData(vector<sample>& vsample, const vector<double>& output)
{
	auto ioutput = output.begin();
	for (auto it = vsample.begin(); it != vsample.end(); it++)
	{
		it->out.push_back(*ioutput);
		ioutput++;
	}
	return;
}


vector<Dy::sample> Dy::getTestData(const char* path, const int& numcol) {

	std::fstream f(path, std::fstream::in);
	string sdata;
	vector<sample> vsample;
	sample s; //中间过程量，用于读取数据并压入vector<sample>
	if (!f.is_open())
	{
		qDebug() << QStringLiteral("文件不存在！请检查路径" )<< endl;
		return vsample;
	}
	if (f.eof()) {
		qDebug() << QStringLiteral("文件为空") << endl;
		return vsample;
	}
	int b = 0;//用来记录当前读取到的列数
	//先把标题读掉
	getline(f, sdata, '\n'); //推进到回车处
	while (f.peek() != EOF)  //注意，这里不能使用!f.eof()作为判断条件，因为读到结束的时候，需要再进行一次循环，才能知道它到达文件尾，peek是当前位置指向的内容
	{
		while (b != numcol) //当前读取行还没到尾端
		{
			if (b == numcol - 1)
			{
				getline(f, sdata, '\n'); //推进到回车处
				s.out.push_back(stod(sdata));//将数据读到sample 的输出
			}
			else
			{
				getline(f, sdata, ',');
				s.in.push_back(stod(sdata));//将数据读到sample 的输入
			}
			++b;
		}
		b = 0;//复位
		vsample.push_back(s);
		s.in.clear();
		s.out.clear();
	}
	return vsample;
}

void Dy::outTestData(const char* path, const vector<sample>& vsample)
{
	std::fstream f(path, std::ios::out);
	string s1 = "input";
	for (int i = 0; i < INNODE; i++)//输入标题
	{
		f << s1 + std::to_string(i) << ',';
	}
	f << "output" << endl;
	for (auto it = vsample.begin(); it != vsample.end(); it++)
	{
		for (auto it2 = it->in.begin(); it2 != it->in.end(); it2++)
		{
			f << std::to_string(*it2) << ',';
		}
		f << std::to_string(it->out.front());//单输出，后面有需要再改吧
		f << endl;
	}
	f.close();
	return;
}

vector<Dy::sample> Dy::getPredData(const char* path) {
	vector<sample> vsample;
	std::fstream f(path, std::ios::in);
	string s;//用于记录读取进来的string
	sample s1;//用于给vector添加元素
	if (!f.is_open())
	{
		qDebug() << QStringLiteral("文件打开失败，请重新尝试") << endl;
		return vsample;
	}
	if (f.eof())
	{
		qDebug() << QStringLiteral("文件为空，请输入好再读取") << endl;
		return vsample;
	}
	std::getline(f, s, '\n');//读掉标题行
	while (f.peek() != EOF) {

		for (int i = 0; i < INNODE; i++)//根据输入层的节点数来读取数据到vector<sample>
		{
			if (i != INNODE - 1)
			{
				getline(f, s, ',');
			}
			else if (i == INNODE - 1)//到最后一次的时候就要把换行符读掉
			{
				getline(f, s, '\n');
			}
			s1.in.push_back(stod(s));
		}
		vsample.push_back(s1);//将这个样本，添加进vector
		s1.in.clear();
	}
	return vsample;

}

void Dy::getInput(double& threshold, int& mostTimes) {
	qDebug() << QStringLiteral("训练及测试数据已从文件读入" )<< endl << endl;
	qDebug() << QStringLiteral("请输入训练最大误差：");   //0.0001最好
	std::cin >> threshold;
	qDebug() << QStringLiteral("请输入训练最大次数：");
	std::cin >> mostTimes;
}

//用例测试代码
void Dy::testForBp()
{
	BpNet bpNet;
	double threshold;
	int mostTimes;
	//下面中，5是输入，14是输出，5和14之间还可以有很多数字，我这里的输出只能有一个，就是最后一个数字
	vector<double> vInput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\JD.csv", 9);
	vector<double> vOnput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据\\JD.csv", 3);
	vector<sample> rawTrainGroup = getTrainData(vInput, vOnput);
	vector<sample> rawPredGroup = getPredData("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据test.csv"); 
	vector<sample> trainGroup = bpNet.normalisation(rawTrainGroup, true);
	vector<sample> predGroup = bpNet.normalisation(rawPredGroup, false);
	getInput(threshold, mostTimes);
	bpNet.doTraining(trainGroup, threshold, mostTimes);
	bpNet.doTesting(predGroup);
	vector<sample> rawTestGroup1 = bpNet.denormalisation(predGroup);
	outTestData("F:\\DESKAPPPLACE\\DOCUMENT\\研究生\\JD汇报\\JD 3.31 含训练数据test.csv", rawTestGroup1);

}