#include"Bpnetwork.h"

namespace Dy
{
	std::default_random_engine e( (unsigned)time(0));
}


//��ʼ��
Dy::BpNet::BpNet() {
	
	error = 100.f;//����100�ȣ���Ϊ�˺����ܹ�����ѵ��
	error_only_abs_denormalisation=0.f;//��һ�ο���ɶ��������
	//��ʼ�������
	for (int i=0; i < INNODE; i++)
	{
		inputLayer[i] = new inputNode();
		for (int j = 0; j < HIDENODE; j++)
		{
			inputLayer[i]->weight.push_back(getRandom());
			inputLayer[i]->wDeltaSum.push_back(0.f);
			
		}
	}
	//��ʼ�����ز�
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
	
	//��ʼ�������
	for (int i = 0; i < OUTNODE; i++)
	{
		outputLayer[i] = new outputNode();
		outputLayer[i]->bias = getRandom();
		outputLayer[i]->bDeltaSum = 0.f;
	}
	
}

//ǰ�򴫲�
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

//���򴫲�
void Dy::BpNet::bp() {

	//������
	for (int i = 0; i < OUTNODE; i++)
	{
		double tmpe = fabs(outputLayer[i]->value - outputLayer[i]->rightValue);//�����ֵ
		error += tmpe * tmpe / 2;//���
		double tmpe1 = fabs(denormalisation_formal(outputLayer[i]->value, minTagOut[i], maxTagOut[i]) - denormalisation_formal(outputLayer[i]->rightValue, minTagOut[i], maxTagOut[i]));
		error_only_abs_denormalisation += tmpe1;
		//error += tmpe1;//���
	}
	
	//�������ƫ�Ƶĵ���
	for (int i = 0; i < OUTNODE; i++)
	{
		//tansigΪ�����ʱ���󵼷���
		double bDelta = (-1) * (outputLayer[i]->rightValue - outputLayer[i]->value) * (1 - pow(outputLayer[i]->value, 2));
		outputLayer[i]->bDeltaSum += bDelta;
		
	}
	
	//�����ز㵽������Ȩ�صĵ���
	for (int i = 0; i < HIDENODE; i++)
	{
		for (int j = 0; j < OUTNODE; j++)
		{
			//tansigΪ�����ʱ���󵼷���
			double wDelta= (-1) * (outputLayer[j]->rightValue - outputLayer[j]->value) * (1 - pow(outputLayer[j]->value, 2))
				* hiddenLayer[i]->value;
			hiddenLayer[i]->wDeltaSum[j] += wDelta;
		}
	}

	//�����ز��ƫ�Ƶĵ���
	for (int i = 0; i < HIDENODE; i++)
	{
		double sum = 0;
		for (int j = 0; j < OUTNODE; j++)//���������Ľ���������sum����Ϊ��ƫ����ʽ�����ز��ƫ���Ǻͽڵ����й�ϵ��
		{
			//tansigΪ�����ʱ���󵼷���
			sum += (-1) * (outputLayer[j]->rightValue - outputLayer[j]->value) * (1 - pow(outputLayer[j]->value, 2))
				* hiddenLayer[i]->weight[j];
		}
		//tansigΪ�����ʱ���󵼷���
		hiddenLayer[i]->bDeltaSum += (sum * (1 - pow(hiddenLayer[i]->value, 2)));
	}

	//�����������ز�Ȩ�صı仯ֵ
	for (int  i = 0; i < INNODE; i++)
	{	//�ⲿ��b1��w1�����ƣ����й���ʽ�ģ������ⲿ�ֵļ���������ͬ
		double sum = 0;
		for (int j = 0; j < HIDENODE; j++)
		{
			for (int k = 0; k < OUTNODE; k++)
			{
				//tansigΪ�����ʱ���󵼷���
				sum += (-1) * (outputLayer[k]->rightValue - outputLayer[k]->value) * (1 - pow(outputLayer[k]->value, 2)) * hiddenLayer[j]->weight[k];
				
			}
			//tansigΪ�����ʱ���󵼷���
			inputLayer[i]->wDeltaSum[j] += (sum * (1 - pow(hiddenLayer[j]->value, 2)) * inputLayer[i]->value);
		}
	}
}
//�������ÿ���ڵ���������ֵ ÿ����������ѵ��ʱ��Ҫ����
void Dy::BpNet::setInValue(vector<double> sampleIn) {
	// ��Ӧһ������ �����ÿ���ڵ������ֵ
	for (int i = 0; i < INNODE; i++) {
		inputLayer[i]->value = sampleIn[i];
	}
}

/**
 * �������ÿ���ڵ�������ȷֵ ÿ����������ѵ��ʱ��Ҫ����
 */
void Dy::BpNet::setOutRightValue(vector<double> sampleOut) {
	// ��Ӧһ������ ������ÿ���ڵ����ȷֵ
	for (int i = 0; i < OUTNODE; i++) {
		outputLayer[i]->rightValue = sampleOut[i];
	}
}

void Dy::BpNet::doTraining(vector<sample> trainGroup, double threshold,int mostTimes) {

	int sampleNum = trainGroup.size();//�ж��ٸ�������һ��������Ӧһ����������
	int trainTimes = 0;
	bool isSuccess = true;
	error = 100;//���³�ʼ������һ�����ֵΪ100
	while (error >= threshold) {//����������ֵ����Ҫ����ѵ��,һ��ʼ������һ��100����Ϊ���ܹ�����ѵ��
		if (trainTimes>mostTimes)//ѵ�������������ѵ������
		{
			isSuccess = false;
			break;
		}

		qDebug() << QStringLiteral("ѵ������:") << trainTimes++ << "\t\t" << QStringLiteral("��ǰ���Ϊ:")<< error << "\t\t" << QStringLiteral("ƽ��������:") << error_only_abs_denormalisation<<endl;
		error = 0.f;
		error_only_abs_denormalisation = 0.f;

		//�������������Ȩ�ص�delta��
		for (int i = 0; i < INNODE; i++)
		{
			inputLayer[i]->wDeltaSum.assign(inputLayer[i]->wDeltaSum.size(), 0.f);
		}
		//�����������ز��Ȩ�غ�ƫ�Ƶĵ�����
		for (int i = 0; i < HIDENODE; i++)
		{
			hiddenLayer[i]->wDeltaSum.assign(hiddenLayer[i]->wDeltaSum.size(), 0.f);
			hiddenLayer[i]->bDeltaSum = 0.f;
		}
		//��������������ƫ�Ƶ�����
		for (int i = 0; i < OUTNODE; i++) {
			outputLayer[i]->bDeltaSum = 0.f;
		}
		for (int i = 0; i < sampleNum; i++)
		{
			setInValue(trainGroup[i].in);     //    �趨��Ӧ���������������ֵ
			setOutRightValue(trainGroup[i].out);
			fp();//�����Ӧ�����ز�����������
			bp();//�����Ӧ��Ȩ�غ�ƫ�Ƶĵ����ͣ�һ��������һ�Σ����ۼӵ�����������ȥ,Ҳ��������һ��
		}
		error_only_abs_denormalisation /= (sampleNum * OUTNODE);//����ƽ������ֵ���
		for (int i = 0; i < INNODE; i++) {
			for (int j = 0; j < HIDENODE; j++) {
				//����Ȩ��
				inputLayer[i]->weight[j] -= LEARNINGRATE * inputLayer[i]->wDeltaSum[j] / sampleNum;
			}
		}
		
		//�޸����ز��Ȩ�غ�ƫ��
		for (int i = 0; i < HIDENODE; i++) {
			// �޸�ÿ���ڵ��ƫ�� ��Ϊһ���ڵ��һ��ƫ�� ���Բ����ڽڵ����ٱ���
			hiddenLayer[i]->bias -= LEARNINGRATE * hiddenLayer[i]->bDeltaSum / sampleNum;

			// �޸�ÿ���ڵ�ĸ�����Ȩ��ֵ
			for (int j = 0; j < OUTNODE; j++) {
				hiddenLayer[i]->weight[j] -= LEARNINGRATE * hiddenLayer[i]->wDeltaSum[j] / sampleNum;
			}
		}

		//�޸�������ƫ��
		for (int i = 0; i < OUTNODE; i++) {
			outputLayer[i]->bias -= LEARNINGRATE * outputLayer[i]->bDeltaSum / sampleNum;
		}

	}
	if (isSuccess) {
		qDebug() << endl << QStringLiteral("ѵ���ɹ�!!!" )<< "\t\t" << QStringLiteral("�������: " )<< error << endl << endl;
	}
	else {
		qDebug() << endl << QStringLiteral("ѵ��ʧ��! ����������!") << "\t\t" << QStringLiteral("�������: ") << error << endl << endl;
	}
	return;
}

void Dy::BpNet::doTesting(vector<sample> & testGroup) {


	int testNum = testGroup.size();
	for (int i = 0; i < testNum; i++)
	{
		testGroup[i].out.clear();//������
		setInValue(testGroup[i].in);//������������
		fp();//ǰ�򴫲���ȡ���Լ������
		for (int j = 0; j < OUTNODE; j++)
		{
			testGroup[i].out.push_back(outputLayer[j]->value);
		}
	}
	return;
}

//��һ����ʽ
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
		//��������һ��vsampleAfter�����ڼ�¼��һ���������
		auto iter = vsample.begin();
		vector<sample> vsampleAfter(vsample);
		for (auto iter1 = vsampleAfter.begin(); iter1 < vsampleAfter.end(); iter1++)
		{
			//��������й�һ��
			for (int num1 = 0; num1 < INNODE; num1++)
			{
				iter1->in[num1] = normalisation_formal(iter1->in[num1], minTagIn[num1], maxTagIn[num1]);
			}
			//��������й�һ��
			for (int num2 = 0; num2 < OUTNODE; num2++)
			{
				iter1->out[num2] = normalisation_formal(iter1->out[num2], minTagOut[num2], maxTagOut[num2]);
			}
		}
		return vsampleAfter;
	}
	else
	{
		//�Բ��Լ����й�һ��
		//��������һ��vsampleAfter�����ڼ�¼��һ���������
		vector<sample> vsampleAfter(vsample);
		for (auto iter1 = vsampleAfter.begin(); iter1 < vsampleAfter.end(); iter1++)
		{
			//��������й�һ��
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
	//��������һ��vsample�����ڼ�¼��һ��ǰ������
	vector<sample> vsample(vsampleAfter);
	for (auto iter1 = vsample.begin(); iter1 < vsample.end(); iter1++)
	{
		//��������з���һ��
		for (int num1 = 0; num1 < INNODE; num1++)
		{
			iter1->in[num1] = denormalisation_formal(iter1->in[num1], minTagIn[num1], maxTagIn[num1]);
		}
		//��������з���һ��
		for (int num2 = 0; num2 < OUTNODE; num2++)
		{
			iter1->out[num2] = denormalisation_formal(iter1->out[num2], minTagOut[num2], maxTagOut[num2]);
		}
	}
	return vsample;
}

void Dy::BpNet::setAllTag(const vector<sample>& vsample) {

	auto iter = vsample.begin();

	//����ǩ��ʼ����ÿһ����������ֵ����Сֵ���Ȱ������ĵ�һ����Ϊ���ʼ��ֵ
	for (vector<double>::const_iterator iter1 = iter->in.begin(); iter1 != iter->in.end(); iter1++)
	{
		maxTagIn.push_back(*iter1);//����ǩ��ʼ��
		minTagIn.push_back(*iter1);//����ǩ��ʼ��
	}
	//���ͬ��
	for (vector<double>::const_iterator iter1 = iter->out.begin(); iter1 != iter->out.end(); iter1++)
	{
		maxTagOut.push_back(*iter1);//����ǩ��ʼ��
		minTagOut.push_back(*iter1);//����ǩ��ʼ��
	}
	//�ó�������������Сֵ�����ֵ��ǩ
	for (; iter < vsample.end(); iter++)
	{
		int num = 0;//���ڼ�¼���ĸ������ˣ���Χ��0��INNODE
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
		num = 0;//��ʱ���ڼ�¼���ĸ�����ˣ���Χ��0��OUTNODE
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
	vector<string> vdata; //���ڴ��Ҫ��ĳ�е�����
	vector<string> vdata1;  //���ڴ�ŵ�ǰ�е�����
	string sdata;//���ڶ�ȡ���ݣ�����ת��Ϊdouble
	int b = 0;//��¼��ǰ��ȡ���ڵ�����
	if (!f.is_open())
	{
		qDebug() << QStringLiteral("�ļ������ڣ�����·��") << endl;
		return vdata;
	}
	if (f.eof()) {
		qDebug() << QStringLiteral("�ļ�Ϊ��") << endl;
		return vdata;
	}
	while (f.peek() != EOF)//ע�⣬���ﲻ��ʹ��!f.eof()��Ϊ�ж���������Ϊ����������ʱ����Ҫ�ٽ���һ��ѭ��������֪���������ļ�β��peek�ǵ�ǰλ��ָ�������
	{
		while (b != NUMCOL) //��ǰ��ȡ�л�û��β��
		{
			if (b == NUMCOL - 1)
			{
				getline(f, sdata, '\n'); //�ƽ����س���
				vdata1.push_back(sdata);
			}
			else
			{
				getline(f, sdata, ',');
				vdata1.push_back(sdata);
			}
			++b;
		}
		b = 0;//��λ
		auto it = vdata1.begin() + position - 1;//ȡ����Ҫ�����ݵĵ�����
		vdata.push_back(*it);
		vdata1.clear();//��vdata��գ�����һ�ζ�ȡ�ṩ������
	}
	return vdata;
	////���Դ���
	//for (auto it = vdata.begin(); it != vdata.end(); ++it)
	//{
	   // qDebug() << *it << endl;
	//}
	//return;
}

vector<double> Dy::data_to_double(const vector<string>& vdata) {
	vector<double> vdoubledata;
	for (auto it = vdata.begin() + 1; it != vdata.end(); it++)//ȥ�����⣬����Ҫbegin+1
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

//��ȡѵ�����ݵ������ػ��汾
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
	sample s; //�м�����������ڶ�ȡ���ݲ�ѹ��vector<sample>
	if (!f.is_open())
	{
		qDebug() << QStringLiteral("�ļ������ڣ�����·��" )<< endl;
		return vsample;
	}
	if (f.eof()) {
		qDebug() << QStringLiteral("�ļ�Ϊ��") << endl;
		return vsample;
	}
	int b = 0;//������¼��ǰ��ȡ��������
	//�Ȱѱ������
	getline(f, sdata, '\n'); //�ƽ����س���
	while (f.peek() != EOF)  //ע�⣬���ﲻ��ʹ��!f.eof()��Ϊ�ж���������Ϊ����������ʱ����Ҫ�ٽ���һ��ѭ��������֪���������ļ�β��peek�ǵ�ǰλ��ָ�������
	{
		while (b != numcol) //��ǰ��ȡ�л�û��β��
		{
			if (b == numcol - 1)
			{
				getline(f, sdata, '\n'); //�ƽ����س���
				s.out.push_back(stod(sdata));//�����ݶ���sample �����
			}
			else
			{
				getline(f, sdata, ',');
				s.in.push_back(stod(sdata));//�����ݶ���sample ������
			}
			++b;
		}
		b = 0;//��λ
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
	for (int i = 0; i < INNODE; i++)//�������
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
		f << std::to_string(it->out.front());//���������������Ҫ�ٸİ�
		f << endl;
	}
	f.close();
	return;
}

vector<Dy::sample> Dy::getPredData(const char* path) {
	vector<sample> vsample;
	std::fstream f(path, std::ios::in);
	string s;//���ڼ�¼��ȡ������string
	sample s1;//���ڸ�vector���Ԫ��
	if (!f.is_open())
	{
		qDebug() << QStringLiteral("�ļ���ʧ�ܣ������³���") << endl;
		return vsample;
	}
	if (f.eof())
	{
		qDebug() << QStringLiteral("�ļ�Ϊ�գ���������ٶ�ȡ") << endl;
		return vsample;
	}
	std::getline(f, s, '\n');//����������
	while (f.peek() != EOF) {

		for (int i = 0; i < INNODE; i++)//���������Ľڵ�������ȡ���ݵ�vector<sample>
		{
			if (i != INNODE - 1)
			{
				getline(f, s, ',');
			}
			else if (i == INNODE - 1)//�����һ�ε�ʱ���Ҫ�ѻ��з�����
			{
				getline(f, s, '\n');
			}
			s1.in.push_back(stod(s));
		}
		vsample.push_back(s1);//�������������ӽ�vector
		s1.in.clear();
	}
	return vsample;

}

void Dy::getInput(double& threshold, int& mostTimes) {
	qDebug() << QStringLiteral("ѵ�������������Ѵ��ļ�����" )<< endl << endl;
	qDebug() << QStringLiteral("������ѵ�������");   //0.0001���
	std::cin >> threshold;
	qDebug() << QStringLiteral("������ѵ����������");
	std::cin >> mostTimes;
}

//�������Դ���
void Dy::testForBp()
{
	BpNet bpNet;
	double threshold;
	int mostTimes;
	//�����У�5�����룬14�������5��14֮�仹�����кܶ����֣�����������ֻ����һ�����������һ������
	vector<double> vInput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\JD.csv", 9);
	vector<double> vOnput = getOnlyDouble("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������\\JD.csv", 3);
	vector<sample> rawTrainGroup = getTrainData(vInput, vOnput);
	vector<sample> rawPredGroup = getPredData("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������test.csv"); 
	vector<sample> trainGroup = bpNet.normalisation(rawTrainGroup, true);
	vector<sample> predGroup = bpNet.normalisation(rawPredGroup, false);
	getInput(threshold, mostTimes);
	bpNet.doTraining(trainGroup, threshold, mostTimes);
	bpNet.doTesting(predGroup);
	vector<sample> rawTestGroup1 = bpNet.denormalisation(predGroup);
	outTestData("F:\\DESKAPPPLACE\\DOCUMENT\\�о���\\JD�㱨\\JD 3.31 ��ѵ������test.csv", rawTestGroup1);

}