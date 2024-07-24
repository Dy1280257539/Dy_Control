#include"endoscopeThread.h"


void endoscopeShowThread::endoscopeShow()//��ȡ�ڿ����������
{
	QMutexLocker Locker(&endoscope_mutex);
	cv::Mat frame;
	*capture >> frame;
	auto image = MatToQImage(frame);
	emit change_endoscope_frame(image); //�����ź�
}

void endoscopeShowThread::run()
{
	//���ܼ�this����this�����������߳���
	m_timer = new QTimer();
	m_timer->setInterval(100);//40msˢ��һ��
	connect(m_timer, &QTimer::timeout, this, &endoscopeShowThread::endoscopeShow, Qt::DirectConnection);
	m_timer->start();
	//һ��Ҫ��exec()
	this->exec();//���̲߳�����ִ��ѭ������
} 

endoscopeShowThread::endoscopeShowThread(QObject* parent,
	cv::VideoCapture* capture,QMutex& endoscope_mutex)
	:QThread(parent),endoscope_mutex(endoscope_mutex), capture(capture), m_timer(nullptr){
	
}

endoscopeShowThread::~endoscopeShowThread()
{
	if (m_timer != nullptr)
		delete m_timer;
}



