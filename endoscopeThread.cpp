#include"endoscopeThread.h"


void endoscopeShowThread::endoscopeShow()//读取内窥镜相机数据
{
	QMutexLocker Locker(&endoscope_mutex);
	cv::Mat frame;
	*capture >> frame;
	auto image = MatToQImage(frame);
	emit change_endoscope_frame(image); //触发信号
}

void endoscopeShowThread::run()
{
	//不能加this，加this就依附于主线程了
	m_timer = new QTimer();
	m_timer->setInterval(100);//40ms刷新一次
	connect(m_timer, &QTimer::timeout, this, &endoscopeShowThread::endoscopeShow, Qt::DirectConnection);
	m_timer->start();
	//一定要有exec()
	this->exec();//让线程不死，执行循环队列
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



