#include "kinectThread.h"

void kinectShowThread::kinectShow()//读取kinect相机数据
{
	emit change_kinect_frame(kinect->capture_frame());
}

void kinectShowThread::run()
{
	//不能加this，加this就依附于主线程了
	m_timer = new QTimer();
	m_timer->setInterval(40);//40ms刷新一次
	connect(m_timer, &QTimer::timeout, this, &kinectShowThread::kinectShow, Qt::DirectConnection);
	m_timer->start();
	//一定要有exec()
	this->exec();//让线程不死，执行循环队列
}

kinectShowThread::kinectShowThread(QObject* parent, KinectDK* kinect)
	:QThread(parent),kinect(kinect),m_timer(nullptr){

}

kinectShowThread::~kinectShowThread()
{
	if (m_timer != nullptr) {
		delete m_timer;
	}
}