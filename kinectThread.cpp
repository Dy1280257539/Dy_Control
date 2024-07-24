#include "kinectThread.h"

void kinectShowThread::kinectShow()//��ȡkinect�������
{
	emit change_kinect_frame(kinect->capture_frame());
}

void kinectShowThread::run()
{
	//���ܼ�this����this�����������߳���
	m_timer = new QTimer();
	m_timer->setInterval(40);//40msˢ��һ��
	connect(m_timer, &QTimer::timeout, this, &kinectShowThread::kinectShow, Qt::DirectConnection);
	m_timer->start();
	//һ��Ҫ��exec()
	this->exec();//���̲߳�����ִ��ѭ������
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