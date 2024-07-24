#include "realSenseThread.h"

void realSenseCameraShowThread::rs_Show()//��ȡkinect�������
{
	realSenseCameraManager->updateFrames();
	emit change_rs_frame(realSenseCameraManager->getAllColorFrames());
}

void realSenseCameraShowThread::run()
{
	//���ܼ�this����this�����������߳���
	m_timer = new QTimer();
	m_timer->setInterval(40);//40msˢ��һ��
	connect(m_timer, &QTimer::timeout, this, &realSenseCameraShowThread::rs_Show, Qt::DirectConnection);
	m_timer->start();
	//һ��Ҫ��exec()
	this->exec();//���̲߳�����ִ��ѭ������
}

realSenseCameraShowThread::realSenseCameraShowThread(QObject* parent, RealSenseCameraManager* realSenseCameraManager)
	:QThread(parent), realSenseCameraManager(realSenseCameraManager), m_timer(nullptr) {

}

realSenseCameraShowThread::~realSenseCameraShowThread()
{
	if (m_timer != nullptr) {
		delete m_timer;
	}
}