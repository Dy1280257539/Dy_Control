#include "realSenseThread.h"

void realSenseCameraShowThread::rs_Show()//读取kinect相机数据
{
	realSenseCameraManager->updateFrames();
	emit change_rs_frame(realSenseCameraManager->getAllColorFrames());
}

void realSenseCameraShowThread::run()
{
	//不能加this，加this就依附于主线程了
	m_timer = new QTimer();
	m_timer->setInterval(40);//40ms刷新一次
	connect(m_timer, &QTimer::timeout, this, &realSenseCameraShowThread::rs_Show, Qt::DirectConnection);
	m_timer->start();
	//一定要有exec()
	this->exec();//让线程不死，执行循环队列
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