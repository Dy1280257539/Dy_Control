#pragma once
#include"RobotThread.h"

void updateRobotDataThread::update() {

}

void updateRobotDataThread::run() {

	//不能加this，加this就依附于主线程了
	m_timer = new QTimer();
	m_timer->setInterval(50);//50ms刷新一次
	connect(m_timer, &QTimer::timeout, this, &updateRobotDataThread::update, Qt::DirectConnection);
	m_timer->start();
	//一定要有exec()
	this->exec();//让线程不死，执行循环队列
}

updateRobotDataThread::updateRobotDataThread(QObject* parent, RTDEReceiveInterface* rtde_r):QThread(parent),rtde_r(rtde_r), m_timer(nullptr){

};

updateRobotDataThread::~updateRobotDataThread(){
	if (m_timer != nullptr)
		delete m_timer;
}
