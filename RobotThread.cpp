#pragma once
#include"RobotThread.h"

void updateRobotDataThread::update() {

}

void updateRobotDataThread::run() {

	//���ܼ�this����this�����������߳���
	m_timer = new QTimer();
	m_timer->setInterval(50);//50msˢ��һ��
	connect(m_timer, &QTimer::timeout, this, &updateRobotDataThread::update, Qt::DirectConnection);
	m_timer->start();
	//һ��Ҫ��exec()
	this->exec();//���̲߳�����ִ��ѭ������
}

updateRobotDataThread::updateRobotDataThread(QObject* parent, RTDEReceiveInterface* rtde_r):QThread(parent),rtde_r(rtde_r), m_timer(nullptr){

};

updateRobotDataThread::~updateRobotDataThread(){
	if (m_timer != nullptr)
		delete m_timer;
}
