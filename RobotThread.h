#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include <ur_rtde/rtde_control_interface.h>
#include<ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

//����������߳�δʹ�ã���Ҫʱ�Լ��޸ģ�
using namespace ur_rtde;

class updateRobotDataThread : public QThread
{
	Q_OBJECT

private:
	QTimer* m_timer;
	RTDEReceiveInterface* rtde_r;
	void update();

public:
	updateRobotDataThread(QObject* parent, RTDEReceiveInterface* rtde_r);
	~updateRobotDataThread();
	void run();
};
