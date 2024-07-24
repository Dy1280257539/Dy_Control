#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include <ur_rtde/rtde_control_interface.h>
#include<ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

//此类机器人线程未使用（必要时自己修改）
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
