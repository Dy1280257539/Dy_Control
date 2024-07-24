#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include<qlabel.h>
#include"RealSenseCameraManager.h"

//此类的实现是为了在子线程中开启定时器，定时触发相机拍摄，并在子线程中更新图片

class realSenseCameraShowThread : public QThread
{
	Q_OBJECT
signals:
	void change_rs_frame(std::map<std::string, rs2::frame>);
private:
	QTimer* m_timer;
	RealSenseCameraManager* realSenseCameraManager;
	void rs_Show();

public:
	realSenseCameraShowThread(QObject* parent, RealSenseCameraManager* realSenseCameraManager);
	~realSenseCameraShowThread();
	void run();
};
