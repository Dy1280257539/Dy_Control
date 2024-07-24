#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include<qlabel.h>
#include"KinectDK.h"

//此类的实现是为了在子线程中开启定时器，定时出发相机拍摄，并在子线程中更新图片

class kinectShowThread : public QThread
{
	Q_OBJECT
signals:
	void change_kinect_frame(k4a::capture);
private:
	QTimer* m_timer;
	KinectDK* kinect;
	void kinectShow();

public:
	kinectShowThread(QObject* parent,KinectDK* kinect);
	~kinectShowThread();
	void run();
};
