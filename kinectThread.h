#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include<qlabel.h>
#include"KinectDK.h"

//�����ʵ����Ϊ�������߳��п�����ʱ������ʱ����������㣬�������߳��и���ͼƬ

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
