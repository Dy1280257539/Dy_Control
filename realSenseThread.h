#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include<qlabel.h>
#include"RealSenseCameraManager.h"

//�����ʵ����Ϊ�������߳��п�����ʱ������ʱ����������㣬�������߳��и���ͼƬ

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
