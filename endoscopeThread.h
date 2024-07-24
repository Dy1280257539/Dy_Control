#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include<qlabel.h>
#include<opencv.hpp>
#include "opencv2/imgproc/types_c.h"
#include"functionSet.h"

//�������ܺ���
QImage MatToQImage(const cv::Mat& mat);


//�����ʵ����Ϊ�������߳��п�����ʱ������ʱ�����ڿ������㣬�������߳��и���ͼƬ

class endoscopeShowThread : public QThread
{
	Q_OBJECT

signals:
	void change_endoscope_frame(QImage);//�����ź�
private:
	QTimer* m_timer;
	cv::VideoCapture* capture;
	QMutex& endoscope_mutex;
	void endoscopeShow();

public:
	endoscopeShowThread(QObject* parent,
		 cv::VideoCapture* capture,QMutex& endoscope_mutex);
	~endoscopeShowThread();
	void run();
};
