#pragma once

#include <QTimer>
#include <QThread>
#include<qmutex.h>
#include<qlabel.h>
#include<opencv.hpp>
#include "opencv2/imgproc/types_c.h"
#include"functionSet.h"

//声明功能函数
QImage MatToQImage(const cv::Mat& mat);


//此类的实现是为了在子线程中开启定时器，定时触发内窥镜拍摄，并在子线程中更新图片

class endoscopeShowThread : public QThread
{
	Q_OBJECT

signals:
	void change_endoscope_frame(QImage);//声明信号
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
