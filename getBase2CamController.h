#pragma once

#include <QMainWindow>
#include "ui_getBase2CamController.h"
#include "KinectDKfunctionSet.h"
#include <QtWidgets/qmessagebox.h>

class getBase2CamController : public QMainWindow
{
	Q_OBJECT

public:
	getBase2CamController(bool * isOpenBase2CamController,QWidget *parent = nullptr);
	~getBase2CamController();

private slots:

	//读取转换矩阵
	void on_outputBase2Cam_button_clicked();

private:
	Ui::getBase2CamControllerClass ui;
	bool* isOpenBase2CamController; //判断是否已经打开Base2Cam控制台的bool值的指针
};
