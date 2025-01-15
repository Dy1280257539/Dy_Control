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

	//��ȡת������
	void on_outputBase2Cam_button_clicked();

private:
	Ui::getBase2CamControllerClass ui;
	bool* isOpenBase2CamController; //�ж��Ƿ��Ѿ���Base2Cam����̨��boolֵ��ָ��
};
