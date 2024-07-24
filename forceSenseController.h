#pragma once

#include <winsock2.h>
#include <QMainWindow>
#include <QtCharts>
QT_CHARTS_USE_NAMESPACE
#include "ui_forceSenseController.h"
#include <ur_rtde/rtde_control_interface.h>
#include<ur_rtde/rtde_receive_interface.h>
#include <QChartView>
#include <QLineSeries>
#include "functionSet.h"

using namespace QtCharts;

using namespace ur_rtde;

class forceSenseController : public QMainWindow
{
	Q_OBJECT

public:
	forceSenseController(bool* openFlagAddress, RTDEControlInterface* rtde_c,RTDEReceiveInterface* rtde_r,QWidget *parent = nullptr);
	~forceSenseController();

private slots:
	void on_readForceAndShowButton_clicked();
	void on_zeroSenseButton_clicked();
	void updateChartData();

	// �����������й� ��������������λ��
	void on_get_once_Button_clicked(); //��ȡ��ǰ��̬��Ӧ������
	void on_caculate_Button_clicked(); //���ݻ�ȡ����Ϣ ��С���˷��������� ������λ��
	void on_clear_Button_clicked();  //��ջ�ȡ������

	// ����������/����
	void on_get_Compensation_Button_clicked(); //��ȡ��/���������Լ���Ӧ ����������ڵ������ת��
	void on_caculate_Compensation_Button_clicked(); // ���㲹������
	void on_clear_Compensation_Button_clicked(); //��ջ�ȡ������
	void on_cancel_Compensation_Button_clicked(); //ȡ����/���ز���
	void update_Compensation_data(); //��ȡ��/��������


private:

	QTimer* updateTimer;
	int dataIndex = 0;
	QSplineSeries* series[6]; // ��ά��������ͼ������ϵ�� 
	QChart* chart[6]; // ��ά����ͼ��
	QString forceName[6];

	//---------------���ڽ��ж��β�������----------------- �ⲿ�ֵıʼ����ݼ���ȸ����������

	std::vector<std::vector<double>> data; //ÿһ����̬��Ӧ������ ÿһ��vector<double> ��Ӧ����������ϵ�� Fx Fy Fz Mx My Mz G'x1 G'y1 G'z1 Mg'x1 Mg'y1 Mg'z1
																								 //0  1  2  3  4  5  6    7     8     9    10    11    ��Ӧ���
	std::vector<cv::Mat> Rots_sensor2base; //ÿһ����̬��Ӧ�� ����������ϵ�� ������ϵ����ת����

	//-------------------------------------------------


	//--------------���ڶ�����/���ز���--------------------- �ⲿ�ֵıʼ����ݼ���ȸ����������

	std::vector<std::vector<double>> Compensation_data; //ÿһ����̬��Ӧ������  ÿһ��vector<double> ��Ӧ����������ϵ�� Fx Fy Fz Mx My Mz �� ����Ϊ��������Ե����ỡ�� ����rtde_r->getTargetQ()[5]��
	bool get_Compensation_data_flag;
	QTimer* QTimerForGetCompensation;

	//-------------------------------------------------
	

private:
	Ui::forceSenseControllerClass ui;
	bool* openFlagAddress;//��ui�����open_forceSenseController_Flag�ĵ�ַ������ʱ��Ϊfalse���ڸ������߳��ѹر�
	RTDEControlInterface* rtde_c;
	RTDEReceiveInterface* rtde_r;
};
