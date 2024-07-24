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

	// 与重力补偿有关 测量重力与重心位置
	void on_get_once_Button_clicked(); //获取当前姿态对应的数据
	void on_caculate_Button_clicked(); //根据获取的信息 最小二乘法计算重力 及重心位置
	void on_clear_Button_clicked();  //清空获取的数据

	// 补偿额外力/力矩
	void on_get_Compensation_Button_clicked(); //获取力/力矩数据以及对应 第六轴相对于第五轴的转角
	void on_caculate_Compensation_Button_clicked(); // 计算补偿力矩
	void on_clear_Compensation_Button_clicked(); //清空获取的数据
	void on_cancel_Compensation_Button_clicked(); //取消力/力矩补偿
	void update_Compensation_data(); //获取力/力矩数据


private:

	QTimer* updateTimer;
	int dataIndex = 0;
	QSplineSeries* series[6]; // 六维度力折线图的数据系列 
	QChart* chart[6]; // 六维度力图表
	QString forceName[6];

	//---------------用于进行二次补偿重力----------------- 这部分的笔记内容见语雀传感器部分

	std::vector<std::vector<double>> data; //每一次姿态对应的数据 每一个vector<double> 对应传感器坐标系下 Fx Fy Fz Mx My Mz G'x1 G'y1 G'z1 Mg'x1 Mg'y1 Mg'z1
																								 //0  1  2  3  4  5  6    7     8     9    10    11    对应序号
	std::vector<cv::Mat> Rots_sensor2base; //每一次姿态对应的 传感器坐标系到 基座标系的旋转矩阵

	//-------------------------------------------------


	//--------------用于额外力/力矩补偿--------------------- 这部分的笔记内容见语雀传感器部分

	std::vector<std::vector<double>> Compensation_data; //每一个姿态对应的数据  每一个vector<double> 对应传感器坐标系下 Fx Fy Fz Mx My Mz θ （θ为第六轴相对第五轴弧度 ，即rtde_r->getTargetQ()[5]）
	bool get_Compensation_data_flag;
	QTimer* QTimerForGetCompensation;

	//-------------------------------------------------
	

private:
	Ui::forceSenseControllerClass ui;
	bool* openFlagAddress;//主ui传入的open_forceSenseController_Flag的地址，析构时置为false用于告诉主线程已关闭
	RTDEControlInterface* rtde_c;
	RTDEReceiveInterface* rtde_r;
};
