#pragma once
#include <QMainWindow>
#include "ui_KinectController.h"
#include"KinectDK.h"
#include"myqopenglwidget.h"
#include <QtCharts>

#include<ur_rtde/rtde_receive_interface.h>

#include<pcl/visualization/cloud_viewer.h>
#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include"kinectThread.h"//kinect相机线程

#include "KinectDKfunctionSet.h"
#include"pcfunctionSet.h"

#include "getBase2CamController.h"

#include<vector>

using namespace ur_rtde;

/*
	注意事项：
	1.手眼标定中，保存数据输出的机器人的姿态数据类似如下：
		1 rx ry rz x y z
		2 rx ry rz x y z
	  是7维度的，第一维是组号，接下来是 rvec，然后是 tvec

*/

class KinectController : public QMainWindow
{
	Q_OBJECT

public:
	KinectController(bool* openFlagAddress,bool * r_connectFlagAddress = nullptr, RTDEReceiveInterface** rtde_r_Address = nullptr,QWidget *parent = nullptr);
	~KinectController();

private slots:
	void on_open_Kinect_Button_clicked();
	void slot_updateUI_kinect_image(k4a::capture);//更新kinect图像区域
	void on_readPCD_clicked();//读取PCD文件，并显示
	void on_show_cur_pointcloud_clicked();//展示当前相机点云（待调整）
	void on_save_cur_pointcloud_clicked();//保存当前保存的点云
	
	//手眼标定
	void on_get_frame_and_pose_Button_clicked();//获取一组数据
	void on_save_all_data_Button_clicked(); //保存当前数据
	void on_clear_all_data_Button_clicked();//清空当前所有数据
	void on_getBase2Cam_Button_clicked(); //输入Base2Cam 转换矩阵

	//点云显示相关
	std::vector<QVector3D> ReadVec3PointCloudPCD(QString path);
	std::vector<QVector3D> PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

private://标签
	bool open_Kinect_Flag;//开启Kinect相机标签

private://点云有关数据
	std::vector<QVector3D> curPointCloud;//当前点云信息

private://与手眼标定相关
	int cur_data_num = 0; //当前手眼标定对应的数据 一组数据含一张图片和一个机器人姿态(base 2 gripper)
	std::vector<std::vector<double>> poses; //机器人gripper的姿态
	std::vector<k4a::capture> photos; // 相机拍摄的每一帧数据
	bool isOpenBase2CamController = false; //是否已经打开了控制台


private://线程
	kinectShowThread* kinect_show_thread; //kinect展示图片子线程

private:
	Ui::KinectControllerClass ui;
	KinectDK* kinect;//kinect相机指针
	bool* openFlagAddress;//主ui传入的open_KinectController_Flag的地址，析构时置为false用于告诉主线程已关闭
	bool* r_connectFlagAddress; //主ui传入的r_connectFlag的地址，用于判断机器人是否连接且是否可读
	RTDEReceiveInterface** rtde_r_Address; //主ui传入的读取机器人信息的指针的地址
	getBase2CamController* get_Base2Cam_Controller;
};
