#pragma once
#include <QMainWindow>
#include "ui_KinectController.h"
#include"KinectDK.h"
#include"myqopenglwidget.h"
#include <QtCharts>

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

#include<vector>

class KinectController : public QMainWindow
{
	Q_OBJECT

public:
	KinectController(bool* openFlagAddress,QWidget *parent = nullptr);
	~KinectController();

private slots:
	void on_open_Kinect_Button_clicked();
	void slot_updateUI_kinect_image(k4a::capture);//更新kinect图像区域
	void on_readPCD_clicked();//读取PCD文件，并显示
	void on_show_cur_pointcloud_clicked();//展示当前相机点云（待调整）
	void on_save_cur_pointcloud_clicked();//保存当前保存的点云

	//点云显示相关
	std::vector<QVector3D> ReadVec3PointCloudPCD(QString path);
	std::vector<QVector3D> PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

private://标签
	bool open_Kinect_Flag;//开启Kinect相机标签

private://点云有关数据
	std::vector<QVector3D> curPointCloud;//当前点云信息

private://线程
	kinectShowThread* kinect_show_thread; //kinect展示图片子线程

private:
	Ui::KinectControllerClass ui;
	KinectDK* kinect;//kinect相机指针
	bool* openFlagAddress;//主ui传入的open_KinectController_Flag的地址，析构时置为false用于告诉主线程已关闭
};
