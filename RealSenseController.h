#pragma once

#include <QMainWindow>
#include "RealSenseCameraManager.h"
#include "realSenseThread.h"
#include "ui_RealSenseController.h"
#include "RSfunctionSet.h"
#include <QtCharts>

#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

class RealSenseController : public QMainWindow
{
	Q_OBJECT

public:
	RealSenseController(bool* openFlagAddress, QWidget* parent = nullptr);
	~RealSenseController();

private slots:
	void on_open_RS_Button_clicked();
	void slot_updateUI_RS_image(std::map<std::string, rs2::frame>);//更新RS图像区域

	void on_readPCD_clicked();//读取PCD文件，并显示
	void on_show_cur_pointcloud_clicked();//展示当前相机点云（待调整）
	void on_save_cur_pointcloud_clicked();//保存当前保存的点云

	//点云显示相关
	std::vector<QVector3D> ReadVec3PointCloudPCD(QString path);
	std::vector<QVector3D> PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

private://标签
	bool open_realSenseCameraManager_Flag;//开启RS相机管理器标签

private://线程
	realSenseCameraShowThread* realSenseCamera_Show_Thread;

private:
	std::unordered_map<int, std::string> num2serial;           //相机号码到序列号的映射
	std::unordered_map<std::string, int> serial2num;           //序列号到相机号码的映射

private://点云有关数据
	std::vector<QVector3D> curPointCloudRS1;//当前RS1点云信息
	std::vector<QVector3D> curPointCloudRS2;//当前RS2点云信息

private:
	Ui::RealSenseControllerClass ui;
	RealSenseCameraManager* realSenseCameraManager;//RealSenseCameraManager指针
	bool* openFlagAddress;//主ui传入的open_RealSenseController_Flag的地址，析构时置为false用于告诉主线程已关闭
};
