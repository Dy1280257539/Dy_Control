#pragma once
#include <QMainWindow>
#include "ui_KinectController.h"
#include"KinectDK.h"
#include"myqopenglwidget.h"
#include <QtCharts>

#include<pcl/visualization/cloud_viewer.h>
#include<iostream>//��׼C++���е�������������ͷ�ļ���
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include"kinectThread.h"//kinect����߳�

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
	void slot_updateUI_kinect_image(k4a::capture);//����kinectͼ������
	void on_readPCD_clicked();//��ȡPCD�ļ�������ʾ
	void on_show_cur_pointcloud_clicked();//չʾ��ǰ������ƣ���������
	void on_save_cur_pointcloud_clicked();//���浱ǰ����ĵ���

	//������ʾ���
	std::vector<QVector3D> ReadVec3PointCloudPCD(QString path);
	std::vector<QVector3D> PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

private://��ǩ
	bool open_Kinect_Flag;//����Kinect�����ǩ

private://�����й�����
	std::vector<QVector3D> curPointCloud;//��ǰ������Ϣ

private://�߳�
	kinectShowThread* kinect_show_thread; //kinectչʾͼƬ���߳�

private:
	Ui::KinectControllerClass ui;
	KinectDK* kinect;//kinect���ָ��
	bool* openFlagAddress;//��ui�����open_KinectController_Flag�ĵ�ַ������ʱ��Ϊfalse���ڸ������߳��ѹر�
};
