#pragma once
#include <QMainWindow>
#include "ui_KinectController.h"
#include"KinectDK.h"
#include"myqopenglwidget.h"
#include <QtCharts>

#include<ur_rtde/rtde_receive_interface.h>

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

#include "getBase2CamController.h"

#include<vector>

using namespace ur_rtde;

/*
	ע�����
	1.���۱궨�У�������������Ļ����˵���̬�����������£�
		1 rx ry rz x y z
		2 rx ry rz x y z
	  ��7ά�ȵģ���һά����ţ��������� rvec��Ȼ���� tvec

*/

class KinectController : public QMainWindow
{
	Q_OBJECT

public:
	KinectController(bool* openFlagAddress,bool * r_connectFlagAddress = nullptr, RTDEReceiveInterface** rtde_r_Address = nullptr,QWidget *parent = nullptr);
	~KinectController();

private slots:
	void on_open_Kinect_Button_clicked();
	void slot_updateUI_kinect_image(k4a::capture);//����kinectͼ������
	void on_readPCD_clicked();//��ȡPCD�ļ�������ʾ
	void on_show_cur_pointcloud_clicked();//չʾ��ǰ������ƣ���������
	void on_save_cur_pointcloud_clicked();//���浱ǰ����ĵ���
	
	//���۱궨
	void on_get_frame_and_pose_Button_clicked();//��ȡһ������
	void on_save_all_data_Button_clicked(); //���浱ǰ����
	void on_clear_all_data_Button_clicked();//��յ�ǰ��������
	void on_getBase2Cam_Button_clicked(); //����Base2Cam ת������

	//������ʾ���
	std::vector<QVector3D> ReadVec3PointCloudPCD(QString path);
	std::vector<QVector3D> PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

private://��ǩ
	bool open_Kinect_Flag;//����Kinect�����ǩ

private://�����й�����
	std::vector<QVector3D> curPointCloud;//��ǰ������Ϣ

private://�����۱궨���
	int cur_data_num = 0; //��ǰ���۱궨��Ӧ������ һ�����ݺ�һ��ͼƬ��һ����������̬(base 2 gripper)
	std::vector<std::vector<double>> poses; //������gripper����̬
	std::vector<k4a::capture> photos; // ��������ÿһ֡����
	bool isOpenBase2CamController = false; //�Ƿ��Ѿ����˿���̨


private://�߳�
	kinectShowThread* kinect_show_thread; //kinectչʾͼƬ���߳�

private:
	Ui::KinectControllerClass ui;
	KinectDK* kinect;//kinect���ָ��
	bool* openFlagAddress;//��ui�����open_KinectController_Flag�ĵ�ַ������ʱ��Ϊfalse���ڸ������߳��ѹر�
	bool* r_connectFlagAddress; //��ui�����r_connectFlag�ĵ�ַ�������жϻ������Ƿ��������Ƿ�ɶ�
	RTDEReceiveInterface** rtde_r_Address; //��ui����Ķ�ȡ��������Ϣ��ָ��ĵ�ַ
	getBase2CamController* get_Base2Cam_Controller;
};
