#pragma once

#include <QMainWindow>
#include "RealSenseCameraManager.h"
#include "realSenseThread.h"
#include "ui_RealSenseController.h"
#include "RSfunctionSet.h"
#include <QtCharts>

#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
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
	void slot_updateUI_RS_image(std::map<std::string, rs2::frame>);//����RSͼ������

	void on_readPCD_clicked();//��ȡPCD�ļ�������ʾ
	void on_show_cur_pointcloud_clicked();//չʾ��ǰ������ƣ���������
	void on_save_cur_pointcloud_clicked();//���浱ǰ����ĵ���

	//������ʾ���
	std::vector<QVector3D> ReadVec3PointCloudPCD(QString path);
	std::vector<QVector3D> PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

private://��ǩ
	bool open_realSenseCameraManager_Flag;//����RS�����������ǩ

private://�߳�
	realSenseCameraShowThread* realSenseCamera_Show_Thread;

private:
	std::unordered_map<int, std::string> num2serial;           //������뵽���кŵ�ӳ��
	std::unordered_map<std::string, int> serial2num;           //���кŵ���������ӳ��

private://�����й�����
	std::vector<QVector3D> curPointCloudRS1;//��ǰRS1������Ϣ
	std::vector<QVector3D> curPointCloudRS2;//��ǰRS2������Ϣ

private:
	Ui::RealSenseControllerClass ui;
	RealSenseCameraManager* realSenseCameraManager;//RealSenseCameraManagerָ��
	bool* openFlagAddress;//��ui�����open_RealSenseController_Flag�ĵ�ַ������ʱ��Ϊfalse���ڸ������߳��ѹر�
};
