#pragma once
#define WIN32_LEAN_AND_MEAN
#define PCL_NO_PRECOMPILE


#include <QtWidgets/QMainWindow>
//#include <QtConcurrent>
#include<QtConcurrent/qtconcurrentrun.h>
//QT_BEGIN_NAMESPACE
#include <QtCharts>
#include "ui_Dy_Control.h"
#include <ur_rtde/rtde_control_interface.h>
#include<ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include<ctime>
#include<iostream>
#include<fstream>
#include<time.h>
#include <sys/timeb.h>
#include<opencv2/opencv.hpp>
#include <QReadWriteLock>
#include <Windows.h>
#include<direct.h>

//���߳�ͷ�ļ�
#include <QtConcurrent/QtConcurrent>
#include"Impedance_control.h"
#include"PID_Control.h"
#include"myqopenglwidget.h"

#include<iostream>//��׼C++���е�������������ͷ�ļ���

#include"endoscopeThread.h"//�ڿ����߳̿�
#include"functionSet.h"//���ܺ�����
#include"KinectController.h" // DK�������̨
#include "RealSenseController.h"//RS�������̨
#include "forceSenseController.h"//������������̨

using namespace ur_rtde;
using namespace std::chrono;
//using namespace QtCharts;
//using namespace cv;


class Dy_Control : public QMainWindow
{
    Q_OBJECT

public:
    Dy_Control(QWidget *parent = nullptr);
    ~Dy_Control();

private slots:
    void on_connect_robot_clicked();
    void on_read_message_clicked();
    void update_force_pose_Data();
    void on_Button_save_clicked();
    void on_button_zeroFSensor_clicked();//����������
    void on_Button_fmode_clicked();//��������ģʽ
    void on_move_x_Button_clicked();
    void on_move_y_Button_clicked();
    void on_move_z_Button_clicked();
    void on_set_init_Button_clicked();//���ó�ʼ��
    void on_move_init_Button_clicked();//�ƶ�����ʼ��
    void on_Rotate_x_Button_clicked();
    void on_Rotate_y_Button_clicked();
    void on_Rotate_z_Button_clicked();
    void on_align_z_Button_clicked();//����Z��
    void on_Show_Now_Button_clicked();
    void on_KinectController_clicked();//����DK�������̨
    void on_RealSenseController_clicked();//����RS�������̨
    void on_forceSenseController_clicked();//��������������̨

    //�ƶ�moveL��Ӧ����
    void move_x_Button_function();
    void move_y_Button_function();
    void move_z_Button_function();
    void Rotate_x_Button_function();
    void Rotate_y_Button_function();
    void Rotate_z_Button_function();

    //�ֶ�ģʽ
    void on_move_byhand_Button_clicked();
    void on_test_Button_clicked(); //���԰�ť

    //����ģʽ
    void on_incremental_Button_clicked();//����ģʽ����
    void incremental_func();//����ģʽ���ܺ���
    void change_incremental_para(int,int,int,int,int,int);//��ť�ؼ��޸����̱߳����Ĳۺ���

    //��һ���ٶ��ƶ�
    void on_move_speedL_Button_clicked();
    void movespeedLThread_function();
    void stop_speedL_function();
    void on_move_speedJ_Button_clicked();
    void movespeedJThread_function();
    void stop_speedJ_function();
    void on_change_Button_clicked(); //stackwidget�л���ť
    void on_change2_Button_clicked();  //stackwidget�л���ť


    //�迹�������
    void on_fmode_Button_clicked();
    void fmode_Button_function();
    void on_icGetInit_Button_clicked();
    void icGetInit_Button_function();
    void on_resetIC_Button_clicked();
    void on_BaseNowIC_Button_clicked();
    void BaseNowIC_Button_function();
    void change_IC_Rotate(int, int);//���ư�ť�����迹���Ƶ�rotate����

    //PID�������������
    void on_PC_Button_clicked();
    void PC_Button_function();

    //�ڿ���������
    void on_endoscopeBuffer_clear_Button_clicked();//���ͼƬ������
    void on_endoscope_read_Button_clicked();//�����ڿ���ͼƬ��������
    void on_endoscope_save_Button_clicked();//����������Ƭ��������
    void on_endoscope_open_Button_clicked();//�����ڿ���
    void slot_updateUI_endoscope_color_image(QImage);//�����ڿ���ͼ������
    
private://����flag��ǩ
    bool all_connectFlag;
    bool r_connectFlag ;
    bool c_connectFlag;
    bool read_message_Flag;
    bool forcemode_Flag ;
    bool movespeedThread_Flag;
    bool movespeedJThread_Flag;
    bool speedL_runoutFlag;
    bool speedJ_runoutFlag;
    bool move_rotate_Flag;
    bool record_trajThread_Flag ;
    bool servoJ_record_traj_Flag ;
    bool io_connectFlag;
    bool show_now_flag;
    bool fmode_flag;//�迹���Ʊ�ǩ
    bool icinit_flag;//�迹���Ƴ�ʼ����ɱ�ǩ
    bool forcemode_Flag1;//�ֶ��ƶ���ģʽ��ǩ
    bool PIDmode_Flag;//PID���Ʊ�ǩ
    bool open_endoscope_Flag;
    bool read_endoscope_Flag;
    bool open_KinectController_Flag;//DK�������̨��ǩ
    bool open_RealSenseController_Flag;//RS�������̨��ǩ
    bool open_forceSenseController_Flag;//������������̨��ǩ
 

private://���ֲ���

    //����Ϣ����ʱ�����ļ�����ʱ�䣨���ڶ���Ϣ���ܣ�
    DWORD dwStart = 0;

    //�ڿ���ͼƬ������
    QReadWriteLock endoscopeBufferLock;
    vector<pair<long long, QImage>> endoscopeBuffer;

    //����ģʽ��д���Լ���Ӧ����
    QReadWriteLock inclock;
    int X = 0,Y = 0, Z = 0, rotateTCPX = 0, rotateTCPY = 0, rotateTCPZ = 0;

    QReadWriteLock rotateLock;
    cv::VideoCapture capture;//����豸
    QMutex endoscope_mutex;//�ڿ��������
    QMutex move_mutex;//�ƶ��߳���
    QTimer* read_message_timer;//��ʱ��ָ��
    QTimer* QTimerForSpeedL;
    QTimer* QTimerForSpeedJ;
    int curPageOfSpeedServe = 0;
    int curPageOfForceMode = 0;
    vector<long long> timedata;
    list<vector<double>> save_excel_worksheet;//������Ϣ����Ҫ�Ķ�ά����
    vector<double> init_pose;//��ǰλ�õļ�¼
    vector<double> vFz;
    vector<double> vXz;

private://�������˲����й����ݺ͹��ܺ���

    int FilterDelay = 200;      //�˲�����ʱ
    double MNoiseCov = 200;     //��������
    double PNoiseCov = 1;       //��������
    vector<list<double>> Euler_Ys;//���������6������,��TCPForce
    vector<list<double>> Euler_Ys_Motion;//���������6������,��MotionForce

private://Dy�����ռ��ڵĲ���
    Dy::Impedance_control* ic; //�迹����
    Dy::PID_Control* pc; //PID��������

private://���ܺ������ʼ������
    vector<double> world_2tcp_force(vector<double> world_force, vector<double>in_pose);//���ڻ�ȡtcp��������world��������ȥ���ܻ�ȡ
    string subreplace(string resource_str, string sub_str, string new_str);//�滻�ַ����ĺ���
    vector<double> Dy_Control::TransAngle(double theta_1, double theta_2, double theta_3, int flag);//����ת������
    LPWSTR ConvertCharToLPWSTR(const char* szString);//��ʽת���Ĺ��ܺ���
    void init();

private: //һЩ����
    Ui::Dy_ControlClass ui;
    RTDEControlInterface* rtde_c;
    RTDEReceiveInterface* rtde_r;
    RTDEIOInterface* rtde_io;
    KinectController* kinectController;
    RealSenseController* realSenseController;
    forceSenseController* fSenseController;

private: //���߳�
    endoscopeShowThread* endoscope_show_thread;//�ڿ�����ʾͼƬ���߳�
};
