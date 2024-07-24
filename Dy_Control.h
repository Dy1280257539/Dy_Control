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

//多线程头文件
#include <QtConcurrent/QtConcurrent>
#include"Impedance_control.h"
#include"PID_Control.h"
#include"myqopenglwidget.h"

#include<iostream>//标准C++库中的输入输出类相关头文件。

#include"endoscopeThread.h"//内窥镜线程库
#include"functionSet.h"//功能函数库
#include"KinectController.h" // DK相机控制台
#include "RealSenseController.h"//RS相机控制台
#include "forceSenseController.h"//力传感器控制台

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
    void on_button_zeroFSensor_clicked();//传感器清零
    void on_Button_fmode_clicked();//机器人力模式
    void on_move_x_Button_clicked();
    void on_move_y_Button_clicked();
    void on_move_z_Button_clicked();
    void on_set_init_Button_clicked();//设置初始点
    void on_move_init_Button_clicked();//移动到初始点
    void on_Rotate_x_Button_clicked();
    void on_Rotate_y_Button_clicked();
    void on_Rotate_z_Button_clicked();
    void on_align_z_Button_clicked();//对齐Z轴
    void on_Show_Now_Button_clicked();
    void on_KinectController_clicked();//开启DK相机控制台
    void on_RealSenseController_clicked();//开启RS相机控制台
    void on_forceSenseController_clicked();//打开力传感器控制台

    //移动moveL对应函数
    void move_x_Button_function();
    void move_y_Button_function();
    void move_z_Button_function();
    void Rotate_x_Button_function();
    void Rotate_y_Button_function();
    void Rotate_z_Button_function();

    //手动模式
    void on_move_byhand_Button_clicked();
    void on_test_Button_clicked(); //测试按钮

    //增量模式
    void on_incremental_Button_clicked();//增量模式开启
    void incremental_func();//增量模式功能函数
    void change_incremental_para(int,int,int,int,int,int);//按钮控件修改主线程变量的槽函数

    //按一定速度移动
    void on_move_speedL_Button_clicked();
    void movespeedLThread_function();
    void stop_speedL_function();
    void on_move_speedJ_Button_clicked();
    void movespeedJThread_function();
    void stop_speedJ_function();
    void on_change_Button_clicked(); //stackwidget切换按钮
    void on_change2_Button_clicked();  //stackwidget切换按钮


    //阻抗控制相关
    void on_fmode_Button_clicked();
    void fmode_Button_function();
    void on_icGetInit_Button_clicked();
    void icGetInit_Button_function();
    void on_resetIC_Button_clicked();
    void on_BaseNowIC_Button_clicked();
    void BaseNowIC_Button_function();
    void change_IC_Rotate(int, int);//控制按钮更新阻抗控制的rotate变量

    //PID法向力控制相关
    void on_PC_Button_clicked();
    void PC_Button_function();

    //内窥镜相机相关
    void on_endoscopeBuffer_clear_Button_clicked();//清空图片缓冲区
    void on_endoscope_read_Button_clicked();//保存内窥镜图片到缓冲区
    void on_endoscope_save_Button_clicked();//将缓冲区的片保存下来
    void on_endoscope_open_Button_clicked();//开启内窥镜
    void slot_updateUI_endoscope_color_image(QImage);//更新内窥镜图像区域
    
private://各种flag标签
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
    bool fmode_flag;//阻抗控制标签
    bool icinit_flag;//阻抗控制初始化完成标签
    bool forcemode_Flag1;//手动移动力模式标签
    bool PIDmode_Flag;//PID控制标签
    bool open_endoscope_Flag;
    bool read_endoscope_Flag;
    bool open_KinectController_Flag;//DK相机控制台标签
    bool open_RealSenseController_Flag;//RS相机控制台标签
    bool open_forceSenseController_Flag;//力传感器控制台标签
 

private://各种参数

    //读信息启动时开启的计数器时间（用于读信息功能）
    DWORD dwStart = 0;

    //内窥镜图片缓冲区
    QReadWriteLock endoscopeBufferLock;
    vector<pair<long long, QImage>> endoscopeBuffer;

    //增量模式读写锁以及对应变量
    QReadWriteLock inclock;
    int X = 0,Y = 0, Z = 0, rotateTCPX = 0, rotateTCPY = 0, rotateTCPZ = 0;

    QReadWriteLock rotateLock;
    cv::VideoCapture capture;//相机设备
    QMutex endoscope_mutex;//内窥镜相机锁
    QMutex move_mutex;//移动线程锁
    QTimer* read_message_timer;//定时器指针
    QTimer* QTimerForSpeedL;
    QTimer* QTimerForSpeedJ;
    int curPageOfSpeedServe = 0;
    int curPageOfForceMode = 0;
    vector<long long> timedata;
    list<vector<double>> save_excel_worksheet;//保存信息所需要的二维数组
    vector<double> init_pose;//当前位置的记录
    vector<double> vFz;
    vector<double> vXz;

private://卡尔曼滤波的有关数据和功能函数

    int FilterDelay = 200;      //滤波器延时
    double MNoiseCov = 200;     //测量噪声
    double PNoiseCov = 1;       //过程噪声
    vector<list<double>> Euler_Ys;//卡尔曼存的6个链表,滤TCPForce
    vector<list<double>> Euler_Ys_Motion;//卡尔曼存的6个链表,滤MotionForce

private://Dy命名空间内的参数
    Dy::Impedance_control* ic; //阻抗控制
    Dy::PID_Control* pc; //PID恒力控制

private://功能函数与初始化函数
    vector<double> world_2tcp_force(vector<double> world_force, vector<double>in_pose);//用于获取tcp的力，将world的力代进去就能获取
    string subreplace(string resource_str, string sub_str, string new_str);//替换字符串的函数
    vector<double> Dy_Control::TransAngle(double theta_1, double theta_2, double theta_3, int flag);//矩阵转换函数
    LPWSTR ConvertCharToLPWSTR(const char* szString);//格式转换的功能函数
    void init();

private: //一些参数
    Ui::Dy_ControlClass ui;
    RTDEControlInterface* rtde_c;
    RTDEReceiveInterface* rtde_r;
    RTDEIOInterface* rtde_io;
    KinectController* kinectController;
    RealSenseController* realSenseController;
    forceSenseController* fSenseController;

private: //子线程
    endoscopeShowThread* endoscope_show_thread;//内窥镜显示图片子线程
};
