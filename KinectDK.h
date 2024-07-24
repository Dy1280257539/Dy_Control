#ifndef KINECTDK_H
#define KINECTDK_H

#include <k4a/k4a.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QtCharts>

class KinectDK {
public:
    KinectDK();
    ~KinectDK();

    // �򿪲���ʼ��Kinect������ɹ�ʱ����true��ʧ��ʱ����false��
    bool open_camera();

    // ���񲢷���һ֡ͼ��������δ�򿪻򲶻�ʧ�ܣ����׳��쳣��
    k4a::capture capture_frame();

    // ��ȡ�����ص�ǰ����֡�ĵ��ơ�������δ�򿪣����׳��쳣��
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_point_cloud();

private:
    k4a::device device;        // Kinect�豸ʵ��
    k4a::capture capture;      // ���ڴ洢����֡�Ķ���
    bool is_camera_open;       // ��ʶ����Ƿ��Ѵ�
    k4a_device_configuration_t config;
    QReadWriteLock captureLock; // ��д�������ڱ��� capture
};

#endif // KINECTDK_H
