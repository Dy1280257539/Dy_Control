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

    // 打开并初始化Kinect相机。成功时返回true，失败时返回false。
    bool open_camera();

    // 捕获并返回一帧图像。如果相机未打开或捕获失败，将抛出异常。
    k4a::capture capture_frame();

    // 获取并返回当前捕获帧的点云。如果相机未打开，将抛出异常。
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_point_cloud();

private:
    k4a::device device;        // Kinect设备实例
    k4a::capture capture;      // 用于存储捕获帧的对象
    bool is_camera_open;       // 标识相机是否已打开
    k4a_device_configuration_t config;
    QReadWriteLock captureLock; // 读写锁，用于保护 capture
};

#endif // KINECTDK_H
