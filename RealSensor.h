// RealSenseCamera.h

#ifndef REALSENSE_CAMERA_H
#define REALSENSE_CAMERA_H

#include <librealsense2/rs.hpp>
#include<iostream>

// RealSenseCamera 类提供了对 RealSense 相机的基本操作。
// 它可以初始化相机、捕获帧、获取点云、彩色帧和深度帧，以及释放相机资源。
class RealSenseCamera {
public:
    RealSenseCamera();             // 构造函数
    ~RealSenseCamera();            // 析构函数，用于资源释放

    bool initialize();             // 初始化相机
    bool captureFrame();           // 捕获一帧数据
    void release();                // 释放相机资源

    rs2::points getPointCloud();   // 获取点云数据
    rs2::frame getColorFrame();    // 获取彩色帧数据
    rs2::frame getDepthFrame();    // 获取深度帧数据

private:
    rs2::pipeline pipe;            // RealSense 管道，用于数据流的管理
    rs2::pointcloud pc;            // 用于计算点云
    rs2::frameset frameset;        // 帧集，包含捕获的帧数据
};

#endif // REALSENSE_CAMERA_H
