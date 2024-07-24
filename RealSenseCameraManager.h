// RealSenseCameraManager.h

#ifndef REALSENSE_CAMERA_MANAGER_H
#define REALSENSE_CAMERA_MANAGER_H

#include <librealsense2/rs.hpp>
#include <map>
#include <unordered_map>
#include <string>
#include <QReadWriteLock>

class RealSenseCameraManager {
public:
    RealSenseCameraManager();
    ~RealSenseCameraManager();

    bool initializeCameras();  // 初始化所有连接的相机
    void updateFrames();       // 更新所有相机的帧数据

    rs2::frame getColorFrame(const std::string& serial); // 获取指定相机的彩色帧
    rs2::frame getDepthFrame(const std::string& serial); // 获取指定相机的深度帧
    rs2::points getPointCloud(const std::string& serial); // 获取指定相机的点云

    std::map<std::string, rs2::frame> getAllColorFrames();    // 获取所有设备的彩色帧
    std::map<std::string, rs2::frame> getAllDepthFrames();    // 获取所有设备的深度帧
    std::map<std::string, rs2::points> getAllPointClouds();   // 获取所有设备的点云

    int getNumOfRS();

    std::unordered_map<std::string, int> getSerial2num();     //获取serial2num
    std::unordered_map<int, std::string> getNum2serial();     //获取serial2num

private:
    rs2::context ctx;   // 用于管理设备的 RealSense 上下文
    std::vector<rs2::pipeline> pipelines; // 每个设备的管道
    std::map<std::string, rs2::frameset> framesets; // 每个设备的最新帧数据
    QReadWriteLock framesetsLock; // 读写锁，用于保护 framesets
    std::map<std::string, rs2::pointcloud> pointclouds; // 每个设备的点云处理对象
    std::unordered_map<int, std::string> num2serial;           //相机号码到序列号的映射
    std::unordered_map<std::string, int> serial2num;           //序列号到相机号码的映射
    int count;                                                 //相机总数量

    void startPipeline(const std::string& serial); // 开始给定序列号的管道
};

#endif // REALSENSE_CAMERA_MANAGER_H
