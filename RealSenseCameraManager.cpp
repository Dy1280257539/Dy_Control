// RealSenseCameraManager.cpp

#include "RealSenseCameraManager.h"
#include <iostream>

RealSenseCameraManager::RealSenseCameraManager():count(0){ /* ... */ }

RealSenseCameraManager::~RealSenseCameraManager() {
    // 释放资源代码
    for (auto& pipeline : pipelines) {
        pipeline.stop();  // 停止每个管道
    }
}

bool RealSenseCameraManager::initializeCameras() {
    try {
        for (auto&& dev : ctx.query_devices()) {
            std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            serial2num[serial] = count;
            num2serial[count++] = serial;
            startPipeline(serial);
            pointclouds[serial] = rs2::pointcloud();
        }
        return true;
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return false;
    }
}

int RealSenseCameraManager::getNumOfRS() {
    return this->count;
}

std::unordered_map<std::string, int> RealSenseCameraManager::getSerial2num() {
    return this->serial2num;
}

std::unordered_map<int, std::string> RealSenseCameraManager::getNum2serial() {
    return this->num2serial;
}


void RealSenseCameraManager::startPipeline(const std::string& serial) {
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(serial); //使能特定设备
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);//配置数据流格式
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
}

void RealSenseCameraManager::updateFrames() {

    QWriteLocker locker(&framesetsLock);
    for (size_t i = 0; i < pipelines.size(); ++i) {
        rs2::frameset fs;
        if (pipelines[i].poll_for_frames(&fs)) {
            std::string serial = pipelines[i].get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            framesets[serial] = fs;
        }
    }
}

rs2::frame RealSenseCameraManager::getColorFrame(const std::string& serial) {

    QReadLocker locker(&framesetsLock);
    return framesets[serial].get_color_frame();
}

rs2::frame RealSenseCameraManager::getDepthFrame(const std::string& serial) {

    QReadLocker locker(&framesetsLock);
    return framesets[serial].get_depth_frame();
}

rs2::points RealSenseCameraManager::getPointCloud(const std::string& serial) {

    QReadLocker locker(&framesetsLock);
    auto depth = framesets[serial].get_depth_frame();
    // 将点云映射到当前彩色帧
    pointclouds[serial].map_to(framesets[serial].get_color_frame());
    return pointclouds[serial].calculate(depth);
}

std::map<std::string, rs2::frame> RealSenseCameraManager::getAllColorFrames() {

    QReadLocker locker(&framesetsLock);
    std::map<std::string, rs2::frame> all_color_frames;
    for (const auto& fs : framesets) {
        all_color_frames[fs.first] = fs.second.get_color_frame();
    }
    return all_color_frames;
}

std::map<std::string, rs2::frame> RealSenseCameraManager::getAllDepthFrames() {

    QReadLocker locker(&framesetsLock);
    std::map<std::string, rs2::frame> all_depth_frames;
    for (const auto& fs : framesets) {
        all_depth_frames[fs.first] = fs.second.get_depth_frame();
    }
    return all_depth_frames;
}

std::map<std::string, rs2::points> RealSenseCameraManager::getAllPointClouds() {

    QReadLocker locker(&framesetsLock);
    std::map<std::string, rs2::points> all_point_clouds;
    for (const auto& pair : framesets) {
        auto depth = pair.second.get_depth_frame();
        pointclouds[pair.first].map_to(pair.second.get_color_frame());
        all_point_clouds[pair.first] = pointclouds[pair.first].calculate(depth);
    }
    return all_point_clouds;
}