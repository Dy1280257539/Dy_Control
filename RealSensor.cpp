// RealSenseCamera.cpp

#include "RealSensor.h"

RealSenseCamera::RealSenseCamera() {
    // 构造函数实现（如果需要）
}

RealSenseCamera::~RealSenseCamera() {
    release();
}

bool RealSenseCamera::initialize() {
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);
        return true;
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return false;
    }
}

bool RealSenseCamera::captureFrame() {
    try {
        frameset = pipe.wait_for_frames();
        return true;
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return false;
    }
}

void RealSenseCamera::release() {
    pipe.stop();
}

rs2::points RealSenseCamera::getPointCloud() {
    auto depth = frameset.get_depth_frame();
    pc.map_to(frameset.get_color_frame());
    return pc.calculate(depth);
}

rs2::frame RealSenseCamera::getColorFrame() {
    return frameset.get_color_frame();
}

rs2::frame RealSenseCamera::getDepthFrame() {
    return frameset.get_depth_frame();
}
