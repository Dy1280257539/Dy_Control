#include "KinectDK.h"
#include <iostream>
#include <stdexcept>

KinectDK::KinectDK() : is_camera_open(false) {
    
}

KinectDK::~KinectDK() {
    if (is_camera_open) {
        device.stop_cameras();
        device.close();
    }
}

bool KinectDK::open_camera() {

    //获取设备数量
    const uint32_t deviceCount = k4a::device::get_installed_count();
    if (deviceCount == 0)
    {
        qDebug() << "no azure kinect devices detected!" << endl;
        return false; // 如果没有设备，直接返回false
    }

    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30; //帧率
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.synchronized_images_only = true;

    device = k4a::device::open(K4A_DEVICE_DEFAULT);
    device.start_cameras(&config);

    is_camera_open = true;
    return true;
}

k4a::capture KinectDK::capture_frame() {

    QWriteLocker locker(&captureLock);
    if (!is_camera_open || !device.get_capture(&capture)) {
        std::cerr << "Failed to capture a frame" << std::endl;
        throw std::runtime_error("Camera is not open or failed to capture a frame");
    }
    return capture;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KinectDK::get_point_cloud() {

    if (!is_camera_open) {
        std::cerr << "Camera is not open" << std::endl;
        throw std::runtime_error("Camera is not open");
    }

    QReadLocker locker(&captureLock);
    if (!capture) {
        std::cerr << "Invalid capture" << std::endl;
        throw std::runtime_error("Invalid capture");
    }

    k4a::image depth_image = capture.get_depth_image();
    if (!depth_image) {
        std::cerr << "Failed to get depth image from capture" << std::endl;
        throw std::runtime_error("Failed to get depth image");
    }

    k4a::transformation transformation(device.get_calibration(config.depth_mode, config.color_resolution));
    k4a::image point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        depth_image.get_width_pixels(),
        depth_image.get_height_pixels(),
        depth_image.get_width_pixels() * 3 * sizeof(int16_t));

    transformation.depth_image_to_point_cloud(depth_image, K4A_CALIBRATION_TYPE_DEPTH, &point_cloud_image);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = static_cast<uint32_t>(depth_image.get_width_pixels());
    cloud->height = static_cast<uint32_t>(depth_image.get_height_pixels());
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    int16_t* point_cloud_data = reinterpret_cast<int16_t*>(point_cloud_image.get_buffer());
    for (int i = 0; i < cloud->width * cloud->height; ++i) {
        pcl::PointXYZ point;
        point.x = static_cast<float>(point_cloud_data[3 * i]) / 1000.0f;  // Kinect depth units are in millimeters
        point.y = static_cast<float>(point_cloud_data[3 * i + 1]) / 1000.0f;
        point.z = static_cast<float>(point_cloud_data[3 * i + 2]) / 1000.0f;
        cloud->points[i] = point;
    }

    return cloud;
}