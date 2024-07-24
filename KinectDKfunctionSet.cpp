#include "KinectDKfunctionSet.h"

QImage capture_to_qimage_color(const k4a::capture& capture) {

    auto color_image = capture.get_color_image();
    if (!color_image) {
        // 如果没有有效的彩色图像，则返回一个空的 QImage
        return QImage();
    }

    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();

    // 假设 Kinect 返回 BGRA 格式的图像
    return QImage(color_image.get_buffer(), width, height, QImage::Format_ARGB32);
}

QImage capture_to_qimage_depth(const k4a::capture& capture) {

    auto depth_image = capture.get_depth_image();
    if (!depth_image) {
        // 如果没有有效的深度图像，则返回一个空的 QImage
        return QImage();
    }

    int width = depth_image.get_width_pixels();
    int height = depth_image.get_height_pixels();
    QImage qimage(width, height, QImage::Format_RGB32);

    uint16_t* depth_data = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint16_t depth_value = depth_data[y * width + x];
            // 将深度值转换为灰度值
            uint8_t intensity = static_cast<uint8_t>((depth_value % 256));
            qimage.setPixel(x, y, qRgb(intensity, intensity, intensity));
        }
    }

    return qimage;
}