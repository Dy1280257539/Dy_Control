#include "KinectDKfunctionSet.h"

QImage capture_to_qimage_color(const k4a::capture& capture) {

    auto color_image = capture.get_color_image();
    if (!color_image) {
        // ���û����Ч�Ĳ�ɫͼ���򷵻�һ���յ� QImage
        return QImage();
    }

    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();

    // ���� Kinect ���� BGRA ��ʽ��ͼ��
    return QImage(color_image.get_buffer(), width, height, QImage::Format_ARGB32);
}

QImage capture_to_qimage_depth(const k4a::capture& capture) {

    auto depth_image = capture.get_depth_image();
    if (!depth_image) {
        // ���û����Ч�����ͼ���򷵻�һ���յ� QImage
        return QImage();
    }

    int width = depth_image.get_width_pixels();
    int height = depth_image.get_height_pixels();
    QImage qimage(width, height, QImage::Format_RGB32);

    uint16_t* depth_data = reinterpret_cast<uint16_t*>(depth_image.get_buffer());
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint16_t depth_value = depth_data[y * width + x];
            // �����ֵת��Ϊ�Ҷ�ֵ
            uint8_t intensity = static_cast<uint8_t>((depth_value % 256));
            qimage.setPixel(x, y, qRgb(intensity, intensity, intensity));
        }
    }

    return qimage;
}