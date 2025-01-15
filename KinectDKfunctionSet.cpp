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



void saveColorImages(const std::vector<k4a::capture>& photos, const std::string& savePath) {
    int frameIndex = 1; // 从 1 开始命名

    // 确保路径分隔符的处理
    std::string correctedPath = savePath;
    if (!correctedPath.empty() && correctedPath.back() != '/' && correctedPath.back() != '\\') {
        correctedPath += '/'; // 自动补充分隔符
    }

    // 遍历每一帧的捕获数据
    for (const auto& capture : photos) {
        // 从捕获数据中获取彩色图像
        k4a::image colorImage = capture.get_color_image();
        if (!colorImage) {
            std::cerr << "Failed to get color image for frame " << frameIndex << std::endl;
            continue; // 跳过当前帧
        }

        // 获取图像宽度、高度和像素数据
        int width = colorImage.get_width_pixels();
        int height = colorImage.get_height_pixels();
        uint8_t* colorData = colorImage.get_buffer();

        // 使用 OpenCV 将彩色图像数据转换为 Mat 格式（BGRA -> BGR）
        cv::Mat colorMat(height, width, CV_8UC4, colorData); // BGRA 格式
        cv::Mat bgrMat;
        cv::cvtColor(colorMat, bgrMat, cv::COLOR_BGRA2BGR); // 转换为 BGR 格式

        // 构造保存文件的完整路径（以数字命名）
        std::string filename = correctedPath + std::to_string(frameIndex) + ".bmp";

        // 保存图像为 BMP 文件
        if (!cv::imwrite(filename, bgrMat)) {
            std::cerr << "Failed to save image: " << filename << std::endl;
        }

        frameIndex++; // 处理下一帧
    }
}