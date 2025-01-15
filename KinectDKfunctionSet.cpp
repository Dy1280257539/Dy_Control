#include "KinectDKfunctionSet.h"

static std::vector<double> base2cam = {0.0,0.0,0.0,0.0,0.0,0.0};

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



void saveColorImages(const std::vector<k4a::capture>& photos, const std::string& savePath) {
    int frameIndex = 1; // �� 1 ��ʼ����

    // ȷ��·���ָ����Ĵ���
    std::string correctedPath = savePath;
    if (!correctedPath.empty() && correctedPath.back() != '/' && correctedPath.back() != '\\') {
        correctedPath += '/'; // �Զ�����ָ���
    }

    // ����ÿһ֡�Ĳ�������
    for (const auto& capture : photos) {
        // �Ӳ��������л�ȡ��ɫͼ��
        k4a::image colorImage = capture.get_color_image();
        if (!colorImage) {
            std::cerr << "Failed to get color image for frame " << frameIndex << std::endl;
            continue; // ������ǰ֡
        }

        // ��ȡͼ���ȡ��߶Ⱥ���������
        int width = colorImage.get_width_pixels();
        int height = colorImage.get_height_pixels();
        uint8_t* colorData = colorImage.get_buffer();

        // ʹ�� OpenCV ����ɫͼ������ת��Ϊ Mat ��ʽ��BGRA -> BGR��
        cv::Mat colorMat(height, width, CV_8UC4, colorData); // BGRA ��ʽ
        cv::Mat bgrMat;
        cv::cvtColor(colorMat, bgrMat, cv::COLOR_BGRA2BGR); // ת��Ϊ BGR ��ʽ

        // ���챣���ļ�������·����������������
        std::string filename = correctedPath + std::to_string(frameIndex) + ".bmp";

        // ����ͼ��Ϊ BMP �ļ�
        if (!cv::imwrite(filename, bgrMat)) {
            std::cerr << "Failed to save image: " << filename << std::endl;
        }

        frameIndex++; // ������һ֡
    }
}


std::vector<double> getBase2Cam() {
    // ���ص�ǰ�� base2cam ֵ
    return base2cam;
}


void updateBase2Cam(const cv::Mat& transformMatrix) {
    // �����������Ƿ��� 4x4
    if (transformMatrix.rows != 4 || transformMatrix.cols != 4) {
        throw std::invalid_argument("Input transformMatrix must be a 4x4 matrix.");
    }

    // ��ȡƽ������ (x, y, z)
    double x = transformMatrix.at<double>(0, 3);
    double y = transformMatrix.at<double>(1, 3);
    double z = transformMatrix.at<double>(2, 3);

    // ��ȡ��ת���� (3x3)
    cv::Mat rotationMatrix = transformMatrix(cv::Range(0, 3), cv::Range(0, 3));

    // ����ת����ת��Ϊ Rodrigues ��ת���� (rx, ry, rz)
    cv::Mat rotationVector;
    cv::Rodrigues(rotationMatrix, rotationVector);

    // ���� base2cam ��ֵ
    base2cam = { x, y, z, rotationVector.at<double>(0), rotationVector.at<double>(1), rotationVector.at<double>(2) };
}