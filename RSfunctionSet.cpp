#include"RSfunctionSet.h"


QImage rsFrameToQImage(const rs2::frame& f) {
    // ���֡�Ƿ���Ч
    if (!f) {
        return QImage();
    }

    // ��ȡ֡�Ŀ�Ⱥ͸߶�
    const int w = f.as<rs2::video_frame>().get_width();
    const int h = f.as<rs2::video_frame>().get_height();

    // ���֡�ĸ�ʽ
    if (f.get_profile().format() == RS2_FORMAT_RGB8) {
        // �������ݵ� QImage
        return QImage((const uchar*)f.get_data(), w, h, QImage::Format_RGB888).copy();
    }
    else if (f.get_profile().format() == RS2_FORMAT_BGR8) {
        // ת�� BGR �� RGB
        auto q = QImage((const uchar*)f.get_data(), w, h, QImage::Format_RGB888);
        return q.rgbSwapped();
    }
    else {
        // ��֧�ֵĸ�ʽ
        std::cerr << "Unsupported frame format: " << f.get_profile().format() << std::endl;
        return QImage();
    }
}


std::vector<QVector3D> convertRsPointsToQVector3D(const rs2::points& points) {
    std::vector<QVector3D> qPoints;

    // ��ȡ���ƵĶ���
    const rs2::vertex* vertices = points.get_vertices();

    // ���������е����е�
    for (int i = 0; i < points.size(); ++i) {
        // ֻ�����Ч�ĵ㣨z ���겻Ϊ 0��
        if (vertices[i].z) {
            QVector3D qPoint(vertices[i].x, vertices[i].y, vertices[i].z);
            qPoints.push_back(qPoint);
        }
    }

    return qPoints;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsByROI(const rs2::points& points,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ��ȡԭʼ���ƵĶ���
    const rs2::vertex* vertices = points.get_vertices();

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& v = vertices[i];

        // �����Ƿ��ڸ��������귶Χ��
        if (v.x >= minX && v.x <= maxX &&
            v.y >= minY && v.y <= maxY &&
            v.z >= minZ && v.z <= maxZ) {
            // ������ӵ��µĵ�����
            filteredCloud->push_back(pcl::PointXYZ(static_cast<double>(v.x),
                static_cast<double>(v.y),
                static_cast<double>(v.z)));
        }
    }

    return filteredCloud;
}
