#include"RSfunctionSet.h"


QImage rsFrameToQImage(const rs2::frame& f) {
    // 检查帧是否有效
    if (!f) {
        return QImage();
    }

    // 获取帧的宽度和高度
    const int w = f.as<rs2::video_frame>().get_width();
    const int h = f.as<rs2::video_frame>().get_height();

    // 检查帧的格式
    if (f.get_profile().format() == RS2_FORMAT_RGB8) {
        // 复制数据到 QImage
        return QImage((const uchar*)f.get_data(), w, h, QImage::Format_RGB888).copy();
    }
    else if (f.get_profile().format() == RS2_FORMAT_BGR8) {
        // 转换 BGR 到 RGB
        auto q = QImage((const uchar*)f.get_data(), w, h, QImage::Format_RGB888);
        return q.rgbSwapped();
    }
    else {
        // 不支持的格式
        std::cerr << "Unsupported frame format: " << f.get_profile().format() << std::endl;
        return QImage();
    }
}


std::vector<QVector3D> convertRsPointsToQVector3D(const rs2::points& points) {
    std::vector<QVector3D> qPoints;

    // 获取点云的顶点
    const rs2::vertex* vertices = points.get_vertices();

    // 遍历点云中的所有点
    for (int i = 0; i < points.size(); ++i) {
        // 只添加有效的点（z 坐标不为 0）
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

    // 获取原始点云的顶点
    const rs2::vertex* vertices = points.get_vertices();

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& v = vertices[i];

        // 检查点是否在给定的坐标范围内
        if (v.x >= minX && v.x <= maxX &&
            v.y >= minY && v.y <= maxY &&
            v.z >= minZ && v.z <= maxZ) {
            // 将点添加到新的点云中
            filteredCloud->push_back(pcl::PointXYZ(static_cast<double>(v.x),
                static_cast<double>(v.y),
                static_cast<double>(v.z)));
        }
    }

    return filteredCloud;
}
