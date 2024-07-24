#include"pcfunctionSet.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr extract_region(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 定义裁剪盒
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    box_filter.setInputCloud(cloud);
    box_filter.filter(*cloud_filtered);

    return cloud_filtered;
}


void filterPointsNearVerticalPlaneX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double plane_x,
    double threshold) {
    // 设置滤波器参数
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(plane_x - threshold, plane_x + threshold);

    // 进行滤波
    pass.filter(*cloud);
    
    return;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsNearPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double a, double b, double c, double d,
    double threshold){

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& point : cloud->points) {
            double distance = std::abs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(a * a + b * b + c * c);
            if (distance <= threshold) {
                filtered_cloud->points.push_back(point);
            }
        }

        return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromPCD(const std::string& file_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        PCL_ERROR("无法读取文件\n");
        return nullptr; // 返回空指针表示读取失败
    }

    return cloud;
}

bool savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& file_path) {
    if (pcl::io::savePCDFile(file_path, *cloud) == -1) {
        PCL_ERROR("保存点云失败\n");
        return false;
    }
    return true;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr smoothPointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    // 创建一个搜索树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);

    // 初始化MLS对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals(false); // 如果不需要法线信息可以设置为false

    // 设置输入点云
    mls.setInputCloud(input_cloud);

    // 设置搜索半径
    mls.setSearchRadius(0.03); // 这个值根据您的数据集进行调整

    // 设置搜索树
    mls.setSearchMethod(tree);

    // 进行MLS平滑
    mls.process(*mls_points);

    return mls_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr smoothAndInterpolatePointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);

    // 初始化MLS对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals(false); // 设置是否计算法线

    // 设置输入点云
    mls.setInputCloud(input_cloud);

    // 设置搜索半径
    mls.setSearchRadius(0.03); // 根据您的数据集调整

    // 设置插补参数
    mls.setPolynomialOrder(2); // 设置多项式阶数
    mls.setPolynomialFit(true); // 开启多项式拟合
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.005); // 设置插补半径
    mls.setUpsamplingStepSize(0.003); // 设置步长

    // 设置搜索树
    mls.setSearchMethod(tree);

    // 进行MLS平滑和插补
    mls.process(*mls_points);

    return mls_points;
}


std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>>
projectPointCloudToYOZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::vector<std::pair<double, double>> positiveY, negativeY;

    for (int i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        if (point.y > 0) {
            positiveY.emplace_back(point.y, point.z); // Assuming Z as the new position
        }
        else if (point.y < 0) {
            negativeY.emplace_back(point.y, point.z);
        }
    }

    return { positiveY, negativeY };
}




