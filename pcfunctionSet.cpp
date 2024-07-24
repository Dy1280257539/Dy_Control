#include"pcfunctionSet.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr extract_region(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // ����ü���
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
    // �����˲�������
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(plane_x - threshold, plane_x + threshold);

    // �����˲�
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

    // ��ȡPCD�ļ�
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        PCL_ERROR("�޷���ȡ�ļ�\n");
        return nullptr; // ���ؿ�ָ���ʾ��ȡʧ��
    }

    return cloud;
}

bool savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& file_path) {
    if (pcl::io::savePCDFile(file_path, *cloud) == -1) {
        PCL_ERROR("�������ʧ��\n");
        return false;
    }
    return true;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr smoothPointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    // ����һ��������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // �������
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);

    // ��ʼ��MLS����
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals(false); // �������Ҫ������Ϣ��������Ϊfalse

    // �����������
    mls.setInputCloud(input_cloud);

    // ���������뾶
    mls.setSearchRadius(0.03); // ���ֵ�����������ݼ����е���

    // ����������
    mls.setSearchMethod(tree);

    // ����MLSƽ��
    mls.process(*mls_points);

    return mls_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr smoothAndInterpolatePointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // �������
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);

    // ��ʼ��MLS����
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals(false); // �����Ƿ���㷨��

    // �����������
    mls.setInputCloud(input_cloud);

    // ���������뾶
    mls.setSearchRadius(0.03); // �����������ݼ�����

    // ���ò岹����
    mls.setPolynomialOrder(2); // ���ö���ʽ����
    mls.setPolynomialFit(true); // ��������ʽ���
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.005); // ���ò岹�뾶
    mls.setUpsamplingStepSize(0.003); // ���ò���

    // ����������
    mls.setSearchMethod(tree);

    // ����MLSƽ���Ͳ岹
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




