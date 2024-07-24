#pragma once
#include<string>
#include <iostream>
#include <vector>
#include <utility>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>


/**
 * 从 pcl::PointCloud<pcl::PointXYZ>::Ptr 中提取特定区域的点云。
 *
 * @param cloud 输入的 pcl::PointCloud<pcl::PointXYZ>::Ptr 点云。
 * @param minX, maxX, minY, maxY, minZ, maxZ 定义感兴趣区域的坐标范围。
 * @return 提取后的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_region(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ);

/**
 * 从 pcl::PointCloud<pcl::PointXYZ>::Ptr 中筛选出接近特定垂直于X轴的平面的点云。
 *
 * @param cloud 输入的 pcl::PointCloud<pcl::PointXYZ>::Ptr 点云。
 * @param plane_x 平面在X轴上的位置。
 * @param threshold 点到平面的最大距离阈值。
 */
void filterPointsNearVerticalPlaneX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double plane_x,
    double threshold);

/**
 * 从 pcl::PointCloud<pcl::PointXYZ>::Ptr 中提取接近特定平面的点云。
 *
 * @param cloud 输入的 pcl::PointCloud<pcl::PointXYZ>::Ptr 点云。
 * @param a, b, c, d 平面方程 ax + by + cz + d = 0 的系数。
 * @param threshold 点到平面的最大距离阈值。
 * @return 提取后的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsNearPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double a, double b, double c, double d,
    double threshold);


/**
 * 从指定路径加载PCD文件并返回一个点云对象。
 *
 * @param file_path PCD文件的路径。
 * @return 加载的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromPCD(const std::string& file_path);

/**
 * 将点云数据保存到指定路径的PCD文件。
 *
 * @param cloud 要保存的点云数据，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 * @param file_path 保存点云数据的PCD文件路径。
 * @return 成功返回true，失败返回false。
 */
bool savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& file_path);


/**
 * 使用移动最小二乘法（MLS）对点云进行平滑处理。
 *
 * @param input_cloud 输入的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 * @return 平滑处理后的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothPointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);



/**
 * 使用移动最小二乘法（MLS）对点云进行平滑处理并进行点插补。
 *
 * @param input_cloud 输入的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 * @return 平滑并插补后的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothAndInterpolatePointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);



/**
 * 将点云投影到YOZ平面，并根据Y坐标的正负分为两组。
 *
 * @param cloud 输入的点云，类型为 pcl::PointCloud<pcl::PointXYZ>::Ptr。
 * @return 两个vector<pair<double, double>>，分别对应y大于0和小于0的点的索引和新位置。
 */
std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>>
projectPointCloudToYOZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

