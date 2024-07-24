#pragma once
#include "RealSenseCameraManager.h"
#include <librealsense2/rs.hpp>
#include <QImage>
#include <QVector3D>

#include <iostream>

//PCL库
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


/**
 * 将 RealSense 的帧转换为 QImage。 //gpt直接给的 未检查
 *
 * @param f rs2::frame，RealSense 帧。
 * @return 返回转换后的 QImage 对象。如果帧无效或格式不支持，则返回空的 QImage。
 */
QImage rsFrameToQImage(const rs2::frame& f);


/**
 * 将 RealSense 的点云（rs2::points）转换为 QVector3D 的向量。  //gpt给的
 *
 * @param points rs2::points，RealSense 点云。
 * @return 返回包含 QVector3D 点的 std::vector。
 */
std::vector<QVector3D> convertRsPointsToQVector3D(const rs2::points& points);

/**
 * 从 RealSense 点云中筛选出落在指定坐标范围内的点，并转换为 PCL 点云格式。
 *
 * @param points RealSense 点云数据。
 * @param minX 最小 X 轴坐标。
 * @param maxX 最大 X 轴坐标。
 * @param minY 最小 Y 轴坐标。
 * @param maxY 最大 Y 轴坐标。
 * @param minZ 最小 Z 轴坐标。
 * @param maxZ 最大 Z 轴坐标。
 * @return 返回一个指向筛选后的 PCL 点云的智能指针。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsByROI(const rs2::points& points,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ);

