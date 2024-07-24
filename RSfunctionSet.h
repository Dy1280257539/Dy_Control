#pragma once
#include "RealSenseCameraManager.h"
#include <librealsense2/rs.hpp>
#include <QImage>
#include <QVector3D>

#include <iostream>

//PCL��
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


/**
 * �� RealSense ��֡ת��Ϊ QImage�� //gptֱ�Ӹ��� δ���
 *
 * @param f rs2::frame��RealSense ֡��
 * @return ����ת����� QImage �������֡��Ч���ʽ��֧�֣��򷵻ؿյ� QImage��
 */
QImage rsFrameToQImage(const rs2::frame& f);


/**
 * �� RealSense �ĵ��ƣ�rs2::points��ת��Ϊ QVector3D ��������  //gpt����
 *
 * @param points rs2::points��RealSense ���ơ�
 * @return ���ذ��� QVector3D ��� std::vector��
 */
std::vector<QVector3D> convertRsPointsToQVector3D(const rs2::points& points);

/**
 * �� RealSense ������ɸѡ������ָ�����귶Χ�ڵĵ㣬��ת��Ϊ PCL ���Ƹ�ʽ��
 *
 * @param points RealSense �������ݡ�
 * @param minX ��С X �����ꡣ
 * @param maxX ��� X �����ꡣ
 * @param minY ��С Y �����ꡣ
 * @param maxY ��� Y �����ꡣ
 * @param minZ ��С Z �����ꡣ
 * @param maxZ ��� Z �����ꡣ
 * @return ����һ��ָ��ɸѡ��� PCL ���Ƶ�����ָ�롣
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsByROI(const rs2::points& points,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ);

