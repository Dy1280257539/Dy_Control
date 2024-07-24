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
 * �� pcl::PointCloud<pcl::PointXYZ>::Ptr ����ȡ�ض�����ĵ��ơ�
 *
 * @param cloud ����� pcl::PointCloud<pcl::PointXYZ>::Ptr ���ơ�
 * @param minX, maxX, minY, maxY, minZ, maxZ �������Ȥ��������귶Χ��
 * @return ��ȡ��ĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_region(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ);

/**
 * �� pcl::PointCloud<pcl::PointXYZ>::Ptr ��ɸѡ���ӽ��ض���ֱ��X���ƽ��ĵ��ơ�
 *
 * @param cloud ����� pcl::PointCloud<pcl::PointXYZ>::Ptr ���ơ�
 * @param plane_x ƽ����X���ϵ�λ�á�
 * @param threshold �㵽ƽ�����������ֵ��
 */
void filterPointsNearVerticalPlaneX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double plane_x,
    double threshold);

/**
 * �� pcl::PointCloud<pcl::PointXYZ>::Ptr ����ȡ�ӽ��ض�ƽ��ĵ��ơ�
 *
 * @param cloud ����� pcl::PointCloud<pcl::PointXYZ>::Ptr ���ơ�
 * @param a, b, c, d ƽ�淽�� ax + by + cz + d = 0 ��ϵ����
 * @param threshold �㵽ƽ�����������ֵ��
 * @return ��ȡ��ĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsNearPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double a, double b, double c, double d,
    double threshold);


/**
 * ��ָ��·������PCD�ļ�������һ�����ƶ���
 *
 * @param file_path PCD�ļ���·����
 * @return ���صĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromPCD(const std::string& file_path);

/**
 * ���������ݱ��浽ָ��·����PCD�ļ���
 *
 * @param cloud Ҫ����ĵ������ݣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 * @param file_path ����������ݵ�PCD�ļ�·����
 * @return �ɹ�����true��ʧ�ܷ���false��
 */
bool savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& file_path);


/**
 * ʹ���ƶ���С���˷���MLS���Ե��ƽ���ƽ������
 *
 * @param input_cloud ����ĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 * @return ƽ�������ĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothPointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);



/**
 * ʹ���ƶ���С���˷���MLS���Ե��ƽ���ƽ���������е�岹��
 *
 * @param input_cloud ����ĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 * @return ƽ�����岹��ĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr smoothAndInterpolatePointCloudWithMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);



/**
 * ������ͶӰ��YOZƽ�棬������Y�����������Ϊ���顣
 *
 * @param cloud ����ĵ��ƣ�����Ϊ pcl::PointCloud<pcl::PointXYZ>::Ptr��
 * @return ����vector<pair<double, double>>���ֱ��Ӧy����0��С��0�ĵ����������λ�á�
 */
std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>>
projectPointCloudToYOZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

