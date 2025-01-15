#pragma once

#include <QImage>
#include<opencv2/opencv.hpp>
#include "functionSet.h"
#include <k4a/k4a.hpp>


/** 
* 将 k4a::capture 中的彩色图像转换为 QImage。
*	
* @param capture Kinect 捕获的数据。
* @return 转换后的 QImage 对象，如果没有有效的彩色图像则为空。
*/
QImage capture_to_qimage_color(const k4a::capture& capture);


/** 
* 将 k4a::capture 中的深度图像转换为 QImage。
* 
* @param capture Kinect 捕获的数据。
* @return 转换后的 QImage 对象，如果没有有效的深度图像则为空。
*/
QImage capture_to_qimage_depth(const k4a::capture& capture);


/**
 * @brief 将 k4a::capture 中的彩色图像保存为 BMP 文件。
 *
 * @param photos 包含所有捕获的帧数据的向量。
 * @param savePath 图像保存的路径，所有图像将保存到此目录。
 *
 * @note 图像将按数字顺序命名为 1.bmp, 2.bmp 等。
 * @note 函数会自动处理路径分隔符问题，无需担心末尾是否有 '/' 或 '\\'。
 */
void saveColorImages(const std::vector<k4a::capture>& photos, const std::string& savePath);


/**
 * @brief 获取 base2cam 的六维表示（x, y, z, rx, ry, rz）。
 *
 * @return 返回一个 std::vector<double>，包含六维表示的值。
 *         前三维是平移 (x, y, z)，后三维是旋转向量 (rx, ry, rz)。
 */
std::vector<double> getBase2Cam();


/**
 * @brief 使用输入的 4x4 变换矩阵更新 base2cam 的六维表示。
 *
 * @param transformMatrix 输入的 4x4 变换矩阵（cv::Mat 类型，大小为 4x4）。
 *                        前三行前三列是旋转矩阵，前三行第四列是平移向量。
 * @throws std::invalid_argument 如果输入矩阵不是 4x4 大小，会抛出异常。
 */
void updateBase2Cam(const cv::Mat& transformMatrix);


