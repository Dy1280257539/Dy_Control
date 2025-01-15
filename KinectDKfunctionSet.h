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

