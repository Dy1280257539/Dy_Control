#pragma once

#include <QImage>
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
