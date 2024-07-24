#pragma once

#include <QImage>
#include <k4a/k4a.hpp>


/** 
* �� k4a::capture �еĲ�ɫͼ��ת��Ϊ QImage��
*	
* @param capture Kinect ��������ݡ�
* @return ת����� QImage �������û����Ч�Ĳ�ɫͼ����Ϊ�ա�
*/
QImage capture_to_qimage_color(const k4a::capture& capture);


/** 
* �� k4a::capture �е����ͼ��ת��Ϊ QImage��
* 
* @param capture Kinect ��������ݡ�
* @return ת����� QImage �������û����Ч�����ͼ����Ϊ�ա�
*/
QImage capture_to_qimage_depth(const k4a::capture& capture);
