#pragma once

#include <QImage>
#include<opencv2/opencv.hpp>
#include "functionSet.h"
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


/**
 * @brief �� k4a::capture �еĲ�ɫͼ�񱣴�Ϊ BMP �ļ���
 *
 * @param photos �������в����֡���ݵ�������
 * @param savePath ͼ�񱣴��·��������ͼ�񽫱��浽��Ŀ¼��
 *
 * @note ͼ�񽫰�����˳������Ϊ 1.bmp, 2.bmp �ȡ�
 * @note �������Զ�����·���ָ������⣬���赣��ĩβ�Ƿ��� '/' �� '\\'��
 */
void saveColorImages(const std::vector<k4a::capture>& photos, const std::string& savePath);

