// RealSenseCamera.h

#ifndef REALSENSE_CAMERA_H
#define REALSENSE_CAMERA_H

#include <librealsense2/rs.hpp>
#include<iostream>

// RealSenseCamera ���ṩ�˶� RealSense ����Ļ���������
// �����Գ�ʼ�����������֡����ȡ���ơ���ɫ֡�����֡���Լ��ͷ������Դ��
class RealSenseCamera {
public:
    RealSenseCamera();             // ���캯��
    ~RealSenseCamera();            // ����������������Դ�ͷ�

    bool initialize();             // ��ʼ�����
    bool captureFrame();           // ����һ֡����
    void release();                // �ͷ������Դ

    rs2::points getPointCloud();   // ��ȡ��������
    rs2::frame getColorFrame();    // ��ȡ��ɫ֡����
    rs2::frame getDepthFrame();    // ��ȡ���֡����

private:
    rs2::pipeline pipe;            // RealSense �ܵ��������������Ĺ���
    rs2::pointcloud pc;            // ���ڼ������
    rs2::frameset frameset;        // ֡�������������֡����
};

#endif // REALSENSE_CAMERA_H
