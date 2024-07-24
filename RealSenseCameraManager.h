// RealSenseCameraManager.h

#ifndef REALSENSE_CAMERA_MANAGER_H
#define REALSENSE_CAMERA_MANAGER_H

#include <librealsense2/rs.hpp>
#include <map>
#include <unordered_map>
#include <string>
#include <QReadWriteLock>

class RealSenseCameraManager {
public:
    RealSenseCameraManager();
    ~RealSenseCameraManager();

    bool initializeCameras();  // ��ʼ���������ӵ����
    void updateFrames();       // �������������֡����

    rs2::frame getColorFrame(const std::string& serial); // ��ȡָ������Ĳ�ɫ֡
    rs2::frame getDepthFrame(const std::string& serial); // ��ȡָ����������֡
    rs2::points getPointCloud(const std::string& serial); // ��ȡָ������ĵ���

    std::map<std::string, rs2::frame> getAllColorFrames();    // ��ȡ�����豸�Ĳ�ɫ֡
    std::map<std::string, rs2::frame> getAllDepthFrames();    // ��ȡ�����豸�����֡
    std::map<std::string, rs2::points> getAllPointClouds();   // ��ȡ�����豸�ĵ���

    int getNumOfRS();

    std::unordered_map<std::string, int> getSerial2num();     //��ȡserial2num
    std::unordered_map<int, std::string> getNum2serial();     //��ȡserial2num

private:
    rs2::context ctx;   // ���ڹ����豸�� RealSense ������
    std::vector<rs2::pipeline> pipelines; // ÿ���豸�Ĺܵ�
    std::map<std::string, rs2::frameset> framesets; // ÿ���豸������֡����
    QReadWriteLock framesetsLock; // ��д�������ڱ��� framesets
    std::map<std::string, rs2::pointcloud> pointclouds; // ÿ���豸�ĵ��ƴ������
    std::unordered_map<int, std::string> num2serial;           //������뵽���кŵ�ӳ��
    std::unordered_map<std::string, int> serial2num;           //���кŵ���������ӳ��
    int count;                                                 //���������

    void startPipeline(const std::string& serial); // ��ʼ�������кŵĹܵ�
};

#endif // REALSENSE_CAMERA_MANAGER_H
