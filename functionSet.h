#pragma once
#include<qlabel.h>
#include<qdebug.h>
#include<opencv.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include<ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include "opencv2/imgproc/types_c.h"
#include<qmath.h>
#include<list>
#include<fstream>

/*
    ע�����
    1.getActualTCPForce �ǻ�ȡ������������,��Ӧ������ϵ�������Ϊ�ڴ���������һ����������ϵ����һ��������ϵ
    2.TCPOffset��Ҫ�����ӿ�ʼʱ�ͳ�ʼ�����ж�Ӧ��init��������Ȼ����Ļ�ȡ��Ϣû����
    3.

*/

#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))
#define DEG_TO_RAD(degrees) ((degrees) * (M_PI / 180.0))


//ת��ö������
enum class RotationAxis {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
};


// getSensorForce��������   ԭ�����https://www.yuque.com/lindong-9iuax/cs7vo1/hh2nkivsldpgwcxd
// ����д�Ĵ�������ƪ  ��Ӧ��ǰ�Ĳ���
struct Compensation_torque {
    double M_e_x5;
    double M_e_y5;
    double M_e_z5;
    double M_x0;
    double M_y0;
    double M_z0;
};

// getSensorForce��������   ԭ�����https://www.yuque.com/lindong-9iuax/cs7vo1/hh2nkivsldpgwcxd
// ����д�Ĵ�������ƪ  ��Ӧ����Ҷ�任
struct Compensation {
    std::vector<double> frequencies;
    std::vector<double> a_k;
    std::vector<double> b_k;
};

//�������ܺ���

using cv::Vec3d;

QImage MatToQImage(const cv::Mat& mat);

cv::Mat QImage2cvMat(QImage image);

/**
 * ��ʼ��TCPOffSet,��Ҫ�ڿ���ǰ����
 *
 * @param ur_rtde::RTDEControlInterface* rtde_c 
 */
void initTCPOffSet(ur_rtde::RTDEControlInterface* rtde_c);

/**
 * ��ȡ�ƶ�����ϵ������ĵ�λ�������ڻ�����ϵ�µı�ʾ��(�ƶ�����ϵ�ڴ�������)
 *
 * @param sixth_axis ����������,��������
 * @param end_effector_direction ĩ���ƶ�����(��������ϵ��xoyƽ��)
 * @param base_frame_z_axis ������z������
 * @param x_axis �ƶ�����ϵx������
 * @param y_axis �ƶ�����ϵy������
 * @param z_axis �ƶ�����ϵz������
 * @param rtde_r RTDEReceiveInterface���ָ��
 */
void computeMotionCoordinateSystem(const Vec3d& sixth_axis,
    const Vec3d& end_effector_direction,
    const Vec3d& base_frame_z_axis,
    Vec3d& x_axis,
    Vec3d& y_axis,
    Vec3d& z_axis,
    ur_rtde::RTDEReceiveInterface* rtde_r);


/**
 * ��������ƽ��Ľ��ߵĵ�λ������
 * ƽ�������ǵķ��������塣
 *
 * @param normal1 ��һ��ƽ��ķ�������
 * @param normal2 �ڶ���ƽ��ķ�������
 * @return ���ߵĵ�λ������ 
 */
cv::Vec3d computeIntersectionLine(const cv::Vec3d& normal1, const cv::Vec3d& normal2);


/**
 * �����һ������ϵ����һ������ϵ����ת����
 *
 * @param axis1_1 ��һ������ϵ�ڻ�����ϵ�µĵ�һ����ĵ�λ������
 * @param axis1_2 ��һ������ϵ�ڻ�����ϵ�µĵڶ�����ĵ�λ������
 * @param axis1_3 ��һ������ϵ�ڻ�����ϵ�µĵ�������ĵ�λ������
 * @param axis2_1 �ڶ�������ϵ�ڻ�����ϵ�µĵ�һ����ĵ�λ������
 * @param axis2_2 �ڶ�������ϵ�ڻ�����ϵ�µĵڶ�����ĵ�λ������
 * @param axis2_3 �ڶ�������ϵ�ڻ�����ϵ�µĵ�������ĵ�λ������
 * @return cv::Mat ���ص�һ������ϵ���ڶ�������ϵ����ת����
 */
cv::Mat calculateRotationMatrix(const cv::Vec3d& axis1_1, const cv::Vec3d& axis1_2, const cv::Vec3d& axis1_3,
    const cv::Vec3d& axis2_1, const cv::Vec3d& axis2_2, const cv::Vec3d& axis2_3);


/**
 * ��ȡ�ӻ�����ϵ��Ŀ������ϵ����ת���󣬲���ΪĿ������ϵ�������ᵥλ�����ڻ�����ϵ�µı�ʾ
 *
 * @param x_axis Ŀ������ϵx������
 * @param y_axis Ŀ������ϵy������
 * @param z_axis Ŀ������ϵz������
 */
cv::Mat getRotationMatrixFromBase(const Vec3d& x_axis,
    const Vec3d& y_axis,
    const Vec3d& z_axis);


/**
 * ����ת���󣬻�ȡ�����ᵥλ�����ڵ�ǰ����ϵ�±�ʾ
 *
 * @param x_axis Ŀ������ϵx������
 * @param y_axis Ŀ������ϵy������
 * @param z_axis Ŀ������ϵz������
 * @param rotation_matrix ��ת����
 */
void getAxisVecFromRotMat(Vec3d& x_axis,
    Vec3d& y_axis,
    Vec3d& z_axis,const cv::Mat& rotation_matrix);


/**
 * ת����ά���������µ�����ϵ��
 *
 * @param force_in_base ��ά�������ڻ�����ϵ�µı�ʾ��ǰ����Ϊ����������Ϊ���أ���
 * @param rotation_matrix �ӻ�����ϵ��������ϵ����ת����
 * @param origin_in_base ������ϵԭ���ڻ�����ϵ�µ����ꡣ
 * @return ��ά��������������ϵ�µı�ʾ��
 */
std::vector<double> transformForceToNewCoordinateSystem(const std::vector<double>& force_in_base,
    const cv::Mat& rotation_matrix,
    const Vec3d& origin_in_base);


/**
* ��ȡ������ķ�����������������
* 
* @param pose ��getActualTCPPose��õ�����
* @return �����᷽������
*/
Vec3d getSixthAxisFromBasePose(std::vector<double> pose);


/**
* ���㹤�ߵ��ٶ� ��δ��֤��
*
* @param tcp_speed ��getActualTCPSpeed��õ�����
* @param tool_offset ���������TCP����ϵ��ƫ�ƣ����ڻ�����ϵ��
* @return ��������ϵ�µ���ά�ٶ�
*/
std::vector<double> calculateToolPointSpeed(const std::vector<double>& tcp_speed,
    const Vec3d& tool_offset);


/**
* ��ȡ�ƶ�����ϵ�µ�����Ϣ  (�ƶ�����ϵ��ԭ��ʹ���������ϵ��ԭ��һ��)
* 
* @param sixth_axis ����������,��������
* @param end_effector_direction ĩ���ƶ�����(��������ϵ��xoyƽ��)
* @param base_frame_z_axis ������z������
* @param rtde_r RTDEReceiveInterface���ָ��
* @return �ƶ�����ϵ�µ�����Ϣ
*/
std::vector<double> getMotionCoordinateSystemForce(const Vec3d& sixth_axis,
    const Vec3d& end_effector_direction,
    const Vec3d& base_frame_z_axis,
    ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* ���ݶ�Ӧ��ǰ����ϵZ/Y/X��תĳ���ǶȻ�ȡ��Ӧ��ת����
*
* @param radians ����
* @param RotationAxis ��ת��
* @return ��ת����
*/
cv::Mat getRotationMatrix(RotationAxis axis, double radians);


/**
 * ��������ת������ϳ�һ����ת����
 *
 * @param rotVec1 ��һ����ת����������Ϊcv::Vec3d
 * @param rotVec2 �ڶ�����ת����������Ϊcv::Vec3d
 * @return ���Ϻ����ת����������Ϊcv::Vec3d
 */
cv::Vec3d combineRotationVectors(const cv::Vec3d& rotVec1, const cv::Vec3d& rotVec2);


/**
* �� ��ά���� ��ת������ Ŀ������ϵ ���������Ŀ������ϵ�µ���ʾ��
*
* @param force ��ά����
* @param RotationMatrix ��ǰ����ϵ��Ŀ������ϵ����ת����
*
* @return Ŀ������ϵ�µ���ά����
*/
std::vector<double> rotateForceToTarget(const std::vector<double>& force, const cv::Mat& rotation_matrix);

/**
* �� ������ת��Ŀ������ϵ �����������Ŀ������ϵ�µ���ʾ��
*
* @param Vec ����
* @param RotationMatrix ��ǰ����ϵ��Ŀ������ϵ����ת����
*
* @return Ŀ������ϵ�µ�������ʾ
*/
cv::Vec3d rotateVecToTarget(const cv::Vec3d& Vec1, const cv::Mat& rotation_matrix);

/**
 * ������һ������ϵת������һ������ϵ (ת������һ������ϵ�±�ʾ)
 *
 * @param force ��ά�����������������������������ط���
 * @param distanceVector �Ӿ�����ϵ��������ϵ�ľ�������
 * @return ������ϵ�µ���ά��������
 */
std::vector<double> TransformForceToNewFrame(const std::vector<double>& force, const cv::Vec3d& distanceVector);

/**
* ����B��C����ת����
*
* @param R_AB A��B����ת����
* @param R_AC A��C����ת����
*
* @return B��C����ת����
*/
cv::Mat calculateRotationMatrix_BC(const cv::Mat& R_AB, const cv::Mat& R_AC);


/**
* ��getActualTCPPose���������ϵ��TCP����ϵ����ת����
*
* @param pose getActualTCPPose��ȡ����λ������
* @return ������ϵ��TCP����ϵ����ת����
*/
cv::Mat calculateRotationMatrix_Base2TCP(const std::vector<double>& pose);



/**
* ��getActualTCPForce�õ�������Ϣת����tcp����ϵ��,���ú�����֤��ֻ�ǵ����������꣨��tcp������ƽ�й�ϵ����
*
* @param world_force getActualTCPForce�õ�������Ϣ
* @param in_pose getActualTCPPose��ȡ����λ������
* @return TCP����ϵ�µ�����Ϣ
*/
std::vector<double> world_2tcp_force(const std::vector<double>& world_force, const std::vector<double> &in_pose);



/**
* �����ƶ�����ϵ��ת�ĽǶȻ�ȡkxita
*
* @param pose getActualTCPPose�õ���λ����Ϣ
* @param speed getActualTCPSpeed�õ����ٶ���Ϣ
* @param theta_x �ƶ�����ϵ����x���ת��
* @param theta_y �ƶ�����ϵ����y���ת��
* @return kxita ����ת������
* @param rtde_r RTDEReceiveInterface���ָ��
*/
Vec3d get_Kxita_From_theta_x_and_theta_y_baseMotion(std::vector<double> pose, std::vector<double> speed
                                                    ,double theta_x, double theta_y, ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* �������������������������������ת��������ʵ�ֵ�һ��������ڶ��������Ķ���
*
* @param Vec1 ��ʼ����
* @param Vec2 Ŀ������
* @return ��ת����
*/
cv::Vec3d CalculateRotationVector(const cv::Vec3d& Vec1, const cv::Vec3d& Vec2);



/**
* ���ṩ����ת��������ȡŷ���ǣ�������ת�����ǰ��� XYZ ˳����ת��
*
* @param rotationMatrix ��ת����
* @return �����Ƕ� �ֱ������ X �ᡢY ��� Z �����ת
*/
cv::Vec3d convertRotaionMatrixToEulerAngles(const cv::Mat& rotationMatrix);


/**
* resize������С
*
* @param vector Ҫ�޸ĵ�����
* @return newLength �޸ĺ������ĳ���
*/
void ResizeVector(cv::Vec3d& vector, double newLength);


/**
* ��ȡTCP����ϵ�µ���
*
* @param rtde_r RTDEReceiveInterface���ָ��
* @return TCP����ϵ�µ���ά��
*/
std::vector<double> getTCPForce(ur_rtde::RTDEReceiveInterface* rtde_r);


/** 
* ��ȡ����������ϵ�µ��� (�����������ϵ�ڷ�������)
*
* @param rtde_r RTDEReceiveInterface���ָ��
* @param isCompensate �Ƿ�������
* @param enableFilter �Ƿ����˲�
* @return ����������ϵ�µ���ά����
*/
std::vector<double> getSensorForce(ur_rtde::RTDEReceiveInterface* rtde_r,bool isCompensate = false,bool enableFilter = false);


/**
* ��ȡ����������ϵ�µ���̬ (�����������ϵ�ڷ�������)
*
* @param rtde_r RTDEReceiveInterface���ָ��
* @return ����������ϵ�µ���̬
*/
std::vector<double> getSensorPose(ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* ��ȡ����������ϵ�µ��ٶ� (�����������ϵ�ڷ�������)  (δ��֤)
*
* @param rtde_r RTDEReceiveInterface���ָ��
* @return ����������ϵ�µ���̬
*/
std::vector<double> getSensorSpeed(ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* ��ȡ ���˹�ʽģ�͵Ĵ���������ϵ��Force�Լ������굽������ϵ����ת���� ,CDS ��coordinate system
*
* @param rtde_r RTDEReceiveInterface���ָ��
* @return ���˹�ʽģ�͵Ĵ���������ϵ��Force�Լ������굽������ϵ����ת����
*/
std::pair<std::vector<double>,cv::Mat> getMyFormulaCDSForceAndBase2Formula(ur_rtde::RTDEReceiveInterface* rtde_r);

/**
* ���ݵ�ǰ��̬����תʸ��������תʸ������������ϵ��������ȡ����̬
*
* @param pose ��ǰ��̬
* @param RotateVec ��תʸ��������תʸ������������ϵ������
* @return ����̬
*/
std::vector<double> getNewPoseFromCurPoseAndRotateVec(const std::vector<double>& pose, const cv::Vec3d& RotateVec);


/**
 * ����һ����ת������ƽ����ת������
 *
 * ����������һϵ�е���ת���������Ƚ�ÿ����ת����ת��Ϊ��Ӧ����ת����
 * Ȼ�������Щ��ת���������ƽ��ֵ����ͨ������ֵ�ֽ��ҵ���ӽ�����������
 * ���ս�����������ת������ת������Ϊƽ����ת�ı�ʾ��
 *
 * @param rotation_vectors һ���������cv::Vec3d��ת������������ÿ����������һ����ά�ռ����ת��
 * @return cv::Vec3d ����ƽ����ת�������䷽���ʾ��ת�ᣬ���ȱ�ʾ��ת�Ƕȣ���λ�����ȣ���
 */
cv::Vec3d averageRotationVector(const std::vector<cv::Vec3d>& rotation_vectors);


/**
* ��ȡ������ϵ����k��� ��תʸ����λ��
*
* @param rtde_r RTDEReceiveInterface���ָ��
* @param k ��k��
* @return vector<double>
*/
std::vector<double> getBase2kAxisPose(ur_rtde::RTDEReceiveInterface* rtde_r, const int num);


/**
* �޸�getSensorForce�������� Compensation_torque  ---��Ӧ��ȸ ��ǰ�Ĳ���
*
* @param M_e_x5 
* @param M_e_y5 
* @param M_e_z5 
* @param M_x0 
* @param M_y0 
* @param M_z0
*/
void change_Compensation_torque(const double& M_e_x5, const double& M_e_y5, const double& M_e_z5,
                                const double& M_x0, const double& M_y0, const double& M_z0, bool isCompensation);


/**
* �޸�getSensorForce�������� Compensation    ---��Ӧ��ȸ ���������� ����Ҷ�任����
*
* @param P_Fx_ak_bk Fx������Ӧ��������ϵ��
* @param P_Fy_ak_bk Fy������Ӧ��������ϵ��
* @param P_Fz_ak_bk Fz������Ӧ��������ϵ��
* @param P_Mx_ak_bk Mx������Ӧ��������ϵ��
* @param P_My_ak_bk My������Ӧ��������ϵ��
* @param Fx_frequencies My������Ӧ��������ϵ��
* @param Fy_frequencies My������Ӧ��������ϵ��
* @param Fz_frequencies My������Ӧ��������ϵ��
* @param Mx_frequencies My������Ӧ��������ϵ��
* @param My_frequencies My������Ӧ��������ϵ��
* @param isCompensation  �Ƿ�������
*/
void change_Compensation(const std::pair<std::vector<double>, std::vector<double>> & P_Fx_ak_bk,
    const std::pair<std::vector<double>, std::vector<double>>& P_Fy_ak_bk,
    const std::pair<std::vector<double>, std::vector<double>>& P_Fz_ak_bk,
    const std::pair<std::vector<double>, std::vector<double>>& P_Mx_ak_bk,
    const std::pair<std::vector<double>, std::vector<double>>& P_My_ak_bk,
    const std::vector<double>& Fx_frequencies,
    const std::vector<double>& Fy_frequencies,
    const std::vector<double>& Fz_frequencies,
    const std::vector<double>& Mx_frequencies,
    const std::vector<double>& My_frequencies,
    bool isCompensation);


/**
 * ʹ�����Һ����һ�����������ں���
 * @param frequencies Ƶ����������λӦ�����ʺ�����ĵ�λ����Hz
 * @param anglesInRadians �Ƕ���������λ�ǻ���
 * @param observedValues ��Ӧ�ǶȵĹ۲�ֵ����
 * @return һ����������������pair���ֱ������Һ�����ϵ������ ak��bk
 */
std::pair<std::vector<double>, std::vector<double>> fitCosineSineComponents(
    const std::vector<double>& frequencies,
    const std::vector<double>& anglesInRadians,
    const std::vector<double>& observedValues);



/**  ----������
* 
* ���Ա궨��������ԭ����봫����ƽ��ľ��룬ʩ�ӵ�������������һ��
* (����ԭ������ά������ƽ�湹��һ���̶�����ϵ���̶�����ϵ�ķ�����TCP����ϵһ�£�
*  ����Ҫ�ҵ�getactualtcpforce()��������������ϵ��ԭ�����̶�����ϵZ���Զ����Ϊ����������ϵֻ��ƽ�ƹ�ϵ��
*  ��������ͨ���ڴ��(0,0,x)��ʩ��һ��������(0,0,x + k)��ʩ��һ������ͨ��������ά�������궨�����������ϵ����)
*
* @param forceA ��һ����A
* @param forceB �ڶ�����B
* @param x ʩ�ӵĵ�һ������������� (0,0,x)
* @param k ���һ����ľ��룬���ڶ����������Ϊ(0,0,x + k)
* @return d getactualtcpforce()��������������ϵ��ԭ�����̶�����ϵZ��ľ���
*/
double calculateD(const std::vector<double>& forceA, const std::vector<double>& forceB, double x, double k);


/**
* ����һ��������̬���棨ģ��URScript�� pose_inv��
*
* @param p_from ���ߵ�ԭʼ��̬��һ������λ�úͷ���Ŀռ�����
* @return ����̬�任��Ҳ��һ���ռ�����
*/
std::vector<double> pose_inv(const std::vector<double>& pose);


/**
 * @brief ��̬�任����������ڹ��߻��Զ�������/��ܽ����ƶ�
 * ��һ������ p_from ���ڱ任�ڶ������� p_from_to��Ȼ�󷵻ؽ��������ζ�Ž���Ǵ� p_from ������ϵ��ʼ��
 * �ڸ�����ϵ���ƶ� p_from_to ��õ�����̬��
 * ����������Դ�������ͬ�ĽǶȿ�����Ҫô��ͨ�� p_from �Ĳ������任����ƽ�ƺ���ת��p_from_to��
 * Ҫô�����ڻ�ȡ���Ƚ��� p_from �ƶ���Ȼ�����Ǹ������Ͻ��� p_from_to �ƶ��Ľ����̬��
 * �������̬��Ϊ�任�������ʾΪ��
 * @verbatim
 * T_world->to = T_world->from * T_from->to
 * T_x->to = T_x->from * T_from->to
 * @endverbatim
 * @param p_from ��ʼ��̬���ռ�������
 * @param p_from_to �������ʼ��̬����̬�仯���ռ�������
 * @return �����̬���ռ�������
 */
std::vector<double> poseTrans(const std::vector<double>& p_from, const std::vector<double>& p_from_to);


/**
* ����һ��ָ���ı任������(wrench)���б任������������ڸı����صĲο��㡣
* ע�⣺�任���ز���任��̬�����򵥣���Ϊ���ص�ת�����ֻ�����ƽ�ƾ���ĳ��ȶ��仯��
*
* ���ر任�Ĺ�ʽΪ w_to = T_from_to * w_from�����У�
* - w_from ��ԭʼ�����أ���ʾΪһ����ά���� [F_x, F_y, F_z, M_x, M_y, M_z]��
*   ���� F_x, F_y, F_z ����������������M_x, M_y, M_z �����ص�����������
* - T_from_to ��һ����ά��������ʾ��ԭʼ�㵽�µ����ת��ƽ�Ʊ任��
*   ǰ������������ת������ͨ������-�Ǳ�ʾ����������������ƽ��������
* - w_to �Ǳ任������أ�Ҳ��һ����ά������
*
* @param T_from_to ��ԭʼ�㵽�µ�ı任����̬��
* @param w_from Ҫ�任�����أ���ʽΪ [F_x, F_y, F_z, M_x, M_y, M_z]
* @return �任������� w_to����ʽΪ [F_x, F_y, F_z, M_x, M_y, M_z]
*/
std::vector<double> wrench_trans(const std::vector<double>& T_from_to, const std::vector<double>& w_from);



/**
 * ʹ����С���˷��������ϵͳ������ʹ�����淽�̷��� ,��x = (ATA)-1 ATb
 *
 * @param A ϵ�����󣬴�СΪ m x n������ m �Ƿ��̵�������n ��δ֪��������
 * @param b �����������СΪ m x 1
 * @return ����������СΪ n x 1�����ϵͳ�޽��򷵻ؿ�����
 */
cv::Mat solveLeastSquaresNormalEquation(const cv::Mat& A, const cv::Mat& b);


/**
 * �������˲����࣬���ڴ���������ƽ�����������١�
 */
class KalmanFilter {
public:
    KalmanFilter(double process_noise, double measurement_noise);

    /**
     * ʹ���µĲ������ݸ��¿������˲�����
     *
     * @param measurement Ҫ����������ݵ㡣
     * @returns ���������Ĺ���״̬��
     */
    double update(double measurement);

private:
    double estimate;           ///< ��ǰ״̬���ơ�
    double error_covariance;   ///< ��ǰ�������Э���
    double process_noise;      ///< ��������Э���
    double measurement_noise;  ///< ��������Э���
    double kalman_gain;        ///< ���������档
};


/**
 * �Դ���������Ӧ�õ�ͨ�˲����Լ��ٸ�Ƶ������         ------��������Ҷ�任����������������ά���������󣬷�������alpha��ֵΪ2 * pi * 1e-3
 *
 * @param current ��ǰ�Ĵ�����������
 * @param previous ��һ�εĹ��˴�����������
 * @param alpha �˲���ƽ�����ӡ�
 * @returns ƽ����Ĵ�����������
 */
std::vector<double> lowPassFilter(const std::vector<double>& current, double alpha);


/**
 * K�˲�����Euler���ٶȺͽǼ��ٶ�
 *
 * @param MeasureData ��������
 * @param MeasureDatas �������ݼ���
 * @param idv_FillterDelay �˲��ӳ�
 * @param idv_MNoiseCov ��������Э����
 * @param idv_PNoiseCov ��������Э����
 * @returns �˲����
 */
double Cal_EulerKFilter(const double& MeasureData, std::list<double>& MeasureDatas, const int& idv_FillterDelay,
    const double& idv_MNoiseCov, const double& idv_PNoiseCov);


/**
 * Kalman�˲����㷨
 *
 * @param ilv_MeasureDatas ��������
 * @param idv_MNoiseCov ��������Э����
 * @param idv_PNoiseCov ��������Э����
 * @return �˲����
 */
double Cal_KalmanFilter(std::vector<double> ilv_MeasureDatas, double idv_MNoiseCov, double idv_PNoiseCov);


/**
 * ��vector<vector<double>>���ݴ洢��CSV�ļ���
 *
 * @param data Ҫ�洢�����ݣ�����Ϊvector<vector<double>>
 * @param path �洢CSV�ļ���·��
 * @return bool �ɹ�����true��ʧ�ܷ���false
 */
bool saveToCsv(const std::vector<std::vector<double>>& data, const std::string& path);


/**
 * ��CSV�ļ���ȡ���ݵ�vector<vector<double>>
 *
 * @param path CSV�ļ���·��
 * @return vector<vector<double>> ��ȡ������
 */
std::vector<std::vector<double>> readFromCsv(const std::string& path);