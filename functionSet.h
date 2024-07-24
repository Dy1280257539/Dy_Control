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
    注意事项：
    1.getActualTCPForce 是获取传感器处的力,对应的坐标系可以理解为在传感器处有一个世界坐标系方向一样的坐标系
    2.TCPOffset需要在连接开始时就初始化，有对应的init函数，不然这里的获取信息没法用
    3.

*/

#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))
#define DEG_TO_RAD(degrees) ((degrees) * (M_PI / 180.0))


//转轴枚举类型
enum class RotationAxis {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
};


// getSensorForce补偿数据   原理见：https://www.yuque.com/lindong-9iuax/cs7vo1/hh2nkivsldpgwcxd
// 个人写的传感器那篇  对应以前的猜想
struct Compensation_torque {
    double M_e_x5;
    double M_e_y5;
    double M_e_z5;
    double M_x0;
    double M_y0;
    double M_z0;
};

// getSensorForce补偿数据   原理见：https://www.yuque.com/lindong-9iuax/cs7vo1/hh2nkivsldpgwcxd
// 个人写的传感器那篇  对应傅里叶变换
struct Compensation {
    std::vector<double> frequencies;
    std::vector<double> a_k;
    std::vector<double> b_k;
};

//声明功能函数

using cv::Vec3d;

QImage MatToQImage(const cv::Mat& mat);

cv::Mat QImage2cvMat(QImage image);

/**
 * 初始化TCPOffSet,需要在控制前调用
 *
 * @param ur_rtde::RTDEControlInterface* rtde_c 
 */
void initTCPOffSet(ur_rtde::RTDEControlInterface* rtde_c);

/**
 * 获取移动坐标系三个轴的单位向量（在基座标系下的表示）(移动坐标系在传感器上)
 *
 * @param sixth_axis 第六轴向量,方向向外
 * @param end_effector_direction 末端移动方向(仅在坐标系下xoy平面)
 * @param base_frame_z_axis 基座标z轴向量
 * @param x_axis 移动坐标系x轴引用
 * @param y_axis 移动坐标系y轴引用
 * @param z_axis 移动坐标系z轴引用
 * @param rtde_r RTDEReceiveInterface类的指针
 */
void computeMotionCoordinateSystem(const Vec3d& sixth_axis,
    const Vec3d& end_effector_direction,
    const Vec3d& base_frame_z_axis,
    Vec3d& x_axis,
    Vec3d& y_axis,
    Vec3d& z_axis,
    ur_rtde::RTDEReceiveInterface* rtde_r);


/**
 * 计算两个平面的交线的单位向量。
 * 平面由它们的法向量定义。
 *
 * @param normal1 第一个平面的法向量。
 * @param normal2 第二个平面的法向量。
 * @return 交线的单位向量。 
 */
cv::Vec3d computeIntersectionLine(const cv::Vec3d& normal1, const cv::Vec3d& normal2);


/**
 * 计算从一个坐标系到另一个坐标系的旋转矩阵。
 *
 * @param axis1_1 第一个坐标系在基座标系下的第一个轴的单位向量。
 * @param axis1_2 第一个坐标系在基座标系下的第二个轴的单位向量。
 * @param axis1_3 第一个坐标系在基座标系下的第三个轴的单位向量。
 * @param axis2_1 第二个坐标系在基座标系下的第一个轴的单位向量。
 * @param axis2_2 第二个坐标系在基座标系下的第二个轴的单位向量。
 * @param axis2_3 第二个坐标系在基座标系下的第三个轴的单位向量。
 * @return cv::Mat 返回第一个坐标系到第二个坐标系的旋转矩阵。
 */
cv::Mat calculateRotationMatrix(const cv::Vec3d& axis1_1, const cv::Vec3d& axis1_2, const cv::Vec3d& axis1_3,
    const cv::Vec3d& axis2_1, const cv::Vec3d& axis2_2, const cv::Vec3d& axis2_3);


/**
 * 获取从基座标系到目标坐标系的旋转矩阵，参数为目标坐标系的三个轴单位向量在基座标系下的表示
 *
 * @param x_axis 目标坐标系x轴引用
 * @param y_axis 目标坐标系y轴引用
 * @param z_axis 目标坐标系z轴引用
 */
cv::Mat getRotationMatrixFromBase(const Vec3d& x_axis,
    const Vec3d& y_axis,
    const Vec3d& z_axis);


/**
 * 从旋转矩阵，获取三个轴单位向量在当前坐标系下表示
 *
 * @param x_axis 目标坐标系x轴引用
 * @param y_axis 目标坐标系y轴引用
 * @param z_axis 目标坐标系z轴引用
 * @param rotation_matrix 旋转矩阵
 */
void getAxisVecFromRotMat(Vec3d& x_axis,
    Vec3d& y_axis,
    Vec3d& z_axis,const cv::Mat& rotation_matrix);


/**
 * 转换六维力向量到新的坐标系。
 *
 * @param force_in_base 六维力向量在基座标系下的表示（前三个为力，后三个为力矩）。
 * @param rotation_matrix 从基座标系到新坐标系的旋转矩阵。
 * @param origin_in_base 新坐标系原点在基座标系下的坐标。
 * @return 六维力向量在新坐标系下的表示。
 */
std::vector<double> transformForceToNewCoordinateSystem(const std::vector<double>& force_in_base,
    const cv::Mat& rotation_matrix,
    const Vec3d& origin_in_base);


/**
* 获取第六轴的方向向量，方向向外
* 
* @param pose 从getActualTCPPose获得的数据
* @return 第六轴方向向量
*/
Vec3d getSixthAxisFromBasePose(std::vector<double> pose);


/**
* 计算工具点速度 （未验证）
*
* @param tcp_speed 从getActualTCPSpeed获得的数据
* @param tool_offset 工具相对与TCP坐标系的偏移（基于基坐标系）
* @return 工具坐标系下的六维速度
*/
std::vector<double> calculateToolPointSpeed(const std::vector<double>& tcp_speed,
    const Vec3d& tool_offset);


/**
* 获取移动坐标系下的力信息  (移动坐标系的原点和传感器坐标系的原点一致)
* 
* @param sixth_axis 第六轴向量,方向向外
* @param end_effector_direction 末端移动方向(仅在坐标系下xoy平面)
* @param base_frame_z_axis 基座标z轴向量
* @param rtde_r RTDEReceiveInterface类的指针
* @return 移动坐标系下的力信息
*/
std::vector<double> getMotionCoordinateSystemForce(const Vec3d& sixth_axis,
    const Vec3d& end_effector_direction,
    const Vec3d& base_frame_z_axis,
    ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* 根据对应当前坐标系Z/Y/X旋转某个角度获取对应旋转矩阵
*
* @param radians 弧度
* @param RotationAxis 旋转轴
* @return 旋转矩阵
*/
cv::Mat getRotationMatrix(RotationAxis axis, double radians);


/**
 * 将两个旋转向量组合成一个旋转向量
 *
 * @param rotVec1 第一个旋转向量，类型为cv::Vec3d
 * @param rotVec2 第二个旋转向量，类型为cv::Vec3d
 * @return 整合后的旋转向量，类型为cv::Vec3d
 */
cv::Vec3d combineRotationVectors(const cv::Vec3d& rotVec1, const cv::Vec3d& rotVec2);


/**
* 将 六维度力 旋转至给定 目标坐标系 （获得力在目标坐标系下的显示）
*
* @param force 六维度力
* @param RotationMatrix 当前坐标系到目标坐标系的旋转矩阵
*
* @return 目标坐标系下的六维度力
*/
std::vector<double> rotateForceToTarget(const std::vector<double>& force, const cv::Mat& rotation_matrix);

/**
* 将 向量旋转至目标坐标系 （获得向量在目标坐标系下的显示）
*
* @param Vec 向量
* @param RotationMatrix 当前坐标系到目标坐标系的旋转矩阵
*
* @return 目标坐标系下的向量表示
*/
cv::Vec3d rotateVecToTarget(const cv::Vec3d& Vec1, const cv::Mat& rotation_matrix);

/**
 * 将力从一个坐标系转换到另一个坐标系 (转换到另一个坐标系下表示)
 *
 * @param force 六维度力，包含三个力分量和三个力矩分量
 * @param distanceVector 从旧坐标系到新坐标系的距离向量
 * @return 新坐标系下的六维度力向量
 */
std::vector<double> TransformForceToNewFrame(const std::vector<double>& force, const cv::Vec3d& distanceVector);

/**
* 计算B到C的旋转矩阵
*
* @param R_AB A到B的旋转矩阵
* @param R_AC A到C的旋转矩阵
*
* @return B到C的旋转矩阵
*/
cv::Mat calculateRotationMatrix_BC(const cv::Mat& R_AB, const cv::Mat& R_AC);


/**
* 由getActualTCPPose计算基座标系到TCP坐标系的旋转矩阵
*
* @param pose getActualTCPPose获取到的位姿数据
* @return 基座标系到TCP坐标系的旋转矩阵
*/
cv::Mat calculateRotationMatrix_Base2TCP(const std::vector<double>& pose);



/**
* 将getActualTCPForce得到的力信息转换到tcp坐标系下,但该函数被证明只是到传感器坐标（与tcp坐标是平行关系）下
*
* @param world_force getActualTCPForce得到的力信息
* @param in_pose getActualTCPPose获取到的位姿数据
* @return TCP坐标系下的力信息
*/
std::vector<double> world_2tcp_force(const std::vector<double>& world_force, const std::vector<double> &in_pose);



/**
* 根据移动坐标系旋转的角度获取kxita
*
* @param pose getActualTCPPose得到的位姿信息
* @param speed getActualTCPSpeed得到的速度信息
* @param theta_x 移动坐标系关于x轴的转角
* @param theta_y 移动坐标系关于y轴的转角
* @return kxita （旋转向量）
* @param rtde_r RTDEReceiveInterface类的指针
*/
Vec3d get_Kxita_From_theta_x_and_theta_y_baseMotion(std::vector<double> pose, std::vector<double> speed
                                                    ,double theta_x, double theta_y, ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* 根据两个给定向量计算出两向量的旋转向量，以实现第一个向量向第二个向量的对齐
*
* @param Vec1 起始向量
* @param Vec2 目标向量
* @return 旋转向量
*/
cv::Vec3d CalculateRotationVector(const cv::Vec3d& Vec1, const cv::Vec3d& Vec2);



/**
* 从提供的旋转矩阵中提取欧拉角，假设旋转矩阵是按照 XYZ 顺序旋转的
*
* @param rotationMatrix 旋转矩阵
* @return 三个角度 分别代表绕 X 轴、Y 轴和 Z 轴的旋转
*/
cv::Vec3d convertRotaionMatrixToEulerAngles(const cv::Mat& rotationMatrix);


/**
* resize向量大小
*
* @param vector 要修改的向量
* @return newLength 修改后向量的长度
*/
void ResizeVector(cv::Vec3d& vector, double newLength);


/**
* 获取TCP坐标系下的力
*
* @param rtde_r RTDEReceiveInterface类的指针
* @return TCP坐标系下的六维力
*/
std::vector<double> getTCPForce(ur_rtde::RTDEReceiveInterface* rtde_r);


/** 
* 获取传感器坐标系下的力 (假设这个坐标系在法兰盘上)
*
* @param rtde_r RTDEReceiveInterface类的指针
* @param isCompensate 是否开启补偿
* @param enableFilter 是否开启滤波
* @return 传感器坐标系下的六维度力
*/
std::vector<double> getSensorForce(ur_rtde::RTDEReceiveInterface* rtde_r,bool isCompensate = false,bool enableFilter = false);


/**
* 获取传感器坐标系下的姿态 (假设这个坐标系在法兰盘上)
*
* @param rtde_r RTDEReceiveInterface类的指针
* @return 传感器坐标系下的姿态
*/
std::vector<double> getSensorPose(ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* 获取传感器坐标系下的速度 (假设这个坐标系在法兰盘上)  (未验证)
*
* @param rtde_r RTDEReceiveInterface类的指针
* @return 传感器坐标系下的姿态
*/
std::vector<double> getSensorSpeed(ur_rtde::RTDEReceiveInterface* rtde_r);


/**
* 获取 本人公式模型的传感器坐标系的Force以及基座标到该坐标系的旋转矩阵 ,CDS 是coordinate system
*
* @param rtde_r RTDEReceiveInterface类的指针
* @return 本人公式模型的传感器坐标系的Force以及基座标到该坐标系的旋转矩阵
*/
std::pair<std::vector<double>,cv::Mat> getMyFormulaCDSForceAndBase2Formula(ur_rtde::RTDEReceiveInterface* rtde_r);

/**
* 根据当前姿态与旋转矢量（该旋转矢量用世界坐标系描述）获取新姿态
*
* @param pose 当前姿态
* @param RotateVec 旋转矢量（该旋转矢量用世界坐标系描述）
* @return 新姿态
*/
std::vector<double> getNewPoseFromCurPoseAndRotateVec(const std::vector<double>& pose, const cv::Vec3d& RotateVec);


/**
 * 计算一组旋转向量的平均旋转向量。
 *
 * 本函数接收一系列的旋转向量，首先将每个旋转向量转换为相应的旋转矩阵。
 * 然后计算这些旋转矩阵的算术平均值，并通过奇异值分解找到最接近的正交矩阵，
 * 最终将此正交矩阵转换回旋转向量作为平均旋转的表示。
 *
 * @param rotation_vectors 一个包含多个cv::Vec3d旋转向量的向量，每个向量代表一个三维空间的旋转。
 * @return cv::Vec3d 返回平均旋转向量，其方向表示旋转轴，长度表示旋转角度（单位：弧度）。
 */
cv::Vec3d averageRotationVector(const std::vector<cv::Vec3d>& rotation_vectors);


/**
* 获取基座标系到第k轴的 旋转矢量与位置
*
* @param rtde_r RTDEReceiveInterface类的指针
* @param k 第k轴
* @return vector<double>
*/
std::vector<double> getBase2kAxisPose(ur_rtde::RTDEReceiveInterface* rtde_r, const int num);


/**
* 修改getSensorForce补偿数据 Compensation_torque  ---对应语雀 以前的猜想
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
* 修改getSensorForce补偿数据 Compensation    ---对应语雀 传感器部分 傅里叶变换补偿
*
* @param P_Fx_ak_bk Fx补偿对应的正余弦系数
* @param P_Fy_ak_bk Fy补偿对应的正余弦系数
* @param P_Fz_ak_bk Fz补偿对应的正余弦系数
* @param P_Mx_ak_bk Mx补偿对应的正余弦系数
* @param P_My_ak_bk My补偿对应的正余弦系数
* @param Fx_frequencies My补偿对应的正余弦系数
* @param Fy_frequencies My补偿对应的正余弦系数
* @param Fz_frequencies My补偿对应的正余弦系数
* @param Mx_frequencies My补偿对应的正余弦系数
* @param My_frequencies My补偿对应的正余弦系数
* @param isCompensation  是否开启补偿
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
 * 使用余弦和正弦基函数拟合周期函数
 * @param frequencies 频率向量，单位应该是适合问题的单位，如Hz
 * @param anglesInRadians 角度向量，单位是弧度
 * @param observedValues 对应角度的观测值向量
 * @return 一个包含两个向量的pair，分别是余弦和正弦系数向量 ak和bk
 */
std::pair<std::vector<double>, std::vector<double>> fitCosineSineComponents(
    const std::vector<double>& frequencies,
    const std::vector<double>& anglesInRadians,
    const std::vector<double>& observedValues);



/**  ----废弃。
* 
* 粗略标定力传感器原点距离传感器平面的距离，施加的两个力尽可能一样
* (具体原理：在六维传感器平面构造一个固定坐标系，固定坐标系的方向与TCP坐标系一致，
*  现在要找到getactualtcpforce()所得力所在坐标系的原点距离固定坐标系Z轴多远，因为这两个坐标系只是平移关系，
*  于是现在通过在大概(0,0,x)处施加一个力，在(0,0,x + k)处施加一个力，通过两组六维度力来标定获得两个坐标系的力)
*
* @param forceA 第一个点A
* @param forceB 第二个点B
* @param x 施加的第一个点的受力坐标 (0,0,x)
* @param k 与第一个点的距离，即第二个点的坐标为(0,0,x + k)
* @return d getactualtcpforce()所得力所在坐标系的原点距离固定坐标系Z轴的距离
*/
double calculateD(const std::vector<double>& forceA, const std::vector<double>& forceB, double x, double k);


/**
* 计算一个给定姿态的逆（模拟URScript中 pose_inv）
*
* @param p_from 工具的原始姿态，一个包含位置和方向的空间向量
* @return 逆姿态变换，也是一个空间向量
*/
std::vector<double> pose_inv(const std::vector<double>& pose);


/**
 * @brief 姿态变换，用于相对于工具或自定义特征/框架进行移动
 * 第一个参数 p_from 用于变换第二个参数 p_from_to，然后返回结果。这意味着结果是从 p_from 的坐标系开始，
 * 在该坐标系中移动 p_from_to 后得到的姿态。
 * 这个函数可以从两个不同的角度看待。要么是通过 p_from 的参数来变换（即平移和旋转）p_from_to。
 * 要么是用于获取首先进行 p_from 移动，然后在那个基础上进行 p_from_to 移动的结果姿态。
 * 如果将姿态视为变换矩阵，则表示为：
 * @verbatim
 * T_world->to = T_world->from * T_from->to
 * T_x->to = T_x->from * T_from->to
 * @endverbatim
 * @param p_from 起始姿态（空间向量）
 * @param p_from_to 相对于起始姿态的姿态变化（空间向量）
 * @return 结果姿态（空间向量）
 */
std::vector<double> poseTrans(const std::vector<double>& p_from, const std::vector<double>& p_from_to);


/**
* 根据一个指定的变换对力矩(wrench)进行变换。这个函数用于改变力矩的参考点。
* 注意：变换力矩不像变换姿态那样简单，因为力矩的转动部分会随着平移距离的长度而变化。
*
* 力矩变换的公式为 w_to = T_from_to * w_from，其中：
* - w_from 是原始的力矩，表示为一个六维向量 [F_x, F_y, F_z, M_x, M_y, M_z]。
*   其中 F_x, F_y, F_z 是力的三个分量，M_x, M_y, M_z 是力矩的三个分量。
* - T_from_to 是一个六维向量，表示从原始点到新点的旋转和平移变换。
*   前三个分量是旋转向量（通常以轴-角表示），后三个分量是平移向量。
* - w_to 是变换后的力矩，也是一个六维向量。
*
* @param T_from_to 从原始点到新点的变换（姿态）
* @param w_from 要变换的力矩，格式为 [F_x, F_y, F_z, M_x, M_y, M_z]
* @return 变换后的力矩 w_to，格式为 [F_x, F_y, F_z, M_x, M_y, M_z]
*/
std::vector<double> wrench_trans(const std::vector<double>& T_from_to, const std::vector<double>& w_from);



/**
 * 使用最小二乘法求解线性系统，具体使用正规方程方法 ,即x = (ATA)-1 ATb
 *
 * @param A 系数矩阵，大小为 m x n，其中 m 是方程的数量，n 是未知数的数量
 * @param b 结果向量，大小为 m x 1
 * @return 解向量，大小为 n x 1，如果系统无解则返回空向量
 */
cv::Mat solveLeastSquaresNormalEquation(const cv::Mat& A, const cv::Mat& b);


/**
 * 卡尔曼滤波器类，用于传感器数据平滑和噪声减少。
 */
class KalmanFilter {
public:
    KalmanFilter(double process_noise, double measurement_noise);

    /**
     * 使用新的测量数据更新卡尔曼滤波器。
     *
     * @param measurement 要纳入的新数据点。
     * @returns 处理测量后的估计状态。
     */
    double update(double measurement);

private:
    double estimate;           ///< 当前状态估计。
    double error_covariance;   ///< 当前估计误差协方差。
    double process_noise;      ///< 过程噪声协方差。
    double measurement_noise;  ///< 测量噪声协方差。
    double kalman_gain;        ///< 卡尔曼增益。
};


/**
 * 对传感器数据应用低通滤波，以减少高频噪声。         ------经过傅里叶变换测量出来传感器六维力传感器后，分析出来alpha的值为2 * pi * 1e-3
 *
 * @param current 当前的传感器读数。
 * @param previous 上一次的过滤传感器读数。
 * @param alpha 滤波的平滑因子。
 * @returns 平滑后的传感器读数。
 */
std::vector<double> lowPassFilter(const std::vector<double>& current, double alpha);


/**
 * K滤波计算Euler角速度和角加速度
 *
 * @param MeasureData 测量数据
 * @param MeasureDatas 测量数据集合
 * @param idv_FillterDelay 滤波延迟
 * @param idv_MNoiseCov 测量噪声协方差
 * @param idv_PNoiseCov 过程噪声协方差
 * @returns 滤波结果
 */
double Cal_EulerKFilter(const double& MeasureData, std::list<double>& MeasureDatas, const int& idv_FillterDelay,
    const double& idv_MNoiseCov, const double& idv_PNoiseCov);


/**
 * Kalman滤波器算法
 *
 * @param ilv_MeasureDatas 测量数据
 * @param idv_MNoiseCov 测量噪声协方差
 * @param idv_PNoiseCov 过程噪声协方差
 * @return 滤波结果
 */
double Cal_KalmanFilter(std::vector<double> ilv_MeasureDatas, double idv_MNoiseCov, double idv_PNoiseCov);


/**
 * 将vector<vector<double>>数据存储到CSV文件中
 *
 * @param data 要存储的数据，类型为vector<vector<double>>
 * @param path 存储CSV文件的路径
 * @return bool 成功返回true，失败返回false
 */
bool saveToCsv(const std::vector<std::vector<double>>& data, const std::string& path);


/**
 * 从CSV文件读取数据到vector<vector<double>>
 *
 * @param path CSV文件的路径
 * @return vector<vector<double>> 读取的数据
 */
std::vector<std::vector<double>> readFromCsv(const std::string& path);