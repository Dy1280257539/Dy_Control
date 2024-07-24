#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <chrono>  // 包含chrono库
#include<qdebug.h>
#include <QtCore/qlogging.h>
#include <utility> 
#include <QReadWriteLock>

#include "pcfunctionSet.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

//——————————————————————对应语雀方法：移动坐标系（圆心与传感器坐标系一致）：力等效到球心————————————————————//
/**
 * @brief 参数结构体，用于保存方程系统的参数
 */
struct param
{
    param() = delete;
    param(double Fx, double Fy, double Fz, double Mx, double My, double Mz, double R, double l, double theta, double My_prime) :
        Fx(Fx), Fy(Fy), Fz(Fz), Mx(Mx), My(My),Mz(Mz), R(R), l(l), theta(theta), My_prime(My_prime) {}
    double Fx;//传感器坐标系下力
    double Fy;
    double Fz;
    double Mx;
    double My;
    double Mz;
    double R; //按摩球半径
    double l; //传感器到球心的距离
    double theta;
    double My_prime;
};


/**
 * @brief 计算给定 x 和参数 p 的方程系统 F(x)
 * @param x 输入向量 x
 * @param p 参数结构体
 * @return 计算得到的方程系统 F(x)
 */
VectorXd F(const VectorXd& x, const param& p);

/**
 * @brief 计算给定 x 和参数 p 的雅可比矩阵 J(x)
 * @param x 输入向量 x
 * @param p 参数结构体
 * @return 计算得到的雅可比矩阵 J(x)
 */
MatrixXd J(const VectorXd& x, const param& p);

/**
 * @brief 执行牛顿-拉弗森方法来求解方程系统 F(x) = 0
 * @param x0 初始猜测的解向量
 * @param p 参数结构体
 * @param tol 收敛容差（默认值: 1e-6）
 * @param max_iter 最大迭代次数（默认值: 100）
 * @return 满足 F(x) = 0 的解向量 x
 */
VectorXd NewtonRaphson(VectorXd x0, const param& p, double tol = 1e-6, int max_iter = 1000);

/**
 * @brief 测试函数，演示使用牛顿-拉弗森方法的用法
 * @return 成功时返回 0
 */
int test();


//——————————————————————————————对应语雀方法：点云求解相关——————————————————————————————————//

/**
 * 对二维点集进行多项式拟合。
 *
 * @param points 输入的点集，类型为 std::vector<std::pair<int, int>>，每个元素代表一个二维点 (x, y)。
 * @param order 拟合多项式的阶数。
 * @return 拟合多项式的系数，存储在 Eigen::VectorXd 中。
 */
VectorXd polynomialFit(const std::vector<std::pair<double, double>>& points, int order);


//——————————————————————移动坐标系（圆心与传感器坐标系一致）：二维：利用二项摩擦理论 + 粒子群算法————————————————————//

//
//
///**
// * @brief 参数结构体，用于保存方程系统的参数
// */
//struct param_for_binomial_friction
//{
//    param_for_binomial_friction() = delete;
//    param_for_binomial_friction(double Fx, double Fz, double My,double R, double l, double theta,double beta,double E_equivalence,double tau0) :
//        Fx(Fx), Fz(Fz), My(My), R(R), l(l), theta(theta), beta(beta), E_equivalence(E_equivalence), tau0(tau0){}
//    double Fx;//传感器坐标系下力
//    double Fz;
//    double My;
//    double R; //按摩球半径
//    double l; //传感器到球心的距离
//    double theta;
//    double beta;
//    double E_equivalence;
//    double tau0;
//};
//
///**
// * 修改 静态变量curVal （curVal是当前从机器人读到的一些数据以及一些几何参数）
// *
// * @param Fx x轴方向的力
// * @param Fy y轴方向的力
// * @param My y轴方向的力矩
// * @param R 球半径
// * @param l 传感器坐标系原点到球心的距离
// * @param beta 压力系数
// * @param E_equivalence 等效杨氏模量
// * @param tau0 初始界面剪切强度
// */
//void change_curVal(const double& Fx, const double& Fz, const double& My, 
//                    const double& R, const double& l, const double& beta, 
//                    const double& E_equivalence, const double& tau0);
//
///**
// * 适应度函数
// *
// * @param particle 粒子群算法中的某一个粒子
// * @return 适应度
// */
//double FitnessFunction(Particle& particle);
//
///**
// * 修改 通过PSO算法获取 θ转角
// * 
// * @return theta 
// */
//double getThetaByPSO();
//
//
