#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <chrono>  // ����chrono��
#include<qdebug.h>
#include <QtCore/qlogging.h>
#include <utility> 
#include <QReadWriteLock>

#include "pcfunctionSet.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

//����������������������������������������������Ӧ��ȸ�������ƶ�����ϵ��Բ���봫��������ϵһ�£�������Ч�����ġ���������������������������������������//
/**
 * @brief �����ṹ�壬���ڱ��淽��ϵͳ�Ĳ���
 */
struct param
{
    param() = delete;
    param(double Fx, double Fy, double Fz, double Mx, double My, double Mz, double R, double l, double theta, double My_prime) :
        Fx(Fx), Fy(Fy), Fz(Fz), Mx(Mx), My(My),Mz(Mz), R(R), l(l), theta(theta), My_prime(My_prime) {}
    double Fx;//����������ϵ����
    double Fy;
    double Fz;
    double Mx;
    double My;
    double Mz;
    double R; //��Ħ��뾶
    double l; //�����������ĵľ���
    double theta;
    double My_prime;
};


/**
 * @brief ������� x �Ͳ��� p �ķ���ϵͳ F(x)
 * @param x �������� x
 * @param p �����ṹ��
 * @return ����õ��ķ���ϵͳ F(x)
 */
VectorXd F(const VectorXd& x, const param& p);

/**
 * @brief ������� x �Ͳ��� p ���ſɱȾ��� J(x)
 * @param x �������� x
 * @param p �����ṹ��
 * @return ����õ����ſɱȾ��� J(x)
 */
MatrixXd J(const VectorXd& x, const param& p);

/**
 * @brief ִ��ţ��-����ɭ��������ⷽ��ϵͳ F(x) = 0
 * @param x0 ��ʼ�²�Ľ�����
 * @param p �����ṹ��
 * @param tol �����ݲĬ��ֵ: 1e-6��
 * @param max_iter ������������Ĭ��ֵ: 100��
 * @return ���� F(x) = 0 �Ľ����� x
 */
VectorXd NewtonRaphson(VectorXd x0, const param& p, double tol = 1e-6, int max_iter = 1000);

/**
 * @brief ���Ժ�������ʾʹ��ţ��-����ɭ�������÷�
 * @return �ɹ�ʱ���� 0
 */
int test();


//��������������������������������������������������������������Ӧ��ȸ���������������ء�������������������������������������������������������������������//

/**
 * �Զ�ά�㼯���ж���ʽ��ϡ�
 *
 * @param points ����ĵ㼯������Ϊ std::vector<std::pair<int, int>>��ÿ��Ԫ�ش���һ����ά�� (x, y)��
 * @param order ��϶���ʽ�Ľ�����
 * @return ��϶���ʽ��ϵ�����洢�� Eigen::VectorXd �С�
 */
VectorXd polynomialFit(const std::vector<std::pair<double, double>>& points, int order);


//���������������������������������������������ƶ�����ϵ��Բ���봫��������ϵһ�£�����ά�����ö���Ħ������ + ����Ⱥ�㷨����������������������������������������//

//
//
///**
// * @brief �����ṹ�壬���ڱ��淽��ϵͳ�Ĳ���
// */
//struct param_for_binomial_friction
//{
//    param_for_binomial_friction() = delete;
//    param_for_binomial_friction(double Fx, double Fz, double My,double R, double l, double theta,double beta,double E_equivalence,double tau0) :
//        Fx(Fx), Fz(Fz), My(My), R(R), l(l), theta(theta), beta(beta), E_equivalence(E_equivalence), tau0(tau0){}
//    double Fx;//����������ϵ����
//    double Fz;
//    double My;
//    double R; //��Ħ��뾶
//    double l; //�����������ĵľ���
//    double theta;
//    double beta;
//    double E_equivalence;
//    double tau0;
//};
//
///**
// * �޸� ��̬����curVal ��curVal�ǵ�ǰ�ӻ����˶�����һЩ�����Լ�һЩ���β�����
// *
// * @param Fx x�᷽�����
// * @param Fy y�᷽�����
// * @param My y�᷽�������
// * @param R ��뾶
// * @param l ����������ϵԭ�㵽���ĵľ���
// * @param beta ѹ��ϵ��
// * @param E_equivalence ��Ч����ģ��
// * @param tau0 ��ʼ�������ǿ��
// */
//void change_curVal(const double& Fx, const double& Fz, const double& My, 
//                    const double& R, const double& l, const double& beta, 
//                    const double& E_equivalence, const double& tau0);
//
///**
// * ��Ӧ�Ⱥ���
// *
// * @param particle ����Ⱥ�㷨�е�ĳһ������
// * @return ��Ӧ��
// */
//double FitnessFunction(Particle& particle);
//
///**
// * �޸� ͨ��PSO�㷨��ȡ ��ת��
// * 
// * @return theta 
// */
//double getThetaByPSO();
//
//
