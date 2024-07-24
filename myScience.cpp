#include"myScience.h"

//����������������������������������������������Ӧ��ȸ�������ƶ�����ϵ��Բ���봫��������ϵһ�£�������Ч�����ġ���������������������������������������//
// ʾ��������
VectorXd F(const VectorXd& x, const param& p) {
    VectorXd f(3);
    double theta = x(0), Fx_prime = x(1), Fz_prime = x(2);

    f(0) = -Fx_prime * cos(theta) + Fz_prime * sin(theta) - p.Fx;
    f(1) = Fx_prime * sin(theta) + Fz_prime * cos(theta) - p.Fz;
    f(2) = p.My_prime - p.l * (-Fx_prime * cos(theta) + Fz_prime * sin(theta)) - p.My;

    return f;
}

// �ſɱȾ���
MatrixXd J(const VectorXd& x, const param& p) {
    MatrixXd j(3, 3);
    double theta = x(0), Fx_prime = x(1), Fz_prime = x(2);

    j(0, 0) = Fx_prime * sin(theta) + Fz_prime * cos(theta);
    j(0, 1) = -cos(theta);
    j(0, 2) = sin(theta);

    j(1, 0) = Fx_prime * cos(theta) - Fz_prime * sin(theta);
    j(1, 1) = sin(theta);
    j(1, 2) = cos(theta);

    j(2, 0) = p.l * (Fx_prime * sin(theta) + Fz_prime * cos(theta));
    j(2, 1) = p.l * cos(theta);
    j(2, 2) = -p.l * sin(theta);

    return j;
}

// ţ��-����ɭ����
VectorXd NewtonRaphson(VectorXd x0, const param& p, double tol, int max_iter) {
    int iter = 0;
    VectorXd x = x0;
    VectorXd delta;

    while (iter < max_iter) {
        VectorXd f_val = F(x, p);
        MatrixXd J_val = J(x, p);
        delta = J_val.fullPivLu().solve(-f_val); //LU�ֽⷨ

        x += delta;

        if (delta.norm() < tol) {
            break;
        }

        iter++;
    }

    return x;
}




int test() {
    VectorXd x0(3);
    x0 << 0, 0, 0;  // ��ʼ�²�

    double tol = 1e-6;  // �ݲ�
    int max_iter = 100;  // ����������

    // ����Ĳ���ֵ
    param p(10, 20, 30, 10, 10, 10, 10, 5, 2, 40);

    auto start = std::chrono::high_resolution_clock::now();  // ��ʼʱ��

    VectorXd solution = NewtonRaphson(x0, p, tol, max_iter);  // ִ��ţ��-����ɭ����

    auto stop = std::chrono::high_resolution_clock::now();  // ����ʱ��
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);  // �������ʱ��

    qDebug() << "Solution: \n" << solution[0] << solution[1] << solution[2] << endl;
    qDebug() << "Time taken: " << duration.count() << " milliseconds";

    return 0;
}


//��������������������������������������������������������������Ӧ��ȸ���������������ء�������������������������������������������������������������������//

VectorXd polynomialFit(const std::vector<std::pair<double, double>>& points, int order) {
    int n = points.size();

    // ���� Vandermonde ����
    MatrixXd X(n, order + 1);
    VectorXd y(n);

    for (int i = 0; i < n; ++i) {
        double x = points[i].first;
        y(i) = points[i].second;

        for (int j = 0; j <= order; ++j) {
            X(i, j) = pow(x, j);
        }
    }

    // ʹ����С���˷��������ʽϵ��
    VectorXd coeffs = X.householderQr().solve(y);
    return coeffs;
}

//���������������������������������������������ƶ�����ϵ��Բ���봫��������ϵһ�£�����ά�����ö���Ħ������ + ����Ⱥ�㷨����������������������������������������//

//static param_for_binomial_friction curVal{.0,.0,.0,.0,.0,.0,.0,.0,.0};
//
//static QReadWriteLock curValLock;
//
//void change_curVal(const double& Fx, const double& Fz, const double& My,
//    const double& R, const double& l, const double& beta,
//    const double& E_equivalence, const double& tau0) {
//    QWriteLocker locker(&curValLock);
//    curVal.Fx = Fx;
//    curVal.Fz = Fz;
//    curVal.My = My;
//    curVal.R = R;
//    curVal.l = l;
//    curVal.beta = beta;
//    curVal.E_equivalence = E_equivalence;
//    curVal.tau0 = tau0;
//}
//
//
////��Ӧ�Ⱥ���
//double FitnessFunction(Particle& particle) {
//    // ��ȡ���ӵ�ǰλ����Ϊtheta
//    double theta = particle.position_[0];
//
//    // ʹ��curVal�е�ֵ
//    double F_x = curVal.Fx;
//    double F_z = curVal.Fz;
//    double R = curVal.R;
//    double l = curVal.l;
//    double M_y = curVal.My;
//    double E_star = curVal.E_equivalence;
//    double beta = curVal.beta;
//    double tau_0 = curVal.tau0;
//
//    // ���巽�����ĸ�������
//    double W = F_z * cos(theta) - F_x * sin(theta);
//    double common_term = tau_0 + beta * W / (M_PI * pow((3 * R * W) / (4 * E_star), 2.0 / 3.0));
//    double term1 = (2.0 / 3.0) * M_PI * R * R * R * common_term;
//    double term2 = 2 * M_PI * common_term * (R * R * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0 + R * pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) / 2.0 - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0);
//    double term3 = R * F_z * sin(theta) + F_x * (R * cos(theta) + l);
//
//    // ���㷽�������Ҳ�Ĳ�ֵ
//    double difference = term1 - term2 - term3 - M_y;
//
//    // ���ط������߲�ľ���ֵ��Ϊ��Ӧ��ֵ
//    return fabs(difference);
//}
//
//
//
//double getThetaByPSO() {
//
//    auto start = std::chrono::high_resolution_clock::now();  // ��ʼʱ��
//
//    PSOPara psopara(1, true);
//    psopara.particle_num_ = 30;		// ���Ӹ���
//    psopara.max_iter_num_ = 700;	// ����������
//    psopara.dt_[0] = 0.1;			// ��һά���ϵ�ʱ�䲽��
//    psopara.wstart_[0] = 0.9;		// ��һά���ϵ���ʼȨ��ϵ��
//    psopara.wend_[0] = 0.4;			// ��һά���ϵ���ֹȨ��ϵ��
//    psopara.C1_[0] = 1.49445;		// ��һά���ϵļ��ٶ�����
//    psopara.C2_[0] = 1.49445;		
//
//    // ��������������ޣ�������������
//    psopara.lower_bound_[0] = -M_PI / 2;	// ��һά����������
//    psopara.upper_bound_[0] = M_PI / 2;	// ��һά����������
//
//    PSOOptimizer psooptimizer(&psopara, FitnessFunction);
//
//    std::srand((unsigned int)time(0));
//    psooptimizer.InitialAllParticles();
//    double fitness = psooptimizer.all_best_fitness_;
//    //double* result = new double[psooptimizer.max_iter_num_];
//
//    for (int i = 0; i < psooptimizer.max_iter_num_; i++)
//    {
//        psooptimizer.UpdateAllParticles();
//        //result[i] = psooptimizer.all_best_fitness_;
//        //std::cout << "��" << i << "�ε��������";
//        //std::cout << "x = " << psooptimizer.all_best_position_[0] << ", " << "y = " << psooptimizer.all_best_position_[1];
//        //std::cout << ", fitness = " << result[i] << std::endl;
//    }
//
//    curVal.theta = psooptimizer.all_best_position_[0];
//
//    auto stop = std::chrono::high_resolution_clock::now();  // ����ʱ��
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);  // �������ʱ��
//
//    qDebug() << "Time taken: " << duration.count() << " milliseconds";
//
//    return curVal.theta;
//}




//������������������������������������������������������������������������������matlab engineʹ�ò��ԡ�����������������������������������������������������������������//

//#include <stdlib.h>
//
//#include <stdio.h>
//
//#include <string.h>
//
//#include "engine.h" // add header file
//#include <runParticleswarm.h>
//
//
//#include "engine.h"
//
//#pragma comment(lib, "ws2_32.lib")
//#pragma comment(lib, "libeng.lib")
//#pragma comment(lib, "libmx.lib")
//#pragma comment(lib, "libmat.lib")
//
//
//int main() {
//    using namespace std::chrono;
//
//    // ��¼���� MATLAB ����Ŀ�ʼʱ��
//    auto start = high_resolution_clock::now();
//
//    Engine* matlabEngine;
//
//    // ���� MATLAB ����
//    if (!(matlabEngine = engOpen(nullptr))) {
//        std::cerr << "�޷����� MATLAB ���棡" << std::endl;
//        return -1;
//    }
//
//    auto end = high_resolution_clock::now();
//    qDebug() << "���� MATLAB ������ʱ: "
//        << duration_cast<milliseconds>(end - start).count()
//        << " ����" << endl;
//
//    engSetVisible(matlabEngine, true);
//
//    // ���� .m �ļ�������·��
//    std::string filePath = "C:\\Users\\Dy\\Desktop\\runParticleswarm.m";
//
//    // ������������
//    std::string command = "run('" + filePath + "')";
//
//    // ��¼ִ�� .m �ļ��Ŀ�ʼʱ��
//    start = high_resolution_clock::now();
//
//    // ִ�� .m �ļ�
//    engEvalString(matlabEngine, command.c_str());
//
//    end = high_resolution_clock::now();
//    qDebug() << "ִ�� .m �ļ���ʱ: "
//        << duration_cast<milliseconds>(end - start).count()
//        << " ����" << endl;
//
//    // �ر� MATLAB ����
//    engClose(matlabEngine);
//
//    return 0;
//}



//���������������������������������������������ƶ�����ϵ��Բ���봫��������ϵһ�£�����ά�����ö���Ħ������ + ����Ӧ����Ⱥ�㷨����������������������������������������//


//static param_for_binomial_friction curVal{.0,.0,.0,.0,.0,.0,.0,.0,.0};
//
//static QReadWriteLock curValLock;
//
//void change_curVal(const double& Fx, const double& Fz, const double& My,
//    const double& R, const double& l, const double& beta,
//    const double& E_equivalence, const double& tau0) {
//    QWriteLocker locker(&curValLock);
//    curVal.Fx = Fx;
//    curVal.Fz = Fz;
//    curVal.My = My;
//    curVal.R = R;
//    curVal.l = l;
//    curVal.beta = beta;
//    curVal.E_equivalence = E_equivalence;
//    curVal.tau0 = tau0;
//}
//
//
////��Ӧ�Ⱥ���
//double FitnessFunction(Particle& particle) {
//    // ��ȡ���ӵ�ǰλ����Ϊtheta
//    double theta = particle.position_[0];
//
//    // ʹ��curVal�е�ֵ
//    double F_x = curVal.Fx;
//    double F_z = curVal.Fz;
//    double R = curVal.R;
//    double l = curVal.l;
//    double M_y = curVal.My;
//    double E_star = curVal.E_equivalence;
//    double beta = curVal.beta;
//    double tau_0 = curVal.tau0;
//
//    // ���巽�����ĸ�������
//    double W = F_z * cos(theta) - F_x * sin(theta);
//    double common_term = tau_0 + beta * W / (M_PI * pow((3 * R * W) / (4 * E_star), 2.0 / 3.0));
//    double term1 = (2.0 / 3.0) * M_PI * R * R * R * common_term;
//    double term2 = 2 * M_PI * common_term * (R * R * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0 + R * pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) / 2.0 - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0);
//    double term3 = R * F_z * sin(theta) + F_x * (R * cos(theta) + l);
//
//    // ���㷽�������Ҳ�Ĳ�ֵ
//    double difference = term1 - term2 - term3 - M_y;
//
//    // ���ط������߲�ľ���ֵ��Ϊ��Ӧ��ֵ
//    return fabs(difference);
//}
//


//double getThetaByPSO() {
//
//    auto start = std::chrono::high_resolution_clock::now();  // ��ʼʱ��
//
//    PSOPara psopara(1, true);
//    psopara.particle_num_ = 30;		// ���Ӹ���
//    psopara.max_iter_num_ = 700;	// ����������
//    psopara.dt_[0] = 0.1;			// ��һά���ϵ�ʱ�䲽��
//    psopara.wstart_[0] = 0.9;		// ��һά���ϵ���ʼȨ��ϵ��
//    psopara.wend_[0] = 0.4;			// ��һά���ϵ���ֹȨ��ϵ��
//    psopara.C1_[0] = 1.49445;		// ��һά���ϵļ��ٶ�����
//    psopara.C2_[0] = 1.49445;		
//
//    // ��������������ޣ�������������
//    psopara.lower_bound_[0] = -M_PI / 2;	// ��һά����������
//    psopara.upper_bound_[0] = M_PI / 2;	// ��һά����������
//
//    PSOOptimizer psooptimizer(&psopara, FitnessFunction);
//
//    std::srand((unsigned int)time(0));
//    psooptimizer.InitialAllParticles();
//    double fitness = psooptimizer.all_best_fitness_;
//    //double* result = new double[psooptimizer.max_iter_num_];
//
//    for (int i = 0; i < psooptimizer.max_iter_num_; i++)
//    {
//        psooptimizer.UpdateAllParticles();
//        //result[i] = psooptimizer.all_best_fitness_;
//        //std::cout << "��" << i << "�ε��������";
//        //std::cout << "x = " << psooptimizer.all_best_position_[0] << ", " << "y = " << psooptimizer.all_best_position_[1];
//        //std::cout << ", fitness = " << result[i] << std::endl;
//    }
//
//    curVal.theta = psooptimizer.all_best_position_[0];
//
//    auto stop = std::chrono::high_resolution_clock::now();  // ����ʱ��
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);  // �������ʱ��
//
//    qDebug() << "Time taken: " << duration.count() << " milliseconds";
//
//    return curVal.theta;
//}


//#include <casadi/casadi.hpp>
//#include <iostream>
//
//using namespace casadi;
//
//int main() {
//    // ������֪��
//    double R = 0.01;
//    double F_z = 5.55744;
//    double F_x = -1.97;
//    double E_star = 16217.43598;
//    double tau_0 = 1.34e5;
//    double beta = -0.2028;
//    double l = 0.055;
//    double M_y = 0.2;
//
//    // ���� theta ��Ϊ�Ż�����
//    SX theta = SX::sym("theta");
//
//    // ���巽��
//    SX equation = fabs((2.0 / 3.0) * M_PI * pow(R, 3) * (tau_0 + beta * (F_z * cos(theta) - F_x * sin(theta)) / (M_PI * pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 2.0 / 3.0)))
//        - 2 * M_PI * (tau_0 + beta * (F_z * cos(theta) - F_x * sin(theta)) / (M_PI * pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 2.0 / 3.0)))
//        * (pow(R, 2) * sqrt(pow(R, 2) - pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0)) / 3
//            + R * pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0) / 2
//            - pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0) * sqrt(pow(R, 2) - pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0)) / 3)
//        - R * F_z * sin(theta) - F_x * (R * cos(theta) + l) - M_y);
//
//    SX objective = pow(equation, 2);
//
//    // �����������������
//    SXDict nlp = { {"x", theta}, {"f", objective} };
//    Function solver = nlpsol("solver", "ipopt", nlp);
//
//    // ���ó�ʼ��ͽ���
//    std::map<std::string, DM> args;
//    args["x0"] = 0;  // ��ʼ�²�
//    args["lbx"] = -M_PI / 2; // theta ���½�
//    args["ubx"] = M_PI / 2; // theta ���Ͻ�
//    args["max_iter"] = 10000;  // ������������������������Ϊ 10000
//    args["tol"] = 1e-4;       // �������̶ȣ���������Ϊ 1e-4
//
//    solver.setOption("ipopt", opts);
//
//    // ���
//    std::map<std::string, DM> sol = solver(args);
//
//    // ������
//    qDebug() << "Solution: theta = " << double(sol["x"]) * (180.0 / M_PI) << endl;
//
//    system("pause");
//
//    return 0;
//}
