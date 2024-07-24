#include"myScience.h"

//——————————————————————对应语雀方法：移动坐标系（圆心与传感器坐标系一致）：力等效到球心————————————————————//
// 示例方程组
VectorXd F(const VectorXd& x, const param& p) {
    VectorXd f(3);
    double theta = x(0), Fx_prime = x(1), Fz_prime = x(2);

    f(0) = -Fx_prime * cos(theta) + Fz_prime * sin(theta) - p.Fx;
    f(1) = Fx_prime * sin(theta) + Fz_prime * cos(theta) - p.Fz;
    f(2) = p.My_prime - p.l * (-Fx_prime * cos(theta) + Fz_prime * sin(theta)) - p.My;

    return f;
}

// 雅可比矩阵
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

// 牛顿-拉弗森方法
VectorXd NewtonRaphson(VectorXd x0, const param& p, double tol, int max_iter) {
    int iter = 0;
    VectorXd x = x0;
    VectorXd delta;

    while (iter < max_iter) {
        VectorXd f_val = F(x, p);
        MatrixXd J_val = J(x, p);
        delta = J_val.fullPivLu().solve(-f_val); //LU分解法

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
    x0 << 0, 0, 0;  // 初始猜测

    double tol = 1e-6;  // 容差
    int max_iter = 100;  // 最大迭代次数

    // 假设的参数值
    param p(10, 20, 30, 10, 10, 10, 10, 5, 2, 40);

    auto start = std::chrono::high_resolution_clock::now();  // 开始时间

    VectorXd solution = NewtonRaphson(x0, p, tol, max_iter);  // 执行牛顿-拉弗森方法

    auto stop = std::chrono::high_resolution_clock::now();  // 结束时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);  // 计算持续时间

    qDebug() << "Solution: \n" << solution[0] << solution[1] << solution[2] << endl;
    qDebug() << "Time taken: " << duration.count() << " milliseconds";

    return 0;
}


//——————————————————————————————对应语雀方法：点云求解相关——————————————————————————————————//

VectorXd polynomialFit(const std::vector<std::pair<double, double>>& points, int order) {
    int n = points.size();

    // 构建 Vandermonde 矩阵
    MatrixXd X(n, order + 1);
    VectorXd y(n);

    for (int i = 0; i < n; ++i) {
        double x = points[i].first;
        y(i) = points[i].second;

        for (int j = 0; j <= order; ++j) {
            X(i, j) = pow(x, j);
        }
    }

    // 使用最小二乘法计算多项式系数
    VectorXd coeffs = X.householderQr().solve(y);
    return coeffs;
}

//——————————————————————移动坐标系（圆心与传感器坐标系一致）：二维：利用二项摩擦理论 + 粒子群算法————————————————————//

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
////适应度函数
//double FitnessFunction(Particle& particle) {
//    // 获取粒子当前位置作为theta
//    double theta = particle.position_[0];
//
//    // 使用curVal中的值
//    double F_x = curVal.Fx;
//    double F_z = curVal.Fz;
//    double R = curVal.R;
//    double l = curVal.l;
//    double M_y = curVal.My;
//    double E_star = curVal.E_equivalence;
//    double beta = curVal.beta;
//    double tau_0 = curVal.tau0;
//
//    // 定义方程左侧的各个部分
//    double W = F_z * cos(theta) - F_x * sin(theta);
//    double common_term = tau_0 + beta * W / (M_PI * pow((3 * R * W) / (4 * E_star), 2.0 / 3.0));
//    double term1 = (2.0 / 3.0) * M_PI * R * R * R * common_term;
//    double term2 = 2 * M_PI * common_term * (R * R * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0 + R * pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) / 2.0 - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0);
//    double term3 = R * F_z * sin(theta) + F_x * (R * cos(theta) + l);
//
//    // 计算方程左侧和右侧的差值
//    double difference = term1 - term2 - term3 - M_y;
//
//    // 返回方程两边差的绝对值作为适应度值
//    return fabs(difference);
//}
//
//
//
//double getThetaByPSO() {
//
//    auto start = std::chrono::high_resolution_clock::now();  // 开始时间
//
//    PSOPara psopara(1, true);
//    psopara.particle_num_ = 30;		// 粒子个数
//    psopara.max_iter_num_ = 700;	// 最大迭代次数
//    psopara.dt_[0] = 0.1;			// 第一维度上的时间步长
//    psopara.wstart_[0] = 0.9;		// 第一维度上的起始权重系数
//    psopara.wend_[0] = 0.4;			// 第一维度上的终止权重系数
//    psopara.C1_[0] = 1.49445;		// 第一维度上的加速度因子
//    psopara.C2_[0] = 1.49445;		
//
//    // 如果有搜索上下限，则设置上下限
//    psopara.lower_bound_[0] = -M_PI / 2;	// 第一维度搜索下限
//    psopara.upper_bound_[0] = M_PI / 2;	// 第一维度搜索上限
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
//        //std::cout << "第" << i << "次迭代结果：";
//        //std::cout << "x = " << psooptimizer.all_best_position_[0] << ", " << "y = " << psooptimizer.all_best_position_[1];
//        //std::cout << ", fitness = " << result[i] << std::endl;
//    }
//
//    curVal.theta = psooptimizer.all_best_position_[0];
//
//    auto stop = std::chrono::high_resolution_clock::now();  // 结束时间
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);  // 计算持续时间
//
//    qDebug() << "Time taken: " << duration.count() << " milliseconds";
//
//    return curVal.theta;
//}




//———————————————————————————————————————matlab engine使用测试—————————————————————————————————//

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
//    // 记录启动 MATLAB 引擎的开始时间
//    auto start = high_resolution_clock::now();
//
//    Engine* matlabEngine;
//
//    // 启动 MATLAB 引擎
//    if (!(matlabEngine = engOpen(nullptr))) {
//        std::cerr << "无法启动 MATLAB 引擎！" << std::endl;
//        return -1;
//    }
//
//    auto end = high_resolution_clock::now();
//    qDebug() << "启动 MATLAB 引擎用时: "
//        << duration_cast<milliseconds>(end - start).count()
//        << " 毫秒" << endl;
//
//    engSetVisible(matlabEngine, true);
//
//    // 定义 .m 文件的完整路径
//    std::string filePath = "C:\\Users\\Dy\\Desktop\\runParticleswarm.m";
//
//    // 构建运行命令
//    std::string command = "run('" + filePath + "')";
//
//    // 记录执行 .m 文件的开始时间
//    start = high_resolution_clock::now();
//
//    // 执行 .m 文件
//    engEvalString(matlabEngine, command.c_str());
//
//    end = high_resolution_clock::now();
//    qDebug() << "执行 .m 文件用时: "
//        << duration_cast<milliseconds>(end - start).count()
//        << " 毫秒" << endl;
//
//    // 关闭 MATLAB 引擎
//    engClose(matlabEngine);
//
//    return 0;
//}



//——————————————————————移动坐标系（圆心与传感器坐标系一致）：二维：利用二项摩擦理论 + 自适应粒子群算法————————————————————//


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
////适应度函数
//double FitnessFunction(Particle& particle) {
//    // 获取粒子当前位置作为theta
//    double theta = particle.position_[0];
//
//    // 使用curVal中的值
//    double F_x = curVal.Fx;
//    double F_z = curVal.Fz;
//    double R = curVal.R;
//    double l = curVal.l;
//    double M_y = curVal.My;
//    double E_star = curVal.E_equivalence;
//    double beta = curVal.beta;
//    double tau_0 = curVal.tau0;
//
//    // 定义方程左侧的各个部分
//    double W = F_z * cos(theta) - F_x * sin(theta);
//    double common_term = tau_0 + beta * W / (M_PI * pow((3 * R * W) / (4 * E_star), 2.0 / 3.0));
//    double term1 = (2.0 / 3.0) * M_PI * R * R * R * common_term;
//    double term2 = 2 * M_PI * common_term * (R * R * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0 + R * pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) / 2.0 - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0) * sqrt(R * R - pow((3 * R * W) / (4 * E_star), 4.0 / 3.0)) / 3.0);
//    double term3 = R * F_z * sin(theta) + F_x * (R * cos(theta) + l);
//
//    // 计算方程左侧和右侧的差值
//    double difference = term1 - term2 - term3 - M_y;
//
//    // 返回方程两边差的绝对值作为适应度值
//    return fabs(difference);
//}
//


//double getThetaByPSO() {
//
//    auto start = std::chrono::high_resolution_clock::now();  // 开始时间
//
//    PSOPara psopara(1, true);
//    psopara.particle_num_ = 30;		// 粒子个数
//    psopara.max_iter_num_ = 700;	// 最大迭代次数
//    psopara.dt_[0] = 0.1;			// 第一维度上的时间步长
//    psopara.wstart_[0] = 0.9;		// 第一维度上的起始权重系数
//    psopara.wend_[0] = 0.4;			// 第一维度上的终止权重系数
//    psopara.C1_[0] = 1.49445;		// 第一维度上的加速度因子
//    psopara.C2_[0] = 1.49445;		
//
//    // 如果有搜索上下限，则设置上下限
//    psopara.lower_bound_[0] = -M_PI / 2;	// 第一维度搜索下限
//    psopara.upper_bound_[0] = M_PI / 2;	// 第一维度搜索上限
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
//        //std::cout << "第" << i << "次迭代结果：";
//        //std::cout << "x = " << psooptimizer.all_best_position_[0] << ", " << "y = " << psooptimizer.all_best_position_[1];
//        //std::cout << ", fitness = " << result[i] << std::endl;
//    }
//
//    curVal.theta = psooptimizer.all_best_position_[0];
//
//    auto stop = std::chrono::high_resolution_clock::now();  // 结束时间
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);  // 计算持续时间
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
//    // 定义已知量
//    double R = 0.01;
//    double F_z = 5.55744;
//    double F_x = -1.97;
//    double E_star = 16217.43598;
//    double tau_0 = 1.34e5;
//    double beta = -0.2028;
//    double l = 0.055;
//    double M_y = 0.2;
//
//    // 定义 theta 作为优化变量
//    SX theta = SX::sym("theta");
//
//    // 定义方程
//    SX equation = fabs((2.0 / 3.0) * M_PI * pow(R, 3) * (tau_0 + beta * (F_z * cos(theta) - F_x * sin(theta)) / (M_PI * pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 2.0 / 3.0)))
//        - 2 * M_PI * (tau_0 + beta * (F_z * cos(theta) - F_x * sin(theta)) / (M_PI * pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 2.0 / 3.0)))
//        * (pow(R, 2) * sqrt(pow(R, 2) - pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0)) / 3
//            + R * pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0) / 2
//            - pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0) * sqrt(pow(R, 2) - pow(3 * R * (F_z * cos(theta) - F_x * sin(theta)) / (4 * E_star), 4.0 / 3.0)) / 3)
//        - R * F_z * sin(theta) - F_x * (R * cos(theta) + l) - M_y);
//
//    SX objective = pow(equation, 2);
//
//    // 创建非线性求解问题
//    SXDict nlp = { {"x", theta}, {"f", objective} };
//    Function solver = nlpsol("solver", "ipopt", nlp);
//
//    // 设置初始解和界限
//    std::map<std::string, DM> args;
//    args["x0"] = 0;  // 初始猜测
//    args["lbx"] = -M_PI / 2; // theta 的下界
//    args["ubx"] = M_PI / 2; // theta 的上界
//    args["max_iter"] = 10000;  // 增加最大迭代次数，例如设置为 10000
//    args["tol"] = 1e-4;       // 调整容忍度，例如设置为 1e-4
//
//    solver.setOption("ipopt", opts);
//
//    // 求解
//    std::map<std::string, DM> sol = solver(args);
//
//    // 输出结果
//    qDebug() << "Solution: theta = " << double(sol["x"]) * (180.0 / M_PI) << endl;
//
//    system("pause");
//
//    return 0;
//}
