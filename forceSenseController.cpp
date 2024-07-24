#include "forceSenseController.h"

forceSenseController::forceSenseController(bool* openFlagAddress, RTDEControlInterface* rtde_c, 
											RTDEReceiveInterface* rtde_r, QWidget* parent)
	: openFlagAddress(openFlagAddress),rtde_c(rtde_c),rtde_r(rtde_r),QMainWindow(parent)
{
	ui.setupUi(this);

    setAttribute(Qt::WA_DeleteOnClose); //子窗体关掉时候可以调用析构函数

    get_Compensation_data_flag = false;
    QTimerForGetCompensation = nullptr;

    forceName[0] = "Fx";
    forceName[1] = "Fy";
    forceName[2] = "Fz";
    forceName[3] = "Mx";
    forceName[4] = "My";
    forceName[5] = "Mz";

	// 创建QSplineSeries和QChart
    for (int i = 0; i < 3; i++) {
        series[i] = new QSplineSeries();
        chart[i] = new QChart();
        chart[i]->legend()->hide();
        chart[i]->setTitle(forceName[i]);
        chart[i]->addSeries(series[i]);
        chart[i]->createDefaultAxes();
        chart[i]->axes(Qt::Vertical).first()->setRange(-10, 10); 
        chart[i]->axes(Qt::Horizontal).first()->setRange(0, 100); // 横坐标
    }

    for (int i = 3; i < 6; i++) {
        series[i] = new QSplineSeries();
        chart[i] = new QChart();
        chart[i]->legend()->hide();
        chart[i]->setTitle(forceName[i]);
        chart[i]->addSeries(series[i]);
        chart[i]->createDefaultAxes();
        chart[i]->axes(Qt::Vertical).first()->setRange(-0.2, 0.2);
        chart[i]->axes(Qt::Horizontal).first()->setRange(0, 100); // 横坐标
    }
    
    ui.Fx_show->setChart(chart[0]);
    ui.Fy_show->setChart(chart[1]);
    ui.Fz_show->setChart(chart[2]);
    ui.Mx_show->setChart(chart[3]);
    ui.My_show->setChart(chart[4]);
    ui.Mz_show->setChart(chart[5]);

	//更新数据定时器
	updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &forceSenseController::updateChartData, Qt::DirectConnection);
}

void forceSenseController::on_readForceAndShowButton_clicked() {
    // 启动或停止定时器
    if (updateTimer->isActive()) {
        updateTimer->stop();
        ui.readForceAndShowButton->setText(QStringLiteral("读取并显示六维力"));
    }
    else {
        updateTimer->start(100); // 更新频率，100毫秒
        ui.readForceAndShowButton->setText(QStringLiteral("停止读取"));
    }
}

void forceSenseController::updateChartData() {

    if (!rtde_r) return;

    auto force = getSensorForce(rtde_r,true);

    //auto joint_angles = rtde_r->getActualQ();

    //force[0] = RAD_TO_DEG(joint_angles[3]);
    //force[1] = RAD_TO_DEG(joint_angles[4]);
    //force[2] = RAD_TO_DEG(joint_angles[5]);

    auto update = [&](int i) {

        // 获取新的数据点
        double newValue = force[i]; // 根据实际情况获取新数据

        // 如果系列中的数据点已经达到80个，移除最左边的点（横坐标最小的点）
        if (series[i]->count() >= 80) {
            series[i]->remove(0);
        }

        // 更新现有点的横坐标，使其向左移动1个单位
        for (int j = 0; j < series[i]->count(); ++j) {
            QPointF point = series[i]->at(j);
            series[i]->replace(j, point.x() - 1, point.y());
        }

        // 在横坐标80处添加新的数据点
        series[i]->append(80, newValue);
    };

    for (int i = 0; i < 6; i++)
        update(i);
}

void forceSenseController::on_zeroSenseButton_clicked() {
    if (!rtde_c) return;
    rtde_c->zeroFtSensor();
}


void forceSenseController::on_get_once_Button_clicked() {

    auto pose = getSensorPose(rtde_r);

    //取个均值
    auto force = std::vector<double>(6,0.0);

    for (int i = 0; i < 200; i++) {
        auto tmp = getSensorForce(rtde_r);
        force[0] += tmp[0];
        force[1] += tmp[1];
        force[2] += tmp[2];
        force[3] += tmp[3];
        force[4] += tmp[4];
        force[5] += tmp[5];
    }

    for (int i = 0; i < 6; i++)
        force[i] /= 200;

    double G_robot = ui.G_robot_lineEdit->text().toDouble();

    auto dx_robot = ui.dx_robot_lineEdit->text().toDouble();

    auto dy_robot = ui.dy_robot_lineEdit->text().toDouble();

    auto dz_robot = ui.dz_robot_lineEdit->text().toDouble();

    cv::Vec3d rot_vec{ pose[3],pose[4],pose[5] };
    cv::Mat R_base2sensor;

    cv::Rodrigues(rot_vec, R_base2sensor);

    cv::Mat R_sensor2base = R_base2sensor.t();

    cv::Vec3d G_robot_vec_base{ 0,0,-G_robot }; //机器人补偿重力在基座标系下的表示

    cv::Vec3d G_robot_vec_sensor = (cv::Mat)(R_sensor2base * G_robot_vec_base);

    //加上对应的G'x1 G'y1 G'z1 Mg'x1 Mg'y1 Mg'z1
    force.push_back(G_robot_vec_sensor[0]);
    force.push_back(G_robot_vec_sensor[1]);
    force.push_back(G_robot_vec_sensor[2]);
    force.push_back(-G_robot_vec_sensor[2] * dy_robot + G_robot_vec_sensor[1] * dz_robot);
    force.push_back(-G_robot_vec_sensor[0] * dz_robot + G_robot_vec_sensor[2] * dx_robot);
    force.push_back(-G_robot_vec_sensor[1] * dx_robot + G_robot_vec_sensor[0] * dy_robot);

    qDebug() << " num :" << data.size() << endl;
    for (int i = 0; i < force.size(); i++)
        qDebug() << force[i] << endl;

    data.push_back(force);
    Rots_sensor2base.push_back(R_sensor2base);

    ui.curNum_lineEdit->setText(QString::number(data.size()));

}


void forceSenseController::on_caculate_Button_clicked() {



    //--------------------第一步最小二乘求 重心位置 ---------------------

    int rows = data.size() * 3; // 每个姿态对应3行
    int cols = 6; // d_x, d_y, d_z, n_1, n_2, n_3 和

    cv::Mat A, b;

    A = cv::Mat(rows, cols, CV_64F, cv::Scalar(0));
    b = cv::Mat(rows, 1, CV_64F, cv::Scalar(0));

    for (size_t i = 0; i < data.size(); ++i) {
        const auto& datum = data[i];
        // 确保每个数据向量包含12个元素
        if (datum.size() != 12) {
            std::cerr << "Error: Each data entry must contain 12 elements." << endl;
            return;
        }

        int baseRow = i * 3;

        // 填充A矩阵
        A.at<double>(baseRow, 1) = datum[2] + datum[8];
        A.at<double>(baseRow, 2) = -(datum[1] + datum[7]);
        A.at<double>(baseRow, 3) = 1;

        A.at<double>(baseRow + 1, 0) = -(datum[2] + datum[8]);
        A.at<double>(baseRow + 1, 2) = datum[0] + datum[6];
        A.at<double>(baseRow + 1, 4) = 1;

        A.at<double>(baseRow + 2, 0) = datum[1] + datum[7];
        A.at<double>(baseRow + 2, 1) = -(datum[0] + datum[6]);
        A.at<double>(baseRow + 2, 5) = 1;

        // 填充b向量
        b.at<double>(baseRow) = datum[3] - datum[9];
        b.at<double>(baseRow + 1) = datum[4] - datum[10];
        b.at<double>(baseRow + 2) = datum[5] - datum[11];
    }

    // 使用最小二乘法求解x
    cv::Mat x;
    cv::solve(A, b, x, cv::DECOMP_SVD);

    ui.dx_real_lineEdit->setText(QString::number(x.at<double>(0,0)));
    ui.dy_real_lineEdit->setText(QString::number(x.at<double>(1,0)));
    ui.dz_real_lineEdit->setText(QString::number(x.at<double>(2,0)));

    //-----------------------------------------------------------------



    //----------------------第二步最小二乘求 重力----------------------------


    rows = Rots_sensor2base.size() * 3; // 每个姿态对应3行
    cols = 6; // 对应 0 0 -G + G’ Fx0 Fy0 Fz0 

    A = cv::Mat(rows, cols, CV_64F, cv::Scalar(0));
    b = cv::Mat(rows, 1, CV_64F, cv::Scalar(0));

    for (size_t i = 0; i < data.size(); ++i) {
        const auto& datum = data[i];
        const auto& Rot_cur = Rots_sensor2base[i];

        // 确保每个数据向量包含12个元素
        if (datum.size() != 12) {
            std::cerr << "Error: Each data entry must contain 12 elements." << endl;
            return;
        }

        int baseRow = i * 3;

        // 填充A矩阵
        A.at<double>(baseRow, 0) = Rot_cur.at<double>(0, 0);
        A.at<double>(baseRow, 1) = Rot_cur.at<double>(0, 1);
        A.at<double>(baseRow, 2) = Rot_cur.at<double>(0, 2);
        A.at<double>(baseRow, 3) = 1.0;

        A.at<double>(baseRow + 1, 0) = Rot_cur.at<double>(1, 0);
        A.at<double>(baseRow + 1, 1) = Rot_cur.at<double>(1, 1);
        A.at<double>(baseRow + 1, 2) = Rot_cur.at<double>(1, 2);
        A.at<double>(baseRow + 1, 4) = 1.0;

        A.at<double>(baseRow + 2, 0) = Rot_cur.at<double>(2, 0);
        A.at<double>(baseRow + 2, 1) = Rot_cur.at<double>(2, 1);
        A.at<double>(baseRow + 2, 2) = Rot_cur.at<double>(2, 2);
        A.at<double>(baseRow + 2, 5) = 1.0;

        // 填充b向量
        b.at<double>(baseRow) = datum[0];
        b.at<double>(baseRow + 1) = datum[1];
        b.at<double>(baseRow + 2) = datum[2];
    }


    // 使用最小二乘法求解x
    x = cv::Mat();
    cv::solve(A, b, x, cv::DECOMP_SVD);

    ui.G_real_lineEdit->setText(QString::number(-x.at<double>(0, 2) + 
        ui.G_robot_lineEdit->text().toDouble()));

    //-----------------------------------------------------------------

    //on_clear_Button_clicked();

}


void forceSenseController::on_clear_Button_clicked() {

    data.clear();
    Rots_sensor2base.clear();

    ui.curNum_lineEdit->setText(QString::number(0));

    return;
}


void forceSenseController::on_get_Compensation_Button_clicked() {

    if (get_Compensation_data_flag) {
        QTimerForGetCompensation->stop();
        delete QTimerForGetCompensation;
        QTimerForGetCompensation = nullptr;
        ui.get_Compensation_Button->setText(QStringLiteral("获取力/力距与转角"));
        get_Compensation_data_flag = false;
        return;
    }
    else {
        QTimerForGetCompensation = new QTimer();
        get_Compensation_data_flag = true;
        QTimer::singleShot(0, this, SLOT(update_Compensation_data()));
        connect(QTimerForGetCompensation, &QTimer::timeout, this, &forceSenseController::update_Compensation_data, Qt::DirectConnection);
        QTimerForGetCompensation->start(10);
        ui.get_Compensation_Button->setText(QStringLiteral("停止获取"));
    }

}

void forceSenseController::update_Compensation_data() {

    std::vector<double> Axis_deg = rtde_r->getTargetQ();

    auto force = getSensorForce(rtde_r);

    Compensation_data.push_back({ force[0],force[1],force[2],force[3],force[4],force[5],Axis_deg[5] });

    ui.curNum_Compensation_lineEdit->setText(QString::number(Compensation_data.size()));
}

void forceSenseController::on_caculate_Compensation_Button_clicked() {

    // 这里填写通过傅里叶变换得出来的频率 挑幅值较大的
    std::vector<double> Fx_frequencies{0.0,0.13387};
    std::vector<double> Fy_frequencies{0.0,0.13387};
    std::vector<double> Fz_frequencies{0.0,0.32139};
    std::vector<double> Mx_frequencies{0.0,0.16064};
    std::vector<double> My_frequencies{0.0,0.16064};

    //std::vector<double> Fx_frequencies{0.0,0.15924};
    //std::vector<double> Fy_frequencies{0.0,0.15924};
    //std::vector<double> Fz_frequencies{0.0,0.15924};
    //std::vector<double> Mx_frequencies{0.0,0.15924};
    //std::vector<double> My_frequencies{0.0,0.15924};


    //第六轴转角
    std::vector<double> anglesInRadians;

    //各个力的观测值
    std::vector<double> Fx_observedValues;
    std::vector<double> Fy_observedValues;
    std::vector<double> Fz_observedValues;
    std::vector<double> Mx_observedValues;
    std::vector<double> My_observedValues;

    for (int i = 0; i < Compensation_data.size(); i++) {
        Fx_observedValues.push_back(Compensation_data[i][0]);
        Fy_observedValues.push_back(Compensation_data[i][1]);
        Fz_observedValues.push_back(Compensation_data[i][2]);
        Mx_observedValues.push_back(Compensation_data[i][3]);
        My_observedValues.push_back(Compensation_data[i][4]);
        anglesInRadians.push_back(Compensation_data[i][6]);
    }

    std::pair<std::vector<double>, std::vector<double>> P_Fx_ak_bk = fitCosineSineComponents(Fx_frequencies,
        anglesInRadians,
        Fx_observedValues);

    std::pair<std::vector<double>, std::vector<double>> P_Fy_ak_bk = fitCosineSineComponents(Fy_frequencies,
        anglesInRadians,
        Fy_observedValues);

    std::pair<std::vector<double>, std::vector<double>> P_Fz_ak_bk = fitCosineSineComponents(Fz_frequencies,
        anglesInRadians,
        Fz_observedValues);

    std::pair<std::vector<double>, std::vector<double>> P_Mx_ak_bk = fitCosineSineComponents(Mx_frequencies,
        anglesInRadians,
        Mx_observedValues);

    std::pair<std::vector<double>, std::vector<double>> P_My_ak_bk = fitCosineSineComponents(My_frequencies,
        anglesInRadians,
        My_observedValues);


    change_Compensation(P_Fx_ak_bk, P_Fy_ak_bk, P_Fz_ak_bk, P_Mx_ak_bk, P_My_ak_bk,
        Fx_frequencies, Fy_frequencies, Fz_frequencies, Mx_frequencies, My_frequencies, true);


    //以下部分对应语雀笔记  -----以前的猜想
    //int n = Compensation_data.size();
    //cv::Mat A(3 * n, 6, CV_64F);
    //cv::Mat b(3 * n, 1, CV_64F);

    //for (int i = 0; i < n; ++i) {
    //    double rad = Compensation_data[i][6];
    //    double cosTheta = cos(rad);
    //    double sinTheta = sin(rad);

    //    A.at<double>(3 * i, 0) = cosTheta;
    //    A.at<double>(3 * i, 1) = sinTheta;
    //    A.at<double>(3 * i, 2) = 0;
    //    A.at<double>(3 * i, 3) = 1;
    //    A.at<double>(3 * i, 4) = 0;
    //    A.at<double>(3 * i, 5) = 0;

    //    A.at<double>(3 * i + 1, 0) = -sinTheta;
    //    A.at<double>(3 * i + 1, 1) = cosTheta;
    //    A.at<double>(3 * i + 1, 2) = 0;
    //    A.at<double>(3 * i + 1, 3) = 0;
    //    A.at<double>(3 * i + 1, 4) = 1;
    //    A.at<double>(3 * i + 1, 5) = 0;

    //    A.at<double>(3 * i + 2, 0) = 0;
    //    A.at<double>(3 * i + 2, 1) = 0;
    //    A.at<double>(3 * i + 2, 2) = 1;
    //    A.at<double>(3 * i + 2, 3) = 0;
    //    A.at<double>(3 * i + 2, 4) = 0;
    //    A.at<double>(3 * i + 2, 5) = 1;

    //    b.at<double>(3 * i, 0) = Compensation_data[i][3];
    //    b.at<double>(3 * i + 1, 0) = Compensation_data[i][4];
    //    b.at<double>(3 * i + 2, 0) = Compensation_data[i][5];
    //}

    //// 使用最小二乘法求解x
    //cv::Mat x;
    //cv::solve(A, b, x, cv::DECOMP_SVD);

    //double M_e_x5 = x.at<double>(0, 0);
    //double M_e_y5 = x.at<double>(1, 0);
    //double M_e_z5 = x.at<double>(2, 0);
    //double M_x0 = x.at<double>(3, 0);
    //double M_y0 = x.at<double>(4, 0);
    //double M_z0 = x.at<double>(5, 0);

    //ui.M_e_x5_lineEdit->setText(QString::number(M_e_x5));
    //ui.M_e_y5_lineEdit->setText(QString::number(M_e_y5));
    //ui.M_e_z5_lineEdit->setText(QString::number(M_e_z5));
    //ui.M_x0_lineEdit->setText(QString::number(M_x0));
    //ui.M_y0_lineEdit->setText(QString::number(M_y0));
    //ui.M_z0_lineEdit->setText(QString::number(M_z0));

    //change_Compensation_torque(M_e_x5, M_e_y5, M_e_z5, M_x0, M_y0, M_z0,true);

    //on_clear_Compensation_Button_clicked();

}


void forceSenseController::on_clear_Compensation_Button_clicked() {

    Compensation_data.clear();

    ui.curNum_Compensation_lineEdit->setText(QString::number(0));

}


void forceSenseController::on_cancel_Compensation_Button_clicked() {


    change_Compensation({}, {}, {}, {}, {},
        {}, {}, {}, {}, {}, false);
}




forceSenseController::~forceSenseController()
{
	*openFlagAddress = false;
}
