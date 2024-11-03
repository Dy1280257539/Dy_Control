#include "Dy_Control.h"

Dy_Control::Dy_Control(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    init();


}

//初始化函数
void Dy_Control::init()
{
    rtde_c = NULL;
    rtde_r = NULL;
    read_endoscope_Flag = false;
    all_connectFlag = false;
    r_connectFlag = false;
    c_connectFlag = false;
    read_message_Flag = false;
    forcemode_Flag = false;
    movespeedThread_Flag = false;
    movespeedJThread_Flag = false;
    move_rotate_Flag = false;
    record_trajThread_Flag = false;
    speedL_runoutFlag = true;
    speedJ_runoutFlag = true;
    servoJ_record_traj_Flag = false;
    io_connectFlag = false;
    show_now_flag = false;
    fmode_flag = false;
    icinit_flag = false;
    forcemode_Flag1 = false;
    PIDmode_Flag = false;
    open_endoscope_Flag = false;
    open_KinectController_Flag = false;
    open_RealSenseController_Flag = false;
    open_forceSenseController_Flag = false;
    ic = nullptr;
    pc = nullptr;
    init_pose.clear();
    endoscope_show_thread = nullptr;
    kinectController = nullptr;
    realSenseController = nullptr;

    //this->setMinimumSize(QSize(941, 561));//固定窗口大小  窗口大小的宽比滚动条那东西长个20
    //this->setMaximumSize(QSize(941, 561));

    Euler_Ys = vector<list<double>>(6, list<double>());//卡尔曼滤波数值初始化
    Euler_Ys_Motion = vector<list<double>>(6, list<double>());
}

void Dy_Control::on_connect_robot_clicked()
{
    qDebug() << "on_connect_robot_clicked" << endl;
    string IP = ui.IP_lineEdit->text().toStdString();
    if (all_connectFlag == false) {
        try {
            rtde_c = new RTDEControlInterface{ IP.data() };
            rtde_r = new RTDEReceiveInterface{ IP.data() };
        }
        catch (...) {
            QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("连接机器人失败请检查网线以及端口"), QMessageBox::Yes , QMessageBox::Yes);
            return;
        }
        QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("连接机器人成功"), QMessageBox::Yes , QMessageBox::Yes);
        all_connectFlag = true;
        r_connectFlag = true;
        c_connectFlag = true;
        initTCPOffSet(rtde_c); //初始化TCPOffset
        ui.connect_robot->setText(QStringLiteral("断开连接"));
        qDebug() << QStringLiteral("连接机器人成功") << endl;
        return;
    }
    else {
        if (read_message_Flag) {
            on_read_message_clicked();
        }
        rtde_c->disconnect();
        rtde_r->disconnect();
        delete rtde_c;
        delete rtde_r;
        rtde_c = NULL;
        rtde_r = NULL;
        all_connectFlag = false;
        r_connectFlag = false;
        c_connectFlag = false;
        ui.connect_robot->setText(QStringLiteral("连接机器人"));
        QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("断开机器人成功"), QMessageBox::Yes , QMessageBox::Yes);
        qDebug() << QStringLiteral("断开机器人") << endl;
    }
}
void Dy_Control::on_read_message_clicked()
{
    qDebug() << "on_read_message_clicked" << endl;
    if ((all_connectFlag && !read_message_Flag) || (r_connectFlag && !read_message_Flag)) {
        //QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("开始显示信息"), QMessageBox::Yes, QMessageBox::Yes);
        read_message_Flag = true;
        ui.read_message->setText(QStringLiteral("停止读信息"));
        Euler_Ys = vector<list<double>>(6, list<double>());//卡尔曼滤波数值初始化
        timedata.clear();
        save_excel_worksheet.clear();//清空之前的数据
        //开启定时器，定时读取更新数据
        read_message_timer = new QTimer(this);
        connect(read_message_timer, SIGNAL(timeout()), this, SLOT(update_force_pose_Data()));//定时更新数据
        read_message_timer->start(10);//  若要设置ms数间隔，可以再start括号内设置 ，如start(50) ,即50ms读取一次信息
        dwStart = GetTickCount64();
    }
    else if (!all_connectFlag || !r_connectFlag) {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
    else if (read_message_Flag) {
        //QMessageBox::warning(NULL, QStringLiteral("停止读取数据提示"), QStringLiteral("停止读取数据成功"), QMessageBox::Yes, QMessageBox::Yes);
        read_message_Flag = false;
        dwStart = 0;
        read_message_timer->stop();
        delete read_message_timer;
        read_message_timer = NULL;
        ui.read_message->setText(QStringLiteral("开始读信息"));
    }
}

void Dy_Control::update_force_pose_Data()
{
    if ( (r_connectFlag && read_message_Flag)|| (r_connectFlag && show_now_flag)){
        DWORD dwCurrent = GetTickCount64();

        //获取开始到现在的时间差
        //DWORD t = dwCurrent - dwStart;//

        struct timeb t1;
        ftime(&t1);
        long long t =  t1.time * 1000 + t1.millitm;

        vector<double> pose = rtde_r->getActualTCPPose();
        vector<double> force = rtde_r->getActualTCPForce();
        vector<double> Axis_deg = rtde_r->getTargetQ();

        pose[0] *= 1000; //初始单位为mm
        pose[1] *= 1000;
        pose[2] *= 1000;
        ui.world_force_1->setText(QString::asprintf("%.4f", force[0]));
        ui.world_force_2->setText(QString::asprintf("%.4f", force[1]));
        ui.world_force_3->setText(QString::asprintf("%.4f", force[2]));
        ui.world_force_4->setText(QString::asprintf("%.4f", force[3]));
        ui.world_force_5->setText(QString::asprintf("%.4f", force[4]));
        ui.world_force_6->setText(QString::asprintf("%.4f", force[5]));

        vector<double> tcp_force = getTCPForce(rtde_r);

        ui.force_1->setText(QString::asprintf("%.4f", tcp_force[0]));
        ui.force_2->setText(QString::asprintf("%.4f", tcp_force[1]));
        ui.force_3->setText(QString::asprintf("%.4f", tcp_force[2]));
        ui.force_4->setText(QString::asprintf("%.4f", tcp_force[3]));
        ui.force_5->setText(QString::asprintf("%.4f", tcp_force[4]));
        ui.force_6->setText(QString::asprintf("%.4f", tcp_force[5]));
        ui.pose_1->setText(QString::asprintf("%.4f", pose[0]));
        ui.pose_2->setText(QString::asprintf("%.4f", pose[1]));
        ui.pose_3->setText(QString::asprintf("%.4f", pose[2]));
        ui.pose_4->setText(QString::asprintf("%.4f", pose[3]));
        ui.pose_5->setText(QString::asprintf("%.4f", pose[4]));
        ui.pose_6->setText(QString::asprintf("%.4f", pose[5]));

        //----------------------------------------------移动坐标系下的力(用于调试)---------------------------------------------
        
        //vector<double> cur_pose = rtde_r->getActualTCPPose();

        //auto sixth_axis = getSixthAxisFromBasePose(cur_pose);

        //Vec3d end_effector_direction{ 1,0,0 };

        //auto sensorForce = getMotionCoordinateSystemForce(sixth_axis, end_effector_direction, { 0,0,1 }, rtde_r);

        //----------------------------------------------移动坐标系下的力---------------------------------------------

        vector<double> sensorForce = getSensorForce(rtde_r,true,true);

        ui.sensorForce_1->setText(QString::asprintf("%.4f", sensorForce[0]));
        ui.sensorForce_2->setText(QString::asprintf("%.4f", sensorForce[1]));
        ui.sensorForce_3->setText(QString::asprintf("%.4f", sensorForce[2]));
        ui.sensorForce_4->setText(QString::asprintf("%.4f", sensorForce[3]));
        ui.sensorForce_5->setText(QString::asprintf("%.4f", sensorForce[4]));
        ui.sensorForce_6->setText(QString::asprintf("%.4f", sensorForce[5]));

        ui.Axis_deg_1->setText(QString::asprintf("%.4f", Axis_deg[0] / M_PI * 180));
        ui.Axis_deg_2->setText(QString::asprintf("%.4f", Axis_deg[1] / M_PI * 180));
        ui.Axis_deg_3->setText(QString::asprintf("%.4f", Axis_deg[2] / M_PI * 180));
        ui.Axis_deg_4->setText(QString::asprintf("%.4f", Axis_deg[3] / M_PI * 180));
        ui.Axis_deg_5->setText(QString::asprintf("%.4f", Axis_deg[4] / M_PI * 180));
        ui.Axis_deg_6->setText(QString::asprintf("%.4f", Axis_deg[5] / M_PI * 180));

        if (!show_now_flag) {
            vector<double> v;
            timedata.push_back(t);
            for (auto _pose : pose) {
                v.push_back(_pose);
            }
            for (auto _force : force) {
                v.push_back(_force);
            }
            for (auto _tcp_force : tcp_force) {
                v.push_back(_tcp_force);
            }
            for (auto _sensorForce : sensorForce) {
                v.push_back(_sensorForce);
            }
            for (auto _Axis_deg : Axis_deg) {
                v.push_back(_Axis_deg / M_PI * 180);
            }
            save_excel_worksheet.push_back(v); //数据加入到二维数组中去
        }
        

     }
}
void Dy_Control::on_Button_save_clicked()
{
    if ((all_connectFlag && read_message_Flag) || (r_connectFlag && read_message_Flag)) {
        on_read_message_clicked();
    }
    if (save_excel_worksheet.size()) {
        
        time_t curtime;
        time(&curtime);
        //cout << "1970到目前经过秒数:" << time(&curtime) << endl;
        //cout << "本地日期和时间:" << ctime(&curtime) << endl;
        //tm* nowtime = localtime(&curtime);
        //// 输出tm结构的年月日
        //cout << "年: " << 1900 + nowtime->tm_year << endl;
        //cout << "月: " << 1 + nowtime->tm_mon << endl;
        //cout << "日: " << nowtime->tm_mday << endl;
        //cout << "时间: " << nowtime->tm_hour << ":";
        //cout << nowtime->tm_min << ":";
        //cout << nowtime->tm_sec << endl;
        QString filename = ui.filename_lineEdit->text();
        if (filename=="default")
        {
            ofstream ofs;
            string str_now_time = ctime(&curtime);
            str_now_time = subreplace(str_now_time, ":", "_");
            str_now_time = subreplace(str_now_time, " ", "_");
            str_now_time = subreplace(str_now_time, "__", "_");
            str_now_time = subreplace(str_now_time, "\n", "");
            str_now_time.erase(0, 4);
            stringstream path;
            string location = ui.Save_location_lineEdit->text().toStdString();
            path << location.c_str() << "/" << str_now_time << ".csv";
            qDebug() << QStringLiteral("保存文件到") << QString::fromStdString(path.str()) << endl;
            ofs.open(path.str(), ios::out | ios::trunc);

            ofs << "x/mm" << ",";
            ofs << "y/mm" << ",";
            ofs << "z/mm" << ",";
            ofs << "rx" << ",";
            ofs << "ry" << ",";
            ofs << "rz" << ",";
            ofs << "Fx_w/N" << ",";
            ofs << "Fy_w/N" << ",";
            ofs << "Fz_w/N" << ",";
            ofs << "Tx_w" << ",";
            ofs << "Ty_w" << ",";
            ofs << "Tz_w" << ",";
            ofs << "Fx_tcp/N" << ",";
            ofs << "Fy_tcp/N" << ",";
            ofs << "Fz_tcp/N" << ",";
            ofs << "Tx_tcp" << ",";
            ofs << "Ty_tcp" << ",";
            ofs << "Tz_tcp" << ",";
            ofs << "sensor_Fx/N" << ",";
            ofs << "sensor_Fy/N" << ",";
            ofs << "sensor_Fz/N" << ",";
            ofs << "Tx_sensor" << ",";
            ofs << "Ty_sensor" << ",";
            ofs << "Tz_sensor" << ",";
            ofs << "Axis1 /deg" << ",";
            ofs << "Axis2 /deg" << ",";
            ofs << "Axis3 /deg" << ",";
            ofs << "Axis4 /deg" << ",";
            ofs << "Axis5 /deg" << ",";
            ofs << "Axis6 /deg" << ",";
            ofs << "time/ms" << ",";
            ofs << std::endl;

            int cur = 0;
            int size = save_excel_worksheet.size();
            for (auto i : save_excel_worksheet) {
                for (auto j : i) {
                    ofs << j << ",";
                }
                ofs << timedata[cur++] << ",";
                if (size-- != 1)//最后一行不输出换行符
                    ofs << std::endl;
            }
            ofs.close();
            save_excel_worksheet.clear();
            timedata.clear();
            QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("保存数据成功"), QMessageBox::Yes, QMessageBox::Yes);
        }
        else //为了防止输入路径中存在中文路径，需要使用QFile来读取路径，如果采用传统的ofstream，open中丢入QString，会乱码，因为编码方式不一样，无法正确打开文件
        {   
            QString location = ui.Save_location_lineEdit->text();
            QString path = location + "/" + filename + ".csv";
            QFile file(path);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream ofs(&file);
            ofs.setCodec("UTF-8");
            ofs << "x/mm" << ",";
            ofs << "y/mm" << ",";
            ofs << "z/mm" << ",";
            ofs << "rx" << ",";
            ofs << "ry" << ",";
            ofs << "rz" << ",";
            ofs << "Fx_w/N" << ",";
            ofs << "Fy_w/N" << ",";
            ofs << "Fz_w/N" << ",";
            ofs << "Tx_w" << ",";
            ofs << "Ty_w" << ",";
            ofs << "Tz_w" << ",";
            ofs << "Fx_tcp/N" << ",";
            ofs << "Fy_tcp/N" << ",";
            ofs << "Fz_tcp/N" << ",";
            ofs << "Tx_tcp" << ",";
            ofs << "Ty_tcp" << ",";
            ofs << "Tz_tcp" << ",";
            ofs << "sensor_Fx/N" << ",";
            ofs << "sensor_Fy/N" << ",";
            ofs << "sensor_Fz/N" << ",";
            ofs << "Tx_sensor" << ",";
            ofs << "Ty_sensor" << ",";
            ofs << "Tz_sensor" << ",";
            ofs << "Axis1 /deg" << ",";
            ofs << "Axis2 /deg" << ",";
            ofs << "Axis3 /deg" << ",";
            ofs << "Axis4 /deg" << ",";
            ofs << "Axis5 /deg" << ",";
            ofs << "Axis6 /deg" << ",";
            ofs << "time/ms" << ",";
            ofs << endl;

            int cur = 0;
            int size = save_excel_worksheet.size();
            for (auto i : save_excel_worksheet) {
                for (auto j : i) {
                    ofs << j << ",";
                }
                ofs << timedata[cur++] << ",";
                if (size-- != 1)//最后一行不输出换行符
                    ofs << endl;
            }

            file.close();
            save_excel_worksheet.clear();
            timedata.clear();
            QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("保存数据成功"), QMessageBox::Yes, QMessageBox::Yes);
        }
        
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("请先读取数据再保存"), QMessageBox::Yes, QMessageBox::Yes);
    }
}
void Dy_Control::on_button_zeroFSensor_clicked()
{
    if (c_connectFlag) {
        rtde_c->zeroFtSensor();
        QMessageBox::information(NULL, QStringLiteral("传感器清零"), QStringLiteral("传感器清零成功"), QMessageBox::Yes, QMessageBox::Yes);
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}
void Dy_Control::on_Button_fmode_clicked()
{
    if (all_connectFlag && !forcemode_Flag) {
        QMessageBox::warning(NULL, QStringLiteral("开启力模式提示"), QStringLiteral("即将开启力模式，请做好安全准备"), QMessageBox::Yes, QMessageBox::Yes);
        //on_button_zeroFSensor_clicked();//先传感器清理

        double force = ui.forcemode_num->text().toDouble();
        vector<double> pose = rtde_r->getActualTCPPose();
        vector<int> selection_vector = { 0, 0, 1, 0, 0, 0 };
        vector<double> wrench = { 0, 0, force, 0, 0, 0 };
        int type = 2;
        vector<double> limits = { 100, 100, 100, 5, 5, 5 };
        rtde_c->forceMode(pose, selection_vector, wrench, type, limits);
        ui.Button_fmode->setText(QStringLiteral("停止恒力模式"));
        forcemode_Flag = true;
    }
    else if (all_connectFlag && forcemode_Flag) {
        rtde_c->forceModeStop();
        forcemode_Flag = false;
        ui.Button_fmode->setText(QStringLiteral("开启恒力模式"));
    }
    else
    {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}
void Dy_Control::on_move_byhand_Button_clicked()
{
    if (all_connectFlag && !forcemode_Flag1) {
        rtde_c->freedriveMode();
        ui.move_byhand_Button->setText(QStringLiteral("停止手动模式"));
        forcemode_Flag1 = true;
    }
    else if (all_connectFlag && forcemode_Flag1) {
        rtde_c->endFreedriveMode();
        forcemode_Flag1 = false;
        ui.move_byhand_Button->setText(QStringLiteral("手动模式"));
    }
    else
    {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

//---------------------------------------------移动功能函数---------------------------------------------------
//move xyz
void Dy_Control::on_move_x_Button_clicked()
{
    if (c_connectFlag) {
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::move_x_Button_function);//创建了一个线程，feature是接收对应的异步计算的结果
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
}
void Dy_Control::move_x_Button_function() {
    move_mutex.lock();
    vector<double> pose = rtde_r->getActualTCPPose();
    pose[0] += 0.001 * int(ui.movex_lineEdit->text().toInt());
    rtde_c->moveL(pose, ui.movex_v_lineEdit->text().toDouble(), ui.movex_a_lineEdit->text().toDouble(), false);
    move_mutex.unlock();
}
void Dy_Control::on_move_y_Button_clicked()
{
    if (c_connectFlag) {
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::move_y_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
}
void Dy_Control::move_y_Button_function() {
    move_mutex.lock();
    vector<double> pose = rtde_r->getActualTCPPose();
    pose[1] += 0.001 * int(ui.movey_lineEdit->text().toInt());
    rtde_c->moveL(pose, ui.movey_v_lineEdit->text().toDouble(), ui.movey_a_lineEdit->text().toDouble(), false);
    move_mutex.unlock();
}
void Dy_Control::on_move_z_Button_clicked()
{
    if (c_connectFlag) {
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::move_z_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
}
void Dy_Control::move_z_Button_function() {
    move_mutex.lock();
    vector<double> pose = rtde_r->getActualTCPPose();
    pose[2] += 0.001 * int(ui.movez_lineEdit->text().toInt());
    rtde_c->moveL(pose, ui.movez_v_lineEdit->text().toDouble(), ui.movez_a_lineEdit->text().toDouble(), false);
    move_mutex.unlock();
}
void Dy_Control::on_set_init_Button_clicked()
{
    if (r_connectFlag) {
        init_pose = rtde_r->getActualTCPPose();
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
}

//move rx ry rz
void Dy_Control::on_move_init_Button_clicked() {
    if (c_connectFlag) {
        rtde_c->moveL(init_pose, 0.05, 0.05, false);
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }

}
void Dy_Control::on_Rotate_x_Button_clicked()
{
    if (c_connectFlag) {
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::Rotate_x_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
    if (forcemode_Flag) {
        on_Button_fmode_clicked();
    }
}
void Dy_Control::Rotate_x_Button_function() {
    move_mutex.lock();
    vector<double>RX_RY_RZ = TransAngle(ui.rotate_x_lineEdit->text().toInt(), 0, 0, 0);
    vector<double> pose2 = rtde_r->getActualTCPPose();
    vector<double> pose_move = { pose2[0], pose2[1], pose2[2], RX_RY_RZ[0], RX_RY_RZ[1], RX_RY_RZ[2] };
    rtde_c->moveL(pose_move, 0.05, 0.05);
    move_mutex.unlock();
}
void Dy_Control::on_Rotate_y_Button_clicked()
{
    if (c_connectFlag) {
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::Rotate_y_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
    if (forcemode_Flag) {
        on_Button_fmode_clicked();
    }
}
void Dy_Control::Rotate_y_Button_function() {
    move_mutex.lock();
    vector<double>RX_RY_RZ = TransAngle(0, ui.rotate_y_lineEdit->text().toInt(), 0, 0);
    vector<double> pose2 = rtde_r->getActualTCPPose();
    vector<double> pose_move = { pose2[0], pose2[1], pose2[2], RX_RY_RZ[0], RX_RY_RZ[1], RX_RY_RZ[2] };
    rtde_c->moveL(pose_move, 0.05, 0.05);
    move_mutex.unlock();
}
void Dy_Control::on_Rotate_z_Button_clicked()
{
    if (c_connectFlag) {
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::Rotate_z_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
    if (forcemode_Flag) {
        on_Button_fmode_clicked();
    }
}
void Dy_Control::Rotate_z_Button_function() {
    move_mutex.lock();
    vector<double>RX_RY_RZ = TransAngle(0, 0, ui.rotate_z_lineEdit->text().toInt(), 0);
    vector<double> pose2 = rtde_r->getActualTCPPose();
    vector<double> pose_move = { pose2[0], pose2[1], pose2[2], RX_RY_RZ[0], RX_RY_RZ[1], RX_RY_RZ[2] };
    rtde_c->moveL(pose_move, 0.05, 0.05);
    move_mutex.unlock();
}
void Dy_Control::on_align_z_Button_clicked()
{
    if (c_connectFlag) {
        vector<double> RX_RY_RZ = TransAngle(180, 0, 0, 1);
        vector<double> pose2 = rtde_r->getActualTCPPose();
        vector<double> pose_move = { pose2[0], pose2[1], pose2[2], RX_RY_RZ[0], RX_RY_RZ[1], RX_RY_RZ[2] };
        rtde_c->moveL(pose_move, 0.05, 0.05);
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
    }
}
void Dy_Control::on_Show_Now_Button_clicked()//展示当前状态
{
    if (!(all_connectFlag|| r_connectFlag ))
    {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes , QMessageBox::Yes);
        return;
    }
    if (read_message_Flag)
    {
        QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("当前正在读取信息"), QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    show_now_flag = true;
    update_force_pose_Data();
    show_now_flag = false;
}


//------------------------------------------速度伺服--------------------------------------------------
//move by speedl
void Dy_Control::on_move_speedL_Button_clicked() {
    if (c_connectFlag) {
        if (!movespeedThread_Flag)
        {
            QTimerForSpeedL = new QTimer(this);
            move_mutex.lock();
            movespeedThread_Flag = true;
            double delay = ui.moveTime_lineEdit->text().toDouble()*1000;//delay单位毫秒
            QTimer::singleShot(0, this, SLOT(movespeedLThread_function()));
            connect(QTimerForSpeedL,SIGNAL(timeout()),this,SLOT(stop_speedL_function()));
            QTimerForSpeedL->start(delay);
            ui.move_speedL_Button->setText(QStringLiteral("停止移动"));
        }
        else
        {
            stop_speedL_function();
        }
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}
void Dy_Control::movespeedLThread_function() {

    double acceleration = ui.Acceleration_lineEdit->text().toDouble();//加速度
    vector<double> tool_speed = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    if (movespeedThread_Flag) {
        speedL_runoutFlag = false;
        tool_speed[0] = 0.001 * ui.speedL_X_lineEdit->text().toDouble();
        tool_speed[1] = 0.001 * ui.speedL_Y_lineEdit->text().toDouble();
        tool_speed[2] = 0.001 * ui.speedL_Z_lineEdit->text().toDouble();
        tool_speed[3] = ui.speedL_Rx_lineEdit->text().toDouble() / 180 * M_PI;
        tool_speed[4] = ui.speedL_Ry_lineEdit->text().toDouble() / 180 * M_PI;
        tool_speed[5] = ui.speedL_Rz_lineEdit->text().toDouble() / 180 * M_PI;
        rtde_c->speedL(tool_speed, acceleration,10000);
    }
}
void Dy_Control::stop_speedL_function() {
    if (movespeedThread_Flag)
    {
        QTimerForSpeedL->stop();
        delete QTimerForSpeedL;
        QTimerForSpeedL = nullptr;
        double deceleration = ui.Deceleration_lineEdit->text().toDouble();//减速度
        move_mutex.unlock();
        movespeedThread_Flag = false;
        ui.move_speedL_Button->setText(QStringLiteral("开始移动"));
        rtde_c->speedStop(deceleration);
    }
    else
    {
        return;
    }
}


//move by speedj
void Dy_Control::on_move_speedJ_Button_clicked() {
    if (c_connectFlag) {
        if (!movespeedJThread_Flag)
        {
            QTimerForSpeedJ = new QTimer(this);
            move_mutex.lock();
            movespeedJThread_Flag = true;
            double delay = ui.speedJ_moveTime_lineEdit->text().toDouble() * 1000;//delay单位毫秒
            QTimer::singleShot(0, this, SLOT(movespeedJThread_function()));
            connect(QTimerForSpeedJ, SIGNAL(timeout()), this, SLOT(stop_speedJ_function()));
            QTimerForSpeedJ->start(delay);
            ui.move_speedJ_Button->setText(QStringLiteral("停止移动"));
        }
        else
        {
            stop_speedJ_function();
        }
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}
void Dy_Control::movespeedJThread_function() {

    double acceleration = ui.speedJ_Acceleration_lineEdit->text().toDouble();//加速度
    vector<double> joint_speed = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    if (movespeedJThread_Flag) {
        speedJ_runoutFlag = false;
        joint_speed[0] = ui.speedJ_1_lineEdit->text().toDouble() / 180 * M_PI;
        joint_speed[1] = ui.speedJ_2_lineEdit->text().toDouble() / 180 * M_PI;
        joint_speed[2] = ui.speedJ_3_lineEdit->text().toDouble() / 180 * M_PI;
        joint_speed[3] = ui.speedJ_4_lineEdit->text().toDouble() / 180 * M_PI;
        joint_speed[4] = ui.speedJ_5_lineEdit->text().toDouble() / 180 * M_PI;
        joint_speed[5] = ui.speedJ_6_lineEdit->text().toDouble() / 180 * M_PI;
        rtde_c->speedJ(joint_speed, acceleration,10000);
    }
}
void Dy_Control::stop_speedJ_function() {
    if (movespeedJThread_Flag)
    {
        QTimerForSpeedJ->stop();
        delete QTimerForSpeedJ;
        QTimerForSpeedJ = nullptr;
        double deceleration = ui.speedJ_Deceleration_lineEdit->text().toDouble();//减速度
        move_mutex.unlock();
        movespeedJThread_Flag = false;
        ui.move_speedJ_Button->setText(QStringLiteral("开始移动"));
        rtde_c->speedStop(deceleration);
    }
    else
    {
        return;
    }
}


void Dy_Control::on_change_Button_clicked() {

    int pageCount = ui.stackedWidget->count();

    curPageOfSpeedServe = (curPageOfSpeedServe + 1) % pageCount;

    ui.stackedWidget->setCurrentIndex(curPageOfSpeedServe);
}

void Dy_Control::on_change2_Button_clicked() {

    int pageCount = ui.stackedWidget_2->count();

    curPageOfForceMode = (curPageOfForceMode + 1) % pageCount;

    ui.stackedWidget_2->setCurrentIndex(curPageOfForceMode);

}


//------------------------------------------------------------------------------------------------


//增量模式功能
void Dy_Control::on_incremental_Button_clicked()
{
    if (all_connectFlag) {
        if (ui.incremental_Button->is_start == false) {
            qDebug() << "start control" << endl;
            connect(ui.incremental_Button, &KeyEventForIncModeButton::changePose, this, &Dy_Control::change_incremental_para);
            ui.incremental_Button->setText(QStringLiteral("关闭"));
            ui.incremental_Button->grabKeyboard();
            ui.incremental_Button->is_start = true;
            QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::incremental_func);
        }
        else if (ui.incremental_Button->is_start == true) {
            qDebug() << "stop control" << endl;
            disconnect(ui.incremental_Button, &KeyEventForIncModeButton::changePose, this, &Dy_Control::change_incremental_para);
            ui.incremental_Button->setText(QStringLiteral("增量模式"));
            ui.incremental_Button->releaseKeyboard();
            ui.incremental_Button->is_start = false;
        }
    }
    else
    {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void Dy_Control::incremental_func() {
    double Ts = 0.01;
    double mutiple = ui.multiple_lineEdit->text().toDouble();//增量倍数
    double Δx = 0.1 * Ts * mutiple; // 默认为：1s走10cm ，0.1是0.1m意思  (后经过倍数调整大小)
    double Δy = 0.1 * Ts * mutiple;
    double Δz = 0.1 * Ts * mutiple;
    double ΔrotateTCPX = (double)10 * Ts * PI / 180 * mutiple; //默认为 1s 转动 10° (后经过倍数调整大小)
    double ΔrotateTCPY = (double)10 * Ts * PI / 180 * mutiple;
    double ΔrotateTCPZ = (double)10 * Ts * PI / 180 * mutiple;
    double curX = 0, curY = 0, curZ = 0, cur_rotateTCPX = 0, cur_rotateTCPY = 0, cur_rotateTCPZ = 0;

    timeb start;
    ftime(&start);//获取毫秒,严格控制每个servoL指令之间的间隔为Ts！
    while (ui.incremental_Button->is_start) {
        
        auto pose = rtde_r->getActualTCPPose();
        if (inclock.tryLockForRead()) {
            curX = X * Δx;
            curY = Y * Δy;
            curZ = Z * Δz;
            cur_rotateTCPX = rotateTCPX * ΔrotateTCPX;
            cur_rotateTCPY = rotateTCPY * ΔrotateTCPY;
            cur_rotateTCPZ = rotateTCPZ * ΔrotateTCPZ;
            inclock.unlock();
        }
        //qDebug() << curX << " " << curY << " " << curZ << " " << cur_rotateTCPX << " " << cur_rotateTCPY << " " << cur_rotateTCPZ << endl;
        pose[0] += curX;
        pose[1] += curY;
        pose[2] += curZ;
        cv::Mat kxita = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
        cv::Mat rotation_matrix;
        cv::Rodrigues(kxita, rotation_matrix);
        cv::Mat Rx_θx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(cur_rotateTCPX), -sin(cur_rotateTCPX), 0, sin(cur_rotateTCPX), cos(cur_rotateTCPX));
        cv::Mat Ry_θy = (cv::Mat_<double>(3, 3) << cos(cur_rotateTCPY), 0, sin(cur_rotateTCPY), 0, 1, 0, -sin(cur_rotateTCPY), 0, cos(cur_rotateTCPY));
        cv::Mat Rz_θz = (cv::Mat_<double>(3, 3) << cos(cur_rotateTCPZ), -sin(cur_rotateTCPZ),0, sin(cur_rotateTCPZ), cos(cur_rotateTCPZ),0, 0, 0, 1);
        rotation_matrix = rotation_matrix * Rx_θx * Ry_θy * Rz_θz;
        cv::Vec3d new_rxryrz;
        cv::Rodrigues(rotation_matrix, new_rxryrz); //得到绕固定坐标系下的kxita（由于这样的相对坐标系下得到的绕固定坐标系下的旋转矩阵是等价的，所以这个矩阵也是绕固定坐标系下的矩阵）
        
        pose[3] = new_rxryrz[0];
        pose[4] = new_rxryrz[1];
        pose[5] = new_rxryrz[2];

        timeb end;
        ftime(&end);//获取毫秒
        int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
        if (deta_t < Ts * 1000) {
            Sleep(Ts * 1000 - deta_t);
        }
        rtde_c->servoL(pose, 0, 0, Ts, 0.03, 500);
        ftime(&start);//获取毫秒
    }
    rtde_c->servoStop(); //关闭
}

void  Dy_Control::change_incremental_para(int X, int Y, int Z, int rotateTCPX, int rotateTCPY, int rotateTCPZ) {
    while (!inclock.tryLockForRead()) {
        continue;
    }
    this->X = X;
    this->Y = Y;
    this->Z = Z;
    this->rotateTCPX = rotateTCPX;
    this->rotateTCPY = rotateTCPY;
    this->rotateTCPZ = rotateTCPZ;
    inclock.unlock();
}

//-------------------------------------------------------------------------------------------------



//-------------------------------阻抗控制相关函数------------------------------------------
void Dy_Control::on_fmode_Button_clicked()
{
    if (c_connectFlag&&!fmode_flag) {
        fmode_flag = true;
        ui.fmode_Button->setText(QStringLiteral("停止基于GABP阻抗控制"));
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::fmode_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else if(c_connectFlag&&fmode_flag)
    {
        fmode_flag = false;
        //没真正开启阻抗控制就调用了这个函数，说明是因为位置偏差过大而返回
        if(ic->startFlag==false)
            QMessageBox::warning(NULL, QStringLiteral("提示"), QStringLiteral("期望位置与当前位置偏差过大"), QMessageBox::Yes, QMessageBox::Yes);
        ui.fmode_Button->setText(QStringLiteral("开启基于GABP阻抗控制（需先初始化）"));
        ic->startFlag = false;
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void Dy_Control::fmode_Button_function(){

    move_mutex.lock();
    //读取对应的数据，用于法向力阻抗控制
    double Fd = ui.Fd_lineEdit->text().toDouble();
    double Xr = ic->getXr(Fd);
    //期望行走的x方向的速度
    double V = ui.v_lineEdit->text().toDouble();
    V /= 1000;
    double Ts = ui.Ts_lineEdit->text().toDouble();
    double tForServo = ui.tForServo_lineEdit->text().toDouble();
    ic->Normal_force_control(Fd, Xr, Ts,V,tForServo);//法向力控制
    move_mutex.unlock();
}

void Dy_Control::on_icGetInit_Button_clicked()
{
    if (c_connectFlag && !fmode_flag) {

        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::icGetInit_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
        ui.icGetInit_Button->setText(QStringLiteral("正在初始化阻抗控制"));
        while (!icinit_flag)
        {
            Sleep(1000);
        }
        QMessageBox::warning(NULL, QStringLiteral("初始化"), QStringLiteral("GABP训练完成，IC初始化完成"), QMessageBox::Yes, QMessageBox::Yes);
        ui.icGetInit_Button->setText(QStringLiteral("已经初始化阻抗控制"));
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void Dy_Control::icGetInit_Button_function() {
    //防止重复初始化ic
    if (!ic)
    {
        move_mutex.lock();
        double M = ui.M_lineEdit->text().toDouble();
        double B = ui.B_lineEdit->text().toDouble();
        double K = ui.K_lineEdit->text().toDouble();
        double threshold = ui.Threshold_lineEdit->text().toDouble();
        double mostTimes = ui.mostTimes_lineEdit->text().toDouble();
        rtde_c->zeroFtSensor();//传感器清零
        //初始化阻抗控制并训练神经网络
        ic = new Dy::Impedance_control(M, B, K, new Dy::BpNet(),rtde_c, rtde_r);
        ic->getInit(threshold, mostTimes);
        //初始化完成
        icinit_flag = true;
        move_mutex.unlock();
    }
}

void Dy_Control::on_resetIC_Button_clicked() {

    if(c_connectFlag) {
        delete ic;
        ic = nullptr;
        icinit_flag = false;
        QMessageBox::warning(NULL, QStringLiteral("重置"), QStringLiteral("重置成功"), QMessageBox::Yes, QMessageBox::Yes);
        ui.icGetInit_Button->setText(QStringLiteral("初始化阻抗控制(下降到指定位置)"));
    }
    else {
    QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void Dy_Control::on_BaseNowIC_Button_clicked() {

    if (c_connectFlag && !fmode_flag) {
        fmode_flag = true;
        ui.BaseNowIC_Button->setText(QStringLiteral("停止基于当前位置的阻抗控制"));
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::BaseNowIC_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else if (c_connectFlag && fmode_flag)
    {
        ic->startFlag = false;
        fmode_flag = false;
        ui.BaseNowIC_Button->setText(QStringLiteral("开启基于当前位置阻抗控制（无需初始化）"));
        Sleep(100);
        delete ic;
        ic = nullptr;
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }

}

void Dy_Control::BaseNowIC_Button_function() {
    move_mutex.lock();
    double Fd = ui.Fd_lineEdit->text().toDouble();
    double V = ui.v_lineEdit->text().toDouble();
    vector<double> _moveUnitDirInWorld{ui.move_dirX_world_lineEdit->text().toDouble(),ui.move_dirY_world_lineEdit->text().toDouble() ,ui.move_dirZ_world_lineEdit->text().toDouble() };
    V /= 1000;
    double Ts = ui.Ts_lineEdit->text().toDouble();
    double tForServo = ui.tForServo_lineEdit->text().toDouble();
    double sigma = ui.sigma_lineEdit->text().toDouble();
    int mode = ui.ICmode_lineEdit->text().toInt();
    double downLimit = ui.downLimit_lineEdit->text().toDouble();
    double R = ui.MassageHeadRadius_lineEdit->text().toDouble();
    double LengthOfSensor2MassageHeadCentre = ui.LengthOfSensor2MassageHeadCentre_lineEdit->text().toDouble();
    double k_formulas = ui.k_formulas_lineEdit->text().toDouble();

    R /= 1000; //单位化成米

    LengthOfSensor2MassageHeadCentre /= 1000;

    //初始化ic，但无需训练BpNet，因为是基于当前位置的阻抗控制
    if (!ic)//如果已经初始化过，就报错，因为之前初始化过ic但不是使用当前功能，即没重置而使用另外一种功能，这两个ic的作用是不一样的，有些初始参数不一样，所以选择报错
    {
        rtde_c->zeroFtSensor();//传感器清零
        double M = ui.M_lineEdit->text().toDouble();
        double B = ui.B_lineEdit->text().toDouble();
        double K = ui.K_lineEdit->text().toDouble();
        double tau_0 = ui.tau_0_lineEdit->text().toDouble();
        double gamma = ui.gamma_lineEdit->text().toDouble();
        double E_star = ui.E_star_lineEdit->text().toDouble();
        ic = new Dy::Impedance_control(M, B, K, tau_0, gamma, E_star, new Dy::BpNet(), rtde_c, rtde_r);
        ic->rotateLock = &rotateLock; //设置读写旋转变量锁
    }
    else {
        throw "ic have init";
        abort();
    }
    ui.icGetInit_Button->setText(QStringLiteral("已经初始化阻抗控制"));

    if(mode == 0) //内窥镜与阻抗控制同时作用所使用，目的是探索与转角有关的影响因素，此时需要控制台与ic的变量相关联
    connect(ui.control_label, &KeyEventLabel::changeRotate, this, &Dy_Control::change_IC_Rotate);//通过label的keyevent修改ic的参数

    ic->Normal_force_control_base_on_now(Fd, Ts,_moveUnitDirInWorld,V, tForServo,sigma,mode,downLimit,R, LengthOfSensor2MassageHeadCentre,k_formulas);
    move_mutex.unlock();
}

void Dy_Control::change_IC_Rotate(int rotateTCPX, int rotateTCPY) {
    if (!ic) {        //若ic不存在
        qDebug() << "ic have not init" << endl;
        return;
    }
    else {
        while (!rotateLock.tryLockForWrite()) { //直到成功锁上为止（一定要写上）
            Sleep(1);
        }
        ic->rotateTCPX = rotateTCPX;
        ic->rotateTCPY = rotateTCPY;
        rotateLock.unlock();
    }
}

//---------------------------------------------------------------------------------------

//---------------------------------------PID恒力控制相关函数-----------------------------

void Dy_Control::on_PC_Button_clicked() {
    if (c_connectFlag && !PIDmode_Flag) {
        PIDmode_Flag = true;
        ui.PC_Button->setText(QStringLiteral("关闭PID恒力控制"));
        QFuture<void> featrue = QtConcurrent::run(this, &Dy_Control::PC_Button_function);//创建了一个线程（异步计算），feature是接收对应的异步计算的结果
    }
    else if (c_connectFlag && PIDmode_Flag)
    {
        pc->startFlag = false;
        PIDmode_Flag = false;
        ui.PC_Button->setText(QStringLiteral("PID恒力控制"));
        Sleep(100);
        delete pc;
        pc = nullptr;
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void Dy_Control::PC_Button_function() {

    QMutexLocker Locker(&move_mutex);//对象管理锁
    double Fd = ui.PID_Fd_lineEdit->text().toDouble();
    double V = ui.PID_v_lineEdit->text().toDouble();
    V /= 1000;
    double Ts = ui.PID_Ts_lineEdit->text().toDouble();
    double tForServo = ui.PID_tForServo_lineEdit->text().toDouble();

    //PID三参数
    double Kp = ui.Kp_lineEdit->text().toDouble();
    double Ki = ui.Ki_lineEdit->text().toDouble();
    double Kd = ui.Kd_lineEdit->text().toDouble();

    //初始化PID控制
    rtde_c->zeroFtSensor();//传感器清零
    pc = new Dy::PID_Control(Kp,Ki,Kd,rtde_c, rtde_r);
    pc->Normal_force_Control_base_on_PID(Fd, Ts, V, tForServo);
}


//---------------------------------------------------------------------------------------

//---------------------------------------Kinect相机相关函数--------------------------------

void Dy_Control::on_KinectController_clicked() {
    if (!open_KinectController_Flag) {
        open_KinectController_Flag = true;
        kinectController = new KinectController(&open_KinectController_Flag);
        kinectController->show();
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("DK相机控制台提示"), QStringLiteral("已打开DK相机控制台"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

//---------------------------------------------------------------------------------------


//--------------------------------------力传感器相关函数-------------------------------------

void Dy_Control::on_forceSenseController_clicked() {

    if (c_connectFlag) {
        if (!open_forceSenseController_Flag) {
            open_forceSenseController_Flag = true;
            fSenseController = new forceSenseController(&open_forceSenseController_Flag, rtde_c,rtde_r);
            fSenseController->show();
        }
        else {
            QMessageBox::warning(NULL, QStringLiteral("力传感器控制台提示"), QStringLiteral("已打开力传感器控制台"), QMessageBox::Yes, QMessageBox::Yes);
        }
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("连接机器人提示"), QStringLiteral("请先连接机器人"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

//---------------------------------------RS相机相关函数-------------------------------------

void Dy_Control::on_RealSenseController_clicked() {

    if (!open_RealSenseController_Flag) {
        open_RealSenseController_Flag = true;
        realSenseController = new RealSenseController(&open_RealSenseController_Flag);
        realSenseController->show();
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("RS相机控制台提示"), QStringLiteral("已打开RS相机控制台"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

//---------------------------------------------------------------------------------------


//---------------------------------------内窥镜相机相关函数---------------------------------

void Dy_Control::on_endoscope_read_Button_clicked() {
    if (!open_endoscope_Flag) {
        QMessageBox::warning(NULL, QStringLiteral("内窥镜相关提示"), QStringLiteral("请先打开内窥镜"), QMessageBox::Yes, QMessageBox::Yes);
    }
    else if (!read_endoscope_Flag) {
        ui.endoscope_read_Button->setText(QStringLiteral("结束读取"));
        read_endoscope_Flag = true;
    }
    else {
        ui.endoscope_read_Button->setText(QStringLiteral("读取内窥镜图片"));
        read_endoscope_Flag = false;
    }
}

void Dy_Control::on_endoscope_save_Button_clicked() {
    if (!open_endoscope_Flag) {
        QMessageBox::warning(NULL, QStringLiteral("内窥镜相关提示"), QStringLiteral("请先打开内窥镜"), QMessageBox::Yes, QMessageBox::Yes);
    }
    else {
          ui.endoscope_read_Button->setText(QStringLiteral("读取内窥镜图片"));
          read_endoscope_Flag = false;
          endoscopeBufferLock.lockForWrite();
          if (endoscopeBuffer.empty()) {
              QMessageBox::warning(NULL, QStringLiteral("内窥镜相关提示"), QStringLiteral("缓冲区无图片"), QMessageBox::Yes, QMessageBox::Yes);
              return;
          } 
          //默认内窥镜图片保存位置
          QString path = "F:/DESKAPPPLACE/DOCUMENT/rtde/Dy_Control/endoscopeImageSaveDir/";
          path += ui.Save_endoscope_lineEdit->text();
          CreateDirectory(path.toStdString().c_str(),NULL);
          if (path.back() != '/')
              path += '/';
          for (int i = 0; i < endoscopeBuffer.size();i++) {
              const long long &index = endoscopeBuffer[i].first;
              const QImage& photo = endoscopeBuffer[i].second;
              QString cur = path + QString::fromStdString(to_string(index)) + ".jpeg";
              photo.save(cur, "JPEG", 100);
          }
          endoscopeBuffer.clear();
          endoscopeBufferLock.unlock();
          QMessageBox::warning(NULL, QStringLiteral("内窥镜相关提示"), QStringLiteral("图片保存成功"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void Dy_Control::on_endoscopeBuffer_clear_Button_clicked() {
    if(read_endoscope_Flag)
        QMessageBox::warning(NULL, QStringLiteral("内窥镜相关提示"), QStringLiteral("请先停止读图片"), QMessageBox::Yes, QMessageBox::Yes);
    else {
        endoscopeBufferLock.lockForWrite();
        endoscopeBuffer.clear();
        endoscopeBufferLock.unlock();
        QMessageBox::warning(NULL, QStringLiteral("内窥镜相关提示"), QStringLiteral("成功清空"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void Dy_Control::on_endoscope_open_Button_clicked() {
    if (!open_endoscope_Flag) {
        //开启相机
        capture = cv::VideoCapture(0);
        ui.endoscope_color_image->setScaledContents(true); //自适应图片

        //开启线程
        endoscope_show_thread = new endoscopeShowThread(this,&capture,endoscope_mutex);
        connect(endoscope_show_thread, &endoscopeShowThread::change_endoscope_frame, this, &Dy_Control::slot_updateUI_endoscope_color_image, Qt::BlockingQueuedConnection);//子线程获取完毕则刷新主线程ui
        endoscope_show_thread->start();
        open_endoscope_Flag = true;
        ui.endoscope_open_Button->setText(QStringLiteral("关闭内窥镜"));
    }
    else {
        //必须将子线程与主线程先断开connect否则无法正常回收线程
        disconnect(endoscope_show_thread, &endoscopeShowThread::change_endoscope_frame, this, &Dy_Control::slot_updateUI_endoscope_color_image);

        //终止并回收子线程
        endoscope_show_thread->quit();
        endoscope_show_thread->wait();
        delete endoscope_show_thread;
        endoscope_show_thread = nullptr;

        ui.endoscope_color_image->clear();
        ui.endoscope_color_image->setText(QStringLiteral("内窥镜图像区域"));

        capture.release(); //释放设备

        open_endoscope_Flag = false;
        ui.endoscope_open_Button->setText(QStringLiteral("开启内窥镜"));
    }
}

//子线程通过connect刷新主线程GUI
void Dy_Control::slot_updateUI_endoscope_color_image(QImage image) {
    this->ui.endoscope_color_image->setPixmap(QPixmap::fromImage(image));

    //如果需要读取图片，则把图片保存到缓冲区
    if (read_endoscope_Flag) {
        endoscopeBufferLock.lockForWrite();
        long long index = 0;
        struct timeb t1;
        ftime(&t1);
        long long t = t1.time * 1000 + t1.millitm;
        index = t;
        endoscopeBuffer.emplace_back(index, image);
        endoscopeBufferLock.unlock();
    }
}

//---------------------------------------------------------------------------------------

//---------------------------------------功能函数-----------------------------------------

//求出tcp的force
vector<double> Dy_Control::world_2tcp_force (vector<double> world_force, vector<double>in_pose)
{
    cv::Mat force = (cv::Mat_<double>(3, 1) << world_force[0], world_force[1], world_force[2]);//
    cv::Mat m_force = (cv::Mat_<double>(3, 1) << world_force[3], world_force[4], world_force[5]);// 3 * 1的力矩向量

    ////tcp坐标系的原点在基座标系下的表示
    //cv::Mat dist = (cv::Mat_<float>(3, 1) << (in_pose[0] / 1000), (in_pose[1] / 1000), (in_pose[2] / 1000));//tcp坐标系与基座标系的距离向量

    cv::Mat r_l = (cv::Mat_<double>(3, 1) << in_pose[3], in_pose[4], in_pose[5]);//旋转向量

    cv::Mat  R_M;
    cv::Rodrigues(r_l, R_M);

    ////基座标系原点在tcp坐标系下的表示   来源：https://blog.csdn.net/chengde6896383/article/details/86738179
    //cv::Mat R_M_T = R_M.t();//求转置矩阵 也就是旋转矩阵的逆矩阵
    //cv::Mat base_dist = -R_M_T * dist;//基座标系原点在tcp坐标系下的表示    

    cv::Mat tcp_force_xyz_mat = R_M * force;

    //cv::Mat tcp_force_mxyz_mat = R_M * m_force + base_dist.cross(tcp_force_xyz_mat);//力矩坐标系转换公式  来源：机器人关节与关节之间的例句转换
    cv::Mat tcp_force_mxyz_mat = R_M * m_force;

    vector<double> tcp_force;
    tcp_force.push_back(tcp_force_xyz_mat.at<double>(0, 0));
    tcp_force.push_back(tcp_force_xyz_mat.at<double>(1, 0));
    tcp_force.push_back(tcp_force_xyz_mat.at<double>(2, 0));
    tcp_force.push_back(tcp_force_mxyz_mat.at<double>(0, 0));
    tcp_force.push_back(tcp_force_mxyz_mat.at<double>(1, 0));
    tcp_force.push_back(tcp_force_mxyz_mat.at<double>(2, 0));

    return tcp_force;
}
//替换字串
string Dy_Control::subreplace(string resource_str, string sub_str, string new_str)
{
    string dst_str = resource_str;
    string::size_type pos = 0;
    while ((pos = dst_str.find(sub_str)) != string::npos)   //替换所有指定子串
    {
        dst_str.replace(pos, sub_str.length(), new_str);
    }
    return dst_str;
}
//矩阵的转换
vector<double> Dy_Control::TransAngle(double theta_1, double theta_2, double theta_3, int flag) {
    qDebug() << QStringLiteral("TransAngle") << endl;
    theta_1 = theta_1 / 180 * M_PI;//转为弧度制
    theta_2 = theta_2 / 180 * M_PI;
    theta_3 = theta_3 / 180 * M_PI;
    cv::Mat Rx = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, cos(theta_1), -sin(theta_1), 0, sin(theta_1), cos(theta_1));
    cv::Mat Ry = (cv::Mat_<float>(3, 3) << cos(theta_2), 0, sin(theta_2), 0, 1, 0, -sin(theta_2), 0, cos(theta_2));
    cv::Mat Rz = (cv::Mat_<float>(3, 3) << cos(theta_3), -sin(theta_3), 0, sin(theta_3), cos(theta_3), 0, 0, 0, 1);
    cv::Mat R1 = Rz * Ry;
    cv::Mat R = R1 * Rx;
    if (flag == 0) { //flag==0的时候是从当前位置绕world轴旋转，最后的结果是期望结果的kxita
        vector<double> pose2 = rtde_r->getActualTCPPose();
        cv::Mat mid_R = (cv::Mat_<float>(1, 3) << pose2[3], pose2[4], pose2[5]);
        cv::Mat  final_R;
        cv::Rodrigues(mid_R, final_R);
        R = final_R * R;
    }
    cv::Mat Rxyz;
    cv::Rodrigues(R, Rxyz);
    Rxyz.at<float>(0, 0);
    vector<double> RX_RY_RZ = { Rxyz.at<float>(0, 0),Rxyz.at<float>(1, 0),Rxyz.at<float>(2, 0) };
    return RX_RY_RZ;
}

//c_str转换为LPWSTR的函数
LPWSTR Dy_Control::ConvertCharToLPWSTR(const char* szString)
{
    int dwLen = strlen(szString) + 1;
    int nwLen = MultiByteToWideChar(CP_ACP, 0, szString, dwLen, NULL, 0);//算出合适的长度
    LPWSTR lpszPath = new WCHAR[dwLen];
    MultiByteToWideChar(CP_ACP, 0, szString, dwLen, lpszPath, nwLen);
    return lpszPath;
}

//---------------------------------------------------------------------------------------



void Dy_Control::on_test_Button_clicked() {
    
    // 下面为求与 平面实验中法向量的差距  在matlab中算出来α以及β--------------------------------------------------------
    /*
    
    double a = 0.823740; //待填写

    double b = 1.351528; //待填写

    vector<double> pose = { 0.128283,-0.32542,-0.0239031,3.09253,0.00052859,0.54544 };//待填写

    Vec3d end_effector_direction{ 1,0,0 };

    Vec3d base_frame_z_axis{ 0,0,1 };

    Vec3d x_axis;
    Vec3d y_axis;
    Vec3d z_axis;

    auto sixth_axis = getSixthAxisFromBasePose(pose);

    computeMotionCoordinateSystem(sixth_axis, end_effector_direction, base_frame_z_axis, x_axis, y_axis, z_axis);

    auto base2motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

    Vec3d normal_motion = { 0.01 * sin(a) * cos(b),0.01 * sin(a) * sin(b),0.01 * cos(a)}; //移动坐标系下的法向量的表示

    normal_motion = cv::Mat(base2motion * normal_motion); //世界坐标系下的法向量的表示

    auto rotVec = CalculateRotationVector(normal_motion, { 0,0,1 }); //平面实验 观察偏差

    qDebug() << " rotVec's size = " << cv::norm(rotVec);
    
    */

    // -----------------------------------------------------------------------------------------------------------

    //double Ts = 0.005;
    //double Vx = 0.002;

    ////初始法向位置位置以及对应力
    //double Xe = 0.5;
    //double Fe = 0;

    ////pid控制算法的三个数值S
    //double value_p = 0.0;
    //double value_i = 0.0;
    //double value_d = 0.0;

    ////上一时刻的环境力 (用于微分环节数值求解)
    //double Fe_last = Fe;

    //double Fd = 5;
    //double A = 1.0; // 正弦波的振幅
    //double f = 1.0 / 5; // 正弦波的频率，单位Hz
    //double phi = 0; // 正弦波的相位，单位弧度
    //double C = Fd; // 正弦波的直流偏置

    //double Kp = 0.006;
    //double Ki = 0;
    //double Kd = 0.004;

    ////切向位置移动
    //double Xd_x = 0;
    //Xd_x += Vx * Ts;

    //double Xc;//传给机器人指令的位置

    //timeb tb;
    //ftime(&tb);//获取毫秒

    //long long startTime = tb.time * 1000 + tb.millitm;


    //while (true)
    //{
    //    timeb start;
    //    ftime(&start);//获取毫秒

    //    long long currentTime = start.time * 1000 + start.millitm;
    //    double t = (currentTime - startTime) / 1000.0; // 转换为秒

    //    // 根据时间动态计算Fd的值
    //    Fd = A * sin(2 * M_PI * f * t) + C;

    //    //PID控制算法本体
    //    value_p = Fe - Fd; //目标偏差
    //    value_i += (Fe - Fd) * Ts; //目标偏差的积分
    //    value_d = (Fe - Fe_last) / Ts; //目标偏差的微分
    //    //法向位置是传给机器人位置伺服闭环，同时与上一时刻和pid数值相加
    //    Xc = Xe + Kp * value_p + Ki * value_i + Kd * value_d;

    //    //切向移动
    //    Xd_x += Vx * Ts;

    //    //输入指令
    //    Xd_x;
    //    Xc;

    //    Xe = Xc;
    //    Fe = (0.5 - Xe) * 200;

    //    qDebug() << "Xc: " << Xc << endl;
    //    qDebug() << "Fe: " << Fe << endl;

    //    ////安全伺服移动
    //    //Safe_servoL(pose, 0, 0, tForServo, 0.1, 300);

    //    //更新参数,只更新法向
    //    Fe_last = Fe;
    //    //Xe = rtde_r->getActualTCPPose()[2];
    //    //Fe = rtde_r->getActualTCPForce()[2];

    //    timeb end;
    //    ftime(&end);//获取毫秒
    //    int deta_t = (end.time * 1000 + end.millitm) - (start.time * 1000 + start.millitm);
    //    if (deta_t < Ts * 1000) {
    //        Sleep(Ts * 1000 - deta_t);
    //    }
    //}
    ////rtde_c->servoStop();

        // 定义输入参数
    std::vector<double> force = { 1.0, 2.0, 3.0, 0.01, 0.05, 0.06 };
    double R = 1.0;
    double l = 1.0;
    bool isCompensation = true;
    double tau_0 = 0.1;
    double gamma = 0.1;
    double E_star = 1.0;
    
    // 创建 QElapsedTimer 实例并启动计时
    QElapsedTimer timer;
    timer.start();

    // 调用函数
    cv::Vec3d result = Dy::Impedance_control::calculateSurfaceNormalVector(force, R, l, isCompensation, tau_0, gamma, E_star);
    
    // 获取时间差（毫秒）
    qint64 elapsed = timer.elapsed();
    qDebug() << "Execution time:" << elapsed << "milliseconds";
    
    // 输出结果（可选）
    qDebug() << "Result: (" << result[0] << ", " << result[1] << ", " << result[2] << ")";

}


//析构函数
Dy_Control::~Dy_Control()
{}

