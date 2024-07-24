#include "RealSenseController.h"

RealSenseController::RealSenseController(bool* openFlagAddress, QWidget* parent)
	: openFlagAddress(openFlagAddress), QMainWindow(parent)
{
	ui.setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose); //子窗体关掉时候可以调用析构函数

    open_realSenseCameraManager_Flag = false;
    realSenseCameraManager = nullptr;
    realSenseCamera_Show_Thread = nullptr;
}

void RealSenseController::on_open_RS_Button_clicked() {
    if (open_realSenseCameraManager_Flag == false) {

        //初始化RS相机管理器
        realSenseCameraManager = new RealSenseCameraManager();
        if (!realSenseCameraManager->initializeCameras()) {
            delete realSenseCameraManager;
            realSenseCameraManager = nullptr;
            QMessageBox::warning(NULL, QStringLiteral("RS相机控制台提示"), 
                QStringLiteral("初始化相机失败"), QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        ui.RS1_color_image->setScaledContents(true);//自适应图片
        ui.RS2_color_image->setScaledContents(true);

        serial2num = realSenseCameraManager->getSerial2num();
        num2serial = realSenseCameraManager->getNum2serial();

        if (num2serial.size() == 0) { //没有相机
            delete realSenseCameraManager;
            realSenseCameraManager = nullptr;
            QMessageBox::warning(NULL, QStringLiteral("RS相机控制台提示"),
                QStringLiteral("未连接到任何相机，请检查端口"), QMessageBox::Yes, QMessageBox::Yes);
            return;
        }

        realSenseCamera_Show_Thread = new realSenseCameraShowThread(this, realSenseCameraManager);
        connect(realSenseCamera_Show_Thread, &realSenseCameraShowThread::change_rs_frame, this,
                &RealSenseController::slot_updateUI_RS_image, Qt::BlockingQueuedConnection);//子线程获取完毕则刷新主线程ui

        realSenseCamera_Show_Thread->start();

        open_realSenseCameraManager_Flag = true;
        ui.open_RS_Button->setText(QStringLiteral("关闭RS相机"));
    }
    else {
        open_realSenseCameraManager_Flag = false;

        ui.open_RS_Button->setText(QStringLiteral("打开RS相机"));

        //必须将子线程与主线程先断开connect否则无法正常回收线程
        disconnect(realSenseCamera_Show_Thread, &realSenseCameraShowThread::change_rs_frame,
                    this, &RealSenseController::slot_updateUI_RS_image);

        //终止线程并回收线程
        realSenseCamera_Show_Thread->quit();
        realSenseCamera_Show_Thread->wait();
        delete realSenseCamera_Show_Thread;
        realSenseCamera_Show_Thread = nullptr;

        //关闭相机管理器
        delete realSenseCameraManager;
        realSenseCameraManager = nullptr;

        serial2num.clear();
        num2serial.clear();

        ui.RS1_color_image->clear();
        ui.RS1_color_image->setText(QStringLiteral("RS1相机彩色图像区域"));

        ui.RS2_color_image->clear();
        ui.RS2_color_image->setText(QStringLiteral("RS2相机彩色图像区域"));

    }
}

void RealSenseController::slot_updateUI_RS_image(std::map<std::string, rs2::frame> all_color_frames) {
    if(num2serial.find(0) != num2serial.end())
        this->ui.RS1_color_image->setPixmap(QPixmap::fromImage(rsFrameToQImage(all_color_frames[num2serial[0]])));
    if (num2serial.find(1) != num2serial.end())
        this->ui.RS1_color_image->setPixmap(QPixmap::fromImage(rsFrameToQImage(all_color_frames[num2serial[1]])));
}

void RealSenseController::on_readPCD_clicked() {

    QString path = QString(ui.pcd_filename_lineEdit->text());
    std::vector<QVector3D> cloud = ReadVec3PointCloudPCD(path);
    //curPointCloud = cloud;
    ui.pointcloud_widget->showPointCloud(cloud);
    return;
}

void RealSenseController::on_show_cur_pointcloud_clicked() {
    if (open_realSenseCameraManager_Flag) {
        double rs1_minX = ui.rs1_minX_lineEdit->text().toDouble();
        double rs1_maxX = ui.rs1_maxX_lineEdit->text().toDouble();
        double rs1_minY = ui.rs1_minY_lineEdit->text().toDouble();
        double rs1_maxY = ui.rs1_maxY_lineEdit->text().toDouble();
        double rs1_minZ = ui.rs1_minZ_lineEdit->text().toDouble();
        double rs1_maxZ = ui.rs1_maxZ_lineEdit->text().toDouble();
        double rs2_minX = ui.rs2_minX_lineEdit->text().toDouble();
        double rs2_maxX = ui.rs2_maxX_lineEdit->text().toDouble();
        double rs2_minY = ui.rs2_minY_lineEdit->text().toDouble();
        double rs2_maxY = ui.rs2_maxY_lineEdit->text().toDouble();
        double rs2_minZ = ui.rs2_minZ_lineEdit->text().toDouble();
        double rs2_maxZ = ui.rs2_maxZ_lineEdit->text().toDouble();
        if(num2serial.find(0) != num2serial.end())
            curPointCloudRS1 = PclPointClouds2vecQvec3D(
                *filterPointsByROI(realSenseCameraManager->getAllPointClouds()[num2serial[0]],
                                  rs1_minX,rs1_maxX,
                                  rs1_minY,rs1_maxY,
                                  rs1_minZ,rs1_maxZ));
        if (num2serial.find(1) != num2serial.end())
            curPointCloudRS2 = PclPointClouds2vecQvec3D(
                *filterPointsByROI(realSenseCameraManager->getAllPointClouds()[num2serial[1]],
                    rs2_minX, rs2_maxX,
                    rs2_minY, rs2_maxY,
                    rs2_minZ, rs2_maxZ));
        ui.pointcloud_widget->showPointCloud(curPointCloudRS1); //目前只有单相机了，呜呜
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("RS相机控制台提示"),
            QStringLiteral("请先打开相机"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void RealSenseController::on_save_cur_pointcloud_clicked() { //目前只有单相机了，呜呜
    if (curPointCloudRS1.empty())
        return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(curPointCloudRS1.size());
    for (int i = 0; i < curPointCloudRS1.size(); i++) {
        cloud->points[i].x = curPointCloudRS1[i].x();
        cloud->points[i].y = curPointCloudRS1[i].y();
        cloud->points[i].z = curPointCloudRS1[i].z();
    }
    pcl::io::savePCDFileASCII(ui.save_filename_lineEdit->text().toStdString(), *cloud);
}


std::vector<QVector3D> RealSenseController::ReadVec3PointCloudPCD(QString path) {
    std::vector<QVector3D> pointcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.toStdString(), *cloud) == -1) //* load the file
    {
        qDebug() << "Couldn't read file test_pcd.pcd \n";
        return {};
    }
    pointcloud = PclPointClouds2vecQvec3D(*cloud);
    return pointcloud;
}


std::vector<QVector3D> RealSenseController::PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
    std::vector<QVector3D> ans;
    for (const auto& point : pointcloud)
        ans.push_back({ point.x , point.y , point.z });
    return ans;
}


RealSenseController::~RealSenseController()
{
    if (open_realSenseCameraManager_Flag)
        on_open_RS_Button_clicked();
    *openFlagAddress = false;
}
