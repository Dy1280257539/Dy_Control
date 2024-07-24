#include "KinectController.h"

KinectController::KinectController(bool* openFlagAddress,QWidget *parent):
    openFlagAddress(openFlagAddress),QMainWindow(parent)
{
	ui.setupUi(this);

    setAttribute(Qt::WA_DeleteOnClose); //子窗体关掉时候可以调用析构函数

    open_Kinect_Flag = false;
    kinect = nullptr;
    kinect_show_thread = nullptr;
}

void KinectController::on_open_Kinect_Button_clicked() {
    if (open_Kinect_Flag == false) {

        //开启相机
        kinect = new KinectDK();
        if (!kinect->open_camera()) {
            delete kinect;
            kinect = nullptr;
            QMessageBox::warning(NULL, QStringLiteral("DK相机控制台提示"),
                QStringLiteral("未找到相机，请检查连接端口"), QMessageBox::Yes, QMessageBox::Yes);
            return;
        }

        ui.Kinect_color_image->setScaledContents(true); //自适应图片
        ui.Kinect_depth_image->setScaledContents(true);

        kinect_show_thread = new kinectShowThread(this, kinect);
        connect(kinect_show_thread, &kinectShowThread::change_kinect_frame, this, &KinectController::slot_updateUI_kinect_image, Qt::BlockingQueuedConnection);//子线程获取完毕则刷新主线程ui
        kinect_show_thread->start();

        open_Kinect_Flag = true;
        ui.open_Kinect_Button->setText(QStringLiteral("关闭Kinect相机"));
    }
    else {
        open_Kinect_Flag = false;

        ui.open_Kinect_Button->setText(QStringLiteral("开启Kinect相机"));

        //必须将子线程与主线程先断开connect否则无法正常回收线程
        disconnect(kinect_show_thread, &kinectShowThread::change_kinect_frame, this, &KinectController::slot_updateUI_kinect_image);

        //终止线程并回收线程
        kinect_show_thread->quit();
        kinect_show_thread->wait();
        delete kinect_show_thread;
        kinect_show_thread = nullptr;

        //关闭相机
        delete kinect;
        kinect = nullptr;

        ui.Kinect_color_image->clear();
        ui.Kinect_color_image->setText(QStringLiteral("彩色图像区域"));

        ui.Kinect_depth_image->clear();
        ui.Kinect_depth_image->setText(QStringLiteral("深度图像区域"));
    }
}

void KinectController::slot_updateUI_kinect_image(k4a::capture capture) {
    this->ui.Kinect_color_image->setPixmap(QPixmap::fromImage(capture_to_qimage_color(capture)));
    this->ui.Kinect_depth_image->setPixmap(QPixmap::fromImage(capture_to_qimage_depth(capture)));
}



void KinectController::on_readPCD_clicked() {

    QString path = QString(ui.pcd_filename_lineEdit->text());
    std::vector<QVector3D> cloud = ReadVec3PointCloudPCD(path);
    curPointCloud = cloud;
    ui.pointcloud_widget->showPointCloud(cloud);
    return;
}

void KinectController::on_show_cur_pointcloud_clicked() {
    if (open_Kinect_Flag) {
        double DK_minX = ui.DK_minX_lineEdit->text().toDouble();
        double DK_maxX = ui.DK_maxX_lineEdit->text().toDouble();
        double DK_minY = ui.DK_minY_lineEdit->text().toDouble();
        double DK_maxY = ui.DK_maxY_lineEdit->text().toDouble();
        double DK_minZ = ui.DK_minZ_lineEdit->text().toDouble();
        double DK_maxZ = ui.DK_maxZ_lineEdit->text().toDouble();
        curPointCloud = PclPointClouds2vecQvec3D(
                        *extract_region(kinect->get_point_cloud(),
                            DK_minX, DK_maxX,
                            DK_minY, DK_maxY,
                            DK_minZ, DK_maxZ)); //要保证在打开相机后才能get_point_cloud
        ui.pointcloud_widget->showPointCloud(curPointCloud);
    }
    else {
        QMessageBox::warning(NULL, QStringLiteral("DK相机控制台提示"),
            QStringLiteral("请先打开相机"), QMessageBox::Yes, QMessageBox::Yes);
    }
}

void KinectController::on_save_cur_pointcloud_clicked() {
    if (curPointCloud.empty())
        return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(curPointCloud.size());
    for (int i = 0; i < curPointCloud.size(); i++) {
        cloud->points[i].x = curPointCloud[i].x();
        cloud->points[i].y = curPointCloud[i].y();
        cloud->points[i].z = curPointCloud[i].z();
    }
    pcl::io::savePCDFileASCII(ui.save_filename_lineEdit->text().toStdString(), *cloud);
}


std::vector<QVector3D> KinectController::ReadVec3PointCloudPCD(QString path) {
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


std::vector<QVector3D> KinectController::PclPointClouds2vecQvec3D(const pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
    std::vector<QVector3D> ans;
    for (const auto& point : pointcloud)
        ans.push_back({ point.x , point.y , point.z });
    return ans;
}

KinectController::~KinectController()
{
    if(open_Kinect_Flag)
        on_open_Kinect_Button_clicked();
    *openFlagAddress = false;
}
