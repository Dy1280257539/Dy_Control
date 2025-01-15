#include "getBase2CamController.h"

getBase2CamController::getBase2CamController(bool* isOpenBase2CamController,QWidget *parent)
	: isOpenBase2CamController(isOpenBase2CamController),QMainWindow(parent)
{
	ui.setupUi(this);

    setAttribute(Qt::WA_DeleteOnClose); //子窗体关掉时候可以调用析构函数
	
	//设置转换矩阵的维度大小
	ui.Base2Cam_tableWidget->setColumnCount(4);
	ui.Base2Cam_tableWidget->setRowCount(4);
}


void getBase2CamController::on_outputBase2Cam_button_clicked() {

    // 获得当前TableWidget控件的行列
    int rows = ui.Base2Cam_tableWidget->rowCount();
    int cols = ui.Base2Cam_tableWidget->columnCount();

    // 创建一个OpenCV的cv::Mat矩阵，类型为CV_64F（double类型）
    cv::Mat matrix = cv::Mat::zeros(rows, cols, CV_64F);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // 先根据索引获取QTableWidget中的每一项Item
            QTableWidgetItem* pItem = ui.Base2Cam_tableWidget->item(i, j);

            // 如果该项没有任何数据或字符填入，pItem == nullptr
            if (pItem == nullptr) {
                QMessageBox::critical(this, "error", "input error!", QMessageBox::Ok);
                return;
            }

            // 获取Item的文本信息，并转换为double数据
            QString text = pItem->text();
            bool OK;
            double num = text.toDouble(&OK);
            if (OK == true) {
                matrix.at<double>(i, j) = num; // 将数据填入矩阵
            }
            else { // 转换失败 表示输入的内容有误
                QMessageBox::critical(this, "error", "input error!", QMessageBox::Ok);
                return;
            }
        }
    }

    updateBase2Cam(matrix);

    //以下是测试代码
    //// 输出输入的矩阵
    //for (int i = 0; i < rows; i++) {
    //    QString line = "";
    //    for (int j = 0; j < cols; j++) {
    //        line += QString::number(matrix.at<double>(i, j)) + " ";
    //    }
    //    qDebug() << line;
    //}

    //qDebug() << endl;

    //auto tmp = getBase2Cam();

    //for (int i = 0; i < tmp.size(); i++) {
    //    qDebug() << tmp[i] << " ";
    //}

}


getBase2CamController::~getBase2CamController()
{
    *isOpenBase2CamController = false;
}
