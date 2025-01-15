#include "getBase2CamController.h"

getBase2CamController::getBase2CamController(bool* isOpenBase2CamController,QWidget *parent)
	: isOpenBase2CamController(isOpenBase2CamController),QMainWindow(parent)
{
	ui.setupUi(this);

    setAttribute(Qt::WA_DeleteOnClose); //�Ӵ���ص�ʱ����Ե�����������
	
	//����ת�������ά�ȴ�С
	ui.Base2Cam_tableWidget->setColumnCount(4);
	ui.Base2Cam_tableWidget->setRowCount(4);
}


void getBase2CamController::on_outputBase2Cam_button_clicked() {

    // ��õ�ǰTableWidget�ؼ�������
    int rows = ui.Base2Cam_tableWidget->rowCount();
    int cols = ui.Base2Cam_tableWidget->columnCount();

    // ����һ��OpenCV��cv::Mat��������ΪCV_64F��double���ͣ�
    cv::Mat matrix = cv::Mat::zeros(rows, cols, CV_64F);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // �ȸ���������ȡQTableWidget�е�ÿһ��Item
            QTableWidgetItem* pItem = ui.Base2Cam_tableWidget->item(i, j);

            // �������û���κ����ݻ��ַ����룬pItem == nullptr
            if (pItem == nullptr) {
                QMessageBox::critical(this, "error", "input error!", QMessageBox::Ok);
                return;
            }

            // ��ȡItem���ı���Ϣ����ת��Ϊdouble����
            QString text = pItem->text();
            bool OK;
            double num = text.toDouble(&OK);
            if (OK == true) {
                matrix.at<double>(i, j) = num; // �������������
            }
            else { // ת��ʧ�� ��ʾ�������������
                QMessageBox::critical(this, "error", "input error!", QMessageBox::Ok);
                return;
            }
        }
    }

    updateBase2Cam(matrix);

    //�����ǲ��Դ���
    //// �������ľ���
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
