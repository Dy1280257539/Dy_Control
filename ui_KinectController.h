/********************************************************************************
** Form generated from reading UI file 'KinectControlleriHnUTB.ui'
**
** Created by: Qt User Interface Compiler version 5.12.9
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef KINECTCONTROLLERIHNUTB_H
#define KINECTCONTROLLERIHNUTB_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "myqopenglwidget.h"

QT_BEGIN_NAMESPACE

class Ui_KinectControllerClass
{
public:
    QWidget *centralWidget;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_8;
    QVBoxLayout *verticalLayout_2;
    QPushButton *open_Kinect_Button;
    QPushButton *show_cur_pointcloud;
    QGridLayout *gridLayout_18;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *save_cur_pointcloud;
    QLineEdit *save_filename_lineEdit;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *readPCD;
    QLineEdit *pcd_filename_lineEdit;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_26;
    QHBoxLayout *horizontalLayout_16;
    QLabel *label_8;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_17;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label_9;
    QLineEdit *DK_minX_lineEdit;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_10;
    QLineEdit *DK_maxX_lineEdit;
    QHBoxLayout *horizontalLayout_20;
    QHBoxLayout *horizontalLayout_21;
    QLabel *label_11;
    QLineEdit *DK_minY_lineEdit;
    QHBoxLayout *horizontalLayout_22;
    QLabel *label_12;
    QLineEdit *DK_maxY_lineEdit;
    QHBoxLayout *horizontalLayout_23;
    QHBoxLayout *horizontalLayout_24;
    QLabel *label_13;
    QLineEdit *DK_minZ_lineEdit;
    QHBoxLayout *horizontalLayout_25;
    QLabel *label_14;
    QLineEdit *DK_maxZ_lineEdit;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *Kinect_color_image;
    QLabel *Kinect_depth_image;
    MyQOpenglWidget *pointcloud_widget;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_16;
    QSpacerItem *horizontalSpacer;
    QLabel *label_15;
    QLineEdit *cur_data_num_lineEdit;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *get_frame_and_pose_Button;
    QPushButton *save_all_data_Button;
    QPushButton *clear_all_data_Button;
    QLabel *label_17;
    QLineEdit *save_caldata_filename_lineEdit;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *KinectControllerClass)
    {
        if (KinectControllerClass->objectName().isEmpty())
            KinectControllerClass->setObjectName(QString::fromUtf8("KinectControllerClass"));
        KinectControllerClass->resize(1229, 851);
        centralWidget = new QWidget(KinectControllerClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(20, 10, 409, 71));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        open_Kinect_Button = new QPushButton(layoutWidget1);
        open_Kinect_Button->setObjectName(QString::fromUtf8("open_Kinect_Button"));
        open_Kinect_Button->setCheckable(false);

        verticalLayout_2->addWidget(open_Kinect_Button);

        show_cur_pointcloud = new QPushButton(layoutWidget1);
        show_cur_pointcloud->setObjectName(QString::fromUtf8("show_cur_pointcloud"));
        show_cur_pointcloud->setCheckable(false);

        verticalLayout_2->addWidget(show_cur_pointcloud);


        horizontalLayout_8->addLayout(verticalLayout_2);

        gridLayout_18 = new QGridLayout();
        gridLayout_18->setSpacing(6);
        gridLayout_18->setObjectName(QString::fromUtf8("gridLayout_18"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        save_cur_pointcloud = new QPushButton(layoutWidget1);
        save_cur_pointcloud->setObjectName(QString::fromUtf8("save_cur_pointcloud"));
        save_cur_pointcloud->setCheckable(false);

        horizontalLayout_6->addWidget(save_cur_pointcloud);

        save_filename_lineEdit = new QLineEdit(layoutWidget1);
        save_filename_lineEdit->setObjectName(QString::fromUtf8("save_filename_lineEdit"));

        horizontalLayout_6->addWidget(save_filename_lineEdit);


        gridLayout_18->addLayout(horizontalLayout_6, 1, 0, 1, 1);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        readPCD = new QPushButton(layoutWidget1);
        readPCD->setObjectName(QString::fromUtf8("readPCD"));
        readPCD->setCheckable(false);

        horizontalLayout_7->addWidget(readPCD);

        pcd_filename_lineEdit = new QLineEdit(layoutWidget1);
        pcd_filename_lineEdit->setObjectName(QString::fromUtf8("pcd_filename_lineEdit"));

        horizontalLayout_7->addWidget(pcd_filename_lineEdit);


        gridLayout_18->addLayout(horizontalLayout_7, 0, 0, 1, 1);


        horizontalLayout_8->addLayout(gridLayout_18);

        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 160, 525, 556));
        verticalLayout_3 = new QVBoxLayout(layoutWidget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_26 = new QHBoxLayout();
        horizontalLayout_26->setSpacing(6);
        horizontalLayout_26->setObjectName(QString::fromUtf8("horizontalLayout_26"));
        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setSpacing(6);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_16->addWidget(label_8);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_18->addWidget(label_9);

        DK_minX_lineEdit = new QLineEdit(layoutWidget);
        DK_minX_lineEdit->setObjectName(QString::fromUtf8("DK_minX_lineEdit"));

        horizontalLayout_18->addWidget(DK_minX_lineEdit);


        horizontalLayout_17->addLayout(horizontalLayout_18);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_19->addWidget(label_10);

        DK_maxX_lineEdit = new QLineEdit(layoutWidget);
        DK_maxX_lineEdit->setObjectName(QString::fromUtf8("DK_maxX_lineEdit"));

        horizontalLayout_19->addWidget(DK_maxX_lineEdit);


        horizontalLayout_17->addLayout(horizontalLayout_19);


        verticalLayout_4->addLayout(horizontalLayout_17);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setSpacing(6);
        horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
        label_11 = new QLabel(layoutWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        horizontalLayout_21->addWidget(label_11);

        DK_minY_lineEdit = new QLineEdit(layoutWidget);
        DK_minY_lineEdit->setObjectName(QString::fromUtf8("DK_minY_lineEdit"));

        horizontalLayout_21->addWidget(DK_minY_lineEdit);


        horizontalLayout_20->addLayout(horizontalLayout_21);

        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setSpacing(6);
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        label_12 = new QLabel(layoutWidget);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_22->addWidget(label_12);

        DK_maxY_lineEdit = new QLineEdit(layoutWidget);
        DK_maxY_lineEdit->setObjectName(QString::fromUtf8("DK_maxY_lineEdit"));

        horizontalLayout_22->addWidget(DK_maxY_lineEdit);


        horizontalLayout_20->addLayout(horizontalLayout_22);


        verticalLayout_4->addLayout(horizontalLayout_20);

        horizontalLayout_23 = new QHBoxLayout();
        horizontalLayout_23->setSpacing(6);
        horizontalLayout_23->setObjectName(QString::fromUtf8("horizontalLayout_23"));
        horizontalLayout_24 = new QHBoxLayout();
        horizontalLayout_24->setSpacing(6);
        horizontalLayout_24->setObjectName(QString::fromUtf8("horizontalLayout_24"));
        label_13 = new QLabel(layoutWidget);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        horizontalLayout_24->addWidget(label_13);

        DK_minZ_lineEdit = new QLineEdit(layoutWidget);
        DK_minZ_lineEdit->setObjectName(QString::fromUtf8("DK_minZ_lineEdit"));

        horizontalLayout_24->addWidget(DK_minZ_lineEdit);


        horizontalLayout_23->addLayout(horizontalLayout_24);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setSpacing(6);
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        label_14 = new QLabel(layoutWidget);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout_25->addWidget(label_14);

        DK_maxZ_lineEdit = new QLineEdit(layoutWidget);
        DK_maxZ_lineEdit->setObjectName(QString::fromUtf8("DK_maxZ_lineEdit"));

        horizontalLayout_25->addWidget(DK_maxZ_lineEdit);


        horizontalLayout_23->addLayout(horizontalLayout_25);


        verticalLayout_4->addLayout(horizontalLayout_23);


        horizontalLayout_16->addLayout(verticalLayout_4);

        horizontalLayout_16->setStretch(0, 1);
        horizontalLayout_16->setStretch(1, 6);

        horizontalLayout_26->addLayout(horizontalLayout_16);

        horizontalLayout_26->setStretch(0, 1);

        verticalLayout_3->addLayout(horizontalLayout_26);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        Kinect_color_image = new QLabel(layoutWidget);
        Kinect_color_image->setObjectName(QString::fromUtf8("Kinect_color_image"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Kinect_color_image->sizePolicy().hasHeightForWidth());
        Kinect_color_image->setSizePolicy(sizePolicy);
        Kinect_color_image->setMinimumSize(QSize(256, 144));
        Kinect_color_image->setMaximumSize(QSize(256, 144));
        Kinect_color_image->setContextMenuPolicy(Qt::NoContextMenu);
        Kinect_color_image->setLayoutDirection(Qt::LeftToRight);
        Kinect_color_image->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n"
"font: 16pt \"Agency FB\";\n"
"border:2px groove gray;"));
        Kinect_color_image->setLineWidth(1);
        Kinect_color_image->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(Kinect_color_image);

        Kinect_depth_image = new QLabel(layoutWidget);
        Kinect_depth_image->setObjectName(QString::fromUtf8("Kinect_depth_image"));
        sizePolicy.setHeightForWidth(Kinect_depth_image->sizePolicy().hasHeightForWidth());
        Kinect_depth_image->setSizePolicy(sizePolicy);
        Kinect_depth_image->setMinimumSize(QSize(256, 144));
        Kinect_depth_image->setMaximumSize(QSize(256, 144));
        Kinect_depth_image->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n"
"font: 16pt \"Agency FB\";\n"
"border:2px groove gray;"));
        Kinect_depth_image->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(Kinect_depth_image);


        verticalLayout->addLayout(horizontalLayout);

        pointcloud_widget = new MyQOpenglWidget(layoutWidget);
        pointcloud_widget->setObjectName(QString::fromUtf8("pointcloud_widget"));
        sizePolicy.setHeightForWidth(pointcloud_widget->sizePolicy().hasHeightForWidth());
        pointcloud_widget->setSizePolicy(sizePolicy);
        pointcloud_widget->setMinimumSize(QSize(512, 288));
        pointcloud_widget->setMaximumSize(QSize(512, 288));

        verticalLayout->addWidget(pointcloud_widget);


        verticalLayout_3->addLayout(verticalLayout);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(20, 90, 521, 65));
        verticalLayout_5 = new QVBoxLayout(layoutWidget2);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_16 = new QLabel(layoutWidget2);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        horizontalLayout_2->addWidget(label_16);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        label_15 = new QLabel(layoutWidget2);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        horizontalLayout_2->addWidget(label_15);

        cur_data_num_lineEdit = new QLineEdit(layoutWidget2);
        cur_data_num_lineEdit->setObjectName(QString::fromUtf8("cur_data_num_lineEdit"));

        horizontalLayout_2->addWidget(cur_data_num_lineEdit);

        horizontalLayout_2->setStretch(0, 1);
        horizontalLayout_2->setStretch(1, 1);
        horizontalLayout_2->setStretch(2, 1);
        horizontalLayout_2->setStretch(3, 1);

        verticalLayout_5->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        get_frame_and_pose_Button = new QPushButton(layoutWidget2);
        get_frame_and_pose_Button->setObjectName(QString::fromUtf8("get_frame_and_pose_Button"));
        get_frame_and_pose_Button->setCheckable(false);

        horizontalLayout_3->addWidget(get_frame_and_pose_Button);

        save_all_data_Button = new QPushButton(layoutWidget2);
        save_all_data_Button->setObjectName(QString::fromUtf8("save_all_data_Button"));
        save_all_data_Button->setCheckable(false);

        horizontalLayout_3->addWidget(save_all_data_Button);

        clear_all_data_Button = new QPushButton(layoutWidget2);
        clear_all_data_Button->setObjectName(QString::fromUtf8("clear_all_data_Button"));
        clear_all_data_Button->setCheckable(false);

        horizontalLayout_3->addWidget(clear_all_data_Button);

        label_17 = new QLabel(layoutWidget2);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        horizontalLayout_3->addWidget(label_17);

        save_caldata_filename_lineEdit = new QLineEdit(layoutWidget2);
        save_caldata_filename_lineEdit->setObjectName(QString::fromUtf8("save_caldata_filename_lineEdit"));

        horizontalLayout_3->addWidget(save_caldata_filename_lineEdit);


        verticalLayout_5->addLayout(horizontalLayout_3);

        KinectControllerClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(KinectControllerClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        KinectControllerClass->setStatusBar(statusBar);

        retranslateUi(KinectControllerClass);

        QMetaObject::connectSlotsByName(KinectControllerClass);
    } // setupUi

    void retranslateUi(QMainWindow *KinectControllerClass)
    {
        KinectControllerClass->setWindowTitle(QApplication::translate("KinectControllerClass", "KinectController", nullptr));
        open_Kinect_Button->setText(QApplication::translate("KinectControllerClass", "\345\274\200\345\220\257Kinect\347\233\270\346\234\272", nullptr));
        show_cur_pointcloud->setText(QApplication::translate("KinectControllerClass", "\345\261\225\347\244\272\345\275\223\345\211\215\347\202\271\344\272\221", nullptr));
        save_cur_pointcloud->setText(QApplication::translate("KinectControllerClass", "\344\277\235\345\255\230\345\275\223\345\211\215\347\202\271\344\272\221", nullptr));
        save_filename_lineEdit->setText(QApplication::translate("KinectControllerClass", "C:/Users/Dy/Desktop/", nullptr));
        readPCD->setText(QApplication::translate("KinectControllerClass", "\350\257\273\347\202\271\344\272\221\345\233\276", nullptr));
        pcd_filename_lineEdit->setText(QApplication::translate("KinectControllerClass", "C:/Users/Dy/Desktop/", nullptr));
        label_8->setText(QApplication::translate("KinectControllerClass", "DK_ROI:", nullptr));
        label_9->setText(QApplication::translate("KinectControllerClass", "DK_minX:", nullptr));
        DK_minX_lineEdit->setText(QApplication::translate("KinectControllerClass", "0.05", nullptr));
        label_10->setText(QApplication::translate("KinectControllerClass", "DK_maxX:", nullptr));
        DK_maxX_lineEdit->setText(QApplication::translate("KinectControllerClass", "0.2", nullptr));
        label_11->setText(QApplication::translate("KinectControllerClass", "DK_minY:", nullptr));
        DK_minY_lineEdit->setText(QApplication::translate("KinectControllerClass", "0.05", nullptr));
        label_12->setText(QApplication::translate("KinectControllerClass", "DK_maxY:", nullptr));
        DK_maxY_lineEdit->setText(QApplication::translate("KinectControllerClass", "0.2", nullptr));
        label_13->setText(QApplication::translate("KinectControllerClass", "DK_minZ:", nullptr));
        DK_minZ_lineEdit->setText(QApplication::translate("KinectControllerClass", "0.05", nullptr));
        label_14->setText(QApplication::translate("KinectControllerClass", "DK_maxZ:", nullptr));
        DK_maxZ_lineEdit->setText(QApplication::translate("KinectControllerClass", "0.2", nullptr));
        Kinect_color_image->setText(QApplication::translate("KinectControllerClass", "\345\275\251\350\211\262\345\233\276\345\203\217\345\214\272\345\237\237", nullptr));
        Kinect_depth_image->setText(QApplication::translate("KinectControllerClass", "\346\267\261\345\272\246\345\233\276\345\203\217\345\214\272\345\237\237", nullptr));
        label_16->setText(QApplication::translate("KinectControllerClass", "\346\211\213\347\234\274\346\240\207\345\256\232(\344\270\200\345\270\247\346\225\260\346\215\256\345\220\253\345\233\276\345\203\217\345\222\214\345\247\277\346\200\201)\357\274\232", nullptr));
        label_15->setText(QApplication::translate("KinectControllerClass", "\345\275\223\345\211\215\346\225\260\346\215\256\346\225\260\357\274\232", nullptr));
        cur_data_num_lineEdit->setText(QApplication::translate("KinectControllerClass", "0", nullptr));
        get_frame_and_pose_Button->setText(QApplication::translate("KinectControllerClass", "\350\216\267\345\217\226\345\275\223\345\211\215\346\225\260\346\215\256", nullptr));
        save_all_data_Button->setText(QApplication::translate("KinectControllerClass", "\344\277\235\345\255\230\346\211\200\346\234\211\346\225\260\346\215\256", nullptr));
        clear_all_data_Button->setText(QApplication::translate("KinectControllerClass", "\346\270\205\347\251\272\346\211\200\346\234\211\346\225\260\346\215\256", nullptr));
        label_17->setText(QApplication::translate("KinectControllerClass", "\344\277\235\345\255\230\345\234\260\345\235\200\357\274\232", nullptr));
        save_caldata_filename_lineEdit->setText(QApplication::translate("KinectControllerClass", "F:\\DESKAPPPLACE\\DOCUMENT\\rtde\\Dy_Control\\calibrationDataSaveDir", nullptr));
    } // retranslateUi

};

namespace Ui {
    class KinectControllerClass: public Ui_KinectControllerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // KINECTCONTROLLERIHNUTB_H
