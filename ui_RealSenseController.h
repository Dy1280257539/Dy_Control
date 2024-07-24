/********************************************************************************
** Form generated from reading UI file 'RealSenseControllerDoefuJ.ui'
**
** Created by: Qt User Interface Compiler version 5.12.9
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef REALSENSECONTROLLERDOEFUJ_H
#define REALSENSECONTROLLERDOEFUJ_H

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

class Ui_RealSenseControllerClass
{
public:
    QWidget *centralWidget;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_8;
    QVBoxLayout *verticalLayout_2;
    QPushButton *open_RS_Button;
    QPushButton *show_cur_pointcloud;
    QGridLayout *gridLayout_18;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *save_cur_pointcloud;
    QLineEdit *save_filename_lineEdit;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *readPCD;
    QLineEdit *pcd_filename_lineEdit;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_26;
    QHBoxLayout *horizontalLayout_15;
    QLabel *label_7;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_12;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QLineEdit *rs1_minX_lineEdit;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_2;
    QLineEdit *rs1_maxX_lineEdit;
    QHBoxLayout *horizontalLayout_13;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_3;
    QLineEdit *rs1_minY_lineEdit;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_4;
    QLineEdit *rs1_maxY_lineEdit;
    QHBoxLayout *horizontalLayout_14;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_5;
    QLineEdit *rs1_minZ_lineEdit;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_6;
    QLineEdit *rs1_maxZ_lineEdit;
    QHBoxLayout *horizontalLayout_16;
    QLabel *label_8;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_17;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label_9;
    QLineEdit *rs2_minX_lineEdit;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_10;
    QLineEdit *rs2_maxX_lineEdit;
    QHBoxLayout *horizontalLayout_20;
    QHBoxLayout *horizontalLayout_21;
    QLabel *label_11;
    QLineEdit *rs2_minY_lineEdit;
    QHBoxLayout *horizontalLayout_22;
    QLabel *label_12;
    QLineEdit *rs2_maxY_lineEdit;
    QHBoxLayout *horizontalLayout_23;
    QHBoxLayout *horizontalLayout_24;
    QLabel *label_13;
    QLineEdit *rs2_minZ_lineEdit;
    QHBoxLayout *horizontalLayout_25;
    QLabel *label_14;
    QLineEdit *rs2_maxZ_lineEdit;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *RS1_color_image;
    QLabel *RS2_color_image;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer;
    MyQOpenglWidget *pointcloud_widget;
    QSpacerItem *horizontalSpacer_2;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *RealSenseControllerClass)
    {
        if (RealSenseControllerClass->objectName().isEmpty())
            RealSenseControllerClass->setObjectName(QString::fromUtf8("RealSenseControllerClass"));
        RealSenseControllerClass->resize(1123, 780);
        centralWidget = new QWidget(RealSenseControllerClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 10, 409, 71));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        open_RS_Button = new QPushButton(layoutWidget1);
        open_RS_Button->setObjectName(QString::fromUtf8("open_RS_Button"));
        open_RS_Button->setCheckable(false);

        verticalLayout_2->addWidget(open_RS_Button);

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
        layoutWidget->setGeometry(QRect(0, 90, 687, 654));
        verticalLayout_5 = new QVBoxLayout(layoutWidget);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_26 = new QHBoxLayout();
        horizontalLayout_26->setSpacing(6);
        horizontalLayout_26->setObjectName(QString::fromUtf8("horizontalLayout_26"));
        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_15->addWidget(label_7);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_3->addWidget(label);

        rs1_minX_lineEdit = new QLineEdit(layoutWidget);
        rs1_minX_lineEdit->setObjectName(QString::fromUtf8("rs1_minX_lineEdit"));

        horizontalLayout_3->addWidget(rs1_minX_lineEdit);


        horizontalLayout_12->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_4->addWidget(label_2);

        rs1_maxX_lineEdit = new QLineEdit(layoutWidget);
        rs1_maxX_lineEdit->setObjectName(QString::fromUtf8("rs1_maxX_lineEdit"));

        horizontalLayout_4->addWidget(rs1_maxX_lineEdit);


        horizontalLayout_12->addLayout(horizontalLayout_4);


        verticalLayout_3->addLayout(horizontalLayout_12);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_5->addWidget(label_3);

        rs1_minY_lineEdit = new QLineEdit(layoutWidget);
        rs1_minY_lineEdit->setObjectName(QString::fromUtf8("rs1_minY_lineEdit"));

        horizontalLayout_5->addWidget(rs1_minY_lineEdit);


        horizontalLayout_13->addLayout(horizontalLayout_5);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_9->addWidget(label_4);

        rs1_maxY_lineEdit = new QLineEdit(layoutWidget);
        rs1_maxY_lineEdit->setObjectName(QString::fromUtf8("rs1_maxY_lineEdit"));

        horizontalLayout_9->addWidget(rs1_maxY_lineEdit);


        horizontalLayout_13->addLayout(horizontalLayout_9);


        verticalLayout_3->addLayout(horizontalLayout_13);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_10->addWidget(label_5);

        rs1_minZ_lineEdit = new QLineEdit(layoutWidget);
        rs1_minZ_lineEdit->setObjectName(QString::fromUtf8("rs1_minZ_lineEdit"));

        horizontalLayout_10->addWidget(rs1_minZ_lineEdit);


        horizontalLayout_14->addLayout(horizontalLayout_10);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_11->addWidget(label_6);

        rs1_maxZ_lineEdit = new QLineEdit(layoutWidget);
        rs1_maxZ_lineEdit->setObjectName(QString::fromUtf8("rs1_maxZ_lineEdit"));

        horizontalLayout_11->addWidget(rs1_maxZ_lineEdit);


        horizontalLayout_14->addLayout(horizontalLayout_11);


        verticalLayout_3->addLayout(horizontalLayout_14);


        horizontalLayout_15->addLayout(verticalLayout_3);

        horizontalLayout_15->setStretch(0, 1);
        horizontalLayout_15->setStretch(1, 6);

        horizontalLayout_26->addLayout(horizontalLayout_15);

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

        rs2_minX_lineEdit = new QLineEdit(layoutWidget);
        rs2_minX_lineEdit->setObjectName(QString::fromUtf8("rs2_minX_lineEdit"));

        horizontalLayout_18->addWidget(rs2_minX_lineEdit);


        horizontalLayout_17->addLayout(horizontalLayout_18);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_19->addWidget(label_10);

        rs2_maxX_lineEdit = new QLineEdit(layoutWidget);
        rs2_maxX_lineEdit->setObjectName(QString::fromUtf8("rs2_maxX_lineEdit"));

        horizontalLayout_19->addWidget(rs2_maxX_lineEdit);


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

        rs2_minY_lineEdit = new QLineEdit(layoutWidget);
        rs2_minY_lineEdit->setObjectName(QString::fromUtf8("rs2_minY_lineEdit"));

        horizontalLayout_21->addWidget(rs2_minY_lineEdit);


        horizontalLayout_20->addLayout(horizontalLayout_21);

        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setSpacing(6);
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        label_12 = new QLabel(layoutWidget);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_22->addWidget(label_12);

        rs2_maxY_lineEdit = new QLineEdit(layoutWidget);
        rs2_maxY_lineEdit->setObjectName(QString::fromUtf8("rs2_maxY_lineEdit"));

        horizontalLayout_22->addWidget(rs2_maxY_lineEdit);


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

        rs2_minZ_lineEdit = new QLineEdit(layoutWidget);
        rs2_minZ_lineEdit->setObjectName(QString::fromUtf8("rs2_minZ_lineEdit"));

        horizontalLayout_24->addWidget(rs2_minZ_lineEdit);


        horizontalLayout_23->addLayout(horizontalLayout_24);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setSpacing(6);
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        label_14 = new QLabel(layoutWidget);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout_25->addWidget(label_14);

        rs2_maxZ_lineEdit = new QLineEdit(layoutWidget);
        rs2_maxZ_lineEdit->setObjectName(QString::fromUtf8("rs2_maxZ_lineEdit"));

        horizontalLayout_25->addWidget(rs2_maxZ_lineEdit);


        horizontalLayout_23->addLayout(horizontalLayout_25);


        verticalLayout_4->addLayout(horizontalLayout_23);


        horizontalLayout_16->addLayout(verticalLayout_4);

        horizontalLayout_16->setStretch(0, 1);
        horizontalLayout_16->setStretch(1, 6);

        horizontalLayout_26->addLayout(horizontalLayout_16);

        horizontalLayout_26->setStretch(0, 1);
        horizontalLayout_26->setStretch(1, 1);

        verticalLayout_5->addLayout(horizontalLayout_26);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        RS1_color_image = new QLabel(layoutWidget);
        RS1_color_image->setObjectName(QString::fromUtf8("RS1_color_image"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(RS1_color_image->sizePolicy().hasHeightForWidth());
        RS1_color_image->setSizePolicy(sizePolicy);
        RS1_color_image->setMinimumSize(QSize(320, 240));
        RS1_color_image->setMaximumSize(QSize(320, 240));
        RS1_color_image->setContextMenuPolicy(Qt::NoContextMenu);
        RS1_color_image->setLayoutDirection(Qt::LeftToRight);
        RS1_color_image->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n"
"font: 16pt \"Agency FB\";\n"
"border:2px groove gray;"));
        RS1_color_image->setLineWidth(1);
        RS1_color_image->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(RS1_color_image);

        RS2_color_image = new QLabel(layoutWidget);
        RS2_color_image->setObjectName(QString::fromUtf8("RS2_color_image"));
        sizePolicy.setHeightForWidth(RS2_color_image->sizePolicy().hasHeightForWidth());
        RS2_color_image->setSizePolicy(sizePolicy);
        RS2_color_image->setMinimumSize(QSize(320, 240));
        RS2_color_image->setMaximumSize(QSize(320, 240));
        RS2_color_image->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n"
"font: 16pt \"Agency FB\";\n"
"border:2px groove gray;"));
        RS2_color_image->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(RS2_color_image);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        pointcloud_widget = new MyQOpenglWidget(layoutWidget);
        pointcloud_widget->setObjectName(QString::fromUtf8("pointcloud_widget"));
        sizePolicy.setHeightForWidth(pointcloud_widget->sizePolicy().hasHeightForWidth());
        pointcloud_widget->setSizePolicy(sizePolicy);
        pointcloud_widget->setMinimumSize(QSize(512, 288));
        pointcloud_widget->setMaximumSize(QSize(512, 288));
        pointcloud_widget->setLayoutDirection(Qt::LeftToRight);

        horizontalLayout_2->addWidget(pointcloud_widget);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalLayout->setStretch(1, 1);

        verticalLayout_5->addLayout(verticalLayout);

        RealSenseControllerClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(RealSenseControllerClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        RealSenseControllerClass->setStatusBar(statusBar);

        retranslateUi(RealSenseControllerClass);

        QMetaObject::connectSlotsByName(RealSenseControllerClass);
    } // setupUi

    void retranslateUi(QMainWindow *RealSenseControllerClass)
    {
        RealSenseControllerClass->setWindowTitle(QApplication::translate("RealSenseControllerClass", "RealSenseController", nullptr));
        open_RS_Button->setText(QApplication::translate("RealSenseControllerClass", "\345\274\200\345\220\257RS\347\233\270\346\234\272", nullptr));
        show_cur_pointcloud->setText(QApplication::translate("RealSenseControllerClass", "\345\261\225\347\244\272\345\275\223\345\211\215\347\202\271\344\272\221", nullptr));
        save_cur_pointcloud->setText(QApplication::translate("RealSenseControllerClass", "\344\277\235\345\255\230\345\275\223\345\211\215\347\202\271\344\272\221", nullptr));
        save_filename_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "C:/Users/Dy/Desktop/", nullptr));
        readPCD->setText(QApplication::translate("RealSenseControllerClass", "\350\257\273\347\202\271\344\272\221\345\233\276", nullptr));
        pcd_filename_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "C:/Users/Dy/Desktop/", nullptr));
        label_7->setText(QApplication::translate("RealSenseControllerClass", "RS1_ROI:", nullptr));
        label->setText(QApplication::translate("RealSenseControllerClass", "RS1_minX:", nullptr));
        rs1_minX_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0.05", nullptr));
        label_2->setText(QApplication::translate("RealSenseControllerClass", "RS1_maxX:", nullptr));
        rs1_maxX_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0.2", nullptr));
        label_3->setText(QApplication::translate("RealSenseControllerClass", "RS1_minY:", nullptr));
        rs1_minY_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0.05", nullptr));
        label_4->setText(QApplication::translate("RealSenseControllerClass", "RS1_maxY:", nullptr));
        rs1_maxY_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0.2", nullptr));
        label_5->setText(QApplication::translate("RealSenseControllerClass", "RS1_minZ:", nullptr));
        rs1_minZ_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0.05", nullptr));
        label_6->setText(QApplication::translate("RealSenseControllerClass", "RS1_maxZ:", nullptr));
        rs1_maxZ_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0.2", nullptr));
        label_8->setText(QApplication::translate("RealSenseControllerClass", "RS2_ROI:", nullptr));
        label_9->setText(QApplication::translate("RealSenseControllerClass", "RS2_minX:", nullptr));
        rs2_minX_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0", nullptr));
        label_10->setText(QApplication::translate("RealSenseControllerClass", "RS2_maxX:", nullptr));
        rs2_maxX_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0", nullptr));
        label_11->setText(QApplication::translate("RealSenseControllerClass", "RS2_minY:", nullptr));
        rs2_minY_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0", nullptr));
        label_12->setText(QApplication::translate("RealSenseControllerClass", "RS2_maxY:", nullptr));
        rs2_maxY_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0", nullptr));
        label_13->setText(QApplication::translate("RealSenseControllerClass", "RS2_minZ:", nullptr));
        rs2_minZ_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0", nullptr));
        label_14->setText(QApplication::translate("RealSenseControllerClass", "RS2_maxZ:", nullptr));
        rs2_maxZ_lineEdit->setText(QApplication::translate("RealSenseControllerClass", "0", nullptr));
        RS1_color_image->setText(QApplication::translate("RealSenseControllerClass", "RS1\347\233\270\346\234\272\345\275\251\350\211\262\345\233\276\345\203\217\345\214\272\345\237\237", nullptr));
        RS2_color_image->setText(QApplication::translate("RealSenseControllerClass", "RS2\347\233\270\346\234\272\345\275\251\350\211\262\345\233\276\345\203\217\345\214\272\345\237\237", nullptr));
    } // retranslateUi

};

namespace Ui {
    class RealSenseControllerClass: public Ui_RealSenseControllerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // REALSENSECONTROLLERDOEFUJ_H
