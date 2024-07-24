/********************************************************************************
** Form generated from reading UI file 'forceSenseControllerLweUJL.ui'
**
** Created by: Qt User Interface Compiler version 5.12.9
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef FORCESENSECONTROLLERLWEUJL_H
#define FORCESENSECONTROLLERLWEUJL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qchartview.h"

QT_BEGIN_NAMESPACE

class Ui_forceSenseControllerClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_2;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QPushButton *readForceAndShowButton;
    QPushButton *zeroSenseButton;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QChartView *Fx_show;
    QSpacerItem *horizontalSpacer;
    QChartView *Fy_show;
    QSpacerItem *horizontalSpacer_2;
    QChartView *Fz_show;
    QHBoxLayout *horizontalLayout;
    QChartView *Mx_show;
    QSpacerItem *horizontalSpacer_3;
    QChartView *My_show;
    QSpacerItem *horizontalSpacer_4;
    QChartView *Mz_show;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_26;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *get_once_Button;
    QPushButton *caculate_Button;
    QPushButton *clear_Button;
    QLabel *label_16;
    QLineEdit *curNum_lineEdit;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_8;
    QLineEdit *G_robot_lineEdit;
    QLabel *label_9;
    QLineEdit *dx_robot_lineEdit;
    QLabel *label_10;
    QLineEdit *dy_robot_lineEdit;
    QLabel *label_11;
    QLineEdit *dz_robot_lineEdit;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_12;
    QLineEdit *G_real_lineEdit;
    QLabel *label_13;
    QLineEdit *dx_real_lineEdit;
    QLabel *label_14;
    QLineEdit *dy_real_lineEdit;
    QLabel *label_15;
    QLineEdit *dz_real_lineEdit;
    QWidget *layoutWidget_2;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_27;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *get_Compensation_Button;
    QPushButton *caculate_Compensation_Button;
    QPushButton *clear_Compensation_Button;
    QPushButton *cancel_Compensation_Button;
    QLabel *label_17;
    QLineEdit *curNum_Compensation_lineEdit;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_18;
    QLineEdit *M_e_x5_lineEdit;
    QLabel *label_19;
    QLineEdit *M_e_y5_lineEdit;
    QLabel *label_20;
    QLineEdit *M_e_z5_lineEdit;
    QLabel *label_21;
    QLineEdit *M_x0_lineEdit;
    QLabel *label_23;
    QLineEdit *M_y0_lineEdit;
    QLabel *label_22;
    QLineEdit *M_z0_lineEdit;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *forceSenseControllerClass)
    {
        if (forceSenseControllerClass->objectName().isEmpty())
            forceSenseControllerClass->setObjectName(QString::fromUtf8("forceSenseControllerClass"));
        forceSenseControllerClass->resize(1121, 703);
        centralWidget = new QWidget(forceSenseControllerClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout_2 = new QGridLayout(centralWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        scrollArea = new QScrollArea(centralWidget);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 1076, 2000));
        scrollAreaWidgetContents->setMinimumSize(QSize(500, 2000));
        readForceAndShowButton = new QPushButton(scrollAreaWidgetContents);
        readForceAndShowButton->setObjectName(QString::fromUtf8("readForceAndShowButton"));
        readForceAndShowButton->setGeometry(QRect(20, 10, 161, 28));
        zeroSenseButton = new QPushButton(scrollAreaWidgetContents);
        zeroSenseButton->setObjectName(QString::fromUtf8("zeroSenseButton"));
        zeroSenseButton->setGeometry(QRect(200, 10, 111, 28));
        layoutWidget = new QWidget(scrollAreaWidgetContents);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 310, 1011, 501));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        Fx_show = new QChartView(layoutWidget);
        Fx_show->setObjectName(QString::fromUtf8("Fx_show"));

        horizontalLayout_2->addWidget(Fx_show);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        Fy_show = new QChartView(layoutWidget);
        Fy_show->setObjectName(QString::fromUtf8("Fy_show"));

        horizontalLayout_2->addWidget(Fy_show);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        Fz_show = new QChartView(layoutWidget);
        Fz_show->setObjectName(QString::fromUtf8("Fz_show"));

        horizontalLayout_2->addWidget(Fz_show);

        horizontalLayout_2->setStretch(0, 5);
        horizontalLayout_2->setStretch(1, 1);
        horizontalLayout_2->setStretch(2, 5);
        horizontalLayout_2->setStretch(3, 1);
        horizontalLayout_2->setStretch(4, 5);

        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        Mx_show = new QChartView(layoutWidget);
        Mx_show->setObjectName(QString::fromUtf8("Mx_show"));

        horizontalLayout->addWidget(Mx_show);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        My_show = new QChartView(layoutWidget);
        My_show->setObjectName(QString::fromUtf8("My_show"));

        horizontalLayout->addWidget(My_show);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_4);

        Mz_show = new QChartView(layoutWidget);
        Mz_show->setObjectName(QString::fromUtf8("Mz_show"));

        horizontalLayout->addWidget(Mz_show);

        horizontalLayout->setStretch(0, 5);
        horizontalLayout->setStretch(1, 1);
        horizontalLayout->setStretch(2, 5);
        horizontalLayout->setStretch(3, 1);
        horizontalLayout->setStretch(4, 5);

        verticalLayout->addLayout(horizontalLayout);

        layoutWidget1 = new QWidget(scrollAreaWidgetContents);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(20, 60, 691, 120));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_26 = new QLabel(layoutWidget1);
        label_26->setObjectName(QString::fromUtf8("label_26"));
        label_26->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_26);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        get_once_Button = new QPushButton(layoutWidget1);
        get_once_Button->setObjectName(QString::fromUtf8("get_once_Button"));

        horizontalLayout_5->addWidget(get_once_Button);

        caculate_Button = new QPushButton(layoutWidget1);
        caculate_Button->setObjectName(QString::fromUtf8("caculate_Button"));

        horizontalLayout_5->addWidget(caculate_Button);

        clear_Button = new QPushButton(layoutWidget1);
        clear_Button->setObjectName(QString::fromUtf8("clear_Button"));

        horizontalLayout_5->addWidget(clear_Button);

        label_16 = new QLabel(layoutWidget1);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_5->addWidget(label_16);

        curNum_lineEdit = new QLineEdit(layoutWidget1);
        curNum_lineEdit->setObjectName(QString::fromUtf8("curNum_lineEdit"));

        horizontalLayout_5->addWidget(curNum_lineEdit);

        horizontalLayout_5->setStretch(0, 1);
        horizontalLayout_5->setStretch(1, 1);
        horizontalLayout_5->setStretch(2, 1);
        horizontalLayout_5->setStretch(3, 1);
        horizontalLayout_5->setStretch(4, 1);

        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_8 = new QLabel(layoutWidget1);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_3->addWidget(label_8);

        G_robot_lineEdit = new QLineEdit(layoutWidget1);
        G_robot_lineEdit->setObjectName(QString::fromUtf8("G_robot_lineEdit"));

        horizontalLayout_3->addWidget(G_robot_lineEdit);

        label_9 = new QLabel(layoutWidget1);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_3->addWidget(label_9);

        dx_robot_lineEdit = new QLineEdit(layoutWidget1);
        dx_robot_lineEdit->setObjectName(QString::fromUtf8("dx_robot_lineEdit"));

        horizontalLayout_3->addWidget(dx_robot_lineEdit);

        label_10 = new QLabel(layoutWidget1);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_3->addWidget(label_10);

        dy_robot_lineEdit = new QLineEdit(layoutWidget1);
        dy_robot_lineEdit->setObjectName(QString::fromUtf8("dy_robot_lineEdit"));

        horizontalLayout_3->addWidget(dy_robot_lineEdit);

        label_11 = new QLabel(layoutWidget1);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        horizontalLayout_3->addWidget(label_11);

        dz_robot_lineEdit = new QLineEdit(layoutWidget1);
        dz_robot_lineEdit->setObjectName(QString::fromUtf8("dz_robot_lineEdit"));

        horizontalLayout_3->addWidget(dz_robot_lineEdit);

        horizontalLayout_3->setStretch(0, 1);
        horizontalLayout_3->setStretch(1, 5);
        horizontalLayout_3->setStretch(2, 1);
        horizontalLayout_3->setStretch(3, 5);
        horizontalLayout_3->setStretch(4, 1);
        horizontalLayout_3->setStretch(5, 5);
        horizontalLayout_3->setStretch(6, 1);
        horizontalLayout_3->setStretch(7, 5);

        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_12 = new QLabel(layoutWidget1);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_4->addWidget(label_12);

        G_real_lineEdit = new QLineEdit(layoutWidget1);
        G_real_lineEdit->setObjectName(QString::fromUtf8("G_real_lineEdit"));

        horizontalLayout_4->addWidget(G_real_lineEdit);

        label_13 = new QLabel(layoutWidget1);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        horizontalLayout_4->addWidget(label_13);

        dx_real_lineEdit = new QLineEdit(layoutWidget1);
        dx_real_lineEdit->setObjectName(QString::fromUtf8("dx_real_lineEdit"));

        horizontalLayout_4->addWidget(dx_real_lineEdit);

        label_14 = new QLabel(layoutWidget1);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout_4->addWidget(label_14);

        dy_real_lineEdit = new QLineEdit(layoutWidget1);
        dy_real_lineEdit->setObjectName(QString::fromUtf8("dy_real_lineEdit"));

        horizontalLayout_4->addWidget(dy_real_lineEdit);

        label_15 = new QLabel(layoutWidget1);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        horizontalLayout_4->addWidget(label_15);

        dz_real_lineEdit = new QLineEdit(layoutWidget1);
        dz_real_lineEdit->setObjectName(QString::fromUtf8("dz_real_lineEdit"));

        horizontalLayout_4->addWidget(dz_real_lineEdit);

        horizontalLayout_4->setStretch(0, 1);
        horizontalLayout_4->setStretch(1, 5);
        horizontalLayout_4->setStretch(2, 1);
        horizontalLayout_4->setStretch(3, 5);
        horizontalLayout_4->setStretch(4, 1);
        horizontalLayout_4->setStretch(5, 5);
        horizontalLayout_4->setStretch(6, 1);
        horizontalLayout_4->setStretch(7, 5);

        verticalLayout_2->addLayout(horizontalLayout_4);

        layoutWidget_2 = new QWidget(scrollAreaWidgetContents);
        layoutWidget_2->setObjectName(QString::fromUtf8("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(20, 190, 721, 101));
        verticalLayout_3 = new QVBoxLayout(layoutWidget_2);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_27 = new QLabel(layoutWidget_2);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(label_27);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        get_Compensation_Button = new QPushButton(layoutWidget_2);
        get_Compensation_Button->setObjectName(QString::fromUtf8("get_Compensation_Button"));

        horizontalLayout_6->addWidget(get_Compensation_Button);

        caculate_Compensation_Button = new QPushButton(layoutWidget_2);
        caculate_Compensation_Button->setObjectName(QString::fromUtf8("caculate_Compensation_Button"));

        horizontalLayout_6->addWidget(caculate_Compensation_Button);

        clear_Compensation_Button = new QPushButton(layoutWidget_2);
        clear_Compensation_Button->setObjectName(QString::fromUtf8("clear_Compensation_Button"));

        horizontalLayout_6->addWidget(clear_Compensation_Button);

        cancel_Compensation_Button = new QPushButton(layoutWidget_2);
        cancel_Compensation_Button->setObjectName(QString::fromUtf8("cancel_Compensation_Button"));

        horizontalLayout_6->addWidget(cancel_Compensation_Button);

        label_17 = new QLabel(layoutWidget_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_6->addWidget(label_17);

        curNum_Compensation_lineEdit = new QLineEdit(layoutWidget_2);
        curNum_Compensation_lineEdit->setObjectName(QString::fromUtf8("curNum_Compensation_lineEdit"));

        horizontalLayout_6->addWidget(curNum_Compensation_lineEdit);

        horizontalLayout_6->setStretch(0, 1);
        horizontalLayout_6->setStretch(1, 1);
        horizontalLayout_6->setStretch(2, 1);
        horizontalLayout_6->setStretch(4, 1);
        horizontalLayout_6->setStretch(5, 1);

        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_18 = new QLabel(layoutWidget_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        horizontalLayout_7->addWidget(label_18);

        M_e_x5_lineEdit = new QLineEdit(layoutWidget_2);
        M_e_x5_lineEdit->setObjectName(QString::fromUtf8("M_e_x5_lineEdit"));

        horizontalLayout_7->addWidget(M_e_x5_lineEdit);

        label_19 = new QLabel(layoutWidget_2);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        horizontalLayout_7->addWidget(label_19);

        M_e_y5_lineEdit = new QLineEdit(layoutWidget_2);
        M_e_y5_lineEdit->setObjectName(QString::fromUtf8("M_e_y5_lineEdit"));

        horizontalLayout_7->addWidget(M_e_y5_lineEdit);

        label_20 = new QLabel(layoutWidget_2);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        horizontalLayout_7->addWidget(label_20);

        M_e_z5_lineEdit = new QLineEdit(layoutWidget_2);
        M_e_z5_lineEdit->setObjectName(QString::fromUtf8("M_e_z5_lineEdit"));

        horizontalLayout_7->addWidget(M_e_z5_lineEdit);

        label_21 = new QLabel(layoutWidget_2);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        horizontalLayout_7->addWidget(label_21);

        M_x0_lineEdit = new QLineEdit(layoutWidget_2);
        M_x0_lineEdit->setObjectName(QString::fromUtf8("M_x0_lineEdit"));

        horizontalLayout_7->addWidget(M_x0_lineEdit);

        label_23 = new QLabel(layoutWidget_2);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        horizontalLayout_7->addWidget(label_23);

        M_y0_lineEdit = new QLineEdit(layoutWidget_2);
        M_y0_lineEdit->setObjectName(QString::fromUtf8("M_y0_lineEdit"));

        horizontalLayout_7->addWidget(M_y0_lineEdit);

        label_22 = new QLabel(layoutWidget_2);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        horizontalLayout_7->addWidget(label_22);

        M_z0_lineEdit = new QLineEdit(layoutWidget_2);
        M_z0_lineEdit->setObjectName(QString::fromUtf8("M_z0_lineEdit"));

        horizontalLayout_7->addWidget(M_z0_lineEdit);

        horizontalLayout_7->setStretch(0, 1);
        horizontalLayout_7->setStretch(1, 5);
        horizontalLayout_7->setStretch(2, 1);
        horizontalLayout_7->setStretch(3, 5);
        horizontalLayout_7->setStretch(4, 1);
        horizontalLayout_7->setStretch(5, 5);

        verticalLayout_3->addLayout(horizontalLayout_7);

        scrollArea->setWidget(scrollAreaWidgetContents);

        gridLayout_2->addWidget(scrollArea, 0, 0, 1, 1);

        forceSenseControllerClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(forceSenseControllerClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        forceSenseControllerClass->setStatusBar(statusBar);

        retranslateUi(forceSenseControllerClass);

        QMetaObject::connectSlotsByName(forceSenseControllerClass);
    } // setupUi

    void retranslateUi(QMainWindow *forceSenseControllerClass)
    {
        forceSenseControllerClass->setWindowTitle(QApplication::translate("forceSenseControllerClass", "forceSenseController", nullptr));
        readForceAndShowButton->setText(QApplication::translate("forceSenseControllerClass", "\350\257\273\345\217\226\345\271\266\346\230\276\347\244\272\345\205\255\347\273\264\345\212\233", nullptr));
        zeroSenseButton->setText(QApplication::translate("forceSenseControllerClass", "\344\274\240\346\204\237\345\231\250\346\270\205\351\233\266", nullptr));
        label_26->setText(QApplication::translate("forceSenseControllerClass", "\344\272\214\346\254\241\351\207\215\345\212\233\350\241\245\345\201\277\347\233\270\345\205\263(\345\220\253'\344\273\243\350\241\250\347\244\272\346\225\231\345\231\250\347\244\272\346\225\260)\357\274\232", nullptr));
        get_once_Button->setText(QApplication::translate("forceSenseControllerClass", "\350\216\267\345\217\226\345\275\223\345\211\215\344\275\215\345\247\277\345\212\233\344\277\241\346\201\257", nullptr));
        caculate_Button->setText(QApplication::translate("forceSenseControllerClass", "\350\256\241\347\256\227\347\234\237\345\256\236\351\207\215\345\212\233", nullptr));
        clear_Button->setText(QApplication::translate("forceSenseControllerClass", "\346\270\205\347\251\272\345\275\223\345\211\215\344\275\215\345\247\277\346\225\260", nullptr));
        label_16->setText(QApplication::translate("forceSenseControllerClass", "\345\275\223\345\211\215\345\267\262\346\234\211\344\275\215\345\247\277\346\225\260:", nullptr));
        label_8->setText(QApplication::translate("forceSenseControllerClass", "G':", nullptr));
        G_robot_lineEdit->setText(QApplication::translate("forceSenseControllerClass", "6.6708", nullptr));
        label_9->setText(QApplication::translate("forceSenseControllerClass", "d'x:", nullptr));
        dx_robot_lineEdit->setText(QApplication::translate("forceSenseControllerClass", "-0.002", nullptr));
        label_10->setText(QApplication::translate("forceSenseControllerClass", "d'y:", nullptr));
        dy_robot_lineEdit->setText(QApplication::translate("forceSenseControllerClass", "0.001", nullptr));
        label_11->setText(QApplication::translate("forceSenseControllerClass", "d'z:", nullptr));
        dz_robot_lineEdit->setText(QApplication::translate("forceSenseControllerClass", "0.035", nullptr));
        label_12->setText(QApplication::translate("forceSenseControllerClass", "G:", nullptr));
        label_13->setText(QApplication::translate("forceSenseControllerClass", "dx:", nullptr));
        label_14->setText(QApplication::translate("forceSenseControllerClass", "dy:", nullptr));
        label_15->setText(QApplication::translate("forceSenseControllerClass", "dz:", nullptr));
        label_27->setText(QApplication::translate("forceSenseControllerClass", "\347\233\270\345\257\271\344\272\216\347\254\254\344\272\224\350\275\264\345\233\272\345\256\232\345\212\233\347\237\251\350\241\245\345\201\277\347\233\270\345\205\263\357\274\232", nullptr));
        get_Compensation_Button->setText(QApplication::translate("forceSenseControllerClass", "\350\216\267\345\217\226\345\212\233/\345\212\233\350\267\235\344\270\216\350\275\254\350\247\222", nullptr));
        caculate_Compensation_Button->setText(QApplication::translate("forceSenseControllerClass", "\350\256\241\347\256\227\350\241\245\345\201\277\345\212\233\347\237\251", nullptr));
        clear_Compensation_Button->setText(QApplication::translate("forceSenseControllerClass", "\346\270\205\347\251\272\345\275\223\345\211\215\344\275\215\345\247\277\346\225\260", nullptr));
        cancel_Compensation_Button->setText(QApplication::translate("forceSenseControllerClass", "\345\217\226\346\266\210\350\241\245\345\201\277", nullptr));
        label_17->setText(QApplication::translate("forceSenseControllerClass", "\345\275\223\345\211\215\345\267\262\346\234\211\344\275\215\345\247\277\346\225\260:", nullptr));
        label_18->setText(QApplication::translate("forceSenseControllerClass", "M_e_x5:", nullptr));
        label_19->setText(QApplication::translate("forceSenseControllerClass", "M_e_y5:", nullptr));
        label_20->setText(QApplication::translate("forceSenseControllerClass", "M_e_z5:", nullptr));
        label_21->setText(QApplication::translate("forceSenseControllerClass", "M_x0:", nullptr));
        label_23->setText(QApplication::translate("forceSenseControllerClass", "M_y0:", nullptr));
        label_22->setText(QApplication::translate("forceSenseControllerClass", "M_z0:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class forceSenseControllerClass: public Ui_forceSenseControllerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // FORCESENSECONTROLLERLWEUJL_H
