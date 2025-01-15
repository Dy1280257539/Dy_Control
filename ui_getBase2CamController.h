/********************************************************************************
** Form generated from reading UI file 'getBase2CamControllerwzJTqk.ui'
**
** Created by: Qt User Interface Compiler version 5.12.9
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef GETBASE2CAMCONTROLLERWZJTQK_H
#define GETBASE2CAMCONTROLLERWZJTQK_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_getBase2CamControllerClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QTableWidget *Base2Cam_tableWidget;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *outputBase2Cam_button;
    QSpacerItem *horizontalSpacer_2;

    void setupUi(QMainWindow *getBase2CamControllerClass)
    {
        if (getBase2CamControllerClass->objectName().isEmpty())
            getBase2CamControllerClass->setObjectName(QString::fromUtf8("getBase2CamControllerClass"));
        getBase2CamControllerClass->resize(342, 361);
        centralWidget = new QWidget(getBase2CamControllerClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        Base2Cam_tableWidget = new QTableWidget(centralWidget);
        Base2Cam_tableWidget->setObjectName(QString::fromUtf8("Base2Cam_tableWidget"));

        verticalLayout->addWidget(Base2Cam_tableWidget);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        outputBase2Cam_button = new QPushButton(centralWidget);
        outputBase2Cam_button->setObjectName(QString::fromUtf8("outputBase2Cam_button"));

        horizontalLayout->addWidget(outputBase2Cam_button);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        horizontalLayout->setStretch(0, 1);
        horizontalLayout->setStretch(1, 1);
        horizontalLayout->setStretch(2, 1);

        verticalLayout->addLayout(horizontalLayout);

        verticalLayout->setStretch(0, 1);
        verticalLayout->setStretch(1, 4);
        verticalLayout->setStretch(2, 1);
        verticalLayout->setStretch(3, 1);

        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

        getBase2CamControllerClass->setCentralWidget(centralWidget);

        retranslateUi(getBase2CamControllerClass);

        QMetaObject::connectSlotsByName(getBase2CamControllerClass);
    } // setupUi

    void retranslateUi(QMainWindow *getBase2CamControllerClass)
    {
        getBase2CamControllerClass->setWindowTitle(QApplication::translate("getBase2CamControllerClass", "getBase2CamController", nullptr));
        label->setText(QApplication::translate("getBase2CamControllerClass", "\350\216\267\345\217\226\345\237\272\345\272\247\346\240\207\345\210\260\347\233\270\346\234\272\345\235\220\346\240\207\347\263\273\347\232\204\346\227\213\350\275\254\347\237\251\351\230\265\357\274\232", nullptr));
        outputBase2Cam_button->setText(QApplication::translate("getBase2CamControllerClass", "\350\276\223\345\207\272\347\237\251\351\230\265", nullptr));
    } // retranslateUi

};

namespace Ui {
    class getBase2CamControllerClass: public Ui_getBase2CamControllerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // GETBASE2CAMCONTROLLERWZJTQK_H
