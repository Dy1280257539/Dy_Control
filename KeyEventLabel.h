#pragma once
#include <QtWidgets>
#include<qpushbutton.h>


//内窥镜控制台类
class KeyEventLabel : public QLabel
{
    Q_OBJECT
signals:
    void changeRotate(int,int);
public:
    KeyEventLabel(QWidget* parent = nullptr) : QLabel(parent), rotateTCPX(0), rotateTCPY(0),is_start(false) {}

protected:
    void mousePressEvent(QMouseEvent* event) override;

    void keyPressEvent(QKeyEvent* event) override;

    void keyReleaseEvent(QKeyEvent* event) override;
private:
    //判断TCP旋转的对应的轴的变量
    int rotateTCPX, rotateTCPY;
    bool is_start;
};


//增量模式专用按钮类
class KeyEventForIncModeButton : public QPushButton
{
    Q_OBJECT
signals:
    void changePose(int,int,int,int,int,int);
public:
    KeyEventForIncModeButton(QWidget* parent = nullptr) : QPushButton(parent), X(0),Y(0),Z(0),rotateTCPX(0), rotateTCPY(0),rotateTCPZ(0), is_start(false) {}

protected:

    void keyPressEvent(QKeyEvent* event) override;

    void keyReleaseEvent(QKeyEvent* event) override;
public:
    //判断TCP旋转的对应的轴的变量
    int X,Y,Z,rotateTCPX,rotateTCPY,rotateTCPZ;
    bool is_start;
};