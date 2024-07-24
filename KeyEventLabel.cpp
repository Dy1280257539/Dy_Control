#include"KeyEventLabel.h"


//------------------------------------内窥镜控制台类--------------------------------


void KeyEventLabel::mousePressEvent(QMouseEvent* event)
{
    if (is_start == false) {
        qDebug() << "start control" << endl;
        this->setText(QStringLiteral("结束控制"));
        this->grabKeyboard();
        is_start = true;
    }
    else if (is_start == true) {
        qDebug() << "stop control" << endl;
        this->setText(QStringLiteral("开始控制"));
        this->releaseKeyboard();
        is_start = false;
    }
}

void KeyEventLabel::keyPressEvent(QKeyEvent* event)
{
    if (is_start == false)
        return;
    if (event->isAutoRepeat()) return; //真正按键触发返回false
    if (event->key() == Qt::Key_Up) {
        qDebug() << "up" << endl;
        rotateTCPY = -1;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
    else if (event->key() == Qt::Key_Down) {
        qDebug() << "down" << endl;
        rotateTCPY = 1;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
    else if (event->key() == Qt::Key_Left) {
        qDebug() << "left" << endl;
        rotateTCPX = -1;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
    else if (event->key() == Qt::Key_Right) {
        qDebug() << "right" << endl;
        rotateTCPX = 1;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
}

void KeyEventLabel::keyReleaseEvent(QKeyEvent* event) {
    if (is_start == false)
        return;
    if (event->isAutoRepeat()) return; //真正松键触发返回false
    if (event->key() == Qt::Key_Up) {
        qDebug() << "up_release" << endl;
        rotateTCPY = 0;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
    else if (event->key() == Qt::Key_Down) {
        qDebug() << "down_release" << endl;
        rotateTCPY = 0;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
    else if (event->key() == Qt::Key_Left) {
        qDebug() << "left_release" << endl;
        rotateTCPX = 0;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
    else if (event->key() == Qt::Key_Right) {
        qDebug() << "right_release" << endl;
        rotateTCPX = 0;
        emit changeRotate(rotateTCPX, rotateTCPY);
    }
}



//----------------------------------------------------------------------------


//------------------------------------增量模式控制按钮类--------------------------------

void KeyEventForIncModeButton::keyPressEvent(QKeyEvent* event)
{
    if (is_start == false)
        return;
    if (event->isAutoRepeat()) return; //真正按键触发返回false
    if (event->key() == Qt::Key_A) {
        qDebug() << "+x" << endl;
        X = 1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_D) {
        qDebug() << "-x" << endl;
        X = -1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_W) {
        qDebug() << "-y" << endl;
        Y = -1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_S) {
        qDebug() << "+y" << endl;
        Y = 1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_J) {
        qDebug() << "+z" << endl;
        Z = 1;
        emit changePose(X,Y,Z,rotateTCPX, rotateTCPY,rotateTCPZ);
    }
    else if (event->key() == Qt::Key_K) {
        qDebug() << "-z" << endl;
        Z = -1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Up) {
        qDebug() << "-rotateTCPY" << endl;
        rotateTCPY = -1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Down) {
        qDebug() << "+rotateTCPY" << endl;
        rotateTCPY = 1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Left) {
        qDebug() << "-rotateTCPX" << endl;
        rotateTCPX = -1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Right) {
        qDebug() << "+rotateTCPX" << endl;
        rotateTCPX = 1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_N) {
        qDebug() << "+rotateTCPZ" << endl;
        rotateTCPZ = 1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_M) {
        qDebug() << "-rotateTCPZ" << endl;
        rotateTCPZ = -1;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
}

void KeyEventForIncModeButton::keyReleaseEvent(QKeyEvent* event) {
    if (is_start == false)
        return;
    if (event->isAutoRepeat()) return; //真正按键触发返回false
    if (event->key() == Qt::Key_A) {
        qDebug() << "+x_release" << endl;
        X = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_D) {
        qDebug() << "-x_release" << endl;
        X = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_W) {
        qDebug() << "-y_release" << endl;
        Y = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_S) {
        qDebug() << "+y_release" << endl;
        Y = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_J) {
        qDebug() << "+z_release" << endl;
        Z = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_K) {
        qDebug() << "-z_release" << endl;
        Z = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Up) {
        qDebug() << "-rotateTCPY_release" << endl;
        rotateTCPY = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Down) {
        qDebug() << "+rotateTCPY_release" << endl;
        rotateTCPY = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Left) {
        qDebug() << "-rotateTCPX_release" << endl;
        rotateTCPX = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_Right) {
        qDebug() << "+rotateTCPX_release" << endl;
        rotateTCPX = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_N) {
        qDebug() << "+rotateTCPZ_release" << endl;
        rotateTCPZ = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
    else if (event->key() == Qt::Key_M) {
        qDebug() << "-rotateTCPZ_release" << endl;
        rotateTCPZ = 0;
        emit changePose(X, Y, Z, rotateTCPX, rotateTCPY, rotateTCPZ);
    }
}



//----------------------------------------------------------------------------