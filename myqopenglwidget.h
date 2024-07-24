#ifndef MYQOPENGLWIDGET_H
#define MYQOPENGLWIDGET_H
#include <QtGui/QWindow>
#include <QtGui/QOpenGLFunctions>
#include <QtWidgets/QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QPainter>
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QOpenGLShaderProgram>
#include <QOpenGLShader>
#include <QTimer>
#include <QVector3D>
#include <QQuaternion>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QTime>
#include "MinBoundingBox.h"

/*
    Dy:
    1.此实现一个openglWidget子类，用于展示点云信息等；
    2.点云存储的数据结构为 vector<QVector3D>，注意转化
    3.

*/

QT_BEGIN_NAMESPACE
class QPainter;
class QOpenGLContext;
class QOpenGLPaintDevice;
class QOpenGLShaderProgram;
class QOpenGLShader;
QT_END_NAMESPACE

class MyQOpenglWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit MyQOpenglWidget(QWidget* parent = 0);
    ~MyQOpenglWidget();
    void showPointCloud(const std::vector<QVector3D>& cloud);
    virtual void resizeGL(int w, int h);
    void setBackgroundColor(QVector3D color);

protected:
    virtual void initializeGL(void);
    virtual void paintGL(void);

    struct VertexInfo
    {
        float pos[3];
        float normal[3];
        float color[4];
    };
private slots:
    void onTimerOut(void);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);
    virtual void wheelEvent(QWheelEvent* e);
    virtual void keyPressEvent(QKeyEvent* e);
    virtual void leaveEvent(QEvent*);
    virtual void enterEvent(QEvent*);

private:
    QTimer* m_Timer;
    QOpenGLContext* m_context;
    QOpenGLShaderProgram* m_Program;
    QOpenGLFunctions_3_3_Core* OpenGLCore;
    QOpenGLShader* m_VertexShader;
    QOpenGLShader* m_FragmentShader;

    // shader spara
    GLuint m_posAttr;
    GLuint m_colAttr;
    GLuint m_norAttr;
    GLuint m_matrixUniform;
    GLuint m_VBO;
    GLuint m_VAO;
    QVector4D m_backgroundColor;

    //store points
    QVector<VertexInfo> m_PointsVertex;

    //为点云提供相关信息的类
    Dy::MinBoundingBox m_box;

    QVector3D m_lineMove;
    QQuaternion m_rotate; //旋转矢量
    QVector3D m_rotationAxis;
    float m_scale;

    QVector2D m_lastPoint;

    //未曾使用
    GLuint createGPUProgram(QString nVertexShaderFile, QString nFragmentShaderFile);

    void GetShaderUniformPara();
    bool InitShader();

    void LineMove(QVector2D posOrgin, QVector2D posEnd);
    void Rotate(QVector2D posOrgin, QVector2D posEnd);//旋转
    void modelZoomInOrOut(bool ZoomInOrOut);//缩放
    QVector3D pixelPosToViewPos(const QVector2D& p);
    void calRotation(QVector2D posOrgin, QVector2D posEnd); //根据两点计算旋转矢量
    void initPointCloud(const std::vector<QVector3D>& cloud);

    //灰度转化为伪彩色
    void gray2Pseudocolor(const QVector3D pos, float color[4]);

    void changePointCloud();
    void setMatrixUniform();

    //重置视图
    void ResetView(); 

    void initCloud();

    //debug
    void debugMsg(QString msg, QTime start);

};

#endif // MYQOPENGLWIDGET_H
