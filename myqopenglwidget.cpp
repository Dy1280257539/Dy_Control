#include "MyQOpenglWidget.h"
#include <QDebug>
#include <QtMath>

static const char* vertexShaderSource =
"attribute highp vec3 posAttr;\n"
"attribute lowp vec4 colAttr;\n"
"varying lowp vec4 col;\n"
"uniform highp mat4 matrix;\n"
"void main() {\n"
"  col=colAttr;\n"
"  gl_Position=matrix * vec4(posAttr,1.0f);\n"
"}\n";

static const char* fragmentShaderSource =
"varying lowp vec4 col;\n"
"void main() {\n"
"   gl_FragColor = col;\n"
"}\n";

MyQOpenglWidget::MyQOpenglWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_context(NULL)
    , m_Program(NULL)
    , OpenGLCore(NULL)
    , m_posAttr(0)
    , m_colAttr(0)
    , m_norAttr(0)
    , m_matrixUniform(0)
    , m_VBO(0)
    , m_VAO(0)
    , m_VertexShader(NULL)
    , m_FragmentShader(NULL)
    , m_scale(1.0f)
{
    m_Timer = new QTimer;
    m_PointsVertex = QVector<VertexInfo>();
    ResetView();
    this->grabKeyboard();
}

MyQOpenglWidget::~MyQOpenglWidget()
{
    if (m_context)
    {
        delete m_context;
        m_context = NULL;
    }
    if (m_Program)
    {
        delete m_Program;
        m_Program = NULL;
    }
    m_PointsVertex.clear();
}

void MyQOpenglWidget::showPointCloud(const std::vector<QVector3D>& cloud)
{
    QTime startTime = QTime::currentTime();
    initPointCloud(cloud);
    debugMsg("initPointCloud =", startTime);

    startTime = QTime::currentTime();
    changePointCloud();
    debugMsg("changePointCloud =", startTime);
    ResetView();
    repaint();
}

void MyQOpenglWidget::initPointCloud(const std::vector<QVector3D>& cloud)
{
    m_PointsVertex.clear();
    m_PointsVertex.resize((int)cloud.size());
    m_box.calculateMinBoundingBox(cloud);
    for (int i = 0; i < cloud.size(); i++)
    {
        //将坐标系原点移动到点云中点
        m_PointsVertex[i].pos[0] = (cloud[i].x() - m_box.getCenterPoint().x());
        m_PointsVertex[i].pos[1] = (cloud[i].y() - m_box.getCenterPoint().y());
        m_PointsVertex[i].pos[2] = (cloud[i].z() - m_box.getCenterPoint().z());
        gray2Pseudocolor(cloud[i], m_PointsVertex[i].color);
        m_PointsVertex[i].normal[0] = 0.0f;
        m_PointsVertex[i].normal[1] = 1.0f;
        m_PointsVertex[i].normal[2] = 0.0f;
    }
}

void MyQOpenglWidget::gray2Pseudocolor(const QVector3D pos, float color[4])
{
    float fmin = m_box.getMinPoint().z();
    float fmax = m_box.getMaxPoint().z();
    //int colortemp = (int)(((pos[2] - fmin) / (fmax - fmin)) * 255);
    int colortemp = (int)(((fmax - pos.z()) / (fmax - fmin)) * 255);
    int r, g, b;
    if (colortemp >= 0 && colortemp < 64)
    {
        r = 0;
        g = 254 - 4 * colortemp;
        b = 255;
    }
    else if (colortemp >= 64 && colortemp < 128) {
        r = 0;
        g = 4 * colortemp - 254;
        b = 510 - 4 * colortemp;
    }
    else if (colortemp >= 128 && colortemp < 192) {
        r = 4 * colortemp - 510;
        g = 255;
        b = 0;
    }
    else if (colortemp >= 192 && colortemp <= 255)
    {
        r = 255;
        g = 1022 - 4 * colortemp;
        b = 0;
    }
    else {
        r = 255;
        g = 255;
        b = 255;
    }
    color[0] = r * 1.0f / 255;
    color[1] = g * 1.0f / 255;
    color[2] = b * 1.0f / 255;
    color[3] = 1.0f;
}

void MyQOpenglWidget::changePointCloud()
{
    if (m_PointsVertex.size() <= 0)
    {
        return;
    }
    OpenGLCore->glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexInfo) * (int)m_PointsVertex.size(), m_PointsVertex.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(m_posAttr);
    glVertexAttribPointer(m_posAttr, 3, GL_FLOAT, GL_FALSE, sizeof(VertexInfo), (void*)(offsetof(VertexInfo, pos)));
    glEnableVertexAttribArray(m_colAttr);
    glVertexAttribPointer(m_colAttr, 4, GL_FLOAT, GL_FALSE, sizeof(VertexInfo), (void*)(offsetof(VertexInfo, color)));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    OpenGLCore->glBindVertexArray(0);
}

void MyQOpenglWidget::ResetView()
{
    m_lineMove = QVector3D();
    m_rotate = QQuaternion();
    m_rotate *= QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 180);
    m_rotationAxis = QVector3D();
    m_scale = 1.0f;
    setBackgroundColor(QVector3D(0.0f, 0.0f, 0.0f));
}

void MyQOpenglWidget::setBackgroundColor(QVector3D color)
{
    m_backgroundColor = QVector4D(color, 1.0f);
}

void MyQOpenglWidget::debugMsg(QString msg, QTime start)
{
    qDebug() << msg << start.msecsTo(QTime::currentTime()) << "ms";
}

void MyQOpenglWidget::resizeGL(int w, int h)
{
    //关于devicePixelRatio()函数的解释：https://blog.csdn.net/danshiming/article/details/126813811
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, w * retinaScale, h * retinaScale);
    repaint();//重绘
}

void MyQOpenglWidget::initializeGL()
{
    bool binit = true;
    binit &= InitShader();
    if (!binit)
    {
        return;
    }
    OpenGLCore = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    OpenGLCore->glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);

    //init need import a point data
    initCloud();
    changePointCloud();

    QObject::connect(m_Timer, SIGNAL(timeout()), this, SLOT(onTimerOut()));
    m_Timer->start(30);
}

bool MyQOpenglWidget::InitShader()
{
    m_Program = new QOpenGLShaderProgram(this);
    bool success = true;
    success &= m_Program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    success &= m_Program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    success &= m_Program->link();
    GetShaderUniformPara();
    return success;
}

void MyQOpenglWidget::GetShaderUniformPara()
{
    m_posAttr = m_Program->attributeLocation("posAttr");
    m_colAttr = m_Program->attributeLocation("colAttr");
    m_matrixUniform = m_Program->uniformLocation("matrix");
    //m_norAttr = m_Program->attributeLocation("norAttr");
}

void MyQOpenglWidget::paintGL()
{
    m_Program->bind();
    glClearColor(m_backgroundColor.x(), m_backgroundColor.y(), m_backgroundColor.z(), m_backgroundColor.w());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BITS);

    OpenGLCore->glBindVertexArray(m_VAO);
    setMatrixUniform();
    glDrawArrays(GL_POINTS, 0, (GLsizei)m_PointsVertex.size());
    OpenGLCore->glPointSize(2.0f);

    OpenGLCore->glBindVertexArray(0);
    m_Program->release();
}

//坐标空间、正射投影、透视投影等学习链接 https://blog.csdn.net/qq_40946921/article/details/105989909
void MyQOpenglWidget::setMatrixUniform()
{
    QMatrix4x4 matrix = QMatrix4x4();
    QMatrix4x4 matrixPerspect = QMatrix4x4(); //投影变换矩阵 将摄像机坐标系上的点投影到2D设备坐标系
    QMatrix4x4 matrixView = QMatrix4x4(); //观察变换矩阵   从世界坐标系变换到摄像机坐标系
    QMatrix4x4 matrixModel = QMatrix4x4(); //模型变换矩阵  变换到世界坐标系

    QVector3D  minPos = (m_box.getMinPoint() - m_box.getCenterPoint());
    QVector3D  maxPos = (m_box.getMaxPoint() - m_box.getCenterPoint());
    float maxAxis;
    maxAxis = qAbs(qMax(qMax(m_box.depth(), m_box.width()), m_box.height()));

    //透视投影
    //matrixPerspect.perspective(45.0f, m_box.width() *1.0f/ m_box.height(), 0.1f, 2*maxAxis);

    /*  正射投影 ortho
        前两个参数指定了平截头体的左右坐标，第三和第四参数指定了平截头体的底部和顶部。
        通过这四个参数我们定义了近平面和远平面的大小，然后第五和第六个参数则定义了近平面和远平面的距离。
        这个投影矩阵会将处于这些x，y，z值范围内的坐标变换为标准化设备坐标。
    */
    matrixPerspect.ortho(2 * minPos[0], 2 * maxPos[0], 2 * minPos[1], 2 * maxPos[1], -2 * maxAxis, 2 * maxAxis);

    //一下三个向量分别为eyes,center,up
    matrixView.lookAt(QVector3D(0, 0, maxAxis), QVector3D(0.0, 0.0, -1), QVector3D(0.0, 1.0, 0.0));

    //m_lineMove.z()为0，而x和y取决于鼠标移动的位置，决定了摄像机
    matrixView.translate(m_lineMove.x(), m_lineMove.y(), m_lineMove.z());

    matrixModel.rotate(m_rotate);
    matrixModel.scale(m_scale);

    matrix = matrixPerspect * matrixView * matrixModel;
    m_Program->setUniformValue(m_matrixUniform, matrix);
}


void MyQOpenglWidget::initCloud()
{
    m_PointsVertex.clear();
    VertexInfo point;
    point.pos[0] = 0.0f;
    point.pos[1] = 0.0f;
    point.pos[2] = 0.0f;
    point.color[0] = m_backgroundColor.x();
    point.color[1] = m_backgroundColor.y();
    point.color[2] = m_backgroundColor.z();
    point.color[3] = m_backgroundColor.w();
    point.normal[0] = 0.0f;
    point.normal[1] = 1.0f;
    point.normal[2] = 0.0f;
    m_PointsVertex.push_back(point);
}

void MyQOpenglWidget::onTimerOut()
{
    if (this->isVisible())
    {
        repaint();
    }
}

void MyQOpenglWidget::mousePressEvent(QMouseEvent* e)
{
    if (e->buttons() & Qt::LeftButton || e->buttons() & Qt::MidButton)
    {
        setMouseTracking(true);
        m_lastPoint = QVector2D(e->localPos());
    }
}

void MyQOpenglWidget::mouseMoveEvent(QMouseEvent* e)
{
    if (e->buttons() & Qt::LeftButton)
    {
        Rotate(QVector2D(m_lastPoint), QVector2D(e->localPos()));
    }
    if (e->buttons() & Qt::MidButton)
    {
        LineMove(m_lastPoint, QVector2D(e->localPos()));
    }
    m_lastPoint = QVector2D(e->localPos());
}

void MyQOpenglWidget::mouseReleaseEvent(QMouseEvent* e)
{
    setMouseTracking(false);
}

void MyQOpenglWidget::wheelEvent(QWheelEvent* e)
{
    if (e->delta() > 0) {
        modelZoomInOrOut(true);
    }
    else {
        modelZoomInOrOut(false);
    }
    e->accept();//防止事件往父部件传递
}

void MyQOpenglWidget::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_R)
    {
        ResetView();
        //return;
    }
    QWidget::keyPressEvent(e);
}

void MyQOpenglWidget::leaveEvent(QEvent*)
{
    //不释放键盘事件抓取，会让编辑框无法输入
    releaseKeyboard();
}

void MyQOpenglWidget::enterEvent(QEvent*)
{
    grabKeyboard();
}

GLuint MyQOpenglWidget::createGPUProgram(QString nVertexShaderFile, QString nFragmentShaderFile)
{
    m_VertexShader = new QOpenGLShader(QOpenGLShader::Vertex);
    bool isOK = m_VertexShader->compileSourceFile(nVertexShaderFile);
    if (!isOK)
    {
        delete m_VertexShader;
        m_VertexShader = nullptr;
        qDebug() << "compile vertex shader fail";
        return 0;
    }

    m_FragmentShader = new QOpenGLShader(QOpenGLShader::Fragment);
    if (!m_FragmentShader->compileSourceFile(nFragmentShaderFile))
    {
        delete m_VertexShader;
        delete m_FragmentShader;
        m_FragmentShader = nullptr;
        qDebug() << "compile fragment shader fail";
        return 0;
    }
    m_Program = new QOpenGLShaderProgram(this);
    m_Program->addShader(m_VertexShader);
    m_Program->addShader(m_FragmentShader);
    m_Program->link();
    GetShaderUniformPara();
    return m_Program->programId();
}

void MyQOpenglWidget::LineMove(QVector2D posOrgin, QVector2D posEnd)
{
    float ratio = 0.003f * sqrt(m_box.width() * m_box.height());
    float xoffset = posEnd.x() - posOrgin.x();
    float yoffset = posEnd.y() - posOrgin.y();

    m_lineMove.setX(m_lineMove.x() + xoffset * ratio);
    m_lineMove.setY(m_lineMove.y() - yoffset * ratio);
}

void MyQOpenglWidget::Rotate(QVector2D posOrgin, QVector2D posEnd)
{
    QVector2D diff = posEnd - posOrgin;
    qreal acc = diff.length() / 100.0;
    if (acc < 0.01f)
    {
        return;
    }
    calRotation(posOrgin, posEnd);
}

void MyQOpenglWidget::modelZoomInOrOut(bool ZoomInOrOut)
{
    if (ZoomInOrOut)//zoom in
    {
        m_scale *= 1.1f;
        //        if(m_scale>2.0f)
        //            m_scale = 2.0f;
    }
    else {                  //zoom out
        m_scale *= 0.9f;
        if (m_scale < 0.5f)
            m_scale = 0.5f;
    }
}

void MyQOpenglWidget::calRotation(QVector2D posOrgin, QVector2D posEnd)
{
    QVector3D orginViewPos = pixelPosToViewPos(posOrgin);
    QVector3D endViewPos = pixelPosToViewPos(posEnd);
    float RotateAngle;
    RotateAngle = qRadiansToDegrees(std::acos(QVector3D::dotProduct(orginViewPos, endViewPos)));
    QVector3D axis;
    axis = QVector3D::crossProduct(orginViewPos, endViewPos);
    axis.normalize();
    m_rotate = QQuaternion::fromAxisAndAngle(axis, RotateAngle) * m_rotate;
}

//像素点到视点的转换
QVector3D MyQOpenglWidget::pixelPosToViewPos(const QVector2D& p)
{
    //viewPos 在 -1 到 1的 范围，且为单位向量
    QVector3D viewPos(2.0 * float(p.x()) / width() - 1.0,
        1.0 - 2.0 * float(p.y()) / height(),
        0);
    float sqrZ = 1 - QVector3D::dotProduct(viewPos, viewPos);
    if (sqrZ > 0)
    {
        viewPos.setZ(std::sqrt(sqrZ));
    }
    else {
        viewPos.normalize();
    }
    return viewPos;
}



