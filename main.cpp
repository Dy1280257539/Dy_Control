#include "Dy_Control.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    //����ͼƬ��������
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);


    //�����windowsƽ̨������ȡ��Ļ��ȵĴ���
    //��Ϊ��qApplicationʵ����ʼ��֮ǰ��QGuiApplication::screens()���޷�ʹ�á�
    DEVMODE NewDevMode;
    //��ȡ��Ļ�����еķֱ���
    EnumDisplaySettings(0, ENUM_CURRENT_SETTINGS, &NewDevMode);
    qreal  cx = NewDevMode.dmPelsWidth;			//��ǰ���õ���Ļ���
    qreal scale = cx / 960.0; // ������Ļ��ȼ���
    scale = qBound(0.2, scale, 1.0); // �������ű����� 0.2x �� 1.0x ֮��
    qputenv("QT_SCALE_FACTOR", QString::number(scale).toLatin1());

    //qputenv("QT_SCALE_FACTOR", "0.8");


    QApplication a(argc, argv);
    Dy_Control w;
    w.show();
    return a.exec();
    
}
