#include "Dy_Control.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    //控制图片缩放质量
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);


    //这个是windows平台用来获取屏幕宽度的代码
    //因为在qApplication实例初始化之前，QGuiApplication::screens()：无法使用。
    DEVMODE NewDevMode;
    //获取屏幕设置中的分辨率
    EnumDisplaySettings(0, ENUM_CURRENT_SETTINGS, &NewDevMode);
    qreal  cx = NewDevMode.dmPelsWidth;			//当前设置的屏幕宽度
    qreal scale = cx / 960.0; // 按照屏幕宽度计算
    scale = qBound(0.2, scale, 1.0); // 限制缩放比例在 0.2x 到 1.0x 之间
    qputenv("QT_SCALE_FACTOR", QString::number(scale).toLatin1());

    //qputenv("QT_SCALE_FACTOR", "0.8");


    QApplication a(argc, argv);
    Dy_Control w;
    w.show();
    return a.exec();
    
}
