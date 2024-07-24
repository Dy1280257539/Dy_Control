#include "Dy_Control.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dy_Control w;
    w.show();
    return a.exec();
    
}
