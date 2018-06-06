#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);

    MainWindow w;
    //Blank w;
    w.setWindowTitle("哈尔滨工业大学(深圳)");

    w.show();

    return a.exec();
}
