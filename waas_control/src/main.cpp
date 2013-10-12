#include "mainwindow.h"
#include <QtCore>
#include <QtGui>

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "waas_control");

    MainWindow w;
    w.show();
    
    return a.exec();
}
