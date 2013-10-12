#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore>
#include <QtGui>

#include <QMainWindow>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>

#include "service_caller.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();

    protected slots:
        void renderImage();
        void publishImage();

    protected:
        double loadRosParam(QString param, double value=0.0f);

    public slots:
        void loadRosParams();


        void rollChangedSlot(double value);
        void pitchChangedSlot(double value);
        void yawChangedSlot(double value);

        void xPosChangedSlot(double value);
        void yPosChangedSlot(double value);
        void zPosChangedSlot(double value);



    signals:
        void triggerParamRefresh();

    private:
        Ui::MainWindow *ui;

        //ROS members
        ros::NodeHandlePtr _nhPtr;
        ros::Publisher _imagePublisher;

        QThread* _srvThread;
        ServiceCaller* _srvCaller;

        //Animation members
        enum AnimateModes { Rainbow, Sweep, Swirl, WarpSpeed };

        QDateTime _animationStartTime;
        quint64 _animationDurationMs;
        quint32 _animationMode;
        QImage* _imageBuffer;
};



#endif // MAINWINDOW_H
