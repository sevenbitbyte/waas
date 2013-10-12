#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    _nhPtr = ros::NodeHandlePtr( new ros::NodeHandle() );

    connect(ui->rollSpin, SIGNAL(valueChanged(double)), this, SLOT(rollChangedSlot(double)));
    connect(ui->pitchSpin, SIGNAL(valueChanged(double)), this, SLOT(pitchChangedSlot(double)));
    connect(ui->yawSpin, SIGNAL(valueChanged(double)), this, SLOT(yawChangedSlot(double)));

    connect(ui->posXSpin, SIGNAL(valueChanged(double)), this, SLOT(xPosChangedSlot(double)));
    connect(ui->posYSpin, SIGNAL(valueChanged(double)), this, SLOT(yPosChangedSlot(double)));
    connect(ui->posZSpin, SIGNAL(valueChanged(double)), this, SLOT(zPosChangedSlot(double)));

    //Load parameters from ROS master
    loadRosParams();

    //Setup service caller in dedicated thread
    _srvThread = new QThread();
    _srvCaller = new ServiceCaller(_nhPtr);
    _srvCaller->moveToThread( _srvThread );

    connect(_srvThread, SIGNAL(finished()), _srvThread, SLOT(deleteLater()));
    connect(this, SIGNAL(triggerParamRefresh()), _srvCaller, SLOT(paramRefreshSlot()));
    _srvThread->start(QThread::LowPriority);
}

MainWindow::~MainWindow()
{
    delete ui;
    //_srvThread->terminate();
}


void MainWindow::renderImage() {

}

void MainWindow::publishImage() {

}

void MainWindow::loadRosParams() {
    ui->rollSpin->setValue( loadRosParam("/waas/orientation/roll") );
    ui->pitchSpin->setValue( loadRosParam("/waas/orientation/pitch") );
    ui->yawSpin->setValue( loadRosParam("/waas/orientation/yaw") );

    ui->posXSpin->setValue( loadRosParam("/waas/position/x") );
    ui->posYSpin->setValue( loadRosParam("/waas/position/y") );
    ui->posZSpin->setValue( loadRosParam("/waas/position/z") );
}

double MainWindow::loadRosParam(QString param, double value){
    if(_nhPtr->hasParam( param.toStdString() )){
         _nhPtr->getParam( param.toStdString(), value );
    }
    else{
        _nhPtr->setParam( param.toStdString(), value );
    }

    return value;
}



void MainWindow::rollChangedSlot(double value) {
    _nhPtr->setParam("/waas/orientation/roll", value);
    emit triggerParamRefresh();
}

void MainWindow::pitchChangedSlot(double value) {
    _nhPtr->setParam("/waas/orientation/pitch", value);
    emit triggerParamRefresh();
}

void MainWindow::yawChangedSlot(double value) {
    _nhPtr->setParam("/waas/orientation/yaw", value);
    emit triggerParamRefresh();
}

void MainWindow::xPosChangedSlot(double value) {
    _nhPtr->setParam("/waas/position/x", value);
    emit triggerParamRefresh();
}

void MainWindow::yPosChangedSlot(double value) {
    _nhPtr->setParam("/waas/position/y", value);
    emit triggerParamRefresh();
}

void MainWindow::zPosChangedSlot(double value) {
    _nhPtr->setParam("/waas/position/z", value);
    emit triggerParamRefresh();
}
