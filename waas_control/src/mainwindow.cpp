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

    connect(ui->downsampleLeafSizeSpin, SIGNAL(valueChanged(double)), this, SLOT(downsampleLeafSizeChangedSlot(double)));
    connect(ui->octreeVoxelSizeSpin, SIGNAL(valueChanged(double)), this, SLOT(octreeVoxelSizeChangedSlot(double)));
    connect(ui->backgroundResetThresholdSpin, SIGNAL(valueChanged(double)), this, SLOT(backgroundResetThresholdChangedSlot(double)));
    connect(ui->clusterJoinDistanceSpin, SIGNAL(valueChanged(double)), this, SLOT(clusterJoinDistanceChangedSlot(double)));
    connect(ui->clusterMinSizeSpin, SIGNAL(valueChanged(double)), this, SLOT(clusterMinSizeChangedSlot(double)));
    connect(ui->clusterMaxSizeSpin, SIGNAL(valueChanged(double)), this, SLOT(clusterMaxSizeChangedSlot(double)));

    connect(ui->globeScaleSpin, SIGNAL(valueChanged(double)), this, SLOT(globesScaleChangedSlot(double)));
    connect(ui->globePositionXSpin, SIGNAL(valueChanged(double)), this, SLOT(globesPositionXChangedSlot(double)));
    connect(ui->globePositionYSpin, SIGNAL(valueChanged(double)), this, SLOT(globesPositionYChangedSlot(double)));
    connect(ui->globeOrientationRollSpin, SIGNAL(valueChanged(double)), this, SLOT(globesOrientationRollChangedSlot(double)));
    connect(ui->globeOrientationPitchSpin, SIGNAL(valueChanged(double)), this, SLOT(globesOrientationPitchChangedSlot(double)));
    connect(ui->globeOrientationYawSpin, SIGNAL(valueChanged(double)), this, SLOT(globesOrientationYawChangedSlot(double)));
    connect(ui->globeSpacingXSpin, SIGNAL(valueChanged(double)), this, SLOT(globesSpacingXChangedSlot(double)));
    connect(ui->globeSpacingYSpin, SIGNAL(valueChanged(double)), this, SLOT(globesSpacingYChangedSlot(double)));

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
    //Cloud orientation
    ui->rollSpin->setValue( loadRosParam("/waas/cloud/orientation/roll") );
    ui->pitchSpin->setValue( loadRosParam("/waas/cloud/orientation/pitch") );
    ui->yawSpin->setValue( loadRosParam("/waas/cloud/orientation/yaw") );

    //Cloud position
    ui->posXSpin->setValue( loadRosParam("/waas/cloud/position/x") );
    ui->posYSpin->setValue( loadRosParam("/waas/cloud/position/y") );
    ui->posZSpin->setValue( loadRosParam("/waas/cloud/position/z") );

    //Globes
    ui->globeScaleSpin->setValue( loadRosParam("/waas/globes/scale") );
    ui->globePositionXSpin->setValue( loadRosParam("/waas/globes/position/x") );
    ui->globePositionYSpin->setValue( loadRosParam("/waas/globes/position/y") );
    ui->globeSpacingXSpin->setValue( loadRosParam("/waas/globes/spacing/x") );
    ui->globeSpacingYSpin->setValue( loadRosParam("/waas/globes/spacing/y") );
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
    _nhPtr->setParam("/waas/cloud/orientation/roll", value);
    emit triggerParamRefresh();
}

void MainWindow::pitchChangedSlot(double value) {
    _nhPtr->setParam("/waas/cloud/orientation/pitch", value);
    emit triggerParamRefresh();
}

void MainWindow::yawChangedSlot(double value) {
    _nhPtr->setParam("/waas/cloud/orientation/yaw", value);
    emit triggerParamRefresh();
}

void MainWindow::xPosChangedSlot(double value) {
    _nhPtr->setParam("/waas/cloud/position/x", value);
    emit triggerParamRefresh();
}

void MainWindow::yPosChangedSlot(double value) {
    _nhPtr->setParam("/waas/cloud/position/y", value);
    emit triggerParamRefresh();
}

void MainWindow::zPosChangedSlot(double value) {
    _nhPtr->setParam("/waas/cloud/position/z", value);
    emit triggerParamRefresh();
}

void MainWindow::downsampleLeafSizeChangedSlot(double value){
    _nhPtr->setParam("/waas/downsample_leaf_size", value);
    emit triggerParamRefresh();
}

void MainWindow::octreeVoxelSizeChangedSlot(double value) {
    _nhPtr->setParam("/waas/octree_voxel_size", value);
    emit triggerParamRefresh();
}

void MainWindow::backgroundResetThresholdChangedSlot(double value) {
    _nhPtr->setParam("/waas/background_reset_threshold", value);
    emit triggerParamRefresh();
}

void MainWindow::clusterJoinDistanceChangedSlot(double value) {
    _nhPtr->setParam("/waas/cluster_join_distance", value);
    emit triggerParamRefresh();
}

void MainWindow::clusterMinSizeChangedSlot(double value) {
    _nhPtr->setParam("/waas/cluster_min_size", value);
    emit triggerParamRefresh();
}

void MainWindow::clusterMaxSizeChangedSlot(double value) {
    _nhPtr->setParam("/waas/cluster_max_size", value);
    emit triggerParamRefresh();
}


void MainWindow::globesScaleChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/scale", value);
    emit triggerParamRefresh();
}

void MainWindow::globesPositionXChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/position/x", value);
    emit triggerParamRefresh();
}

void MainWindow::globesPositionYChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/position/y", value);
    emit triggerParamRefresh();
}

void MainWindow::globesOrientationRollChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/orientation/roll", value);
    emit triggerParamRefresh();
}

void MainWindow::globesOrientationPitchChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/orientation/pitch", value);
    emit triggerParamRefresh();
}

void MainWindow::globesOrientationYawChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/orientation/yaw", value);
    emit triggerParamRefresh();
}


void MainWindow::globesSpacingXChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/spacing/x", value);
    emit triggerParamRefresh();
}

void MainWindow::globesSpacingYChangedSlot(double value) {
    _nhPtr->setParam("/waas/globes/spacing/y", value);
    emit triggerParamRefresh();
}
