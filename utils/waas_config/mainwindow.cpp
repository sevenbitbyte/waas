#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    _nextRun = NULL;
    _ola = new OlaManager(this);
    _pixelMap = new PixelMapper(_ola, this);

    _pixelMap->_outputLabel = ui->imageLabel;
    ui->imageLabel->setScaledContents(true);

    _dmxStart.universe = 1;
    _dmxStart.offset = 0;

    _dmxEnd.universe = 1;
    _dmxEnd.offset = 0;

    _dmxCursor = _dmxStart;
    _animationStage = 0;
    _cursorValue.setRgb(0, 0, 0);
    _animateTimer = new QTimer(this);
    _animateTimer->setInterval(50);
    _animateTimer->setSingleShot(false);

    connect(_animateTimer, SIGNAL(timeout()), this, SLOT(animateDmxCursor()));

    //Buttons
    connect(ui->forwardButton, SIGNAL(clicked()), this, SLOT(cursorForwardSlot()));
    connect(ui->backwardButton, SIGNAL(clicked()), this, SLOT(cursorBackwardSlot()));
    connect(ui->resetButton, SIGNAL(clicked()), this, SLOT(cursorResetSlot()));
    connect(ui->runStartButton, SIGNAL(clicked()), this, SLOT(runStartSlot()));
    connect(ui->runEndButton, SIGNAL(clicked()), this, SLOT(runEndSlot()));

    connect(ui->cursorAddrEdit, SIGNAL(textEdited(QString)), this, SLOT(updateCursorAddrSlot(QString)));

    //Pan tilt
    //connect(ui->st)
    //ui->startAddrSpin
    connect(ui->startAddrSpin, SIGNAL(valueChanged(int)), this, SLOT(setMoverStartAddr(int)));
    connect(ui->panSlider, SIGNAL(valueChanged(int)), this, SLOT(setMoverPan(int)));
    connect(ui->tiltSlider, SIGNAL(valueChanged(int)), this, SLOT(setMoverTilt(int)));
    connect(ui->zoomSlider, SIGNAL(valueChanged(int)), this, SLOT(setMoverZoom(int)));
    connect(ui->redSpin, SIGNAL(valueChanged(int)), this, SLOT(setMoverRed(int)));
    connect(ui->greenSpin, SIGNAL(valueChanged(int)), this, SLOT(setMoverGreen(int)));
    connect(ui->blueSpin, SIGNAL(valueChanged(int)), this, SLOT(setMoverBlue(int)));
    connect(ui->whiteSpin, SIGNAL(valueChanged(int)), this, SLOT(setMoverWhite(int)));
    connect(ui->intensitySlide, SIGNAL(valueChanged(int)), this, SLOT(setMoverIntensity(int)));


    //Tools tab
    connect(ui->blackoutButton, SIGNAL(clicked()), this, SLOT(blackoutSlot()));
    connect(ui->whiteButton, SIGNAL(clicked()), this, SLOT(whiteoutSlot()));

    connect(ui->arrayRed, SIGNAL(valueChanged(int)), this, SLOT(setArrayRed(int)));
    connect(ui->arrayGreen, SIGNAL(valueChanged(int)), this, SLOT(setArrayGreen(int)));
    connect(ui->arrayBlue, SIGNAL(valueChanged(int)), this, SLOT(setArrayBlue(int)));

    //connect(ui->intensitySlider, SIGNAL(valueChanged(int)), this, SLOT(whiteoutItensitySlot(int)));


    connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(savePixelMap()));
    connect(ui->actionLoad, SIGNAL(triggered()), this, SLOT(loadPixelMap()));

    _moverStart.universe = 1;
    _moverStart.offset = 496;
    _red = 0;
    _green = 0;
    _blue = 0;
    _pan = 0;
    _tilt = 0;
    _intensity = 100;
    _zoom = 0;

    _animateTimer->start();
    updateUi();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::moveDmxCursor(bool forward){
    _ola->setPixel(_dmxCursor, QColor(0,0,0));

    if(forward){
        _dmxCursor = _dmxCursor.add(3);
    }
    else{
        _dmxCursor = _dmxCursor.add(-3);
    }
    updateUi();
}

void MainWindow::animateDmxCursor(){
    if(_animationStage == 0){
        int value = _cursorValue.red();

        if(value < 255){
            value = qMin(value + 20, 255);
            _cursorValue.setRed( value );
            _cursorValue.setGreen( value );
            _cursorValue.setBlue( value );
        }

        if(value >= 255){
            _animationStage=1;
        }
    }

    if(_animationStage == 1){
        int value = _cursorValue.red();

        if(value > 0){
            value = qMax(value - 20, 0);
            _cursorValue.setRed( value );
            _cursorValue.setGreen( value );
            _cursorValue.setBlue( value );
        }

        if(value <= 0){
            _animationStage=0;
        }
    }

    _frameCount++;
    //if(_frameCount%2 == 0 || _forceRender){
        _pixelMap->render();
        _forceRender = false;
    //}

    _ola->setPixel(_dmxCursor, _cursorValue);
    _ola->sendBuffers();
}

void MainWindow::setMoverStartAddr(int i) {
    _moverStart.universe = 1;
    _moverStart.offset = i;

    updateMover();
}

void MainWindow::setMoverPan(int i) {
    _pan = i;
    updateMover();
}

void MainWindow::setMoverTilt(int i) {
    _tilt = i;
    updateMover();
}

void MainWindow::setMoverZoom(int i){
    _zoom = i;
    updateMover();
}

void MainWindow::setMoverRed(int i) {
    _moverColor.setRed(i);
    updateMover();
}

void MainWindow::setMoverGreen(int i) {
    _moverColor.setGreen(i);
    updateMover();
}

void MainWindow::setMoverBlue(int i) {
    _moverColor.setBlue(i);
    updateMover();
}

void MainWindow::setMoverWhite(int i) {
    _moverWhite = i;
    updateMover();
}

void MainWindow::setMoverIntensity(int i) {
    _intensity = i;
    updateMover();
}

void MainWindow::updateMover(){
    /*
     *
     *int _pan;
        int _tilt;
        int _intensity;
        int _red;
        int _green;
        int blue;
     *
     */

    int i=0;
    while(i < _moverStart.offset){
        //QColor c(_red, _green, _blue);

        _ola->setPixel(_moverStart.universe, i, _pixelColor);
        i+=3;
    }

    _ola->setValue(_moverStart.universe, _moverStart.offset, 255);
    _ola->setValue(_moverStart.universe, _moverStart.offset + 1, _intensity);
    _ola->setValue(_moverStart.universe, _moverStart.offset + 2, _pan);
    _ola->setValue(_moverStart.universe, _moverStart.offset + 13, _zoom);
    _ola->setValue(_moverStart.universe, _moverStart.offset + 4, _tilt);
    _ola->setValue(_moverStart.universe, _moverStart.offset + 8, _moverColor.red());
    _ola->setValue(_moverStart.universe, _moverStart.offset + 9, _moverColor.green());
    _ola->setValue(_moverStart.universe, _moverStart.offset + 10, _moverColor.blue());
    _ola->setValue(_moverStart.universe, _moverStart.offset + 11, _moverWhite);
    _forceRender = true;
}

void MainWindow::cursorResetSlot(){
    _dmxCursor.universe = 1;
    _dmxCursor.offset = 0;
    _ola->blackout();

    if(_nextRun != NULL){
        _nextRun->dmxEnd = _dmxCursor;
        _forceRender = true;
    }

    updateUi();
}

void MainWindow::cursorForwardSlot(){
    moveDmxCursor(true);

    if(_nextRun != NULL){
        _nextRun->dmxEnd = _dmxCursor;
        _forceRender = true;
    }
}

void MainWindow::cursorBackwardSlot(){
    moveDmxCursor(false);

    if(_nextRun != NULL){
        _nextRun->dmxEnd = _dmxCursor;
        _forceRender = true;
    }
}


void MainWindow::updateUi(){
    ui->cursorAddrEdit->setText( _dmxCursor.toString() );

    if(_nextRun != NULL){
        ui->runStartAddrEdit->setText( _nextRun->dmxStart.toString() );
        ui->runEndAddrEdit->setText( _nextRun->dmxEnd.toString() );
    }
}


void MainWindow::updateCursorAddrSlot(QString text){
    if(text.isEmpty()){
        return;
    }

    QStringList values = text.split(".");

    if(values.size() < 2){
        return;
    }

    _dmxCursor.universe = values[0].toInt();
    _dmxCursor.offset = values[1].toInt();
}


void MainWindow::setArrayRed(int i) {
    _pixelColor.setRed(i);
    updateMover();
}

void MainWindow::setArrayGreen(int i) {
    _pixelColor.setGreen(i);
    updateMover();
}

void MainWindow::setArrayBlue(int i) {
    _pixelColor.setBlue(i);
    updateMover();
}

void MainWindow::blackoutSlot(){
    _ola->blackout();
}

void MainWindow::whiteoutSlot(){
    _ola->lightsOn(255);
}

/*void MainWindow::whiteoutItensitySlot(int value){
    _ola->lightsOn(value);
}*/


void MainWindow::runStartSlot(){
    if(_nextRun != NULL){
        delete _nextRun;
    }

    _nextRun = new LedRun();
    _nextRun->dmxStart = _dmxCursor;
    _nextRun->dmxEnd = _dmxCursor;

    _pixelMap->testRun = _nextRun;
    _pixelMap->start();

    updateUi();
}


void MainWindow::runEndSlot(){
    if(_nextRun == NULL){
        return;
    }

    _nextRun->dmxEnd = _dmxCursor.add(2);


    if(ui->layoutCombo->currentIndex() != 0){
        _nextRun->reverse = true;
    }

    int column = ui->columnSpin->text().toInt();
    _pixelMap->insertRun(column, _nextRun);

    _pixelMap->start();

    _nextRun = NULL;
    _pixelMap->testRun = NULL;

    updateUi();
}


void MainWindow::savePixelMap(){
    QString fileName = QFileDialog::getSaveFileName(this, "Save pixel map", "~/", "*.json");

    QJsonDocument doc = _pixelMap->toJson();

    QByteArray jsonText = doc.toJson();

    QFile outputFile(fileName);

    if(outputFile.open(QFile::WriteOnly)){
        outputFile.write(jsonText);
        outputFile.close();
    }

}

void MainWindow::loadPixelMap(){
    QString fileName = QFileDialog::getOpenFileName(this, "Load pixel map", "~/", "*.json");

    QFile pixelMapFile(fileName);

    if(!pixelMapFile.open(QIODevice::ReadOnly)){
        qCritical() << "Failed to open file: " << fileName;
    }

    QByteArray fileContent = pixelMapFile.readAll();

    qDebug() << "Loaded " << fileContent.size() << " bytes from " << fileName;

    QJsonDocument doc=QJsonDocument::fromJson(fileContent);

    _pixelMap->fromJson(doc);

    if(!_pixelMap->isRunning()){
        qDebug() << "Starting pixel mapper";
        _pixelMap->start();
    }

    pixelMapFile.close();
}
