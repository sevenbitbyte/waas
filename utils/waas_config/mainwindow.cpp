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


    //Tools tab
    connect(ui->blackoutButton, SIGNAL(clicked()), this, SLOT(blackoutSlot()));
    connect(ui->whiteButton, SIGNAL(clicked()), this, SLOT(whiteoutSlot()));
    connect(ui->intensitySlider, SIGNAL(valueChanged(int)), this, SLOT(whiteoutItensitySlot(int)));


    connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(savePixelMap()));

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


void MainWindow::blackoutSlot(){
    _ola->blackout();
}

void MainWindow::whiteoutSlot(){
    _ola->lightsOn(255);
}

void MainWindow::whiteoutItensitySlot(int value){
    _ola->lightsOn(value);
}


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

