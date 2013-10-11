#include "pixelmapper.h"

PixelMapper::PixelMapper(OlaManager* ola, QObject *parent) :
    QObject(parent)
{
    _ola = ola;
    _image = new QImage(34, 34, QImage::Format_RGB32);

    QPainter painter;

    QRect bounds(0,0, 34, 34);
    QBrush fillBrush( QColor(255,0,0) );

    painter.begin(_image);
    painter.fillRect(bounds, fillBrush);
    painter.end();

    _renderTimer = new QTimer(this);
    _renderTimer->setInterval(100);
    _renderTimer->setSingleShot(false);

    _outputLabel = NULL;
    testRun = NULL;

    animationMode = FillWhite;

    //connect(_renderTimer, SIGNAL(timeout()), this, SLOT(render()));

    //_renderTimer->start();
}


void PixelMapper::setPixel(QPoint position, QColor color){
    QPainter painter;

    painter.begin(_image);

    painter.setPen(color);

    painter.drawPoint(position);

    painter.end();
}


void PixelMapper::insertRun(int column, LedRun* run){
    _colToLedRun.insert(column, run);
}

bool PixelMapper::isRunning(){
    return _renderTimer->isActive();
}

void PixelMapper::render(){
    QList<int> columns = _colToLedRun.keys();

    if(animationForward){
        if(animationStep < /*_image->height()*/ 360){
            animationStep++;
        }
        else{
            animationStep=0;
            animationForward = true;
            //animationForward = false;
        }
    }

    if(!animationForward){
        if(animationStep > 1){
            animationStep--;
        }
        else{
            animationStep = 360;
            animationForward = true;
        }
    }

    _image->fill(Qt::black);

    if(animationMode == FillWhite){
        _image->fill(Qt::white);
    }
    else if(animationMode == Swirl){

        QPainter painter;

        painter.begin(_image);

        QConicalGradient conicalGrad(17,17, animationStep*10);
        conicalGrad.setColorAt(0, Qt::red);
        conicalGrad.setColorAt(90.0/360.0, Qt::green);
        conicalGrad.setColorAt(180.0/360.0, Qt::blue);
        conicalGrad.setColorAt(270.0/360.0, Qt::magenta);
        conicalGrad.setColorAt(360.0/360.0, Qt::yellow);


        //QLinearGradient linearGrad(QPointF(0, 0), QPointF(0, _image->height()));
        //linearGrad.setColorAt(0, Qt::white);
        //linearGrad.setColorAt(0.3, Qt::darkGray);
        /*linearGrad.setColorAt(0.2, Qt::red);
        linearGrad.setColorAt(0.4, Qt::white);
        linearGrad.setColorAt(0.8, Qt::blue);*/
        //linearGrad.setColorAt(1, Qt::black);

        QBrush fillBrush( conicalGrad );
        QRect fillRect(0,0, 34, _image->height());

        //painter.setBrush(fillBrush);
        painter.fillRect(fillRect, fillBrush);

        painter.end();
    }

    //Update DMX values via OLA
    for(int i=0; i<columns.size(); i++){
        int col = columns[i];
        LedRun* run = _colToLedRun[col];

        DmxAddress addr = run->dmxStart;

        int row = 0;

        if(!run->reverse){
            while(addr.isBefore(run->dmxEnd)){
                //Get pixel data
                QRgb pixelData = _image->pixel(col, row);

                _ola->setPixel(addr, QColor(pixelData));

                row++;
                addr = addr.add(3);
            }
        }
        else{
            addr = run->dmxEnd.add(-2);
            DmxAddress lastAddr = run->dmxStart.add(-1);
            while(addr.isAfter(lastAddr)){
                //Get pixel data
                QRgb pixelData = _image->pixel(col, row);

                _ola->setPixel(addr, QColor(pixelData));


                row++;
                addr = addr.add(-3);
            }
        }
    }

    if(testRun != NULL){
        LedRun* run = testRun;
        DmxAddress addr = run->dmxStart;
        int row = 0;

        while(addr.isBefore(run->dmxEnd)){
            //Get pixel data
            QRgb pixelData = _image->pixel(0, row);

            _ola->setPixel(addr, QColor(Qt::yellow));

            row++;
            addr = addr.add(3);
        }
    }


    if(_outputLabel != NULL){

        QPixmap pix = QPixmap::fromImage(*_image);

        QSize labelSize = _outputLabel->sizeHint();

        if(labelSize.width() > labelSize.height()){
            _outputLabel->setPixmap( pix.scaledToWidth( labelSize.width() ) );
        }
        else{
            _outputLabel->setPixmap( pix.scaledToHeight( labelSize.height() ) );
        }

        //_outputLabel->setPixmap( pix.scaled(_outputLabel->sizeHint(), Qt::KeepAspectRatio) );
    }
}



QJsonDocument PixelMapper::toJson(){
    QJsonDocument jsonDoc;

    QJsonArray ledRunArray;

    QList<int> columns = _colToLedRun.keys();

    for(int i=0; i<columns.size(); i++){
        int col = columns[i];

        LedRun* ledRun = _colToLedRun[col];

        QJsonObject obj;

        obj.insert("col", QJsonValue(col));
        obj.insert("run", QJsonValue(ledRun->toJson()));

        ledRunArray.push_back( obj );
    }

    jsonDoc.setArray(ledRunArray);

    return jsonDoc;
}

bool PixelMapper::fromJson(QJsonDocument &doc){
    if(!doc.isArray()){
        qWarning() << "PixelMapper::fromJson() - Document does not contain led run array";
        //return false;
    }

    if(doc.isObject()){
        qDebug() << "Doc is an object " << doc.object().keys();
    }

    QJsonArray ledRunArray = doc.array();


    for(int i=0; i<ledRunArray.size(); i++){
        QJsonValue itemValue = ledRunArray.at(i);

        if(!itemValue.isObject()){
            qCritical() << "PixelMapper::fromJson() - Item at index " << i << " is not an object!";
            return false;
        }

        QJsonObject itemObj = itemValue.toObject();

        QJsonValue colValue = itemObj.value("col");
        QJsonValue runValue = itemObj.value("run");

        if(colValue.isUndefined()){
            qCritical() << "PixelMapper::fromJson() - Undefined column";
            return false;
        }

        if(runValue.isUndefined()){
            qCritical() << "PixelMapper::fromJson() - Undefined led run";
            return false;
        }

        LedRun* ledRun = new LedRun();

        QJsonObject runObj = runValue.toObject();

        if( !ledRun->fromJson(runObj) ){
            return false;
        }

        qDebug() << "Inserted col=" << colValue.toVariant().toInt();

        _colToLedRun.insert(colValue.toVariant().toInt(), ledRun);
    }

    qDebug() << "PixelMapper::fromJson() - Loaded " << ledRunArray.size() << " items";
    //

    return true;
}

void PixelMapper::start(){
    _renderTimer->start();

    animationStep = 0;
    animationForward = true;
}

void PixelMapper::stop(){
    _renderTimer->stop();
}
