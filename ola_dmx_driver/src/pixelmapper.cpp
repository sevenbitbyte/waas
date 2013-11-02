#include "pixelmapper.h"

PixelMapper::PixelMapper(OlaManager* ola, QObject *parent) :
    QObject(parent)
{
    _ola = ola;
    _image = new QImage(32, 32, QImage::Format_RGB32);
    clearImage();
}

void PixelMapper::clearImage(QColor color){
    QPainter painter;
    QRect bounds(0, 0, _image->width()-1, _image->height()-1);
    QBrush fillBrush( color );

    painter.begin(_image);
    painter.fillRect(bounds, fillBrush);
    painter.end();
    _imageDirty = true;
}


void PixelMapper::setSize(int width, int height){
    _imageLock.lock();
    delete _image;
    _image = new QImage(width, height, QImage::Format_RGB32);
    _imageLock.unlock();

    clearImage();
}

bool PixelMapper::isDirty() const {
    return _imageDirty;
}

int PixelMapper::width() const {
    return _image->width();
}

int PixelMapper::height() const {
    return _image->height();
}

void PixelMapper::updateImage(const QImage &image){
    _imageLock.lock();

    if(_image != NULL){
        delete _image;
    }

    _image = new QImage(image);

    _imageDirty = true;
    _imageLock.unlock();
}

void PixelMapper::updateImage(const sensor_msgs::ImagePtr& rosImage){
    int step = rosImage->step;

    _imageLock.lock();

    for(int y=0; y < rosImage->height; y++){
        for(int x=0; x < rosImage->width; x++){
            int idx = (y*step) + (x*3);

            QColor c(   rosImage->data[idx],
                        rosImage->data[idx+1],
                        rosImage->data[idx+2]);

            QRgb oldColor = _image->pixel(x,y);

            c.setRed( (qRed(oldColor) + c.red()) / 2 );
            c.setGreen( (qGreen(oldColor) + c.green()) / 2 );
            c.setBlue( (qBlue(oldColor) + c.blue()) / 2 );

            _image->setPixel(x, y, c.rgb());
        }
    }

    _imageDirty = true;
    _imageLock.unlock();
}

QMap<int, QPair<QPoint, QRgb> > PixelMapper::getGlobeData() const {
    QMap<int, QPair<QPoint, QRgb> > globeData;

    QMap<int, LedRun*>::const_iterator runIter =  _colToLedRun.begin();

    for(runIter; runIter != _colToLedRun.end(); runIter++){
        LedRun* run = runIter.value();

        int x = runIter.key();
        int startY = 0;
        int endY = run->length();

        for(int i=startY; i<endY; i++){
            int id = x + (i*width());
            QPoint pt(x, i);

            globeData.insert(id, qMakePair(pt, _image->pixel(pt)));
        }
    }

    return globeData;
}



void PixelMapper::insertRun(int column, LedRun* run){
    _colToLedRun.insert(column, run);
}



void PixelMapper::render(){
    QList<int> columns = _colToLedRun.keys();

    _imageLock.lock();

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

    _ola->sendBuffers();
    _imageDirty = false;
    _imageLock.unlock();
}


void PixelMapper::setBackgroundColor(QColor c) {
    _imageLock.lock();
    _image->fill(c);
    _imageDirty = true;
    _imageLock.unlock();
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

bool PixelMapper::fromFile(QString filePath){
    if(filePath.isEmpty()){
        filePath = QDir::homePath();
        filePath.append("/.waas/pixel_map.json");
    }

    QFile pixelMapFile(filePath);

    if(!pixelMapFile.open(QIODevice::ReadOnly)){
        qCritical() << "PixelMapper::fromFile - Failed to load pixel mape from file: " << filePath;
        return false;
    }

    QByteArray fileContent = pixelMapFile.readAll();

    qDebug() << "Loaded " << fileContent.size() << " bytes from " << filePath;

    QJsonDocument doc=QJsonDocument::fromJson(fileContent);

    bool success = this->fromJson(doc);

    pixelMapFile.close();

    return success;
}

bool PixelMapper::fromJson(QJsonDocument &doc){
    if(!doc.isArray()){
        qWarning() << "PixelMapper::fromJson() - Document does not contain led run array";
        //return false;
    }

    int minWidth = 0;
    int minHeight = 0;

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

        int column = colValue.toVariant().toInt();
        int length = ledRun->length();

        qDebug() << "PixelMapper::fromJson() - Column " << column << " has " << length << " globes";

        //Update image dimensions
        if(column > minWidth){
            minWidth = column;
        }

        if(length > minHeight){
            minHeight = length;
        }



        _colToLedRun.insert(column, ledRun);
    }

    qDebug() << "PixelMapper::fromJson() - Loaded " << ledRunArray.size() << " items";

    //Update image size
    setSize( qMax(minWidth, width()), qMax(minHeight,height()) );

    qDebug() << "PixelMapper::fromJson() - Image dimensions (" << width() << ", " << height() << ")";

    return true;
}
