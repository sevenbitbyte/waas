#ifndef PIXELMAPPER_H
#define PIXELMAPPER_H

#include <QObject>
#include <QtCore>
#include <QtGui>

#include <QLabel>

#include "utils.h"
#include "ledrun.h"
#include "olamanager.h"

class PixelMapper : public QObject
{
        Q_OBJECT
    public:
        explicit PixelMapper(OlaManager* ola, QObject *parent = 0);
        
        QPicture* getPicture();

        void insertRun(int column, LedRun* run);
        bool isRunning();

        QJsonDocument toJson();
        bool fromJson(QJsonDocument& doc);

        LedRun* testRun;
        QLabel* _outputLabel;

    signals:
        

    public slots:
        void setPixel(QPoint position, QColor);
        void render();

        void start();
        void stop();

    private:
        QImage* _image;
        QTimer* _renderTimer;





        QMap<int, LedRun*> _colToLedRun;   //Map image columns to LedRun

        OlaManager* _ola;

        enum AnimationModes { FillWhite, Swirl=1, Space, FillBlack };

        int animationMode;
        int animationStep;
        bool animationForward;
};

#endif // PIXELMAPPER_H
