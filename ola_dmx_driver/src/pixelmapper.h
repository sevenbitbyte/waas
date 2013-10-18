#ifndef PIXELMAPPER_H
#define PIXELMAPPER_H

#include <QObject>
#include <QtCore>
#include <QtGui>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "utils.h"
#include "ledrun.h"
#include "olamanager.h"

class PixelMapper : public QObject
{
        Q_OBJECT
    public:
        explicit PixelMapper(OlaManager* ola, QObject *parent = 0);
        
        void updateImage(const sensor_msgs::ImagePtr& rosImage);
        //QPicture* getPicture();

        void insertRun(int column, LedRun* run);
        bool isRunning();

        QJsonDocument toJson();
        bool fromJson(QJsonDocument& doc);

        /**
         * @brief   Read the pixel map from the specified file. If file path is empty
         *          defaults to "~/.waas/pixel_map.json"
         * @param filePath
         * @return
         */
        bool fromFile(QString filePath=QString());

        void clearImage(QColor color=QColor());
        void setSize(int width, int height);

        int width() const;
        int height() const;

        /**
         * @brief   Creates a mapping of globe positions and values
         * @return
         */
        QMap<int, QPair<QPoint, QRgb> > getGlobeData() const;

    signals:
        

    public slots:
        void setPixel(QPoint position, QColor);
        void render();


    private:
        QMutex _imageLock;
        QImage* _image;


        QMap<int, LedRun*> _colToLedRun;   //Map image columns to LedRun

        OlaManager* _ola;

};

#endif // PIXELMAPPER_H
