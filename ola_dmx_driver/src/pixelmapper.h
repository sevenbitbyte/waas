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
        
        void updateImage(const QImage& image);
        void updateImage(const sensor_msgs::ImagePtr& rosImage);
        QImage* getImage();

        void insertRun(int column, LedRun* run);

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

        /**
         * @brief   Returns true if the image has been modified since the last time it was rendered
         * @return
         */
        bool isDirty() const;

        int width() const;
        int height() const;

        /**
         * @brief   Creates a mapping of globe positions and values
         * @return
         */
        QMap<int, QPair<QPoint, QRgb> > getGlobeData() const;

    signals:
        

    public slots:
        /**
         * @brief Send image to OLA pixel display
         */
        void render();


    private:
        QMutex _imageLock;
        QImage* _image;
        bool _imageDirty;

        QColor _backgroundColor;

        QMap<int, LedRun*> _colToLedRun;   //Map image columns to LedRun
        OlaManager* _ola;
};

#endif // PIXELMAPPER_H
