#ifndef LEDRUN_H
#define LEDRUN_H

#include "utils.h"
#include <QtGui>

class LedRun
{
    public:
        LedRun();

        //int dmxPixels();
        //int imgPixels();

        bool reverse;

        DmxAddress dmxStart;
        DmxAddress dmxEnd;

        QJsonObject toJson() const;
        bool fromJson(QJsonObject& obj);

        /*QPoint imgStart;
        QPoint imgEnd;*/
};

#endif // LEDRUN_H
