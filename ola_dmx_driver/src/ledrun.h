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

        int length();

        QJsonObject toJson() const;
        bool fromJson(QJsonObject& obj);

};

#endif // LEDRUN_H
