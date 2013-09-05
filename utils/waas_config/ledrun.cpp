#include "ledrun.h"

LedRun::LedRun()
{
    reverse = false;
}


QJsonObject LedRun::toJson() const {
    QJsonObject obj;

    obj.insert("start", dmxStart.toJson());
    obj.insert("end", dmxEnd.toJson());
    obj.insert("reverse", reverse);

    return obj;
}
