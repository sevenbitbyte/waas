#include "ledrun.h"

LedRun::LedRun()
{
    reverse = false;
}

int LedRun::length(){
    int startPos = dmxStart.getGlobalOffset();
    int endPos = dmxEnd.getGlobalOffset() + 3;

    return (endPos - startPos) / 3;
}

QJsonObject LedRun::toJson() const {
    QJsonObject obj;

    obj.insert("start", dmxStart.toJson());
    obj.insert("end", dmxEnd.toJson());
    obj.insert("reverse", reverse);

    return obj;
}


bool LedRun::fromJson(QJsonObject& obj) {
    QJsonValue dmxStartValue = obj.value("start");
    QJsonValue dmxEndValue = obj.value("end");
    QJsonValue reverseValue = obj.value("reverse");

    if(dmxStartValue.isUndefined() || dmxEndValue.isUndefined()){

        qCritical() << "Missing DMX start or end address";

        return false;
    }

    bool parseSuccess = true;

    QJsonObject dmxStartObj = dmxStartValue.toObject();
    QJsonObject dmxEndObj = dmxEndValue.toObject();

    parseSuccess &= dmxStart.fromJson( dmxStartObj );
    parseSuccess &= dmxEnd.fromJson( dmxEndObj );
    reverse = reverseValue.toBool();

    return parseSuccess;
}
