#include "utils.h"


int DmxAddress::getGlobalOffset(){
    return (universe*512) + offset;
}

DmxAddress DmxAddress::add(int length){
    DmxAddress next;

    next.universe = universe;
    next.offset = offset + length;

    if(next.offset >= 510){
        next.universe += 1;
        next.offset = next.offset % 510;
    }
    else if(next.offset < 0){
        next.universe -= 1;
        next.offset += next.offset + 513;
    }

    return next;
}

QJsonObject DmxAddress::toJson() const {
    QJsonValue universeValue(universe);
    QJsonValue offsetValue(offset);

    QJsonObject obj;

    obj.insert("universe", universeValue);
    obj.insert("offset", offsetValue);

    return obj;
}

bool DmxAddress::fromJson(QJsonObject& obj) {
    QJsonValue universeValue = obj.value("universe");
    QJsonValue offsetValue = obj.value("offset");

    if(universeValue.isUndefined() || offsetValue.isUndefined()){
        qCritical() << "Missing universe or offset";
        return false;
    }

    universe = universeValue.toVariant().toInt();
    offset = offsetValue.toVariant().toInt();

    return true;
}

bool DmxAddress::isBefore(DmxAddress& other){
    if(other.universe > universe){
        return true;
    }

    if(other.universe == universe){
        return other.offset > offset;
    }

    return false;
}

bool DmxAddress::isEqual(DmxAddress& other){
    return (other.universe == universe) && (other.offset == offset);
}

bool DmxAddress::isAfter(DmxAddress& other){
    return (!isEqual(other) && !isBefore(other));
}

QString DmxAddress::toString(){
    QString string;
    QTextStream stream(&string);

    stream << universe << "." << offset;

    return string;
}
