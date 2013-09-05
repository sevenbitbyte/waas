#ifndef UTILS_H
#define UTILS_H

#include <QtCore>

struct DmxAddress {
    int universe;
    int offset;

    int getGlobalOffset();

    DmxAddress add(int length);

    bool isBefore(DmxAddress& other);

    bool isEqual(DmxAddress& other);

    bool isAfter(DmxAddress& other);

    QString toString();
    QJsonObject toJson() const;

};

#endif // UTILS_H
