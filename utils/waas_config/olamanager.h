#ifndef OLAMANAGER_H
#define OLAMANAGER_H

#include <QtCore>
#include <QtGui>
#include <QObject>

#include <ola/DmxBuffer.h>
#include <ola/Logging.h>
#include <ola/StreamingClient.h>

#include "utils.h"

class OlaManager : public QObject
{
    Q_OBJECT
    public:
        explicit OlaManager(QObject *parent = 0);

        void updateBuffers(QMap<int,ola::DmxBuffer> data);
        void updateBuffer(int universe, ola::DmxBuffer& data);

        void setPixel(DmxAddress address, QColor color);
        void setPixel(int univ, int offset, QColor color);
        void setValue(int univ, int offset, int value);

        ola::DmxBuffer* getBuffer(int universe);

        void blackout();
        void lightsOn(int value);
        
    signals:

    public slots:
        void sendBuffers();

        

    private:
        ola::StreamingClient* _client;
        QMap<int,ola::DmxBuffer*> _buffers;
};

#endif // OLAMANAGER_H
