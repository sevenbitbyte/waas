#ifndef SERVICE_CALLER_H
#define SERVICE_CALLER_H

#include <QtCore>
#include <QtGui>

#include <ros/ros.h>
#include <ros/console.h>


class ServiceCaller : public QObject
{
    Q_OBJECT
    public:
        ServiceCaller(ros::NodeHandlePtr nodePtr, QObject* parent=NULL);
        ~ServiceCaller();

    public slots:
        void paramRefreshSlot();

    private:
        ros::NodeHandlePtr _nhPtr;
};

#endif  //SERVICE_CALLER_H
