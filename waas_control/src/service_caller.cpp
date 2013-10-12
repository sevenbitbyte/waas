#include "service_caller.h"
#include "point_downsample/RefreshParams.h"

ServiceCaller::ServiceCaller(ros::NodeHandlePtr nodePtr, QObject* parent)
    :QObject(parent)
{
    _nhPtr = nodePtr;
}

ServiceCaller::~ServiceCaller() {
    //
}

void ServiceCaller::paramRefreshSlot() {
    ros::ServiceClient client = _nhPtr->serviceClient<point_downsample::RefreshParams::Request>("refresh_params");

    point_downsample::RefreshParams srv;

    if(client.call(srv)){
        qDebug() << "Refresh success";
    }
    else{
        qDebug() << "Refresh failure";
    }
}
