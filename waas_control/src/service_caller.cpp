#include "service_caller.h"
#include "point_downsample/RefreshParams.h"
#include "ola_dmx_driver/RefreshParams.h"

ServiceCaller::ServiceCaller(ros::NodeHandlePtr nodePtr, QObject* parent)
    :QObject(parent)
{
    _nhPtr = nodePtr;
}

ServiceCaller::~ServiceCaller() {
    //
}

void ServiceCaller::paramRefreshSlot() {
    ros::ServiceClient client = _nhPtr->serviceClient<point_downsample::RefreshParams::Request>("/point_downsample/refresh_params");

    point_downsample::RefreshParams srv;

    if(client.call(srv)){
        qDebug() << "Point_downsample Refresh success";
    }
    else{
        qDebug() << "Point_downsample Refresh failure";
    }

    ros::ServiceClient olaClient = _nhPtr->serviceClient<ola_dmx_driver::RefreshParams::Request>("/pixel_map_node/refresh_params");

    ola_dmx_driver::RefreshParams olaSrv;

    if(olaClient.call(olaSrv)){
        qDebug() << "ola_dmx_driver Refresh success";
    }
    else{
        qDebug() << "ola_dmx_driver Refresh failure";
    }
}
