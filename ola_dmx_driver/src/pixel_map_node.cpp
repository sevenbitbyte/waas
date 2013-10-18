#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ola/DmxBuffer.h>
#include <ola/OlaClient.h>
#include <ola/StreamingClient.h>


#include <string>
#include <iostream>
#include <sstream>

#include <QtGui>
#include <QtCore>

//#include "point_downsample/RefreshParams.h"
#include "utils.h"
#include "olamanager.h"
#include "pixelmapper.h"

#include "ola_dmx_driver/RefreshParams.h"

using namespace std;
using namespace ola_dmx_driver;

#define DEFAULT_GLOBE_HEIGHT (3.0f)

ros::NodeHandlePtr _nhPtr;


//Animation_host Publishers
ros::Publisher _framePub;


//Animation host Subscribers
ros::Subscriber _blobSub;

ros::Subscriber _frameSub;

ros::ServiceServer _refreshParamServ;

ros::Publisher _lightVizPub;
tf::TransformListener* _tfListener = NULL;
tf::TransformBroadcaster* _tfBroadcaster = NULL;

//Service callbackes
bool refreshParams(RefreshParams::Request &request, RefreshParams::Response &response);

double loadRosParam(std::string param, double value=0.0f);
void reloadParameters();

void renderImage(const ros::TimerEvent& event);
void publishGlobeTransform(const ros::TimerEvent& event);
void publishGlobeMarkers();

//Members
OlaManager* _olaManager;
PixelMapper* _pixelMapper;


geometry_msgs::Point _globesScale;
tf::Vector3 _globesOrigin;
tf::Quaternion _globesOrientation;

geometry_msgs::Point _globeSpacing;

QImage* _animationHostImage;

//Callbacks
void frameCallback(const sensor_msgs::ImagePtr& frame);
void blobCallback(const visualization_msgs::MarkerArrayPtr& markers);

int main(int argc, char** argv){
    _olaManager = new OlaManager();
    _olaManager->blackout();

    _pixelMapper = new PixelMapper(_olaManager);

    if(!_pixelMapper->fromFile()){
        ROS_ERROR("Failed to load pixel map, exiting!");
        return -1;
    }

    ros::init (argc, argv, "pixel_map_node");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());
    _tfListener = new tf::TransformListener();
    tf::TransformBroadcaster _broadcaster;
    _tfBroadcaster = &_broadcaster;

    _lightVizPub = _nhPtr->advertise<visualization_msgs::MarkerArray> ("/pixel_map_node/globes/markers", 1);
    _framePub = _nhPtr->advertise<sensor_msgs::Image> ("/pixel_map_node/animation/image", 1);

    _frameSub = _nhPtr->subscribe ("/pixel_map_node/animation/image", 1, frameCallback);
    _blobSub = _nhPtr->subscribe("/point_downsample/markers", 1, blobCallback);

    //Load parameters
    reloadParameters();

    //Services
    _refreshParamServ = _nhPtr->advertiseService("/pixel_map_node/refresh_params", refreshParams);



    //Setup animation image
    _animationHostImage = new QImage(34, 34, QImage::Format_RGB32);
    QPainter painter;
    QRect bounds(0,0, 34, 34);
    QBrush fillBrush( QColor(0,0,0) );
    painter.begin(_animationHostImage);
    painter.fillRect(bounds, fillBrush);
    painter.end();

    publishGlobeTransform(ros::TimerEvent());
    publishGlobeMarkers();

    ros::Timer transformTimer = _nhPtr->createTimer(ros::Duration(0.2), publishGlobeTransform);
    ros::Timer renderTimer = _nhPtr->createTimer(ros::Duration(0.033), renderImage);    //30 FPS

    ros::spin();

    delete _pixelMapper;
    delete _olaManager;

	return 0;
}

void renderImage(const ros::TimerEvent& event){
    _pixelMapper->render();
}

void publishGlobeTransform(const ros::TimerEvent& event){
    tf::Transform transform;

    transform.setOrigin( _globesOrigin );
    transform.setRotation( _globesOrientation );
    _tfBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "globes_link"));

    if(_lightVizPub.getNumSubscribers() > 0){
        publishGlobeMarkers();;
    }
}

void publishGlobeMarkers(){
    QMap<int, QPair<QPoint, QRgb> > pixelData = _pixelMapper->getGlobeData();

    visualization_msgs::MarkerArrayPtr markerArray(new visualization_msgs::MarkerArray);

    QMap<int, QPair<QPoint, QRgb> >::iterator pixelIter = pixelData.begin();

    int id=0;
    for(pixelIter; pixelIter != pixelData.end(); pixelIter++){
        visualization_msgs::Marker globeMarker;
        globeMarker.header.frame_id = "/globes_link";
        globeMarker.ns = "pixel_map_node";
        globeMarker.id = id;
        globeMarker.type = visualization_msgs::Marker::CUBE;
        globeMarker.action = visualization_msgs::Marker::ADD;
        globeMarker.pose.position.x = pixelIter.value().first.x() * _globeSpacing.x;
        globeMarker.pose.position.y = pixelIter.value().first.y() * _globeSpacing.y;
        globeMarker.pose.position.z = 0;
        globeMarker.pose.orientation.x = 0.0;
        globeMarker.pose.orientation.y = 0.0;
        globeMarker.pose.orientation.z = 0.0;
        globeMarker.pose.orientation.w = 1.0;
        globeMarker.scale.x = 0.05;
        globeMarker.scale.y = 0.05;
        globeMarker.scale.z = 0.05;
        globeMarker.color.a = 1.0;
        globeMarker.color.r = qRed(pixelIter.value().second);
        globeMarker.color.g = qGreen(pixelIter.value().second);
        globeMarker.color.b = qBlue(pixelIter.value().second);

        markerArray->markers.push_back(globeMarker);

        id++;
    }

    _lightVizPub.publish(markerArray);
}



void frameCallback(const sensor_msgs::ImagePtr& frame) {
    _pixelMapper->updateImage(frame);
}

/*
double loadRosParam(std::string param, double value){
    if(_nhPtr->hasParam( param )){
         _nhPtr->getParam( param, value );
    }
    else{
        _nhPtr->setParam( param, value );
    }

    return value;
}

bool refreshParams(RefreshParams::Request &request, RefreshParams::Response &response){
    reloadParameters();

    return true;
}

void reloadParameters(){
    std::cout << "Reloading parameters ... ";

    std::cout << "done!" << std::endl;
}
*/

void blobCallback(const visualization_msgs::MarkerArrayPtr& markers) {

    if(!_tfListener->canTransform("base_link", "camera_link", ros::Time())){
        std::cout<<"blobCallback() - Can't transform"<<std::endl;
        return;
    }

    QPainter painter;
    painter.begin( _animationHostImage );

    _animationHostImage->fill(QColor());

    for(int i=0; i<markers->markers.size(); i++){
        visualization_msgs::Marker& marker = markers->markers.at(i);

        geometry_msgs::PoseStamped poseInput;
        poseInput.header = marker.header;
        poseInput.pose = marker.pose;

        geometry_msgs::PoseStamped baseLinkPose;
        _tfListener->transformPose("globes_link", poseInput, baseLinkPose);

        if(marker.type == visualization_msgs::Marker::CUBE){

            double widthPx = marker.scale.x * _globesScale.x;
            double depthPx = marker.scale.y * _globesScale.y;

            double centerXPx = (baseLinkPose.pose.position.x * _globesScale.x);
            double centerYPx = (baseLinkPose.pose.position.y * _globesScale.y);

            QRectF bounds(centerXPx - (widthPx/2.0f),
                          centerYPx - (depthPx/2.0f),
                          widthPx,
                          depthPx);

            cout << "(" << baseLinkPose.pose.position.x << "," << baseLinkPose.pose.position.y << ") -> (" <<centerXPx << "," << centerYPx << ")" <<endl;

            QConicalGradient conicalGrad(centerXPx,centerYPx, 0);
            conicalGrad.setColorAt(0, Qt::red);
            conicalGrad.setColorAt(90.0/360.0, Qt::green);
            conicalGrad.setColorAt(180.0/360.0, Qt::blue);
            conicalGrad.setColorAt(270.0/360.0, Qt::magenta);
            conicalGrad.setColorAt(360.0/360.0, Qt::yellow);

            QBrush fillBrush( conicalGrad );
            QPainterPath fillPath;

            fillPath.addEllipse(bounds);

            painter.fillPath(fillPath, fillBrush);
            painter.save();

        }
    }

    //painter.end();



    sensor_msgs::Image frame;

    frame.width = _animationHostImage->width();
    frame.height = _animationHostImage->height();

    frame.header.frame_id = "base_link";
    frame.header.stamp = ros::Time();

    frame.encoding = sensor_msgs::image_encodings::RGB8;

    frame.data.clear();
    frame.step = _animationHostImage->width() * 3;

    for(int j=0; j<_animationHostImage->height(); j++){
        for(int i=0; i<_animationHostImage->width(); i++){

            QRgb pixel = _animationHostImage->pixel(i, j);

            frame.data.push_back( qRed(pixel) );
            frame.data.push_back( qGreen(pixel) );
            frame.data.push_back( qBlue(pixel) );

        }
    }

    _framePub.publish( frame );
}


double loadRosParam(std::string param, double value){
    if(_nhPtr->hasParam( param )){
         _nhPtr->getParam( param, value );
    }
    else{
        _nhPtr->setParam( param, value );
    }

    return value;
}

bool refreshParams(RefreshParams::Request &request, RefreshParams::Response &response){
    reloadParameters();

    return true;
}


void reloadParameters(){
    std::cout << "Reloading parameters ... ";

    //Update position
    _globesOrigin.setX( loadRosParam("/waas/globes/position/x", 0.0f) );
    _globesOrigin.setY( loadRosParam("/waas/globes/position/y", 0.0f) );
    _globesOrigin.setZ( loadRosParam("/waas/globes/position/z", DEFAULT_GLOBE_HEIGHT));

    double deg2radCoef = M_PI / 180.0f;

    //Update orientation
    _globesOrientation.setRPY(
                                deg2radCoef * loadRosParam("/waas/globes/orientation/roll"),
                                deg2radCoef * loadRosParam("/waas/globes/orientation/pitch"),
                                deg2radCoef * loadRosParam("/waas/globes/orientation/yaw")
                              );

    _globesScale.x = loadRosParam("/waas/globes/scale", 1.0f/0.2032f);
    _globesScale.y = _globesScale.x;

    _globeSpacing.x = loadRosParam("/waas/globes/spacing/x", 0.2032);    //Default to 8in
    _globeSpacing.y = loadRosParam("/waas/globes/spacing/y", 0.2032);    //Default to 8in

    std::cout << "done!" << std::endl;
}

