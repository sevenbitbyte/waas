#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>



#include <ola/DmxBuffer.h>
//#include <ola/OlaClient.h>
#include <ola/StreamingClient.h>

#include <string>
#include <iostream>
#include <sstream>

#include <QtGui>
#include <QtCore>

//#include "point_downsample/RefreshParams.h"
#include "animationhost.h"
#include "animations.h"
#include "starfield.h"

#include "ola_dmx_driver/RefreshParams.h"
//#include "starfield.h"

using namespace std;
using namespace ola_dmx_driver;

#define DEFAULT_GLOBE_HEIGHT (3.0f)

ros::NodeHandlePtr _nhPtr;

//Animation_host Publishers
ros::Publisher _framePub;


//Animation host Subscribers
ros::Subscriber _blobSub;


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
QList<BlobInfo*> _pendingBlobs;
QSharedPointer<RenderData> _dataPtr;
BlobTracker* _blobTracker;
AnimationHost* _animationHost;


geometry_msgs::Point _globesScale;
tf::Vector3 _globesOrigin;
tf::Quaternion _globesOrientation;

geometry_msgs::Point _globeSpacing;



//Callbacks
void blobCallback(const visualization_msgs::MarkerArrayPtr& markers);

int main(int argc, char** argv){

    ros::init (argc, argv, "pixel_map_node");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());

    std::string pixelMapPath = QDir::homePath().toStdString();
    pixelMapPath.append("/.waas/pixel_map.json");
    _nhPtr->param("pixel_map", pixelMapPath, pixelMapPath);

    _dataPtr = QSharedPointer<RenderData>( new RenderData );
    _dataPtr->timestamp = ros::Time::now();
    _blobTracker = new BlobTracker(_dataPtr);
    _animationHost = new AnimationHost(QString(pixelMapPath.c_str()), _dataPtr);

    Animation* fill = new FillFade();
    _animationHost->insertLayer(0, fill);

    //Animation* starsim = new StarSim();
    //_animationHost->insertLayer(1, starsim);

    Animation* starPath = new StarPath();
    _animationHost->insertLayer(1, starPath);

    _tfListener = new tf::TransformListener();
    tf::TransformBroadcaster _broadcaster;
    _tfBroadcaster = &_broadcaster;


    //Load parameters
    reloadParameters();


    _lightVizPub = _nhPtr->advertise<visualization_msgs::MarkerArray> ("/pixel_map_node/globes/markers", 1);
    _framePub = _nhPtr->advertise<sensor_msgs::Image> ("/pixel_map_node/animation/image", 1);

    _blobSub = _nhPtr->subscribe("/point_downsample/markers", 1, blobCallback);

    //Services
    _refreshParamServ = _nhPtr->advertiseService("/pixel_map_node/refresh_params", refreshParams);


    publishGlobeTransform(ros::TimerEvent());
    qDebug() << "Published globe TF";


    ros::Timer transformTimer = _nhPtr->createTimer(ros::Duration(0.05), publishGlobeTransform);
    ros::Timer renderTimer = _nhPtr->createTimer(ros::Duration(0.033), renderImage);    //30 FPS

    ros::spin();

    delete _animationHost;
    delete _blobTracker;

	return 0;
}


void renderImage(const ros::TimerEvent& event){
    std::cout << "renderImage()" << std::endl;
    _dataPtr->timestamp = ros::Time::now();

    _blobTracker->updateBlobs( _pendingBlobs );
    _pendingBlobs.clear();

    QImage* image = _animationHost->renderAll();

    if(image == NULL) {
            std::cout << "renderImage() - null" << std::endl;
        return;
    }

    std::cout << "renderImage() - transmit" << std::endl;
    _animationHost->transmit();

    sensor_msgs::Image frame;

    frame.width = image->width();
    frame.height = image->height();

    frame.header.frame_id = "base_link";
    frame.header.stamp = ros::Time();

    frame.encoding = sensor_msgs::image_encodings::RGB8;

    frame.data.clear();
    frame.step = image->width() * 3;

    for(int j=0; j<image->height(); j++){
        for(int i=image->width()-1; i >= 0; i--){

            QRgb pixel = QColor(Qt::black).rgb();
            if(image->width() > i && image->height() < j) {
                pixel = image->pixel(i, j);
            }


            frame.data.push_back( qRed(pixel) );
            frame.data.push_back( qGreen(pixel) );
            frame.data.push_back( qBlue(pixel) );

        }
    }

    _framePub.publish( frame );
    std::cout << "renderImage() - done" << std::endl;
}

void publishGlobeTransform(const ros::TimerEvent& event){
    std::cout << "publishGlobeTransform()" << std::endl;
    tf::Transform transform;

    transform.setOrigin( _globesOrigin );
    transform.setRotation( _globesOrientation );
    _tfBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "globes_link"));

    if(_lightVizPub.getNumSubscribers() > 0){
        publishGlobeMarkers();
    }
    std::cout << "publishGlobeTransform() - done" << std::endl;
}

void publishGlobeMarkers(){
    std::cout << "publishGlobeMarkers()" << std::endl;
    //Collect pixel data
    QMap<int, QPair<QPoint, QRgb> > pixelData = _animationHost->getPixelMapper()->getGlobeData();

    visualization_msgs::Marker globeMarker;
    globeMarker.header.frame_id = "/globes_link";
    globeMarker.ns = "pixel_map_node";
    globeMarker.id = 0;
    globeMarker.type = visualization_msgs::Marker::CUBE;
    globeMarker.action = visualization_msgs::Marker::ADD;
    globeMarker.pose.position.x = 0;
    globeMarker.pose.position.y = 0;
    globeMarker.pose.position.z = 0;
    globeMarker.pose.orientation.x = 0.0;
    globeMarker.pose.orientation.y = 0.0;
    globeMarker.pose.orientation.z = 0.0;
    globeMarker.pose.orientation.w = 1.0;
    globeMarker.scale.x = 0.05;
    globeMarker.scale.y = 0.05;
    globeMarker.scale.z = 0.05;
    /*globeMarker.color.a = 1.0;
    globeMarker.color.r = qRed(pixelIter.value().second);
    globeMarker.color.g = qGreen(pixelIter.value().second);
    globeMarker.color.b = qBlue(pixelIter.value().second);*/


    QMap<int, QPair<QPoint, QRgb> >::iterator pixelIter = pixelData.begin();
    for(; pixelIter != pixelData.end(); pixelIter++){
        //Load position
        geometry_msgs::Point pt;
        pt.x = pixelIter.value().first.x() * _globeSpacing.x;
        pt.y = pixelIter.value().first.y() * _globeSpacing.y;

        globeMarker.points.push_back( pt );

        //Load color
        QColor qColor(pixelIter.value().second);
        std_msgs::ColorRGBA c;
        c.a = 1.0;
        c.r = qColor.redF();
        c.g = qColor.greenF();
        c.b = qColor.blueF();

        globeMarker.colors.push_back(c);
    }

    //Publish
    visualization_msgs::MarkerArrayPtr markerArray(new visualization_msgs::MarkerArray);
    markerArray->markers.push_back(globeMarker);
    _lightVizPub.publish(markerArray);
    std::cout << "publishGlobeMarkers() - done" << std::endl;
}




void blobCallback(const visualization_msgs::MarkerArrayPtr& markers) {

    try{
        if(!_tfListener->canTransform("base_link", "globes_link", ros::Time())){
            std::cout<<"blobCallback() - Can't transform"<<std::endl;
            return;
        }
    }
    catch(...){
        std::cout<<"blobCallback() - Caught transform exception"<<std::endl;
        return;
    }

    std::cout << "blobCallback() with " << markers->markers.size() << std::endl;


    for(unsigned int i=0; i<markers->markers.size(); i++){
        visualization_msgs::Marker& marker = markers->markers.at(i);

        geometry_msgs::PoseStamped poseInput;
        poseInput.header = marker.header;
        poseInput.pose = marker.pose;

        if(marker.type == visualization_msgs::Marker::CUBE){

            geometry_msgs::PoseStamped globeLinkPose;

            try {
                _tfListener->transformPose("globes_link", poseInput, globeLinkPose);
            }
            catch(...){
                std::cout << "TF Error" << std::endl;
                continue;
            }

            double deltaXPx = (marker.scale.x * _globesScale.x) / 1.75f; //1.75 is aestecic not real conversion
            double deltaYPx = (marker.scale.y * _globesScale.y) / 1.75f;
            double deltaZPx = (marker.scale.z * _globesScale.z) / 1.75f;


            double centerXPx = (globeLinkPose.pose.position.x * _globesScale.x);
            double centerYPx = (globeLinkPose.pose.position.y * _globesScale.y);

            BlobInfo* blob = new BlobInfo;

            blob->realDimensions.setValue( marker.scale.x, marker.scale.y, marker.scale.z );
            blob->bounds.setValue( deltaXPx, deltaYPx, deltaZPx );
            blob->centroid.setValue( centerXPx, centerYPx, globeLinkPose.pose.position.z );
            blob->timestamp = ros::Time::now();

            _pendingBlobs.push_back( blob );
        }
    }
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

