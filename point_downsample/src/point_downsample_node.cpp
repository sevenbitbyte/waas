#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// PCL specific includessresetd
#include <pcl/ModelCoefficients.h>
#include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.6/pcl/filters/voxel_grid.h>
//#include <pcl-1.7/pcl/filters/plane_clipper3D.h>
#include <pcl-1.6/pcl/features/normal_3d.h>
#include <pcl-1.6/pcl/filters/extract_indices.h>
#include <pcl-1.6/pcl/filters/passthrough.h>

#include <pcl-1.6/pcl/sample_consensus/model_types.h>
#include <pcl-1.6/pcl/sample_consensus/method_types.h>
#include <pcl-1.6/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.6/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.6/pcl/filters/statistical_outlier_removal.h>

#include <pcl-1.6/pcl/features/normal_3d.h>
#include <pcl-1.6/pcl/kdtree/kdtree.h>
#include <pcl-1.6/pcl/octree/octree.h>
//#include <pcl-1.6/pcl
#include <pcl-1.6/pcl/segmentation/extract_clusters.h>

#include "point_downsample/RefreshParams.h"

//#include <QtGui>

ros::NodeHandlePtr _nhPtr;

ros::Publisher _pointsPub;
ros::Publisher _backgroundPub;
ros::Publisher _foregroundPub;
ros::Publisher _clustersPub;
ros::Publisher _visualizerPub;

ros::Subscriber _pointCloudSub;

ros::ServiceServer _refreshParamServ;

tf::TransformBroadcaster* _tfBroadcaster = NULL;
tf::TransformListener* _tfListener = NULL;



#define DEFAULT_downsample_leaf_size        (0.05f)
#define DEFAULT_octree_voxel_size           (0.2f)
#define DEFAULT_background_reset_threshold  (0.5f)
#define DEFAULT_cluster_join_distance       (0.15f)
#define DEFAULT_cluster_min_size            (200)
#define DEFAULT_cluster_max_size            (3000)

struct CloudProcessParams{
    double downsample_leaf_size;
    double octree_voxel_size;
    double background_reset_threshold;
    double cluster_join_distance;
    double cluster_min_size;
    double cluster_max_size;
};

tf::Vector3 _kinectPosition;
tf::Quaternion _kinectOrientation;
CloudProcessParams _cloudParams;

using namespace point_downsample;

/*Function Prototypes*/
void publishTransform(const ros::TimerEvent& event);
double loadRosParam(std::string param, double value=0.0f);
void reloadParameters();

//Subscriber callbacks
void pointCloudCallback (const sensor_msgs::PointCloud2Ptr& input);

//Service callbackes
bool refreshParams(RefreshParams::Request &request, RefreshParams::Response &response);

//Helper functions
//void updateTransform();

visualization_msgs::MarkerArrayPtr generateMarkers(float centroid[3], float maxValue[3], float minValue[3], int id);

struct point3d {
    point3d(float values[3]){
        data[0]=values[0];
        data[1]=values[1];
        data[2]=values[2];
    }

    union{
        float data[4];
        struct {
          float x;
          float y;
          float z;
        };
    };
};



int main(int argc, char** argv){
    ros::init (argc, argv, "point_downsample_node");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());
    _tfListener = new tf::TransformListener();
    tf::TransformBroadcaster _broadcaster;
    _tfBroadcaster = &_broadcaster;

    //Load parameters
    reloadParameters();

    /*
     *  TODO: Make topics relative rather than absolute
     */

    //Subscribers
    _pointCloudSub = _nhPtr->subscribe ("/camera/depth/points", 1, pointCloudCallback);

    //Publishers
    _pointsPub = _nhPtr->advertise<sensor_msgs::PointCloud2> ("/point_downsample/points", 1);
    _backgroundPub = _nhPtr->advertise<sensor_msgs::PointCloud2> ("/point_downsample/background", 1);
    _clustersPub = _nhPtr->advertise<sensor_msgs::PointCloud2> ("/point_downsample/clusters", 1);
    _foregroundPub = _nhPtr->advertise<sensor_msgs::PointCloud2> ("/point_downsample/foreground", 1);
    _visualizerPub = _nhPtr->advertise<visualization_msgs::MarkerArray>( "/point_downsample/markers", 0 );

    //Services
    _refreshParamServ = _nhPtr->advertiseService("/point_downsample/refresh_params", refreshParams);


    ros::Timer timer = _nhPtr->createTimer(ros::Duration(0.2), publishTransform);

    //Lift off
    ros::spin();
}

void publishTransform(const ros::TimerEvent& event){
    tf::Transform transform;

    transform.setOrigin( _kinectPosition );
    transform.setRotation( _kinectOrientation );
    _tfBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr backgroundCloud;
sensor_msgs::PointCloud2 backgroundSensor;

void pointCloudCallback (const sensor_msgs::PointCloud2Ptr& input) {
    if(input->data.size() <= 0){
        std::cout << "Input cloud size " << input->data.size() << std::endl;
        return;
    }



    sensor_msgs::PointCloud2 downSampledInput;


    //Downsample input point cloud
    float leafSize = _cloudParams.downsample_leaf_size;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> downsample;
    downsample.setInputCloud(input);
    downsample.setLeafSize(leafSize, leafSize, leafSize);
    downsample.filter(downSampledInput);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg (downSampledInput, *pclCloud);

    if(backgroundCloud.get() == NULL) {
        std::cout << "Background cloud size " << input->data.size() << ", downsampled size " << downSampledInput.data.size() <<  std::endl;
        backgroundCloud = pclCloud;
        pcl::toROSMsg(*backgroundCloud, backgroundSensor);

        if(_backgroundPub.getNumSubscribers() > 0) {
            _backgroundPub.publish(backgroundSensor);
        }

        std::cout << "Octree ready" << std::endl;
    }

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree ( _cloudParams.octree_voxel_size );
    octree.setInputCloud(backgroundCloud);
    octree.addPointsFromInputCloud();

    octree.switchBuffers();

    octree.setInputCloud(pclCloud);
    octree.addPointsFromInputCloud();


    std::vector<int> newPointIdxVector;

     // Get vector of point indices from octree voxels which did not exist in previous buffer
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);

    pcl::PointCloud<pcl::PointXYZ> foregroundCloud(*pclCloud, newPointIdxVector);

    float foregroundPerecent = (float)foregroundCloud.points.size() / (float)backgroundCloud->points.size();

    if(foregroundPerecent > _cloudParams.background_reset_threshold){
        backgroundCloud.reset();
        std::cout << "Reseting foreground percent=" << foregroundPerecent << std::endl;
    }


    vector<point3d> centroids;

    if(foregroundCloud.points.size() > 0){
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud ( foregroundCloud.makeShared() );


        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance ( _cloudParams.cluster_join_distance );
        ec.setMinClusterSize ( _cloudParams.cluster_min_size );
        ec.setMaxClusterSize ( _cloudParams.cluster_max_size );
        ec.setSearchMethod (tree);
        ec.setInputCloud ( foregroundCloud.makeShared() );
        ec.extract (cluster_indices);

        int index=0;

        pcl::PointCloud<pcl::PointXYZ> clusterCloud;

        visualization_msgs::MarkerArrayPtr markers( new visualization_msgs::MarkerArray );

        //Loop over ever cluster
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

            float maxValues[3] = {-9000, -9000, -9000};
            float minValues[3] = {9000, 9000, 9000};
            float centroid[3] = {0, 0, 0};

            pcl::PointCloud<pcl::PointXYZ> localCluster(foregroundCloud, it->indices);
            clusterCloud += localCluster;

            //Loop over every point
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {

                for(int i=0; i<3; i++){
                    if(foregroundCloud.points[*pit].data[i] > maxValues[i]){
                        maxValues[i] = foregroundCloud.points[*pit].data[i];
                    }

                    if(foregroundCloud.points[*pit].data[i] < minValues[i]){
                        minValues[i] = foregroundCloud.points[*pit].data[i];
                    }

                    centroid[i] += foregroundCloud.points[*pit].data[i];
                }
            }

            for(int i=0; i<3; i++){
                centroid[i] = centroid[i] / it->indices.size();
            }

            centroids.push_back( (point3d) centroid );

            visualization_msgs::MarkerArrayPtr tempMarkers = generateMarkers(centroid, maxValues, minValues, index++);

            markers->markers.insert(markers->markers.begin(), tempMarkers->markers.begin(), tempMarkers->markers.end());
        }

        if(_visualizerPub.getNumSubscribers() > 0 && markers->markers.size() > 0){
            _visualizerPub.publish(markers);
        }

        sensor_msgs::PointCloud2 clusterSensor;
        pcl::toROSMsg(clusterCloud, clusterSensor);

        clusterSensor.header.frame_id = input->header.frame_id;

        _clustersPub.publish(clusterSensor);
    }

    if(_foregroundPub.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 foregroundSensor;
        pcl::toROSMsg(foregroundCloud, foregroundSensor);
        _foregroundPub.publish(foregroundSensor);
    }

    if(_pointsPub.getNumSubscribers() > 0){
        _pointsPub.publish(downSampledInput);
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
    _kinectPosition.setX( loadRosParam("/waas/cloud/position/x") );
    _kinectPosition.setY( loadRosParam("/waas/cloud/position/y") );
    _kinectPosition.setZ( loadRosParam("/waas/cloud/position/z") );

    double deg2radCoef = M_PI / 180.0f;

    //Update orientation
    _kinectOrientation.setRPY(
                                deg2radCoef * loadRosParam("/waas/cloud/orientation/roll"),
                                deg2radCoef * loadRosParam("/waas/cloud/orientation/pitch"),
                                deg2radCoef * loadRosParam("/waas/cloud/orientation/yaw")
                              );

    //Update point cloud processing parameters
    _cloudParams.downsample_leaf_size = loadRosParam("/waas/downsample_leaf_size", DEFAULT_downsample_leaf_size);
    _cloudParams.octree_voxel_size = loadRosParam("/waas/octree_voxel_size", DEFAULT_octree_voxel_size);
    _cloudParams.background_reset_threshold = loadRosParam("/waas/background_reset_threshold", DEFAULT_background_reset_threshold);
    _cloudParams.cluster_join_distance = loadRosParam("/waas/cluster_join_distance", DEFAULT_cluster_join_distance);
    _cloudParams.cluster_min_size = loadRosParam("/waas/cluster_min_size", DEFAULT_cluster_min_size);
    _cloudParams.cluster_max_size = loadRosParam("/waas/cluster_max_size", DEFAULT_cluster_max_size);

    std::cout << "done!" << std::endl;
}


visualization_msgs::MarkerArrayPtr generateMarkers(float centroid[3], float maxValue[3], float minValue[3], int id){
    visualization_msgs::Marker centroidMarker;
    centroidMarker.header.frame_id = "/camera_depth_optical_frame";
    centroidMarker.header.stamp = ros::Time();
    centroidMarker.ns = "point_downsample";
    centroidMarker.id = id;
    centroidMarker.type = visualization_msgs::Marker::SPHERE;
    centroidMarker.action = visualization_msgs::Marker::ADD;
    centroidMarker.pose.position.x = centroid[0];
    centroidMarker.pose.position.y = centroid[1];
    centroidMarker.pose.position.z = centroid[2];
    centroidMarker.pose.orientation.x = 0.0;
    centroidMarker.pose.orientation.y = 0.0;
    centroidMarker.pose.orientation.z = 0.0;
    centroidMarker.pose.orientation.w = 1.0;
    centroidMarker.scale.x = 0.3;
    centroidMarker.scale.y = 0.3;
    centroidMarker.scale.z = 0.3;
    centroidMarker.color.a = 0.3;
    centroidMarker.color.r = 1.0;
    centroidMarker.color.g = 1.0;
    centroidMarker.color.b = 0.0;

    float center[3];
    float range[3];

    for(int i=0; i<3; i++){
        range[i] = maxValue[i] - minValue[i];
        center[i] = (range[i] / 2.0f) + minValue[i];

        //cout << "range=" << range[i] << endl;
    }

    visualization_msgs::Marker boundsMarker;
    boundsMarker.header.frame_id = "/camera_depth_optical_frame";
    boundsMarker.header.stamp = ros::Time();
    boundsMarker.ns = "point_downsample";
    boundsMarker.id = id+100;
    boundsMarker.type = visualization_msgs::Marker::CUBE;
    boundsMarker.action = visualization_msgs::Marker::ADD;
    boundsMarker.pose.position.x = center[0];
    boundsMarker.pose.position.y = center[1];
    boundsMarker.pose.position.z = center[2];
    boundsMarker.pose.orientation.x = 0.0;
    boundsMarker.pose.orientation.y = 0.0;
    boundsMarker.pose.orientation.z = 0.0;
    boundsMarker.pose.orientation.w = 1.0;
    boundsMarker.scale.x = range[0];
    boundsMarker.scale.y = range[1];
    boundsMarker.scale.z = range[2];
    boundsMarker.color.a = 0.2;
    boundsMarker.color.r = 0.0;
    boundsMarker.color.g = 0.0;
    boundsMarker.color.b = 1.0;


    visualization_msgs::MarkerArrayPtr markerArray( new visualization_msgs::MarkerArray );


    markerArray->markers.push_back(centroidMarker);
    markerArray->markers.push_back(boundsMarker);

    return markerArray;
}


float getValueByRange(float upper, float lower, float percent, bool reverse){
    float value = (upper - lower) * percent;

    if(reverse){
        value = upper - value;
    }
    else{
        value = lower + value;
    }

    return value;
}
