#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// PCL specific includessresetd
#include <pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/ros/conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
//#include <pcl-1.7/pcl/filters/plane_clipper3D.h>
#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/filters/passthrough.h>

#include <pcl-1.7/pcl/sample_consensus/model_types.h>
#include <pcl-1.7/pcl/sample_consensus/method_types.h>
#include <pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.7/pcl/filters/statistical_outlier_removal.h>


#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/kdtree/kdtree.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>

//Message types
#include "blob_tracker/Blob.h"
#include "blob_tracker/BlobStamped.h"
#include "blob_tracker/BlobList.h"
#include "blob_tracker/BlobStampedList.h"

//Service types
#include "blob_tracker/RefreshParams.h"

ros::NodeHandlePtr _nhPtr;

ros::Publisher _blobListPub;
ros::Subscriber _pointCloudSub;

ros::ServiceServer _refreshParamServ;

tf::TransformBroadcaster* _tfBroadcaster = NULL;
tf::TransformListener* _tfListener = NULL;

using namespace blob_tracker;


/*Function Prototypes*/
//Subscriber callbacks
void pointCloudCallback (const sensor_msgs::PointCloud2Ptr& input);

//Service callbacks
bool refreshParams(RefreshParams::Request &request, RefreshParams::Response &response);

int main(int argc, char** argv){
    ros::init(argc, argv, "blob_tracker");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());
    _tfListener = new tf::TransformListener();
    tf::TransformBroadcaster _broadcaster;
    _tfBroadcaster = &_broadcaster;


    _pointCloudSub = _nhPtr->subscribe("/point_downsample/points", 1, pointCloudCallback);

    _blobListPub = _nhPtr->advertise<sensor_msgs::PointCloud2> ("/point_downsample/points", 1);

    _refreshParamServ = _nhPtr->advertiseService("refresh_params", refreshParams);


    ros::spin();
}


void pointCloudCallback (const sensor_msgs::PointCloud2Ptr& input) {
    if(input->data.size() <= 0){
        std::cout << "Input cloud size " << input->data.size() << std::endl;
        return;
    }

    if(_blobListPub.getNumSubscribers() < 1){
        //Short circuit
        return;
    }

    //Convert to native PCL cloud type
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ> tree;// (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::Search<pcl::PointXYZ>::Ptr treePtr(&tree);
    tree.setInputCloud(cloud.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.05); // 5cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod ( treePtr );
    ec.setInputCloud(cloud.makeShared());
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cloud_cluster->points.push_back (cloud.points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        j++;
    }
}


bool refreshParams(RefreshParams::Request &request, RefreshParams::Response &response){
    //TODO

    return false;
}
