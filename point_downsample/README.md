point_downsample
===

ROS node which will perform point cloud down sampling and transformation to keep the cloud correctly oriented using either Imu or ground plane finding.


ROS Default Input Topics
---
* /tf
* /imu
* /camera/depth/points


ROS Output Topics
---
* /point_downsample/points


ROS Services Provided
---
* /point_downsample/refresh_settings
* /point_downsample/set_position
* /point_downsample/set_orientation


ROS Parameters
---
* /point_downsample/input/imu_topic
* /point_downsample/input/cloud_topic
* /point_downsample/orientation/use_imu
* /point_downsample/leaf_size
* /point_downsample/orientation/find_ground
* /point_downsample/orientation/position/x
* /point_downsample/orientation/position/y
* /point_downsample/orientation/position/z




