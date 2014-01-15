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
* /point_downsample/refresh_params
* /point_downsample/store_params

ROS Parameters
---
* /waas/cloud/position/x
* /waas/cloud/position/y
* /waas/cloud/position/z
* /waas/cloud/orientation/roll
* /waas/cloud/orientation/pitch
* /waas/cloud/orientation/yaw
* /waas/downsample_leaf_size
* /waas/octree_voxel_size
* /waas/background_reset_threshold
* /waas/cluster_join_distance
* /waas/cluster_min_size
* /waas/cluster_max_size




