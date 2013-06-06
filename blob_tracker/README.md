blob_tracker
===

A ROS node which takes a point cloud and seperates out all distinct blobs using euclidean cluster extraction. Extracted blobs are reported and if similar to previously found blobs additional statistics are computed allowing tracking of individual blobs.


ROS Default Input Topics
---
* /tf
* /point_downsample/points


ROS Output Topics
---
* /blob_tracker/blobs


ROS Services
---
* /blob_tracker/refresh_params


ROS Custom Messages
---
* /blob_tracker/Blob
* /blob_tracker/BlobStamped
* /blob_tracker/BlobList
* /blob_tracker/BlobListStamped


ROS Parameters
--
* /blob_tracker/min_blob_points
* /blob_tracker/min_join_distance
* /blob_tracker/max_join_distance
* /blob_tracker/max_blobs
* /blob_tracker/compute_motion
