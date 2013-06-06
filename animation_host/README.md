animation_host
===
A ROS node which computes an animation stream based upon incoming tracked blobs.


ROS Input Topics
---
* /blob_tracker/blobs


ROS Output Topics
---
* /animation_host/frame
* /animation_host/status


ROS Services
---
* /animation_status/refresh_params


ROS Parameters
---
* /animation_status/selected_animation
* /animation_status/blob_topic

