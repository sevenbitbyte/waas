Hacking
===
To hack on waas you will need:
* [Ubunut 13.04](http://www.ubuntu.com/)
* [ROS Groovy](http://www.ros.org/wiki/ROS/Installation)
* [Qt5](http://qt-project.org/wiki/Qt_5.0) via the [Ubuntu SDK](http://developer.ubuntu.com/get-started/) install directions
* [OLA 0.8.28](http://code.google.com/p/open-lighting/downloads/detail?name=ola-0.8.28.tar.gz&can=2&q=) built from source

Progress
---
* point_downsample implementation under way
  * blocked on PCL VoxelFilter::applyFilter crash, work around found needs testing


Todo
---
* Complete point_downsample
  * service calls
  * parameter retrieval
  * port plane detection from fuerte version of height_filter
* Implement blob_tracker
  * Test PCL kdtree perf. with euclidean clustering
  * Develop Blob messages
  * Research trajectory APIs already present in ROS or Qt
* Implement animation_host
  * Leverage Qt animation framework + Python bindings for Qt and ROS
  * Develop status message
* Implement ola_dmx_driver
  * Identify communication datatypes
  * Develop status message
  * Settle on pixel map file format
* Implement waas_control
  * Prototype making ROS service calls
  * Research rviz Qt widget
  * animation visualization
  * Rviz plugin vs. standalone Qt application

* Performance tuning
  * End-to-end latency?
  * Network traffic if distributed?
* System image
  * Ubuntu 12.10 vs. 13.04?
  * ROS with patches
  * OLA compiled with http, c++ client lib, and e1.31 support
    * E1.31 configuration file
  * Qt5


Problems and Work Arounds
---

###point_downsample crashes with "Illegal Instruction" in VoxelFilter::applyFilter()###
* Install PCL from source using instructions found on ansers.ros.org [1]
  [1]: http://answers.ros.org/question/62979/how-do-i-use-pcl-17-with-groovy/
