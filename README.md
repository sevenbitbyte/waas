waas
===
This repository represents the software developed in support of the We Are All Stars installation for Critical NW 2013.


Overview
---
This project consists of several modular components developed using [ROS](http://www.ros.org/wiki/), [Qt](http://qt-project.org/), and [OLA](http://www.opendmx.net/index.php/Open_Lighting_Project). Leveraging ROS allows any component to be run remotely allowing for scalability and enforcing clean architecture.


Modules
---
* point_downsample - Responsible for transforming and trimming point cloud data
* blob_tracker - Extracts intersting blobs from downsampled point cloud
* animation_host - Produces image frames based upon isolated point cloud blobs
* ola_dmx_driver - Translates image frames into relevant DMX commands, performs pixel mapping
* waas_control - Controls all stages of point cloud processing, animation, and LED driving. Provides debug and setup utilities


