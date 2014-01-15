Hacking
===
To hack on waas you will need:

* [Ubunut 12.10 or 13.04](http://www.ubuntu.com/)
* [ROS Hydro](http://www.ros.org/wiki/ROS/Installation)
* [OLA 0.8.28](http://code.google.com/p/open-lighting/downloads/detail?name=ola-0.8.28.tar.gz&can=2&q=) built from source
  * [protobuf 2.3 w/headers](https://code.google.com/p/protobuf/downloads/detail?name=protobuf-2.3.0.tar.gz&can=2&q=)

Progress
---
 * point_downsamwple
 * ola_dmx_driver

Todo
---
* Performance tuning
  * End-to-end latency?
  * Network traffic if distributed?
* System image
  * Ubuntu 12.10 vs. 13.04?
  * ROS with source build PCL1.6, see work arounds section
  * OLA compiled with http, c++ client lib, and e1.31 support
    * E1.31 configuration file
  * Qt5

