# laser_assembler_implementation

## Overview

The laser_assembler_implementation package produces a 3D map of the surrounding environment using the vertical laser scanner (i.e. laser_lsm). The conv_laser_to_cloud node converts the laser scans, from laser_lsm, from [LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) into [PointCloud] in the map_world_frame (which is defined by the hector_mapping_implementation package), using the [laser_geometry](http://wiki.ros.org/laser_geometry) package. The PointClouds are then passed into the [laser_assembler](http://wiki.ros.org/laser_assembler) package which joins each [PointCloud] into a 3D map. The call_laser_assembler_srv node then calls the assemble_scans service (from the laser_assembler package) which publishes the map. The conv_cloud_to_cloud2 node then converts the [PointCloud] into a [PointCloud2] message (This message can be used to produce an octomap, which only accepts PointCloud2 msgs).

The laser_assembler_implementation package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Authors:**

Brendan Emery

**Contact:** 

Brendan.Emery@student.uts.edu.au

**Affiliation: Centre for Autonomous Systems (CAS), University of Technology, Sydney (UTS)**

***
## Bugs & Feature Requests

There are no known bugs. Please report bugs and request features by emailing the author.

[ROS]: http://www.ros.org
[PointCloud]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
[PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html