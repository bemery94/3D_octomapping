# laser_conversions 

## Overview

The laser_conversions package produces a 3D map of the surrounding environment using the vertical laser scanner (i.e. laser_lsm) and horizontal laser scanner (laser_lsl). The conv_laser_to_cloud node converts the laser scans, from laser_lsm, from [LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) into [PointCloud], using the [laser_geometry](http://wiki.ros.org/laser_geometry) package. The conv_cloud_to_cloud2 node then converts the [PointCloud] into a [PointCloud2] message (This message can be used to produce an octomap, which only accepts PointCloud2 msgs).

The laser_conversions package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

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
