# master_laser_scanner

## Overview

The master_laser_scanner package is used to provide a single launch file which runs all mapping
nodes. At this stage, the launch file launches the 2D hector mapping from the
slam_2D package, the 3D mapping from the slam_3D package and also the laser_conversions package. It
also runs rviz with a custom config file.

The master_laser_scanner package has been tested under [ROS] Indigo and Ubuntu 14.04. This is
research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Authors:**

Brendan Emery

**Contact:** 

Brendan.Emery@student.uts.edu.au

**Affiliation: Centre for Autonomous Systems (CAS), University of Technology, Sydney (UTS)**

***
## Bugs & Feature Requests

There are no known bugs. Please report bugs and request features by emailing the author.

[ROS]: http://www.ros.org
[sensor_msgs/PointCloud]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html