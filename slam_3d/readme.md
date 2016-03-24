# slam_3D

## Overview

The slam_3D package produces a 3D map of the surrounding environment using the vertical laser
scanner (i.e. laser_lsm). The package calls the octomap_server package which produces an octomap
based on the PointCloud2 message input from the laser_conversions package.

The slam_3D package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code,
expect that it changes often and any fitness for a particular purpose is disclaimed.

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