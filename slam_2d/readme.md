# slam_2D

## Overview

The slam_2D package runs the [hector_mapping](http://wiki.ros.org/hector_mapping) package which
performs 2D SLAM. The hector mapping node subscribes to the laser scans from the horizontal laser
(i.e. laser_lsl) and it publishes a 2D map and the robot's pose relative to a fixed map frame.
The pose of the robot is then used by the slam_3D package for 3D mapping.
 
The slam_2D package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code,
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
