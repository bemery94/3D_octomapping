# laser_scanner repository

## Overview

This repository contains packages that records and processes imu, camera, tf and laser scan data in
order to produce a 3D map of the surrounding environment. The purpose of each package is as follows
(see the readme.md file in each package for more details):

* ```uts_sensor_box```

This package provides functionality to run the laser scanner hardware and also to record the imu,
camera, tf and laser scan data.

* ```slam_2D```

This package produces a 2D map of the surrounding environment using the horizontal laser scanner. It
uses hector_mapping to produce a 2D map and localise, giving the robot pose relative to the world
frame.

* ```slam_3D```

This package produces a 3D map of the surrounding environment using the robot pose from the
slam_2D package and the vertical laser scanner. It uses the octomap package to produce a map.

* ```laser_conversions```

This package uses the laser_assembler package to assemble vertical laser scans over periods of 1
second to feed into the slam_3D package.

This package also has functionality to perform transformations of message types between LaserScan,
PointCloud, PointCloud2. It can also combine the horizontal and vertical laser scanners into a
single point cloud. (Note. these functions are not currently being used).

* ```master_laser_scanner```

This package provides a single launch file that can run all of the data processing/mapping
functionality.

### Building

In order to install, clone the latest version from this repository into your catkin workspace and
compile using:

    cd catkin_workspace/src
    git clone https://codeine.research.uts.edu.au/b.emery94/laser_scanner.git
    cd ../
    catkin_make -j1

Note. This will clone the entire workspace.
