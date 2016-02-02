# laser_scanner repository

## Overview

This repository contains packages that records imu, camera and laser scan data in order to produce a 3D map of the surrounding environment. The purpose of each package is as follows (see the readme.md file in each package for more details):

* ```uts_sensor_box```

This package provides functionality to run the laser scanner hardware and also to record the imu, camera and laser scan data.

* ```hector_mapping_implementation```

This package produces a 2D map of the surrounding environment using the horizontal laser scanner.

* ```laser_assembler_implementation```

This package produces a 3D map of the surrounding environment using the vertical laser scanner.

* ```master_laser_scanner```

This package provides a single launch file that can run all of the data processing/mapping functionality.

### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile using:

    cd catkin_workspace/src
    git clone https://codeine.research.uts.edu.au/b.emery94/laser_scanner.git
    cd ../
    catkin_make

Note. This will clone the entire workspace.