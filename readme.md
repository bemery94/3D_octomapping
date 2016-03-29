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

### Instructions for running code

#### Set up robot

* You can connect to the robot via an ethernet cable. When using Ubuntu, you can set up the connection by clicking on the Wifi/connections button in the tray in the top right of the screen, click "Edit Connections", "Add" and then select "Ethernet" from the drop down menu. Now select the newly created ethernet connection in the Network Connections window, click "Edit", "IPv4 Settings" and in the "Method" drop down box, select "Shared to other computers".

* When the robot is turned on and plugged in via ethernet, ensure that the connection that you created above is selected. You can now type in terminal "ssh odroid@10.42.0.10" to access the robot's odroid.

#### Setting up roscore

* You can either run roscore on the odroid, or you can set your local computer to be the master so the odroid uses your roscore. If you set roscore on the odroid, then you can unplug your computer and the data_recorder package/any other ROS software that is running will continue to run. If you set roscore locally on your computer, you cannot unplug your computer, however you can record bags, echo ros topics etc. that are running on the odroid locally on your computer (Note. you have to source the .bashrc file or restart the terminal every time you make a change to the .bashrc file)
* To run roscore on the odroid, access the ~/.bashrc file on the odroid and comment out the last 2 lines that say: "export ROS_OP=10.42.0.10" and "export ROS_MASTER_URI=http://10.42.0.1:11311". 
* To run roscore locally from your computer, ensure that the 2 lines mentioned above are uncommented. 

#### Running the software

The robot works in 2 stages, the first is recording the data and the second is processing the data to produce the map. They can either be done concurrently or the data can be recorded in a rosbag and replayed at a later stage. 

* To run the data recording, ensure that the workspace has been built and that you have ssh'ed into the odroid. Type "roslaunch data_recorder data_recorder.launch". This will begin publishing the laser scan messages.

* If you would like to build the map in real time, then ensure that roscore is set to your local computer (see "Setting up roscore" above), the parameter /use_sim_time is set to false using "rosparam set /use_sim_time false" and type "roslaunch master_laser_scanner master_laser_scanner.launch" from your computer.

* If you would like to save the data to be played back at a later time, type (either in the odroid's or you local computer's terminal depending on where roscore is set) "rosbag record /scan_lsl /scan_lsm /tf /myahrs_imu". You can then play this bag back at any time by setting /use_sim_time to be true with "rosparam set /use_sim_time true" and typing "rosbag play --clock your_bag.bag"
