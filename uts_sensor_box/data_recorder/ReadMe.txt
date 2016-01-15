File contains information on the data recording software data_rec
Author: Dave Hunt

Software designed to subscribe to each of the available sensor topics, recording the data to the specified ROS_BAG.

****************TO INSTALL*****************



****************TO RUN*******************
roscore
roslaunch data_recorder data_recorder.launch


*************TROUBLESHOOTING*************
"The hokuyo driver failed"
Probably an issue with udev/naming convention. Check the launch file and fix the /dev/ttyACM? to the systems assigned dev.
