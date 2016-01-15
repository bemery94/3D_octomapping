This file contains information about using the myahrs_plus_imu software
Author: Dave Hunt

The bulk of the work is done in /myahrs_plus_imu/include/myahrs_plus.hpp written by WithRobot.
This includes serial comms and all the basic commands required for basic IMU data streaming.
Check the available commands/streams in the below files. Useful ones are summarised below.

Source:		http://github.com/withrobot/myAHRS_plus.com

**************Useful Commands****************
To get euler angles:
sensor.cmd_ascii_data_format("RPY")			//change the mode into roll/pitch/yaw
sensor.cmd_mode("AC")						//ASCII and Continuous mode
EulerAngle& e = sensor_data.euler_angle;	//access the euler angles
//e.roll , e.pitch , e.yaw



**************TO INSTALL******************
TODO




***************TO RUN*********************
TODO
