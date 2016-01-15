#!/bin/bash
#
# install.bash
#
# Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
# All rights reserved.
#
# This software was developed as a part of an industry based research project on Assistive Robotic Devices.
#
# Author: Liyang Liu
# Date: 25/06/2013
#
# Rev History:
#       0.0 - Liyang Liu
#


#
# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash [<ip_address>] [<net_if>]
#
# where 
# <ip_address> is the dedicated ip address of your host, and must be a value other than 127.0.0.1 for
# communication with android tablet 
# <net_if> is whatever network interface you want to set the robot
# up for.  wlan0 is the default.
#

HOST_NAME=odrobox
USER_NAME=odroid
USER_PASSWORD=odroid
USER_HOME=/home/${USER_NAME}
ros_setup_file=${USER_HOME}/ros/Indigo/devel/setup.bash
SB_LOG_FILE=${USER_HOME}/log/sensor_box.log
SB_HOME=${USER_HOME}/ros/Indigo/src/uts_sensor_box
STOP_SB_FILE=/var/sensor_box/stop.txt
LAUNCH_PID_FILE=/var/sensor_box/sb_launch_pid

CONFIG_FILE=/etc/sb/setup.bash
IP_ADDRESS=127.0.0.1
#START_SERVICE_FILE=/usr/sbin/sb_start_bootstrap.sh

launch_pid_dirname=`dirname $LAUNCH_PID_FILE`

echo "Installing start, stop, service scripts ..."
cp sb_start.sh  /usr/sbin/sb_start.sh
chmod +x /usr/sbin/sb_start.sh

cp sb_stop.sh  /usr/sbin/sb_stop.sh
chmod +x /usr/sbin/sb_stop.sh

cp sb_kill.sh /usr/sbin/sb_kill.sh
chmod +x /usr/sbin/sb_kill.sh

cp sb_shutdown.sh /usr/sbin/sb_shutdown.sh
chmod +x /usr/sbin/sb_shutdown.sh

cp sb_root_shutdown.sh /usr/sbin/sb_root_shutdown.sh
chmod +x /usr/sbin/sb_root_shutdown.sh

cp sb_reboot.sh /usr/sbin/sb_reboot.sh
chmod +x /usr/sbin/sb_reboot.sh

cp find_old.sh /usr/sbin/find_old.sh
chmod +x /usr/sbin/find_old.sh

cp sb_service.conf  /etc/init/sb_service.conf

chmod a+x $SB_HOME/sb_run.sh

#cp "#!/bin/bash" > $START_SERVICE_FILE
#cp "su $USER_NAME sb_start.sh" >> $START_SERVICE_FILE
#chmod +x $START_SERVICE_FILE

#echo "Install device rules ..."
#cp ../udevrules/*.rules /etc/udev/rules.d/

# create folders for runtime operation
echo "Creating folders for runtime operation ..."

mkdir -p ${USER_HOME}/log
chown -R $USER_NAME ${USER_HOME}/log

mkdir -p $launch_pid_dirname
chown -R $USER_NAME $launch_pid_dirname

# Copy files into /etc/ros/fuerte/hoist
mkdir -p `dirname $CONFIG_FILE`
#mkdir -p /etc/sb_hoist/fuerte
# cat sb.launch > /etc/ros/fuerte/sb.launch

#echo '. /opt/ros/fuerte/setup.bash; export ROS_PACKAGE_PATH=/home/hoistpc/ROS:${ROS_PACKAGE_PATH}' > /etc/ros/setup.bash
echo "Creating configuration file at $CONFIG_FILE ..."

echo -e "source ${ros_setup_file}\n" > $CONFIG_FILE
echo -e "export SB_HOME=${SB_HOME}\n" >> $CONFIG_FILE
echo -e "export STOP_SB_FILE=${STOP_SB_FILE}\n" >> $CONFIG_FILE
echo -e "export SB_LOG_FILE=${SB_LOG_FILE}\n" >> $CONFIG_FILE
echo -e "export USER_HOME=${USER_HOME}\n" >> $CONFIG_FILE
echo -e "export HOST_NAME=${HOST_NAME}\n" >> $CONFIG_FILE
echo -e "export LAUNCH_PID_FILE=${LAUNCH_PID_FILE}\n" >> $CONFIG_FILE
echo -e "export USER_NAME=${USER_NAME}\n" >> $CONFIG_FILE
echo -e "export USER_PASSWORD=${USER_PASSWORD}\n" >> $CONFIG_FILE

echo "Done installation !"

