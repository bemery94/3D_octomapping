#!/bin/bash
#
#
# Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
# All rights reserved.
#
# This software was developed as a part of an industry based research project on Assistive Robotic Devices.
#
# Author: Liyang Liu
# Date: 11/07/2013
#
# Rev History:
#   0.0 - Liyang Liu
#

CONFIG_FILE=/etc/sb/setup.bash

EXE_NAME=$0

DATE=`date`

if [ -s $CONFIG_FILE ]; then
    source $CONFIG_FILE
else
    echo "$DATE: $EXE_NAME is unable to configuration file $CONFIG_FILE, quitting ." >> $SB_LOG_FILE
    exit 1
fi

echo -e "\n-------------------------------------------" >> $SB_LOG_FILE
echo -e "$DATE: $EXE_NAME starts" >> $SB_LOG_FILE
echo -e "-------------------------------------------\n" >> $SB_LOG_FILE
echo "$DATE: $EXE_NAME using setup file $CONFIG_FILE" >> $SB_LOG_FILE

rm -f $STOP_SB_FILE

echo $(date) "=================$EXE_NAME new session =================" >> $SB_LOG_FILE

echo $(date) "Run fan_mode" >> $SB_LOG_FILE

source ${USER_HOME}/fan_full_speed.sh

echo "$DATE: $EXE_NAME on interface $HOST_IFACE" >> $SB_LOG_FILE

ROS_IP=192.168.0.50

if [ "$ROS_IP" = "" ]; then
    echo "$DATE: $EXE_NAME can't run with empty ROS_IP." >> $SB_LOG_FILE
    exit 1
fi


export ROS_MASTER_URI=http://${ROS_IP}:11311
export ROS_HOSTNAME=${ROS_IP}
export ROSCONSOLE_FORMAT='[${severity} ] - [ ${node} ]@[ ${time} ] : ${message}'

export HOME=${USER_HOME}

echo "$DATE: $EXE_NAME setting ROS_MASTER_URI=$ROS_MASTER_URI" >> $SB_LOG_FILE
echo "$DATE: $EXE_NAME setting ROS_HOSTNAME=$ROS_HOSTNAME" >> $SB_LOG_FILE


#execute hoist launch here
echo $(date) 'Launching sb modules!'
echo $(date) 'Launching sb modules!' >> $SB_LOG_FILE

sleep 10

while [ ! -f $STOP_SB_FILE ];
do
    roslaunch --pid=$LAUNCH_PID_FILE $SB_HOME/launch/data_recorder.launch  
    sleep 2
done

rm -f $LAUNCH_PID_FILE
rm -f $STOP_SB_FILE








