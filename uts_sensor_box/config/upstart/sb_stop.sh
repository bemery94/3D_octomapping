#!/bin/bash
#
# sb_stop.sh
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

CONFIG_FILE=/etc/sb/setup.bash

EXE_NAME=$0

DATE=`date`

log_file=/tmp/sb_upstart.log

echo "$DATE: $EXE_NAME" >> $log_file


if [ -s $CONFIG_FILE ]; then
    echo "$DATE: $EXE_NAME using setup file $CONFIG_FILE" >> $log_file
    source $CONFIG_FILE
else
    echo "$DATE: $EXE_NAME is unable to configuration file $CONFIG_FILE, quitting ." >> $log_file
    exit 1
fi

#
# Permanently shut-down sb
#
#touch $STOP_SB_FILE

#sudo kill -TERM $(cat $LAUNCH_PID_FILE)
echo "$DATE: calling sb_kill.sh to kill sb_run.sh" >> $log_file

setuidgid $USER_NAME sb_kill.sh

#shutdown -h now

sudo killall screen


# 
# for i in $( rosnode list ); do
#     rosnode kill $i;
# done
# 
# killall screen
# killall roslaunch

