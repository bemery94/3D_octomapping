#!/bin/bash
#
# sb_kill.sh
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
# Set environment
#
CONFIG_FILE=/etc/sb/setup.bash

EXE_NAME=$0

source $CONFIG_FILE
rm -f $STOP_SB_FILE      #start from a fresh state
DATE=`date`

#
# Decide should restart
#
RESTART=0
if [ $# -gt 0 ]; then
    if [ "$1" = "restart" ]; then
        RESTART=1
    fi
fi
echo "$EXE_NAME called with RESTART=$RESTART" >> $SB_LOG_FILE

#
# Kill current sb_launch, communicates to sb service that whether shut-down is required by creating $STOP_SB_FILE
#

if [ $RESTART -eq 1 ]; then
    echo "Restart sb ... " >> $SB_LOG_FILE
else
    echo "Shut-down sb ... " >> $SB_LOG_FILE
fi
touch $STOP_SB_FILE

kill -TERM $(cat $LAUNCH_PID_FILE)
sleep 8

echo "Killed sb_run.sh" >> $SB_LOG_FILE

# 
# for i in $( rosnode list ); do
#     rosnode kill $i;
# done
# 
# killall screen
# killall roslaunch

