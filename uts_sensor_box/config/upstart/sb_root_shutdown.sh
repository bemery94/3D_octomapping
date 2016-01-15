#!/bin/bash
#
# sb_root_shutdown.sh
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

# !!!
# This script Requires root previledge
#

EXE_NAME=$0
REBOOT=$1

CONFIG_FILE=/etc/sb/setup.bash

DATE=`date`

USER_PASSWORD=

log_file=/tmp/sb_upstart.log

source $CONFIG_FILE

echo "$DATE: $EXE_NAME $REBOOT" >> $log_file

#
# Permanently shut-down sb
#
#touch $STOP_SB_FILE

#sudo kill -TERM $(cat $LAUNCH_PID_FILE)
echo "$DATE: calling sb_kill.sh to kill sb launch " >> $log_file

setuidgid $USER_NAME sb_kill.sh

#echo "$DATE: stop sb_service " >> $log_file
#sleep 20

# service sb_service stop

if [ "$REBOOT" == "" ]; then
	#echo "$DATE: shutdown $HOST_NAME " >> $log_file
	echo $USER_PASSWORD | sudo -S shutdown -h now
	#shutdown -h +1
else
	#echo "$DATE: reboot $HOST_NAME " >> $log_file
	echo $USER_PASSWORD | sudo -S shutdown -r now
        #sleep 20
fi



# 
# for i in $( rosnode list ); do
#     rosnode kill $i;
# done
# 
# killall screen
# killall roslaunch

