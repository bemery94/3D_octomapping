#!/bin/bash
#
# sb_shutdown.sh
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
REBOOT=$1

DATE=`date`


#log_file=/tmp/sb_upstart.log
#rm -f $log_file

source $CONFIG_FILE

echo "$DATE: $0 $REBOOT starts ... " >> $SB_LOG_FILE

echo "$DATE: $0 calls sb_root_shutdown.sh $REBOOT ..." >> $SB_LOG_FILE

echo $USER_PASSWORD | sudo -S sb_root_shutdown.sh $REBOOT &

