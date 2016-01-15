#!/bin/bash
#
# arf_shutdown.sh
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

IS_TEST=1

EXE_NAME=$0

DATE=`date`

MAX_TABLET_LINK_CHECKS=6

MAX_TABLET_LINK_RESETS=5

MAX_TABLET_PINGS=3

#log_file=/tmp/arf_upstart.log
#rm -f $log_file

CONFIG_FILE=/etc/arf_hoist/setup.bash
source $CONFIG_FILE

function reboot_host ()
{
    if [ $IS_TEST -eq 1 ]; then
        echo "fake reboot" >> $ARF_LOG_FILE
        return
        #exit 2
    else
        sleep 30    # give admin user 30 sec time before reboot to debug system, e.g. delete script to get out of reboot cycle in next start-up

        #echo "$USER_PASSWORD" | sudo -S reboot now
    fi
}

echo -e "\n\n-------------------------------------------">> $ARF_LOG_FILE
echo "$DATE: $EXE_NAME starts ... " >> $ARF_LOG_FILE
echo "-------------------------------------------">> $ARF_LOG_FILE




#--------------------------------------------------------------------------
#
#  Make sure network interface is up
#
#--------------------------------------------------------------------------

# grep the network interface roscore and arf_manager is tied to.
echo -e "\n*** Check host network interface $HOST_IFACE ..." >> $ARF_LOG_FILE
iface_status=`ifconfig | grep '^[a-w]' | awk ' { print $1 } ' | sort | uniq | grep $HOST_IFACE`

# check result
if [ "$iface_status" = "" ]; then
    echo "Network interface $HOST_IFACE not detected, rebooting ..." >> $ARF_LOG_FILE

    reboot_host

else
    echo "=> Detected Network interface $HOST_IFACE successfully"  >> $ARF_LOG_FILE
fi


#--------------------------------------------------------------------------
#
#  Make sure network interface is active
#
#--------------------------------------------------------------------------
echo -e "\n*** Check tablet network interface $TABLET_IFACE ..." >> $ARF_LOG_FILE
num_reset=0
check_success="xxx" # will be set to "0" if check successful
while [ $num_reset -lt $MAX_TABLET_LINK_RESETS ]; do
    check_success="xxx"
    num_check=0
    while [ $num_check -lt $MAX_TABLET_LINK_CHECKS ]; do
        $ARF_HOME/arf_configure/upstart/netiface/ethtest $TABLET_IFACE >> $ARF_LOG_FILE 2>&1
        check_success=$?
        if [ "$check_success" == "0" ]; then  break ; fi
        sleep 0.5
        num_check=$((num_check + 1))
    done
    if [ "$check_success" == "0" ]; then  break ; fi

    num_reset=$((num_reset + 1))
    $ARF_HOME/arf_configure/upstart/netiface/ethtest $TABLET_IFACE  reset >> $ARF_LOG_FILE 2>&1
done

#check result
if [ "$check_success" != "0" ]; then
    echo "Network interface $TABLET_IFACE not ready, rebooting ..." >> $ARF_LOG_FILE

    reboot_host

else
    echo "=> Detected tablet interface $TABLET_IFACE successfully"  >> $ARF_LOG_FILE
fi

#--------------------------------------------------------------------------
#
#  Make sure tablet routes OK
#
#--------------------------------------------------------------------------
echo -e "\n*** Check routing status on tablet network interface $TABLET_IFACE ..." >> $ARF_LOG_FILE
num_ping=0
while [ $num_ping -lt $MAX_TABLET_PINGS ]; do
    ping $TABLET_IP -q -c 10 -w 10  >> $ARF_LOG_FILE 2>&1
    ping_result=$?
    if [ "$ping_result" == "0" ]; then  break ; fi
    num_ping=$((num_ping + 1))
done
if [ "$ping_result" != "0" ]; then
    echo "Ping on $TABLET_IFACE failed, rebooting ..." >> $ARF_LOG_FILE

    reboot_host

else
    echo "=> Routing from host to tablet successful"  >> $ARF_LOG_FILE
fi

echo -e "\n*** Data channel to tablet OK, bootstrap successful !"  >> $ARF_LOG_FILE

