#!/bin/bash

CONFIG_FILE=/etc/sb/setup.bash

EXE_NAME=$0

DATE=`date`

if [ -s $CONFIG_FILE ]; then
    source $CONFIG_FILE
else
    echo "$DATE: $EXE_NAME is unable to configuration file $CONFIG_FILE, quitting ." 
    exit 1
fi

find_old.sh $USER_HOME/.ros 30 | xargs rm -fr
