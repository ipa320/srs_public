#!/bin/bash

#TARGETPATH=$HOME/log_screenshots

# we will specify path as argument to the script
TARGETPATH=$1

# In case we want to store to external USB HDD:
# TARGETPATH=/media/screenshots/log_screenshots

# check if target folder exists and create it if not
if [ ! -d $TARGETPATH ]; then
mkdir -p $TARGETPATH
fi

# set time interval in seconds for taking screenshots
interval=3;

i=0;
while true;
do
  import -window root $TARGETPATH/`date "+%Y-%m-%d_%H-%M-%S_%N"`.png;
  sleep $interval;
  let i=$i+1;
done
