#!/bin/sh

# A script to start up the necessary terminals for multi-robot SLAM.

##### Constants

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

CATKIN_WS_PATH="${SCRIPT_PATH}/../../../.." 

cd $CATKIN_WS_PATH"