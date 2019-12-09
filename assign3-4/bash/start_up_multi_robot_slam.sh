#!/bin/sh

# A script to start up the necessary terminals for multi-robot SLAM.

##### Constants

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

# Setup CATKIN WS by running the devel/setup.sh script
. $SCRIPT_PATH/setup_catkin_ws.sh

# Export TB3 model
. $SCRIPT_PATH/setup_turtlebot3_model.sh

## LAUNCHING TERMINALS

# Launch terminal with Gazebo world and robots
x-terminal-emulator -e roslaunch turtlebot3_gazebo multi_turtlebot3.launch

# Wait for Gazebo
sleep 2s

# Launch terminals with Gmapping for each robot
x-terminal-emulator -e "export ROS_NAMESPACE=tb3_0 && roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_0/base_footprint set_odom_frame:=tb3_0/odom set_map_frame:=tb3_0/map"

x-terminal-emulator -e "export ROS_NAMESPACE=tb3_1 && roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_1/base_footprint set_odom_frame:=tb3_1/odom set_map_frame:=tb3_1/map"

x-terminal-emulator -e "export ROS_NAMESPACE=tb3_2 && roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_2/base_footprint set_odom_frame:=tb3_2/odom set_map_frame:=tb3_2/map"

# Launch terminal with multi_map_merge
x-terminal-emulator -e roslaunch turtlebot3_gazebo multi_map_merge.launch

# Launch terminal with RViz
x-terminal-emulator -e rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam.rviz