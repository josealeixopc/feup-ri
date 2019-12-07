# Gazebo Multi-Robot SLAM

## TODO

- Create simple RL environment for ONE robot
- Scale single RL environment to two robots

## Warnings

-  The first time the `turtlebot3_house.world` is used in Gazebo, it takes quite a long time to start, because it is downloading the map.

## Creation

- Install TurtleBot3-related packages.
- Follow the steps on section 11.2.1.6. of [this page](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo).

## Running

### Starting Gazebo and Robots

The following command does several things:

```
roslaunch coop_mapping 2_robots.launch 
```

1. Starts ROSCore if it's not running already
1. Starts the Gazebo World with Gazebo ROS
1. Spawns **two** TurtleBot 3 robots
1. Launches GMapping for each robot
1. Launches Multi Map Merger to merge each robot's map
1. Launches RViz

### Starting the RL environment

TODO
