# Gazebo Multi-Robot SLAM

## Installation

### Pre-requisites

- Python 2.7
- ROS-Melodic (may work in other distros, but not tested)

### Before running

**Change your working directory to `catkin_ws` and run the following commands.**

Install all dependencies from the `src` folder.

`rosdep install --from-paths src --ignore-src -r -y`

Source your distro `setup.bash`:

`source /opt/ros/$ROS_DISTRO/setup.bash`

Build the package in the `src` folder:

`catkin_make`

Source the new Catkin environment:

`source devel/setup.bash`

You shouldn't need to run anything else, but in case you head into some environment variables related problem run:

`export ENV=devel`

`export ROS_WS=$PWD`

## Running

### Warnings

- The first time the `turtlebot3_house.world` is used in Gazebo, it takes quite a long time to start, because it is downloading the map.
- There may appear some Protobuf related errors that are not so easily solved. But you may look at the `README.md` inside the `src/external-packages/pgm_map_creator-master` folder.

### Starting the RL environment

```
roslaunch coop_mapping 2_robots.launch 
```


### Starting Gazebo and Robots

#### Starting 1 Robot mapping

```
roslaunch coop_mapping 1_robot.launch 
```

1. Starts ROSCore if it's not running already
1. Starts the Gazebo World with Gazebo ROS
1. Spawns **one** TurtleBot 3 robots
1. Launches GMapping for the robot
1. Launches RViz

#### Starting 2 Robot mapping

```
roslaunch coop_mapping 2_robots.launch 
```

1. Starts ROSCore if it's not running already
1. Starts the Gazebo World with Gazebo ROS
1. Spawns **two** TurtleBot 3 robots
1. Launches GMapping for each robot
1. Launches Multi Map Merger to merge each robot's map
1. Launches RViz

## Generating maps from world files

This is done using the `pgm_map_creator` inside `external packages`.

### Add the map and insert the plugin
1. Add your world file to world folder
2. Add this line at the end of the world file, before `</world>` tag:
`<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>`

### Create the pgm map file
1. Open a terminal, run gzerver with the map file
`rosrun gazebo_ros gazebo <catkin_ws_path>/src/pgm_map_creator/world/<map file>`
2. Open another terminal, launch the request_publisher node
`roslaunch pgm_map_creator request_publisher.launch`
3. Wait for the plugin to generate map. It will be located in the map folder


## Tips and Warnings for Development

- Some problems can be solved by reading/downloading the code of the `openai_ros`, `open_ai_gym_construct` and `openai_examples_projects` from the [Construct BitBucket](https://bitbucket.org/theconstructcore/).
    - The `turtlebot3` package in this repo is the original one from ROBOTIS. However, the one OpenAI_ROS works with is the one they have in their own BitButcket: [turtlebot3](https://bitbucket.org/theconstructcore/turtlebot3/src/master/).
- Generate the `requirements.txt` command by running `pip freeze -l > requirements.txt`.
    - The `-l` flag makes sure that only packages inside the virtualenv are put in the file. As `PYTHONPATH` is set by ROS and adds many packages to the `pip list`, you'll need to use the flag to cut out the ROS packages.

## Creation

- Install TurtleBot3-related packages.
- Follow the steps on section 11.2.1.6. of [this page](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo).
