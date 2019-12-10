# Gazebo Multi-Robot SLAM

## TODO

- Create simple RL environment for ONE robot
    - Try to use the OPEN AI ROS environments
        - Don't forget the `max_episode_steps` problem and the `.` and `:` problem in `task_envs_list.py`.
- Scale single RL environment to two robots

## Tips and Warnings for Development

- Some problems can be solved by reading/downloading the code of the `openai_ros`, `open_ai_gym_construct` and `openai_examples_projects` from the [Construct BitBucket](https://bitbucket.org/theconstructcore/).
    - The `turtlebot3` package in this repo is the original one from ROBOTIS. However, the one OpenAI_ROS works with is the one they have in their own BitButcket: [turtlebot3](https://bitbucket.org/theconstructcore/turtlebot3/src/master/).
- Generate the `requirements.txt` command by running `pip freeze -l > requirements.txt`.
    - The `-l` flag makes sure that only packages inside the virtualenv are put in the file. As `PYTHONPATH` is set by ROS and adds many packages to the `pip list`, you'll need to use the flag to cut out the ROS packages.

## Creation

- Install TurtleBot3-related packages.
- Follow the steps on section 11.2.1.6. of [this page](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo).

## Running

### Warnings

- The first time the `turtlebot3_house.world` is used in Gazebo, it takes quite a long time to start, because it is downloading the map.

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
