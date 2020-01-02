# catkin_ws

## TODO

- Figure out problem with rospy.log* and try to understand where the program is hanging.
- Create simple RL environment for ONE robot
    - Try to use the OPEN AI ROS environments
        - Don't forget the `max_episode_steps` problem and the `.` and `:` problem in `task_envs_list.py`.
    - Try to make the observation include the current map that the robot is building.
    - Try to disable Gazebo's ODE (physics engine) to accelerate learning.
        - Maybe [this](https://answers.ros.org/question/218744/gazebo-disable-physics-or-enable-kinematics-through-codecommand-line/)?
        - Or [this](http://gazebosim.org/tutorials?tut=preset_manager&cat=physics).
        - Or [this](http://answers.gazebosim.org/question/14384/changing-physics-engine-through-urdf/).
        - Or [this](https://answers.ros.org/question/218744/gazebo-disable-physics-or-enable-kinematics-through-codecommand-line/).
- Scale single RL environment to two robots
- Say that `task_envs` in `openai_ros` has been changed to accomodate our tasks inside `turtlebot3_my_envs`.

## Debugging a ROSPY script

1. Install the ROS extension for VS Code
1. Make sure your root directory in VS Code is the Catkin workspace (in this case, the `catkin_ws` folder).
1. Create a `ROS: Attach` debug configuration.
1. Start ROS Core and the other services you need **except** the script.
1. Start the script. Make sure you have a loop in the beginning of your Python script to wait for the debugger to attach.
1. Run the debug configuration, and select the process corresponding to your script.
1. Exit the loop and debug!