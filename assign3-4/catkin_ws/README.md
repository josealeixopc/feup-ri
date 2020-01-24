# catkin_ws

## coop_mapping

### Environmental variables

The `ROS_WS` environment variable is **mandatory** and should be set to the ROS workspace path of the machine. By default, it is set to my own machine, so you'll need to change it to run in a different machine.

The `ENV` variable determines what gets launched during training. The default value is `dev-no-gazebo`. These are the available values:

- `dev-no-gazebo` launches including RViz but runs Gazebo headless (runs only `gzserver` and not `gzclient`).
- `dev-gazebo` launches RViz **and** Gazebo client (Gazebo client is quite heavy, so processing will be a lot slower).
- `deploy` does not launch RViz nor Gazebo client, so that the training may run in a headless server. 

### Running



## Debugging a ROSPY script

1. Install the ROS extension for VS Code
1. Make sure your root directory in VS Code is the Catkin workspace (in this case, the `catkin_ws` folder).
1. Create a `ROS: Attach` debug configuration.
1. Start ROS Core and the other services you need **except** the script.
1. Start the script. Make sure you have a loop in the beginning of your Python script to wait for the debugger to attach.
1. Run the debug configuration, and select the process corresponding to your script.
1. Exit the loop and debug!