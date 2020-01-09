# catkin_ws

## Debugging a ROSPY script

1. Install the ROS extension for VS Code
1. Make sure your root directory in VS Code is the Catkin workspace (in this case, the `catkin_ws` folder).
1. Create a `ROS: Attach` debug configuration.
1. Start ROS Core and the other services you need **except** the script.
1. Start the script. Make sure you have a loop in the beginning of your Python script to wait for the debugger to attach.
1. Run the debug configuration, and select the process corresponding to your script.
1. Exit the loop and debug!