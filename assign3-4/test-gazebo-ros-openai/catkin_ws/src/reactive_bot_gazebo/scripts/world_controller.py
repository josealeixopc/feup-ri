#! /usr/bin/env python

import os
import platform

# ROS imports
import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import *
from geometry_msgs.msg import *


def main():

    print("Running controller script with Python version: {}".format(
            platform.python_version()))

    rospy.init_node('world_controller_node', disable_signals=True)

    print("Waiting for gazebo services...")
    # Run `rosservice list` to verify the Gazebo service names
    # rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    print("Gazebo has been launched.")

    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    # Get URDF for robot model
    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    reactive_bot_description_path = rospack.get_path('reactive_bot_description')
    reactive_bot_urdf_path = reactive_bot_description_path + os.path.sep + "urdf" + os.path.sep + "reactive_bot.xacro"

    print reactive_bot_urdf_path

    with open(reactive_bot_urdf_path, "r") as f:
        reactive_bot_urdf = f.read()

    
    for i in range(5):
        position = Point(x=i*2, y=0, z=0)
        orientation = Quaternion(x=0, y=0, z=0, w=0)
        model_pose = Pose(position, orientation)

        spawn_model("robot{}".format(i), reactive_bot_urdf, "", model_pose, "world")

if __name__ == '__main__':
    main()
