#! /usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# from tf import transformations
# from datetime import datetime

# Util imports
import random
import math
import time
import platform
import sys

# The topic to which the sensor's plugin is publihing results
# (from the gazebo.xacro file)
SENSOR_TOPIC = '/reactive_bot/laser/scan'
CONTROLLER_TOPIC = '/reactive_bot/controller/position'

# The sensor update rate in Hz (from the gazebo.xacro file)
SENSOR_UPDATE_RATE = 20


def laser_scan_callback(message):
    pass


def main():

    print("Running controller script with Python version: {}".format(
        platform.python_version()))

    print(sys.executable)

    print('Creating publisher and subscriber...')

    # Publishers/Subscribers API detailed here:
    # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    laser_scan_subscriber = rospy.Subscriber(
        SENSOR_TOPIC, LaserScan, callback=laser_scan_callback, queue_size=1)

    controller_publisher = rospy.Publisher(
        CONTROLLER_TOPIC, Twist, queue_size=1)

    print(laser_scan_subscriber)
    print(controller_publisher)


if __name__ == '__main__':
    main()
