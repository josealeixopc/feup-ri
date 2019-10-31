#! /usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# from tf import transformations
# from datetime import datetime

# Util imports
# import random
# import math
# import time
import platform
# import sys

# The topic to which the sensor's plugin is publihing results
# (from the gazebo.xacro file)
SENSOR_TOPIC = '/reactive_bot/laser/scan'
CONTROLLER_TOPIC = '/reactive_bot/controller/position'

# The sensor definitions (from the reactive_bot.xacro file,
# sensor definition)
SENSOR_UPDATE_RATE = 20
SENSOR_HORIZONTAL_SAMPLING = 1080
SENSOR_HORIZONTAL_RESOLUTION = 1
SENSOR_HORIZONTAL_MIN_ANGLE = -3.14  # (in radians)
SENSOR_HORIZONTAL_MAX_ANGLE = 3.14  # (in radians)
SENSOR_RANGE_MIN = 0.1  # (in meters)
SENSOR_RANGE_MAX = 10   # (in meters)
SENSOR_RANGE_RESOLUTION = 0.001


def laser_scan_callback(message):
    print(message)
    pass


def main():

    print("Running controller script with Python version: {}".format(
        platform.python_version()))

    rospy.init_node('controller_node')

    print('Creating publisher and subscriber...')

    # Publishers/Subscribers API detailed here:
    # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    laser_scan_subscriber = rospy.Subscriber(
        SENSOR_TOPIC, LaserScan, callback=laser_scan_callback, queue_size=1)

    controller_publisher = rospy.Publisher(
        CONTROLLER_TOPIC, Twist, queue_size=1)

    print(laser_scan_subscriber)
    print(controller_publisher)

    print('Executing controller...')

    num_iterations = 0
    rate = rospy.Rate(SENSOR_UPDATE_RATE)
    while not rospy.is_shutdown():
        num_iterations = num_iterations + 1
        controller_msg = Twist()

        # # State Dispatcher
        # if state_ == 0:
        #     msg = random_wandering()
        # elif state_ == 1:
        #     msg = following_wall()
        # elif state_ == 2:
        #     msg = rotating()
        # else:
        #     rospy.logerr('Unknown state!')

        controller_publisher.publish(controller_msg)

        rate.sleep()


if __name__ == '__main__':
    main()
