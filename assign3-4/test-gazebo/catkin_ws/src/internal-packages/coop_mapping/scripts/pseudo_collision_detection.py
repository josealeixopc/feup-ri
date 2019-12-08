#! /usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# from tf import transformations
# from datetime import datetime

# Util imports
# import random
import math
# import time
import platform
# import sys
import matplotlib.pyplot as plt

# The topic to which the sensor's plugin is publihing results
# (from the gazebo.xacro file)
SENSOR_TOPIC = '/scan'
CONTROLLER_TOPIC = '/cmd_vel'

# The sensor definitions (from the reactive_bot.xacro file,
# sensor definition)
SENSOR_UPDATE_RATE = 5
SENSOR_HORIZONTAL_RESOLUTION = 1
SENSOR_RANGE_RESOLUTION = 0.015

# Some properties can be retrived from the messages that the
# Laser Scan sends. Because in Gazebo the reading instantenous,
# the time_incremente and scan_time of the Laser Scan message
# will always be 0. Therefore we need to set the update rate
# manually. These are set by the set_sensor_settings function.
SENSOR_ALREADY_SETUP = False
SENSOR_HORIZONTAL_SAMPLING = None
SENSOR_HORIZONTAL_MIN_ANGLE = None  # (in radians)
SENSOR_HORIZONTAL_MAX_ANGLE = None  # (in radians)
SENSOR_RANGE_MIN = None  # (in meters)
SENSOR_RANGE_MAX = None   # (in meters)

# The states of the robot
STATE_FIND_WALL = 0
STATE_GO_TO_WALL = 1
STATE_FOLLOW_WALL = 2

# Wall-following settings
WALL_FOLLOW_DISTANCE = 0.5  # in meter
WALL_FOLLOW_DISTANCE_TOLERANCE = 0.3
WALL_FOLLOW_ANGLE_TOLERANCE = 0.05

# Speed settings
SPEED_LINEAR_MAX = 0.2  # in meter/second
SPEED_ANGULAR_MAX = 0.1  # in radian/second

# Last message received
MESSAGE_CURRENT = None

# UTILITIES


def calculate_closest_point(message):
    """
    Takes a message from the LaserScan and returns a tuple
    (alpha1, d), which translates into the closest wall point from
    the bot
    """

    closest_point_distance = min(message.ranges)
    closest_point_index = message.ranges.index(closest_point_distance)

    closest_point_angle = calculate_angle_of_ray(closest_point_index)

    return (closest_point_angle, closest_point_distance)


def calculate_angle_of_ray(angle_index):
    return calculate_angle_of_ray_ext(SENSOR_HORIZONTAL_MIN_ANGLE,
                                      SENSOR_HORIZONTAL_MAX_ANGLE,
                                      SENSOR_HORIZONTAL_SAMPLING,
                                      angle_index)


def calculate_angle_of_ray_ext(min_angle, max_angle, resolution, angle_index):
    step = (max_angle - min_angle) / resolution
    angle_of_ray = min_angle + step * angle_index

    return angle_of_ray


def set_sensor_settings(message):
    """
    Using a LaserScan message, it sets up most SENSOR-related
    constants.

    This saves the trouble of changing the constants when
    changing the sensor's XML.
    """
    global SENSOR_ALREADY_SETUP, SENSOR_HORIZONTAL_SAMPLING
    global SENSOR_HORIZONTAL_MIN_ANGLE, SENSOR_HORIZONTAL_MAX_ANGLE
    global SENSOR_RANGE_MIN, SENSOR_RANGE_MAX

    if SENSOR_ALREADY_SETUP:
        pass

    SENSOR_HORIZONTAL_SAMPLING = len(message.ranges)
    SENSOR_HORIZONTAL_MIN_ANGLE = message.angle_min
    SENSOR_HORIZONTAL_MAX_ANGLE = message.angle_max
    SENSOR_RANGE_MIN = message.range_min
    SENSOR_RANGE_MAX = message.range_max

    SENSOR_ALREADY_SETUP = True


def laser_scan_callback(message):
    global MESSAGE_CURRENT

    if not SENSOR_ALREADY_SETUP:
        set_sensor_settings(message)

    MESSAGE_CURRENT = message  # Set global message to the last one received

    (angle, distance) = calculate_closest_point(message)

    print(distance)
    print(SENSOR_RANGE_MIN)

    tolerance = 0.005

    if (distance <= SENSOR_RANGE_MIN + tolerance):
        print ("Pseudo-collision detected. Laser scan distance has reached minimum range.")

def main():

    global MESSAGE_CURRENT

    try:

        print("Running controller script with Python version: {}".format(
            platform.python_version()))

        rospy.init_node('collision_detection_node', disable_signals=True)

        print('Creating subscriber...')

        # Publishers/Subscribers API detailed here:
        # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        laser_scan_subscriber = rospy.Subscriber(
            SENSOR_TOPIC,
            LaserScan,
            callback=laser_scan_callback,
            queue_size=1)

        print(laser_scan_subscriber)

        print('Executing controller...')

        rate = rospy.Rate(SENSOR_UPDATE_RATE)
        while not rospy.is_shutdown():

            if not SENSOR_ALREADY_SETUP:
                continue

            rate.sleep()

    finally:
        # If any exception is thrown, remember to tell the robot to stop
        pass


if __name__ == '__main__':
    main()