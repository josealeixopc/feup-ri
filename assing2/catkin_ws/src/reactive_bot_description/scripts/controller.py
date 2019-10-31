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

# The topic to which the sensor's plugin is publihing results
# (from the gazebo.xacro file)
SENSOR_TOPIC = '/reactive_bot/laser/scan'
CONTROLLER_TOPIC = '/cmd_vel'

# The sensor definitions (from the reactive_bot.xacro file,
# sensor definition)
SENSOR_UPDATE_RATE = 20
SENSOR_HORIZONTAL_RESOLUTION = 1
SENSOR_RANGE_RESOLUTION = 0.001

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
STATE_FOLLOW_WALL = 1

# Wall-following settings
WALL_FOLLOW_DISTANCE = 0.3  # in meter

# Speed settings
SPEED_LINEAR_MAX = 0.3  # in meter/second
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


class MovementController(object):
    def __init__(self):
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.speed = .5
        self.turn = 1

        self.moveBindings = {
            'f': (1, 0),  # front
            'fr': (1, -1),  # front-right
            'l': (0, 1),  # left
            'r': (0, -1),  # right
            'fl': (1, 1),  # front-left
            'b': (-1, 0),  # back
            'bl': (-1, 1),  # back-left
            'br': (-1, -1),  # back-right
        }

    def print_vels(speed, turn):
        print "currently:\tspeed %s\tturn %s " % (speed, turn)

    def generate_twist(self):
        self.target_speed = self.speed * self.x
        self.target_turn = self.turn * self.th

        if self.target_speed > self.control_speed:
            self.control_speed = min(self.target_speed,
                                     self.control_speed + 0.02)
        elif self.target_speed < self.control_speed:
            self.control_speed = max(self.target_speed,
                                     self.control_speed - 0.02)
        else:
            self.control_speed = self.target_speed

        if self.target_turn > self.control_turn:
            self.control_turn = min(self.target_turn, self.control_turn + 0.1)
        elif self.target_turn < self.control_turn:
            self.control_turn = max(self.target_turn, self.control_turn - 0.1)
        else:
            self.control_turn = self.target_turn

        twist = Twist()
        twist.linear.x = self.control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.control_turn

        return twist

    def get_twist_find_wall(self):
        self.th = 1
        self.x = 1.5

        return self.generate_twist()

    def get_twist_follow_wall(self, message):

        (angle_to_wall, distance_to_wall) = calculate_closest_point(message)

        if angle_to_wall == 0:
            self.th = 0
        elif angle_to_wall > 0:
            self.th = -1
        elif angle_to_wall < 0:
            self.th = 1

        self.x = 1

        return self.generate_twist()


class ReactiveBot(object):
    def __init__(self):
        self.state = STATE_FIND_WALL
        self.movement_controller = MovementController()

    def get_twist(self, message):
        """
        Generates the twist message which the controller will publish
        to the controller topic.
        """
        if self.state == STATE_FIND_WALL:
            return self.movement_controller.get_twist_find_wall()
        elif self.state == STATE_FOLLOW_WALL:
            return self.movement_controller.get_twist_follow_wall(message)
        else:
            print ("State is unkown. Has value '{}'.".format(self.state))
            return Twist

    def is_wall_visible(self, ranges):
        return not all(math.isinf(r) for r in ranges)

    def update_state(self, message):
        if self.is_wall_visible(message.ranges):
            self.state = STATE_FOLLOW_WALL
        else:
            self.state = STATE_FIND_WALL


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


def main():

    global STATE_CURRENT, MESSAGE_CURRENT

    try:

        print("Running controller script with Python version: {}".format(
            platform.python_version()))

        rospy.init_node('controller_node', disable_signals=True)

        print('Creating publisher and subscriber...')

        # Publishers/Subscribers API detailed here:
        # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        laser_scan_subscriber = rospy.Subscriber(
            SENSOR_TOPIC,
            LaserScan,
            callback=laser_scan_callback,
            queue_size=1)

        controller_publisher = rospy.Publisher(
            CONTROLLER_TOPIC, Twist, queue_size=5)

        print(laser_scan_subscriber)
        print(controller_publisher)

        print('Executing controller...')

        bot = ReactiveBot()

        num_iterations = 0
        STATE_CURRENT = STATE_FIND_WALL

        rate = rospy.Rate(SENSOR_UPDATE_RATE)
        while not rospy.is_shutdown():
            if not SENSOR_ALREADY_SETUP:
                continue

            bot.update_state(MESSAGE_CURRENT)
            controller_msg = bot.get_twist(MESSAGE_CURRENT)

            # print controller_msg
            # print "Robot state: {}".format(bot.state)
            controller_publisher.publish(controller_msg)

            rate.sleep()
            num_iterations = num_iterations + 1

    finally:
        # If any exception is thrown, remember to tell the robot to stop
        # Twist() returns all 0's for linear and angular velocity
        controller_publisher.publish(Twist())


if __name__ == '__main__':
    main()
