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
            'i': (1, 0),
            'o': (1, -1),
            'j': (0, 1),
            'l': (0, -1),
            'u': (1, 1),
            ',': (-1, 0),
            '.': (-1, 1),
            'm': (-1, -1),
        }

        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9),
        }

    def print_controls(self):
        msg = """
Control Your Turtlebot!
---------------------------
Moving around:
u    i    o
j    k    l
m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

        print (msg)

    def print_vels(speed, turn):
        print "currently:\tspeed %s\tturn %s " % (speed, turn)

    def get_twist(self, key):
        if key in self.moveBindings.keys():
            self.x = self.moveBindings[key][0]
            self.th = self.moveBindings[key][1]
            self.count = 0
        elif key in self.speedBindings.keys():
            self.speed = self.speed * self.speedBindings[key][0]
            self.turn = self.turn * self.speedBindings[key][1]
            self.count = 0

            self.print_vels(self.speed, self.turn)
            if (self.status == 14):
                self.print_controls()
            self.status = (self.status + 1) % 15
        elif key == ' ' or key == 'k':
            self.x = 0
            self.th = 0
            self.control_speed = 0
            self.control_turn = 0
        else:
            self.count = self.count + 1
            if self.count > 4:
                self.x = 0
                self.th = 0

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
            return self.movement_controller.get_twist('o')
        else:
            return Twist()

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


def calculate_possible_walls(ranges):
    """
    Takes a message from the LaserScan and returns a list with tuples
    (alpha1, alpha2, d), which translates into a possible wall that is
    between angles alpha1 and alpha2 (rad), with closest point at
    distance d (meters)
    """

    # possible_walls = []

    # current_wall_begin_angle = None
    # current_wall_end_angle = None
    # current_wall_closes_distance = None

    # for r in ranges:
    #     break

    # pass


def laser_scan_callback(message):
    global MESSAGE_CURRENT

    if not SENSOR_ALREADY_SETUP:
        set_sensor_settings(message)

    MESSAGE_CURRENT = message  # Set global message to the last one received


def calculate_angle_of_ray(min_angle, max_angle, resolution, angle_index):
    step = (max_angle - min_angle) / resolution
    angle_of_ray = min_angle + step * angle_index

    return angle_of_ray


def act():
    # if STATE_CURRENT
    pass


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
            SENSOR_TOPIC, LaserScan, callback=laser_scan_callback, queue_size=1)

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
