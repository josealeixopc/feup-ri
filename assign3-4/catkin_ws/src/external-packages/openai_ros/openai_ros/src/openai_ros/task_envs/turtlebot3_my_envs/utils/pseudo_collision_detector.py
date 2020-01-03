#! /usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math
import platform

class PseudoCollisionDetector:
    def __init__(self):
        # The topic to which the sensor's plugin is publihing results
        # (from the gazebo.xacro file)
        self._sensor_topic = '/scan'
        self._controller_topic = '/cmd_vel'


        # Some properties can be retrived from the messages that the
        # Laser Scan sends. Because in Gazebo the reading instantenous,
        # the time_incremente and scan_time of the Laser Scan message
        # will always be 0. Therefore we need to set the update rate
        # manually. These are set by the set_sensor_settings function.
        self._sensor_already_setup = False
        self._sensor_horizontal_sampling = None
        self._sensor_horizontal_min_angle = None  # (in radians)
        self._sensor_horizontal_max_angle = None  # (in radians)
        self._sensor_range_min = None # (in meters)
        self._sensor_range_max = None # (in meters)


        # Last message received
        self._last_message_received = None

    # UTILITIES

    def calculate_closest_point(self, message):
        """
        Takes a message from the LaserScan and returns a tuple
        (alpha1, d), which translates into the closest wall point from
        the bot
        """

        closest_point_distance = min(message.ranges)
        closest_point_index = message.ranges.index(closest_point_distance)

        closest_point_angle = self.calculate_angle_of_ray(closest_point_index)

        return (closest_point_angle, closest_point_distance)


    def calculate_angle_of_ray(self, angle_index):
        return self.calculate_angle_of_ray_ext(self._sensor_horizontal_min_angle,
                                        self._sensor_horizontal_max_angle,
                                        self._sensor_horizontal_sampling,
                                        angle_index)


    def calculate_angle_of_ray_ext(self, min_angle, max_angle, resolution, angle_index):
        step = (max_angle - min_angle) / resolution
        angle_of_ray = min_angle + step * angle_index

        return angle_of_ray


    def set_sensor_settings(self, message):
        """
        Using a LaserScan message, it sets up most SENSOR-related
        constants.

        This saves the trouble of changing the constants when
        changing the sensor's XML.
        """

        if self._sensor_already_setup:
            pass

        self._sensor_horizontal_sampling = len(message.ranges)
        self._sensor_horizontal_min_angle = message.angle_min
        self._sensor_horizontal_max_angle = message.angle_max
        self._sensor_range_min = message.range_min
        self._sensor_range_max = message.range_max

        self._sensor_already_setup = True


    def collision_detected(self, message, max_distance_for_collision=0.005):

        if not self._sensor_already_setup:
            self.set_sensor_settings(message)

        self._last_message_received = message  # Set global message to the last one received

        (angle, distance) = self.calculate_closest_point(message)

        if (distance <= self._sensor_range_min + max_distance_for_collision):
            return True
        else:
            return False