#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

MAP_FILE_NAME = "/tmp/ros_merge_map"

def callback(map_data):
    # Based on this: https://github.com/ros-planning/navigation/blob/melodic-devel/map_server/src/map_saver.cpp

    threshold_occupied = 65
    threshold_free = 25
    f = open(MAP_FILE_NAME + ".pgm", "w")

    f.write("P5\n# CREATOR: my_map_saver.py {} m/pix\n{} {}\n255\n".format(map_data.info.resolution, 
                                                                            map_data.info.width,
                                                                            map_data.info.height))

    for y in range(map_data.info.height):
        for x in range(map_data.info.width):
            i = x + (map_data.info.height - y - 1) * map_data.info.width

            if map_data.data[i] >= 0 and map_data.data[i] <= threshold_free:
                f.write(chr(254))
            elif map_data.data[i] >= threshold_occupied:
                f.write(chr(0))
            else:
                f.write(chr(205))

    f.close()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('map', OccupancyGrid, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()