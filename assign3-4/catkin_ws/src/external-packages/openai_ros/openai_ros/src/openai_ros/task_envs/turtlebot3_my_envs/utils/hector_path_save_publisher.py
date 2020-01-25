#!/usr/bin/env python

# Based on: https://answers.ros.org/question/267199/how-can-i-publish-exactly-once-when-the-node-is-run/
# Running `rostopic pub syscommand std_msgs/String "savegeotiff"` from a script.

import rospy

from std_msgs.msg import String

def publish_once(namespace):
    pub = rospy.Publisher('{}/syscommand'.format(namespace), String, queue_size=3)
    msg = String(data="savegeotiff")

    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(msg)
            rospy.loginfo("Published message to savegeotiff.")
            break

        rate.sleep()
    

if __name__ == "__main__":
    rospy.init_node('hector_save_publisher_py')

    publish_once("/tb3_1")
