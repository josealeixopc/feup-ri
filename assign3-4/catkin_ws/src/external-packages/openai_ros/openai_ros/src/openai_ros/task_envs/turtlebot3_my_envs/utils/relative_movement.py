import rospy
import tf2_ros

from geometry_msgs.msg import Twist
import sys

def generate_move_forward(distance, linear_velocity, namespace=""):
    twist = Twist()
    
    twist.linear.x = linear_velocity
    twist.angular.z = 0.0
    twist.linear.y = 0.0

def get_robot_position_in_map(tf_buffer, namespace=""):
    my_base_link = namespace[1:] + "/base_link" # with tf2, frames cannot start with "/" so we remove it
    my_map = "map"

    position = None
    rotation = None
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        try:
            trans = tf_buffer.lookup_transform(my_map, my_base_link, rospy.Time(0))

            position = [trans.transform.translation.x,
                        trans.transform.translation.y, 
                        trans.transform.translation.z]

            rotation = [trans.transform.rotation.x,
                        trans.transform.rotation.y, 
                        trans.transform.rotation.z]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        break

    return position, rotation

if __name__ == "__main__":
    rospy.init_node('relative_movement_py')

    position, rotation = get_robot_position_in_map("/tb3_0")

    print("Position: ", position)
    print("Rotation: ",  rotation)


