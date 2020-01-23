import rospy
import tf

from geometry_msgs.msg import Twist
import sys

def generate_move_forward(distance, linear_velocity, namespace=""):
    twist = Twist()
    
    twist.linear.x = linear_velocity
    twist.angular.z = 0.0
    twist.linear.y = 0.0

def get_position_in_map(namespace=""):
    tf_listener = tf.TransformListener()

    my_base_link = namespace + "/base_link"
    my_map = "/map"

    position = None
    quaternion = None

    timeout = 5.0

    # t = rospy.Time(0)
    # tf_listener.waitForTransform(my_base_link, my_map, t, rospy.Duration(timeout))

    while not rospy.is_shutdown():
        try:
            position, quaternion = tf_listener.lookupTransform(my_map, my_base_link, rospy.Time(0))
        except:
            continue

        break

    return position, quaternion

if __name__ == "__main__":
    rospy.init_node('relative_movement_py')

    position, quaternion = get_position_in_map("/tb3_0")

    print("Position: ", position)
    print("Quaternion: ",  quaternion)
    print("Euler from quaternion: ", tf.transformations.euler_from_quaternion(quaternion))


