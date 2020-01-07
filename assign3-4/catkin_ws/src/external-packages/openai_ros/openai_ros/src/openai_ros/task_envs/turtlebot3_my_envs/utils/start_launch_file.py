import roslaunch
import rospy

rospy.init_node('two_robot_mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/internal-packages/coop_mapping/launch/spawn_2_robots_mapping.launch"])
launch.start()
rospy.loginfo("started")

rospy.sleep(3)
# 3 seconds later
launch.shutdown()