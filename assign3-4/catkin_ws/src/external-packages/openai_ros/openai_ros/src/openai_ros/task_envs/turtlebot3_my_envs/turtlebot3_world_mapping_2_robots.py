import rospy
import rospkg
import roslaunch
import numpy
from gym import spaces
import turtlebot3_two_robots_env
from gym.envs.registration import register
from geometry_msgs.msg import Vector3
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from utils.pseudo_collision_detector import PseudoCollisionDetector
from utils.image_similarity_ros import compare_current_map_to_actual_map

class TurtleBot3WorldMapping2RobotsEnv(turtlebot3_two_robots_env.TurtleBot3TwoRobotsEnv):
    def __init__(self):
        """
        This Task Env is designed for having two TurtleBot3 robots in the turtlebot3 world closed room with columns.

        It will learn how to move around without crashing.
        """
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there            
        ros_ws_abspath = rospy.get_param("/turtlebot3/ros_ws_abspath", None)
        if os.environ.get('ROS_WS') != None:
            ros_ws_abspath = os.environ.get('ROS_WS')

        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        # Depending on which environment we're in, decide to launch Gazebo with or without GUI.
        gazebo_launch_file = "start_empty_tb3_world.launch"

        if os.environ.get('ENV') == 'deploy':
            gazebo_launch_file = "start_empty_tb3_world_no_gui.launch"

        ROSLauncher(rospackage_name="coop_mapping",
                    launch_file_name=gazebo_launch_file,
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/turtlebot3_my_envs/config",
                               yaml_file_name="turtlebot3_world_mapping.yaml")


        # Here we will add any init functions prior to starting the MyRobotEnv
        super(TurtleBot3WorldMapping2RobotsEnv, self).__init__(ros_ws_abspath, 
                                                        ros_launch_file_package="coop_mapping", 
                                                        ros_launch_file_name="spawn_2_robots.launch")

        # Only variable needed to be set here
        self.number_actions = rospy.get_param('/turtlebot3/n_actions')
        self.number_robots = len(self.robot_namespaces) # should be 2

        # 3x3 possible actions (a % 3 -> robot 1 action, a / 3 -> robot 2 action)
        self.action_space = spaces.Discrete(pow(self.number_actions, self.number_robots)) 
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        
        # Actions and Observations
        self.linear_forward_speed = rospy.get_param('/turtlebot3/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/turtlebot3/linear_turn_speed')
        self.angular_speed = rospy.get_param('/turtlebot3/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/turtlebot3/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/turtlebot3/init_linear_turn_speed')
        
        self.new_ranges = rospy.get_param('/turtlebot3/new_ranges')
        self.min_range = rospy.get_param('/turtlebot3/min_range')
        self.max_laser_value = rospy.get_param('/turtlebot3/max_laser_value')
        self.min_laser_value = rospy.get_param('/turtlebot3/min_laser_value')
        self.max_linear_aceleration = rospy.get_param('/turtlebot3/max_linear_aceleration')
        
        
        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        laser_scan = self.get_laser_scan(self.robot_namespaces[0])
        num_laser_readings = len(laser_scan.ranges)/self.new_ranges
        high = numpy.full((num_laser_readings), self.max_laser_value)
        low = numpy.full((num_laser_readings), self.min_laser_value)
        
        """
        An observation is a MultiDiscrete element with R elements, where R is the number of laser scan rays we are using for observation.
        """

        multi_discrete_shape = [round(self.max_laser_value)] * (self.new_ranges * self.number_robots)

        # We only use two integers
        self.observation_space = spaces.MultiDiscrete(multi_discrete_shape)
        
        rospy.loginfo("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.loginfo("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        self.forwards_reward = rospy.get_param("/turtlebot3/forwards_reward")
        self.turn_reward = rospy.get_param("/turtlebot3/turn_reward")
        self.end_episode_points = rospy.get_param("/turtlebot3/end_episode_points")

        self.no_crash_reward_points = rospy.get_param("/turtlebot3/no_crash_reward_points")
        self.crash_reward_points = rospy.get_param("/turtlebot3/crash_reward_points")
        self.exploration_multi_factor = rospy.get_param("/turtlebot3/exploration_multi_factor")

        self.cumulated_steps = 0.0

        # Init dictionary for both robots actions
        self.last_action = {}

        # Set the logging system
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('coop_mapping')

        # Control variables for launching nodes from .launch files
        self._gmapping_launch_file = pkg_path + os.path.sep + 'launch' + os.path.sep + 'spawn_2_robots_mapping.launch'
        self._gmapping_running = False
        self._gmapping_launch = None

        self._map_merge_launch_file = pkg_path + os.path.sep + 'launch' + os.path.sep + '2_robots_multi_map_merge.launch'
        self._map_merge_running = False
        self._map_merge_launch = None

        # Variables for map comparison
        self.actual_map_file = "turtlebot3_world_map.pgm"
        self.current_min_map_difference = 1 # The minimum difference that has been observed

        # Start subscriber to /map to save it to file
        self._map_file_name = "/tmp/ros_merge_map"
        rospy.Subscriber('map', OccupancyGrid, self._map_callback)


    def _set_init_pose(self):
        """Sets the Robots in its init pose
        """
        for ns in self.robot_namespaces:
            self.move_base( self.init_linear_forward_speed,
                            self.init_linear_turn_speed,
                            ns,
                            epsilon=0.01,
                            update_rate=10)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False

        # (Re)Start GMapping
        self._stop_gmapping()
        self._start_gmapping()

        # Start MapMerge
        self._stop_map_merge()
        self._start_map_merge()

        # Set initial map difference
        # rospy.logwarn("Running initial map comparison.")
        # self.current_min_map_difference = compare_current_map_to_actual_map(self.actual_map_file)
        # rospy.logwarn("Initial map difference: {}".format(self.current_min_map_difference))

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the two TurleBot3 based on the action value.
        """

        robot_actions = {}
        
        # First robot has action -> action % 3
        robot_actions[self.robot_namespaces[0]] = action % self.number_actions
        # Second robot has action -> action // 3
        robot_actions[self.robot_namespaces[1]] = action // self.number_actions
        
        for ns in self.robot_namespaces:
            current_robot_action = robot_actions[ns]
            rospy.loginfo("Start Set Action for Robot {} ==> ".format(ns) + str(current_robot_action))
            # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
            if current_robot_action == 0: #FORWARD
                linear_speed = self.linear_forward_speed
                angular_speed = 0.0
                self.last_action[ns] = "FORWARDS"
            elif current_robot_action == 1: #LEFT
                linear_speed = self.linear_turn_speed
                angular_speed = self.angular_speed
                self.last_action[ns] = "TURN_LEFT"
            elif current_robot_action == 2: #RIGHT
                linear_speed = self.linear_turn_speed
                angular_speed = -1*self.angular_speed
                self.last_action[ns] = "TURN_RIGHT"
            
            # We tell TurtleBot3 the linear and angular speed to set to execute
            self.move_base(linear_speed, angular_speed, ns, epsilon=0.01, update_rate=10)
            
            rospy.loginfo("END Set Action for Robot {} ==>".format(ns)+str(action))

    def _get_obs(self):
        """
        Here we define the observation. In this case, it is composed of both robots laser scans.
        """
        rospy.loginfo("Start Get Observation ==>")
        all_robots_observations = []

        for ns in self.robot_namespaces:
            # We get the laser scan data
            laser_scan = self.get_laser_scan(ns)
            discretized_observations = self.discretize_scan_observation(laser_scan, self.new_ranges)
            all_robots_observations.extend(discretized_observations)

        rospy.loginfo("Observations from all robots==>"+str(all_robots_observations))
        rospy.loginfo("END Get Observation ==>")
        return all_robots_observations
        

    def _is_done(self, observations):
        
        if self._episode_done:
            rospy.logerr("A TurtleBot3 is Too Close to wall==>")
        else:
            rospy.loginfo("No TurtleBot3 is close to a wall ==>")
            
        return self._episode_done

    def _compute_reward(self, observations, done):
        """
        The current reward depends only on the first robot!
        """
        rospy.logwarn("Running map comparison...")
        new_map_difference = compare_current_map_to_actual_map(self._map_file_name, self.actual_map_file)
        new_min_map_difference = min(new_map_difference, self.current_min_map_difference)

        # If the new difference is big, it's possibly a bug
        if new_min_map_difference > 0.5:
            exploration_reward = 0
        else:
            exploration_reward = self.current_min_map_difference - new_min_map_difference

        rospy.logwarn("Old map dif - new map dif: {}-{} = {}".format(self.current_min_map_difference, new_min_map_difference, exploration_reward))

        if not done:
            reward = self.no_crash_reward_points + exploration_reward * self.exploration_multi_factor
        else:
            reward = self.crash_reward_points

        self.current_min_map_difference = new_min_map_difference

        rospy.loginfo("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.loginfo("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.loginfo("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward

    # Internal TaskEnv Methods
    
    def discretize_scan_observation(self,data,new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """        
        discretized_ranges = []
        mod = len(data.ranges)/new_ranges
        
        # rospy.loginfo("data=" + str(data))
        rospy.loginfo("new_ranges=" + str(new_ranges))
        rospy.loginfo("mod=" + str(mod))
        
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if item == float ('Inf') or numpy.isinf(item):
                    discretized_ranges.append(self.max_laser_value)
                elif numpy.isnan(item):
                    discretized_ranges.append(self.min_laser_value)
                else:
                    discretized_ranges.append(int(item))
                    
            if (self.min_range > item > 0):
                rospy.logerr("done Validation >>> item=" + str(item)+"< "+str(self.min_range))
                self._episode_done = True
            else:
                rospy.loginfo("NOT done Validation >>> item=" + str(item)+"< "+str(self.min_range))
                    

        return discretized_ranges
        
        
    def get_vector_magnitude(self, vector):
        """
        It calculated the magnitude of the Vector3 given.
        This is usefull for reading imu accelerations and knowing if there has been 
        a crash
        :return:
        """
        contact_force_np = numpy.array((vector.x, vector.y, vector.z))
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude

    def check_if_crashed(self, ns):
        """
        Check if any of the robots has crashed. 
        """
        laser_scan_message = self.get_laser_scan(ns)

        collision_detector = PseudoCollisionDetector()
        
        collision_detected = collision_detector.collision_detected(laser_scan_message, self.min_range)

        if collision_detected:
            return True

        return False

    ### OVERRIDE

    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    def move_base(self, linear_speed, angular_speed, namespace, epsilon=0.05, update_rate=10):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achived reading from the odometry topic.
        :param linear_speed: Speed in the X axis of the robot base frame
        :param angular_speed: Speed of the angular turning of the robot base frame
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return: 
        """
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        rospy.loginfo("TurtleBot3 Base Twist Cmd>>" + str(cmd_vel_value))
        self._check_publishers_connection(namespace)
        self._cmd_vel_pub[namespace].publish(cmd_vel_value)
        self.wait_until_twist_achieved(cmd_vel_value,
                                        epsilon,
                                        update_rate,
                                        namespace)

        # After moving, stop, so the other robots can move
        stop_twist = Twist()
        self._cmd_vel_pub[namespace].publish(stop_twist)
        self.wait_until_twist_achieved(stop_twist,
                                        epsilon,
                                        update_rate,
                                        namespace)

    
    def wait_until_twist_achieved(self, cmd_vel_value, epsilon, update_rate, namespace):
        """
        We wait for the cmd_vel twist given to be reached by the robot reading
        from the odometry.
        :param cmd_vel_value: Twist we want to wait to reach.
        :param epsilon: Error acceptable in odometry readings.
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        rospy.loginfo("START wait_until_twist_achieved...")
        
        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 0.05
        
        rospy.loginfo("Desired Twist Cmd>>" + str(cmd_vel_value))
        rospy.loginfo("epsilon>>" + str(epsilon))
        
        linear_speed = cmd_vel_value.linear.x
        angular_speed = cmd_vel_value.angular.z
        
        linear_speed_plus = linear_speed + epsilon
        linear_speed_minus = linear_speed - epsilon
        angular_speed_plus = angular_speed + epsilon
        angular_speed_minus = angular_speed - epsilon
        
        while not rospy.is_shutdown():
            current_odometry = self._check_odom_ready(namespace)
            # IN turtlebot3 the odometry angular readings are inverted, so we have to invert the sign.
            odom_linear_vel = current_odometry.twist.twist.linear.x
            odom_angular_vel = current_odometry.twist.twist.angular.z
            
            rospy.loginfo("Linear VEL of {}=".format(namespace) + str(odom_linear_vel) + ", ?RANGE=[" + str(linear_speed_minus) + ","+str(linear_speed_plus)+"]")
            rospy.loginfo("Angular VEL of {}=".format(namespace) + str(odom_angular_vel) + ", ?RANGE=[" + str(angular_speed_minus) + ","+str(angular_speed_plus)+"]")
            
            linear_vel_are_close = (odom_linear_vel <= linear_speed_plus) and (odom_linear_vel > linear_speed_minus)
            angular_vel_are_close = (odom_angular_vel <= angular_speed_plus) and (odom_angular_vel > angular_speed_minus)
            
            if linear_vel_are_close and angular_vel_are_close:
                rospy.loginfo("{} reached Velocity!".format(namespace))
                end_wait_time = rospy.get_rostime().to_sec()
                break
            
            if self.check_if_crashed(namespace):
                rospy.logerr("{} has crashed while trying to achieve Twist.".format(namespace))
                self._episode_done = True
                break

            rospy.loginfo("{} is not there yet, keep waiting...".format(namespace))
            rate.sleep()
        delta_time = end_wait_time- start_wait_time
        rospy.loginfo("[Wait Time=" + str(delta_time)+"]")
        
        rospy.loginfo("END wait_until_twist_achieved...")
        
        return delta_time

    def _start_map_merge(self):
        if not self._map_merge_running:
            rospy.loginfo("Creating launch parent for MapMerge launch file.")
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            self._map_merge_launch = roslaunch.parent.ROSLaunchParent(uuid, [self._map_merge_launch_file])

            self._map_merge_launch.start()
            rospy.loginfo("Started MapMerge launch file.")

            self._map_merge_running = True
    
    def _start_gmapping(self):
        if not self._gmapping_running:
            rospy.loginfo("Creating launch parent for Gmapping launch file.")
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self._gmapping_launch = roslaunch.parent.ROSLaunchParent(uuid, [self._gmapping_launch_file])

            self._gmapping_launch.start()
            rospy.loginfo("Started Gmapping launch file.")

            self._gmapping_running = True

    def _stop_map_merge(self):
        if self._map_merge_running:
            self._map_merge_launch.shutdown()
            rospy.loginfo("Stopped MapMerge launch file.")

            self._map_merge_running = False
    
    def _stop_gmapping(self):
        if self._gmapping_running:
            self._gmapping_launch.shutdown()
            rospy.loginfo("Stopped Gmapping launch file.")

            self._gmapping_running = False

    def _map_callback(self, map_data):
        # Based on this: https://github.com/ros-planning/navigation/blob/melodic-devel/map_server/src/map_saver.cpp

        threshold_occupied = 65
        threshold_free = 25
        f = open(self._map_file_name + ".pgm", "w")

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