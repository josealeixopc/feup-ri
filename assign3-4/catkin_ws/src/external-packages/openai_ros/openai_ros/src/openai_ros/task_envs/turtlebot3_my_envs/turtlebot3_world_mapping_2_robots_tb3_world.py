import os

import turtlebot3_world_mapping_2_robots

class TurtleBot3WorldMapping2RobotsTB3WorldEnv(turtlebot3_world_mapping_2_robots.TurtleBot3WorldMapping2RobotsEnv):
    def __init__(self):
        # Export TB3 robots position to ENV variable so they can be accessible in the launch file
        os.environ["GAZEBO_WORLD"] = "turtlebot3_world.world"

        os.environ["FIRST_TB3_X"] = "2.0"
        os.environ["FIRST_TB3_Y"] = "0.0"
        os.environ["FIRST_TB3_Z"] = "0.0"
    
        os.environ["SECOND_TB3_X"] = "-2.0"
        os.environ["SECOND_TB3_Y"] = "0.0"
        os.environ["SECOND_TB3_Z"] = "0.0"

        super(TurtleBot3WorldMapping2RobotsTB3WorldEnv, self).__init__("turtlebot3_world_mapping_tb3_world.yaml")