import turtlebot3_world_mapping_2_robots

class TurtleBot3WorldMapping2RobotsTB3WorldEnv(turtlebot3_world_mapping_2_robots.TurtleBot3WorldMapping2RobotsEnv):
    def __init__(self):
         super(TurtleBot3WorldMapping2RobotsTB3WorldEnv, self).__init__("turtlebot3_world_mapping_tb3_world.yaml")