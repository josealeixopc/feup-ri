#!/usr/bin/env python

import os
from datetime import datetime
import gym
import numpy
import time
import qlearn
from gym import wrappers

# ROS packages required
import rospy
import rospkg
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment

# Keras
from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

VS_ROS_DEBUG = 1
ENV_NAME = 'TurtleBot3WorldMapping2Robots-v0'

def create_dir(path):
    try:
        os.makedirs(os.path.dirname(path))
    except OSError as exc:  # Guard against race condition
        if exc.errno != errno.EEXIST:
            raise

if __name__ == '__main__':
    """DQN agent implementation based from https://github.com/keras-rl/keras-rl/blob/master/examples/dqn_cartpole.py
    """

    if VS_ROS_DEBUG:
        raw_input('Waiting for VS ROS debugger to be attached... Press a key and ENTER once it has been attached: ')

    ### Export ENV variables BEGIN
    
    # Add node name to ROS logging messages 
    os.environ['ROSCONSOLE_FORMAT']='[${severity}] [${time}]: ${node}: ${message}'
    
    # Set TB3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # Set WS path if no env variable is set
    if os.environ.get('ROS_WS') is None:
            os.environ['ROS_WS'] = '/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws'
    
    ### Export ENV variables END

    rospy.init_node('turtlebot3_world_mapping_dqn', anonymous=True, log_level=rospy.DEBUG)

    # Init OpenAI_ROS ENV
    task_and_robot_environment_name = ENV_NAME
    env = StartOpenAI_ROS_Environment(task_and_robot_environment_name)
    
    # MAKE SURE TO USE loginfo INSTEAD OF logdebug! 
    # logdebug doesn't appear in \rosout for some reason (check rospy API), therefore it won't appear in rosconsole.
    # Create the Gym environment
    rospy.loginfo("Gym environment done")
    rospy.loginfo("Starting Learning")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('coop_mapping')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    # Next, we build a very simple model.
    nb_actions = env.action_space.n

    model = Sequential()
    model.add(Flatten(input_shape=(1,) + env.observation_space.shape))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(nb_actions))
    model.add(Activation('linear'))
    print(model.summary())

    # Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
    # even the metrics!
    memory = SequentialMemory(limit=50000, window_length=1)
    policy = BoltzmannQPolicy()
    dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10,
                target_model_update=1e-2, policy=policy)
    dqn.compile(Adam(lr=1e-3), metrics=['mae'])

    # Okay, now it's time to learn something! We visualize the training here for show, but this
    # slows down training quite a lot. You can always safely abort the training prematurely using
    # Ctrl + C.
    dqn.fit(env, nb_steps=10, visualize=False, verbose=2)

    # After training is done, we save the final weights.
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('coop_mapping')

    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d-%H-%M-%S")

    file_name = pkg_path + os.path.sep + "training_weights" + os.path.sep + "{}-dqn_{}_weights.h5f".format(current_time, ENV_NAME)
    create_dir(file_name)

    dqn.save_weights(file_name, overwrite=True)

    # Finally, evaluate our algorithm for 5 episodes.
    # dqn.test(env, nb_episodes=5, visualize=False)
