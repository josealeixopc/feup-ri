import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
import json


Y_REWARDS = 'r'
Y_EPISODE_LENGTH = 'l'
Y_TIME_ELAPSED = 't'

lines = ["-", "--", ":", "-."]
markers = ['o', 'x', '+', '^']
colors = ['#000000', '#222222', '#444444', '#666666']

def smooth_moving_average(x, y, window_size):
    # Understanding convolution with window size: https://stackoverflow.com/a/20036959/7308982
    y_new = np.convolve(y, np.ones((window_size,)) / window_size, mode='valid')

    # We need to trim the last values, because the "valid" mode returns a list with size max(M, N) - min(M, N) + 1.
    # See here: https://docs.scipy.org/doc/numpy/reference/generated/numpy.convolve.html
    x_trimmed = x[:-(window_size - 1)]

    return x_trimmed, y_new

if __name__ == '__main__':

    episodes_0_200_file = "/home/jazz/Documents/ri-results/tb3_world_800_episodes_85_percent/2020-01-29-17-45-21-dqn-0-200/openaigym.episode_batch.0.26249.stats.json"
    episodes_200_400_file = "/home/jazz/Documents/ri-results/tb3_world_800_episodes_85_percent/2020-01-30-20-33-28-dqn-200-400/openaigym.episode_batch.0.28736.stats.json"
    episodes_400_800_file = "/home/jazz/Documents/ri-results/tb3_world_800_episodes_85_percent/2020-02-03-13-12-56-dqn-400-800/openaigym.episode_batch.0.1379.stats.json"

    with open(episodes_0_200_file) as f:
        episodes_0_200_json = json.load(f)
    
    with open(episodes_200_400_file) as f:
        episodes_200_400_json = json.load(f)

    with open(episodes_400_800_file) as f:
        episodes_400_800_json = json.load(f)


    all_rewards = episodes_0_200_json['episode_rewards'] + episodes_200_400_json['episode_rewards'] # + episodes_400_800_json['episode_rewards']
    all_lengths = episodes_0_200_json['episode_lengths'] + episodes_200_400_json['episode_lengths'] # + episodes_400_800_json['episode_lengths']

    plt.style.use('ggplot')

    x, y = smooth_moving_average(range(len(all_rewards)), all_rewards, 10)


    plt.plot(x, y, linewidth=1)
    plt.xlim(left=0)
    plt.xlabel("Number of Episodes")
    plt.ylabel("Rewards per Episode")
    plt.tight_layout()
    ax = plt.gca()
    ratio = 0.4
    xleft, xright = ax.get_xlim()
    ybottom, ytop = ax.get_ylim()
    ax.set_aspect(abs((xright-xleft)/(ybottom-ytop))*ratio)
    plt.savefig("rewards.eps", format='eps', bbox_inches='tight')
    plt.show()

    x, y = smooth_moving_average(range(len(all_lengths)), all_lengths, 10)

    plt.plot(x, y, linewidth=1)
    plt.xlim(left=0)
    plt.xlabel("Number of Episodes")
    plt.ylabel("Timesteps per Episode")
    plt.tight_layout()
    ax = plt.gca()
    ratio = 0.4
    xleft, xright = ax.get_xlim()
    ybottom, ytop = ax.get_ylim()
    ax.set_aspect(abs((xright-xleft)/(ybottom-ytop))*ratio)
    plt.savefig("timesteps.eps", format='eps', bbox_inches='tight')
    plt.show()
