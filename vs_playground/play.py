import time
import gym
import vs_simulator
import vs_playground
from vs_playground.utils import *
import numpy as np
import math

config_file_path = 'vs_point2point.yaml'

if __name__ == '__main__':
    config_file_path = vs_playground.lib_path + '/configs/' + config_file_path
    env_name = configure(config_file_path)['env_name']
    print('loading env {}'.format(env_name))
    env = gym.make(env_name, config_file=config_file_path)
    MAX_NUM_EPISODES = 10000
    NUM_STEPS = 1000
    for i in range(MAX_NUM_EPISODES):
        obs = env.reset()  # obs: current position x, y;  target position x, y
        for j in range(NUM_STEPS):
            s_time = time.monotonic()
            action = np.random.uniform(low=-math.pi/2, high=math.pi/2, size=(2,))   # joint velocity control
            obs, reward, done, info = env.step(action)  # joint velocity control
            print('==Episode {:4d}, step {:4d}/{:4d}, duration {:.2f}s, reward {:+.1f}, distance to target {:3.3f}'.format(i, j, NUM_STEPS, time.monotonic()-s_time,reward, info['dist']))
