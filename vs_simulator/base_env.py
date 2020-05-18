#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Jun Jin
# Created Date: Fri May 15 MDT 2020
# Revised Date: N/A
# github      : https://github.com/atlas-jj/visual-servoing-simulator-pybullet
# =============================================================================
"""
Franka Panda point 2 point task
 - Vision & Robotics Lab, Department of Computing Science
 - University of Alberta, Canada
"""

import yaml
import gym
import pybullet
from gym.utils import seeding
from pybullet_utils.bullet_client import BulletClient


class BaseEnv(gym.Env):
    def __init__(self, config_file):
        self.configs, self.bc = None, None
        # load configs
        with open(config_file) as confs:
            try:
                self.configs = yaml.safe_load(confs)['simulation']
            except Exception as err:
                print('load configs failed! {}'.format(err))
                raise err
        self.bc = self.config_bullet()

    def config_bullet(self):
        if self.configs['gui']:
            bullet_client = BulletClient(connection_mode=pybullet.GUI)
        else:
            bullet_client = BulletClient(connection_mode=pybullet.DIRECT)
        # enable planar reflection
        bullet_client.configureDebugVisualizer(bullet_client.COV_ENABLE_PLANAR_REFLECTION, 1)
        # enable shadows
        bullet_client.configureDebugVisualizer(bullet_client.COV_ENABLE_SHADOWS, 0)
        if self.configs['debug']:
            bullet_client.configureDebugVisualizer(bullet_client.COV_ENABLE_GUI, 1)
        else:
            bullet_client.configureDebugVisualizer(bullet_client.COV_ENABLE_GUI, 0)
        bullet_client.setGravity(0, 0, -self.configs['physics']['gravity'])
        bullet_client.setPhysicsEngineParameter(
            fixedTimeStep=self.configs['physics']['timestep'],
            contactERP=self.configs['physics']['contact_erp']
        )
        return bullet_client

    def close(self):
        self.bc.disconnect()

    def reset(self):
        """
        # reset robot to initial cofnigs.
        # reset scene
        :return:
        """
        raise NotImplementedError("")

    def seed(self, seed=None):
        if seed is not None:
            seeding.np_random(abs(seed))
        return [seed]

    def get_reward(self):
        raise NotImplementedError("")

    def get_space_attributes(self):
        raise NotImplementedError("")

    def step(self, action):
        raise NotImplementedError("")






