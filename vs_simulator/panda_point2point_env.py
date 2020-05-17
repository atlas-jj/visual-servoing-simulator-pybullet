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
import vs_simulator
from vs_simulator.robots.franka_panda import FrankaPanda
from vs_simulator.base_env import BaseEnv


class PandaPoint2PointEnv(BaseEnv):
    def __init__(self, config_file):
        BaseEnv.__init__(self, config_file=config_file)
        self.config_robot = self.configs['robot']
        self.config_scene = self.configs['scene']
        self.config_cameras = self.configs['cameras']
        self.robot = FrankaPanda(self.config_robot, self.bc)




