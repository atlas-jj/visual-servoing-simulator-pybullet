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

from gym import spaces
import numpy as np
from vs_simulator.utils import *
import vs_simulator
from vs_simulator.robots.franka_panda import FrankaPanda
from vs_simulator.base_env import BaseEnv
import random


class PandaPoint2PointEnv(BaseEnv):
    def __init__(self, config_file):
        BaseEnv.__init__(self, config_file=config_file)
        self.config_robot = self.configs['robot']
        self.config_scene = self.configs['scene']
        self.config_cameras = self.configs['cameras']
        self.robot = FrankaPanda(self.config_robot, self.bc)
        self.robot.select_control_joints([1, 3])  # only control the 2nd and 4th joints.
        self.sphere_in_hand, self.sphere_target = None, None
        # load plane
        ground_id = self.bc.loadURDF(fileName=vs_simulator.lib_path+'/data/scene/planes/plane.urdf')
        self.bc.changeDynamics(ground_id, -1)
        self.load_scene()

    def load_scene(self, inner_num_steps=5):
        """
        1 create a sphere for the robot to chase with.
        2 create a sphere for the robot to grasp with.
        :return:
        """
        self.sphere_in_hand = create_sphere(0.04, [0, 0, 1.0, 1.0], [0.5549266, 0.011935757, 0.48], self.bc)
        k = 2.2
        # the problem only controls 2 DOF, it requires sphere in hand and the target sphere are in the same 3D plane.
        self.sphere_target = create_sphere(0.04, [1.0, 0, 0., 1.0], [0.5549266*k, 0.011935757*k, 0.75], self.bc, fixed=True)
        self.bc.stepSimulation()
        for _ in range(inner_num_steps):
            self.bc.stepSimulation()
        self.robot.close_gripper(0.015)
        self.bc.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=-6.4, cameraPitch=-8.8,
                                           cameraTargetPosition=[0.449, 0.4026, 0.445])

    def reset(self):
        """
        # reset robot to initial cofnigs.
        # reset scene
        :return:
        """
        self.robot.reset_whole_body_joints()
        # remove the spheres
        self.bc.removeBody(self.sphere_target)
        self.bc.removeBody(self.sphere_in_hand)
        self.load_scene()
        # TODO randomly change target ball's location
        return self.get_observation()

    def step(self, action):
        """
        action: 2dof joint velocity.
        :param action:
        :return:
        """
        self.robot.move_by_joint_velocities(action, inner_num_steps=10)
        info = self.check_termination()
        return self.get_observation(), self.get_reward(), info['done'], info

    def get_space_attributes(self):
        self.action_space = spaces.Box(low=-math.pi, high=math.pi, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-500, high=500, shape=(4,), dtype=np.float32)
        self.reward_range = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)

    def get_reward(self, info=None):
        return random.random()

    def get_observation(self):
        """
        can be changed to whatever observation format that you want.
        :return:
        """
        current_pos, _ = self.bc.getBasePositionAndOrientation(self.sphere_in_hand)
        target_pos, _ = self.bc.getBasePositionAndOrientation(self.sphere_target)
        x, y, _ = current_pos
        x_t, y_t, _ = target_pos
        return np.array([x, y, x_t, y_t])

    def check_termination(self):
        """
        should be implemented to your own needs.
        :return:
        """
        current_pos, _ = self.bc.getBasePositionAndOrientation(self.sphere_in_hand)
        target_pos, _ = self.bc.getBasePositionAndOrientation(self.sphere_target)
        dist = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
        info={'dist': dist,
              'done': False}
        if dist < 0.001:
            info['done'] = True
        return info
