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

import vs_simulator
from vs_simulator.utils import *


class FrankaPanda:
    MAX_GRIPPING_FORCE = 50  # 50 N
    JOINTS = []
    FINGERS = []

    def __init__(self, _configs, _bullet_client):
        self.robot_id = None
        self.configs = _configs
        self.bc = _bullet_client
        self.bc.setAdditionalSearchPath(vs_simulator.lib_path + '/data/robots/franka_panda')
        try:
            self.robot_id = self.bc.loadURDF(
                fileName='panda.urdf',
                basePosition=self.configs['base']['position'],
                baseOrientation=euler_angles_to_quaternions(self.configs['base']['orientation'], self.bc),
                useFixedBase=True,
                flags=(self.bc.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | self.bc.URDF_USE_SELF_COLLISION |
                       self.bc.URDF_MAINTAIN_LINK_ORDER)
            )
            self.num_joints = self.bc.getNumJoints(self.robot_id)
            # set initial joints
            self.reset_pose = self.configs['initial']['joints']
            self.reset_whole_body_joints()
            # get controllable joints
            self.JOINTS = [i for i in range(self.num_joints) if
                           self.bc.getJointInfo(self.robot_id, i)[2] == self.bc.JOINT_REVOLUTE]
            self.FINGERS = [i for i in range(self.num_joints) if
                            self.bc.getJointInfo(self.robot_id, i)[2] == self.bc.JOINT_PRISMATIC]
            # get joint limits
            self.joint_lower_limits, self.joint_higher_limits, self.joint_ranges = self.get_joint_limits(self.JOINTS)
            self.finger_lower_limits, self.finger_higher_limits, self.finger_ranges = self.get_joint_limits(self.FINGERS)
            # get max joint velocities
            self.joint_max_forces = [self.bc.getJointInfo(self.robot_id, i)[10] for i in self.JOINTS]
            self.joint_max_velocities = [self.bc.getJointInfo(self.robot_id, i)[11] for i in self.JOINTS]
        except Exception as ex:
            print('{}, {}'.format(type(ex), ex.args[0]))

    def reset_whole_body_joints(self):
        for i in range(self.num_joints):
            self.bc.resetJointState(bodyUniqueId=self.robot_id,
                                    jointIndex=i,
                                    targetValue=self.reset_pose[i],
                                    targetVelocity=0)

    def get_joint_limits(self, joint_idx):
        lower_limits, higher_limits, j_ranges = [], [], []
        for i in joint_idx:
            joint_info = self.bc.getJointInfo(self.robot_id, i)
            lower_limits.append(joint_info[8])
            higher_limits.append(joint_info[9])
            j_ranges.append(joint_info[9] - joint_info[8])
        return lower_limits, higher_limits, j_ranges

    def move_by_joint_pose(self, target_joints):
        return

    def move_by_joint_torques(self, target_torques):
        return

    def move_by_joint_velocities(self, target_joint_velocities):
        return

    def move_by_cart_pose(self, target_pose):
        return

    def move_by_diff_cart_action(self, diff_cart_action):
        return

    def close_gripper(self, gripping_force=None):
        return

    def open_gripper(self):
        return

    def get_state(self):
        """
        get robot state
        :return: 7 joints' pos, velocities.
        """
        return


