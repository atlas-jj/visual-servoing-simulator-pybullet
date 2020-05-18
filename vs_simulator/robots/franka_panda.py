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
import numpy as np


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
            self.selected_joints_index = [i for i in range(len(self.JOINTS))]
            self.controlled_joints = self.JOINTS
            # get joint limits
            self.joint_lower_limits, self.joint_higher_limits, self.joint_ranges = self.get_joint_limits(self.JOINTS)
            self.finger_lower_limits, self.finger_higher_limits, self.finger_ranges = self.get_joint_limits(
                self.FINGERS)
            # get max joint velocities
            self.joint_max_forces = [self.bc.getJointInfo(self.robot_id, i)[10] for i in self.JOINTS]
            self.joint_max_velocities = [self.bc.getJointInfo(self.robot_id, i)[11] for i in self.JOINTS]
            # add constraint to the two joint fingers
            c = self.bc.createConstraint(self.robot_id,
                                         self.FINGERS[0],
                                         self.robot_id,
                                         self.FINGERS[1],
                                         jointType=self.bc.JOINT_GEAR,
                                         jointAxis=[1, 0, 0],
                                         parentFramePosition=[0, 0, 0],
                                         childFramePosition=[0, 0, 0])
            self.bc.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=100)
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

    def select_control_joints(self, selected_index):
        """
        select joints to control
        :param selected_index:
        :return:
        """
        self.selected_joints_index = selected_index
        self.controlled_joints = []
        for i in self.selected_joints_index:
            self.controlled_joints.append(self.JOINTS[i])

    def move_by_joint_pose(self, target_joints, max_num_inner_step=50):
        # check limits
        joints_in_range = True
        pos_t = 0
        for i in range(len(target_joints)):
            if target_joints[i] < self.joint_lower_limits[self.selected_joints_index[i]] or \
                    target_joints[i] > self.joint_higher_limits[self.selected_joints_index[i]]:
                joints_in_range = False
                pos_t = i
                break
        assert joints_in_range, 'target joints out of range limits. Check joint index {}'.format(pos_t)
        # fill in the uncontrolled joints
        target_joints_temp = self.get_joint_states()['positions']
        for i in range(len(self.selected_joints_index)):
            target_joints_temp[self.selected_joints_index[i]] = target_joints[i]
        self.bc.setJointMotorControlArray(self.robot_id, self.JOINTS,
                                          controlMode=self.bc.POSITION_CONTROL,
                                          targetPositions=target_joints_temp)
        # run multiple steps to reach target pose
        for _ in range(max_num_inner_step):
            self.bc.stepSimulation()
            diff = abs(np.array(target_joints_temp) - np.array(self.get_joint_states()['positions']))
            if np.linalg.norm(diff) < 0.001:
                break
        return True

    def move_by_joint_torques(self, target_torques, inner_num_steps=5):
        target_joints_temp = np.zeros((len(self.JOINTS)))
        for i in range(len(self.selected_joints_index)):
            target_joints_temp[self.selected_joints_index[i]] = target_torques[i]
        self.bc.setJointMotorControlArray(self.robot_id, self.JOINTS,
                                          controlMode=self.bc.TORQUE_CONTROL,
                                          forces=target_joints_temp)
        for _ in range(inner_num_steps):
            self.bc.stepSimulation()
        return True

    def move_by_joint_velocities(self, target_joint_velocities, inner_num_steps=10):
        target_joints_temp = np.zeros((len(self.JOINTS)))
        for i in range(len(self.selected_joints_index)):
            target_joints_temp[self.selected_joints_index[i]] = target_joint_velocities[i]
        self.bc.setJointMotorControlArray(self.robot_id, self.JOINTS,
                                          controlMode=self.bc.VELOCITY_CONTROL,
                                          targetVelocities=target_joints_temp)
        for _ in range(inner_num_steps):
            self.bc.stepSimulation()
        return True

    def move_by_cart_pose(self, target_position, target_orientation):
        joint_pose = self.bc.calculateInverseKinematics(self.robot_id,
                                                        endEffectorLinkIndex=11,
                                                        targetPosition=target_position,
                                                        targetOrientation=target_orientation,
                                                        lowerLimits=self.joint_lower_limits,
                                                        upperLimits=self.joint_higher_limits,
                                                        jointRanges=self.joint_ranges
                                                        )
        target_joints = []
        for i in self.JOINTS:
            target_joints.append(joint_pose[i])
        return self.move_by_joint_pose(target_joints)

    def move_by_cart_velocity(self, cart_velocity):
        raise NotImplementedError("Cart_velocity will be mapped to joint velocity by Jacobian.")

    def close_gripper(self, distance=0):
        return self.move_gripper([distance, distance], max_num_inner_step=40)

    def open_gripper(self, distance=0.04, max_num_inner_step=20):
        return self.move_gripper([distance, distance], max_num_inner_step=max_num_inner_step)

    def move_gripper(self, target_pose, max_num_inner_step=20):
        self.bc.setJointMotorControlArray(self.robot_id, self.FINGERS,
                                          controlMode=self.bc.POSITION_CONTROL,
                                          targetPositions=target_pose)
        # run multiple steps to reach target pose
        for _ in range(max_num_inner_step):
            self.bc.stepSimulation()
            diff = abs(np.array(target_pose) - np.array(self.get_finger_states()['positions']))
            if np.linalg.norm(diff) < 0.001:
                break
        return True

    def get_joint_states(self):
        return self.get_states(self.JOINTS)

    def get_finger_states(self):
        return self.get_states(self.FINGERS)

    def get_states(self, joint_idx):
        """
        get robot state
        :return: 7 joints' pos, velocities.
        """
        joint_states = self.bc.getJointStates(self.robot_id, joint_idx)
        joint_states_dict = {'positions': [joint_states[i][0] for i in range(len(joint_idx))],
                             'velocities': [joint_states[i][1] for i in range(len(joint_idx))],
                             'torques': [joint_states[i][3] for i in range(len(joint_idx))]}
        return joint_states_dict

    def get_end_effector_state(self):
        link_state = self.bc.getLinkState(self.robot_id, linkIndex=11,
                                          computeLinkVelocity=1, computeForwardKinematics=1)
        end_effector_state = {'position': link_state[0],
                              'orientation': link_state[1],
                              'linear velocity': link_state[6],
                              'angular velocity': link_state[7]}
        return end_effector_state
