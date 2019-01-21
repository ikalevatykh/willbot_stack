#!/usr/bin/env python
from __future__ import print_function

import numpy as np
from gym.utils import seeding

from willbot_utils.arm import UR5
from willbot_utils.hand import RobotiqHand
from willbot_utils.scene import StandardScene


class WillbotEnv(object):
    def __init__(self):
        self._np_random = None
        self._seed = 0

        print('Connecting arm ...')
        arm = UR5()
        arm.set_planner_id("RRTConnectkConfigDefault")
        # arm.set_max_velocity_scaling_factor(0.5)
        self._arm = arm

        workspace = np.array([[0.3, -0.20, 0.050], [0.7, 0.20, 0.250]])
        arm.set_workspace(workspace.flatten())
        self._workspace = workspace
        print('- Arm OK')

        print('Connecting hand ...')
        hand = RobotiqHand()
        hand.mode = 1
        hand.target_velocity = 5
        hand.target_effort = 5
        self._hand = hand
        print('- Hand OK')

        print('Preparing scene ...')
        self._scene = StandardScene()
        self._arm.set_support_surface_name('rubber')
        print('- Scene OK')

    def reset(self):
        """Resets the state of the environment and returns an initial observation."""
        raise NotImplementedError

    def step(self, action):
        """Run one timestep of the environment's dynamics."""

        if 'tool_position' in action:
            pos = action.get('tool_position')
            orn = action.get('tool_orientation', None)
            self._arm.cartesian().move(pos, orn).execute(wait=True)
        elif 'joint_position' in action:
            joints = action.get('joint_position')
            self._arm.go(joints, wait=True)
        elif 'linear_velocity' in action:
            #'angular_velocity'
            raise NotImplementedError
        elif 'joint_velocity' in action:
            raise NotImplementedError                      

        if 'gripper_open' in action:
            self._hand.open(wait=True)
        elif 'gripper_close' in action:
            self._hand.close(wait=True)

    def seed(self, seed):
        """Sets the seed for this env's random number generator(s)."""
        np_random, seed = seeding.np_random(seed)
        self._np_random = np_random
        self._seed = seed
        return [seed]
