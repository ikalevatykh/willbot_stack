#!/usr/bin/env python
from __future__ import print_function

import json
import numpy as np

from willbot_env import WillbotEnv


class WillbotPickEnv(WillbotEnv):
    def __init__(self):
        super(WillbotPickEnv, self).__init__()
        self._hand._close_position = 0.7
        cube_pos = (0.30, 0.00, 0.041)
        cube_dim = (0.06, 0.06, 0.06)
        self._scene.add_box(
            'box', cube_dim, cube_pos, color=(1, 0, 0))
        self._cube_position = cube_pos

    def reset(self):
        tool_limits = [0.3, -0.20, 0.150], [0.7, 0.20, 0.250]
        tool_pos = self._np_random.uniform(*tool_limits)

        cube_limits = [0.3, -0.20, 0.041], [0.7, 0.20, 0.041]
        cube_pos = self._np_random.uniform(*cube_limits)

        plan = self._arm.pick_place(self._hand, 'box')
        success = plan.pick(self._cube_position) and plan.place(cube_pos)
        if not success:
            raise RuntimeError('Cannot move a cube')
        self._cube_position = cube_pos

        self._arm.set_pose_target(tool_pos + [-0.7, 0.7, 0.0, 0.0])
        if not self._arm.go():
            raise RuntimeError('Cannot move an arm')

        return self.observation()

    def step(self, action):
        super(WillbotPickEnv, self).step(action)
        return self.observation()

    def observation(self):
        return dict(cube_pos=self._cube_position)
