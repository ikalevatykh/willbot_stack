import numpy as np
import rospy

from base_env import WillbotEnv


class PickEnv(WillbotEnv):
    def __init__(self):
        super(PickEnv, self).__init__()
        self._arm.detach_object()
        self._scene.remove_world_object('box')

        plan = self._arm.cartesian()
        plan.move((0.30, 0.00, 0.20), (np.pi, 0, -np.pi/2))
        if not plan.execute():
            raise RuntimeError('Cannot move an arm')

        cube_pos = (0.30, 0.00, 0.041)
        cube_dim = (0.06, 0.06, 0.06)
        self._scene.add_box(
            'box', cube_dim, cube_pos, color=(1, 0, 0))
        self._cube_position = cube_pos

    def reset(self):
        tool_limits = [0.3, -0.20, 0.15], [0.7, 0.20, 0.25]
        tool_pos = self._np_random.uniform(*tool_limits)

        cube_limits = [0.3, -0.20, 0.041], [0.7, 0.20, 0.041]
        cube_pos = self._np_random.uniform(*cube_limits)

        plan = self._arm.pick_place(self._hand, 'box')
        plan.pick(self._cube_position)
        plan.place(cube_pos)
        if not plan.execute():
            raise RuntimeError('Cannot move a cube')
        self._cube_position = cube_pos

        plan = self._arm.cartesian()
        plan.move(tool_pos, (np.pi, 0, -np.pi/2))
        if not plan.execute():
            raise RuntimeError('Cannot move an arm')

        return self.observation()

    def step(self, action):
        super(PickEnv, self).step(action)

        reward = 0.0
        done = False
        info = {}
        return self.observation(), reward, done, info

    def observation(self):
        return dict(cube_pos=self._cube_position.tolist())
