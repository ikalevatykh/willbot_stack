import numpy as np
import rospy

from base_env import WillbotEnv


class ReachEnv(WillbotEnv):
    """Reach environment. 
    
    Agent's goal is to move an arm close to the target in xy plane.
    """

    def __init__(self):
        super(ReachEnv, self).__init__()
        self._arm.detach_object()
        self._scene.remove_world_object('target')

        plan = self._arm.cartesian()
        plan.move((0.30, 0.00, 0.20), (np.pi, 0, -np.pi/2))
        if not plan.execute():
            raise RuntimeError('Cannot move an arm')

        cube_pos = (0.30, 0.00, 0.041)
        cube_dim = (0.06, 0.06, 0.06)
        self._scene.add_box(
            'target', cube_dim, cube_pos)
        self._target_position = cube_pos

        self._success_tolerance = 0.01

    def reset(self, target_pos=None, tool_pos=None):
        if tool_pos is None:
            tool_limits = [0.3, -0.20, 0.15], [0.7, 0.20, 0.25]
            tool_pos = self._np_random.uniform(*tool_limits)
        if target_pos is None:
            target_limits = [0.3, -0.20, 0.041], [0.7, 0.20, 0.041]
            target_pos = self._np_random.uniform(*target_limits)

        plan = self._arm.pick_place(self._hand, 'target')
        plan.pick(self._target_position)
        plan.place(target_pos)
        if not plan.execute():
            raise RuntimeError('Cannot move a target')
        self._target_position = target_pos

        plan = self._arm.cartesian()
        plan.move(tool_pos, (np.pi, 0, -np.pi/2))
        if not plan.execute():
            raise RuntimeError('Cannot move an arm')

        return self.observation()

    def step(self, action):
        super(ReachEnv, self).step(action)

        reward = 0.0
        info = {}
        return self.observation(), reward, self.done(), info

    def observation(self):
        return dict(target_position=self._target_position.tolist()[:2])

    def done(self):
        """Returns True if target and tool in xy plane is close """
        return np.isclose(
            self._target_position[:2],
            self._arm.get_current_position()[:2],
            atol=self._success_tolerance)
