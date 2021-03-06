import numpy as np
import rospy

from base_env import WillbotEnv


class ReachEnv(WillbotEnv):
    """Reach environment.

    Agent's goal is to move an arm close to the target in xy plane.
    """

    def __init__(self, cube_size=0.06, update_scene=True):
        super(ReachEnv, self).__init__()
        self._tool_orn = (np.pi, 0, np.pi/2)
        self._success_tolerance = 0.01
        self._update_scene = update_scene
        self._target_size = cube_size
        self._target_position = None
        self._target = None
        self._setup = False

    def setup(self):
        # open hand
        self._hand.open()

        # move arm to a predefined position
        init_pos = (0.30, 0.00, 0.20), (np.pi, 0, np.pi/2)
        if not self._arm.joint_move(*init_pos):
            raise RuntimeError('Cannot move an arm')

        # add target model to a scene
        dim = self._target_size
        cube_pos = (0.30, 0.00, 0.012 + dim / 2)
        cube_dim = (dim, dim, dim - 0.004)
        if self._update_scene:
            self._target = self._scene.add_box('target', cube_dim, cube_pos)
        self._target_position = (0.30, 0.00, 0.05)

        self._setup = True

    def reset(self, target_pos=None, tool_pos=None):
        # setup environment only once
        if not self._setup:
            self.setup()

        # generate random positions if not specified
        if target_pos is None:
            target_limits = [0.3, -0.20, 0.05], [0.7, 0.20, 0.05]
            target_pos = self._np_random.uniform(*target_limits)

        if tool_pos is None:
            tool_limits = [0.3, -0.20, 0.15], [0.7, 0.20, 0.25]
            tool_pos = self._np_random.uniform(*tool_limits)

        # pick and place target to a new (random) place
        plan = self._arm.pick_place(self._target)
        plan.pick(self._target_position, object_width=self._target_size)
        plan.place(target_pos)
        if not plan.execute():
            raise RuntimeError('Cannot move a target')
        self._target_position = target_pos

        # move arm to a (random) position
        if not self._arm.linear_move(tool_pos):
            raise RuntimeError('Cannot move an arm')

        return self.observation()

    def step(self, action):
        super(ReachEnv, self).step(action)

        reward = 0.0
        info = {}
        return self.observation(), reward, self.done(), info

    def observation(self):
        """Target position in xy plane."""
        return dict(target_position=self._target_position.tolist()[:2])

    def done(self):
        """True if target and tool in xy plane is close."""
        return np.allclose(
            self._target_position.tolist()[:2],
            self._arm.get_current_position()[:2],
            atol=self._success_tolerance)
