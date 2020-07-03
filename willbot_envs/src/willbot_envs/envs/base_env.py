import numpy as np
import rospy

import gym
from gym.utils import seeding

from willbot_utils.arm import UR5
from willbot_utils.hand import RobotiqHand
from willbot_utils.scene import StandardScene


class WillbotEnv(gym.Env):
    def __init__(self):
        self._np_random = None
        self._seed = 0

        rospy.loginfo('Connecting hand ...')
        hand = RobotiqHand()
        hand.mode = 1
        hand.target_velocity = 15
        hand.target_effort = 5
        self._hand = hand
        rospy.loginfo('- Hand OK')

        rospy.loginfo('Connecting arm ...')
        self._arm = UR5(hand=hand)

        workspace = np.array([[0.3, -0.20, 0.050], [0.7, 0.20, 0.250]])
        self._arm.set_workspace(workspace.flatten())
        self._workspace = workspace
        rospy.loginfo('- Arm OK')

        rospy.loginfo('Preparing scene ...')
        self._scene = StandardScene(self._arm)
        rospy.loginfo('- Scene OK')

    def reset(self):
        """Resets the state of the environment and returns an initial observation."""
        raise NotImplementedError

    def step(self, action):
        """Run one timestep of the environment's dynamics."""

        if 'tool_position' in action:
            pos = action.get('tool_position')
            orn = action.get('tool_orientation', None)
            plan = self._arm.cartesian()
            plan.move(pos, orn)
            plan.execute(wait=True)
        elif 'joint_position' in action:
            joints = action.get('joint_position')
            self._arm.go(joints, wait=True)
        elif 'linear_velocity' in action:
            # 'angular_velocity'
            raise NotImplementedError
        elif 'joint_velocity' in action:
            raise NotImplementedError

        if 'gripper_open' in action:
            self._hand.open(wait=True)
        elif 'gripper_close' in action:
            self._hand.close(wait=True)
        elif 'grip_velocity' in action:
            vel = action.get('grip_velocity')
            deadzone = 0.01
            if vel < -deadzone:
                self._hand.close(wait=False)
            elif vel > deadzone:
                self._hand.open(wait=False)
            else:
                self._hand.stop()

    def seed(self, seed):
        """Sets the seed for this env's random number generator(s)."""
        np_random, seed = seeding.np_random(seed)
        self._np_random = np_random
        self._seed = seed
        return [seed]

    def close(self):
        """Override close in your subclass to perform any necessary cleanup."""
        self._arm.stop()
        self._hand.stop()

