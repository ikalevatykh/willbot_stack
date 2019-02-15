import sys
import gym
from gym.utils import seeding

import rospy

from willbot_envs.utils import load_env


class EnvironmentDirect(gym.Env):
    """Wrapper for ROS environment server. 

    Provides gym interface for remote environment. 
    """

    def __init__(self, environment_id, init_node=True, **params):
        """Connects to environment server and init standalone environment.

        Arguments:
            environment_id(string): unique environment id.
        """

        if init_node:
            rospy.myargv(argv=sys.argv)
            rospy.init_node('willbot_env_direct', anonymous=False)        

        env_cls = load_env(environment_id)
        self._environment = env_cls(**params)

    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state."""

        return self._environment.step(action)

    def reset(self, **params):
        """Resets the state of the environment and returns an initial observation."""

        return self._environment.reset(**params)

    def seed(self, seed=None):
        """Sets the seed for this env's random number generator(s)."""

        return self._environment.seed(seed)

    def render(self, mode='human', close=False):
        """Renders the environment."""
        raise NotImplementedError

    def close(self):
        """Override _close in your subclass to perform any necessary cleanup."""

        self._environment.close()

    def __enter__(self):
        """Just return self."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """close connection at exit."""
        self.close()
