import json

import gym
from gym.utils import seeding

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from willbot_envs.msg import EnvResetAction, EnvResetGoal
from willbot_envs.msg import EnvStepAction, EnvStepGoal


class EnvironmentClient(gym.Env):
    """Client for ROS environment server. 
    
    Provides gym interface for remote environment. 
    Can be used from python 3.x. 
    """

    def __init__(self, environment_id):
        """"""
        self._reset_client = actionlib.SimpleActionClient(
            '/willbot_env/reset', EnvResetAction)
        self._step_client = actionlib.SimpleActionClient(
            '/willbot_env/step', EnvStepAction)
        timeout = rospy.Duration(5.0)

        if not (self._reset_client.wait_for_server(timeout) and
                self._step_client.wait_for_server(timeout)):
            raise RuntimeError('Cannot connect to an Environment Server')

        self._environment_id = environment_id
        self._episode_id = 0
        self._seed = []

    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state."""

        goal = EnvStepGoal(
            episode_id=self._episode_id,
            action=json.dumps(action)
        )
        self._step_client.send_goal(goal)

        finished = self._step_client.wait_for_result(rospy.Duration(30.0))
        if not finished:
            raise RuntimeError('Environment server not responding')

        state = self._step_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError(self._step_client.get_goal_status_text())

        result = self._step_client.get_result()
        observation = json.loads(result.observation)
        reward = result.reward
        done = result.done
        info = json.loads(result.info)

        return observation, reward, done, info

    def reset(self):
        """Resets the state of the environment and returns an initial observation."""

        seed = EnvResetGoal.NO_SEED
        if self._seed:
            seed = self._seed.pop()

        goal = EnvResetGoal(
            environment_id=self._environment_id,
            seed=seed
        )
        self._reset_client.send_goal(goal)

        finished = self._reset_client.wait_for_result(rospy.Duration(30.0))
        if not finished:
            raise RuntimeError('Environment server not responding')

        state = self._reset_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError(self._reset_client.get_goal_status_text())

        result = self._reset_client.get_result()
        self._episode_id = result.episode_id
        observation = json.loads(result.observation)
        return observation

    def close(self):
        """Override _close in your subclass to perform any necessary cleanup."""

        self._reset_client.cancel_all_goals()
        self._step_client.cancel_all_goals()

    def seed(self, seed=None):
        """Sets the seed for this env's random number generator(s)."""

        self._seed = [seed]
        return self._seed

    def render(self, mode='human', close=False):
        """Renders the environment."""
        raise NotImplementedError
