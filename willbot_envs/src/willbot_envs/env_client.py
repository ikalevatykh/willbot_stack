import sys
import gym
from gym.utils import seeding

import rospy
import actionlib

from willbot_envs.utils import loads, dumps

from actionlib_msgs.msg import GoalStatus
from willbot_envs.msg import EnvResetAction, EnvResetGoal
from willbot_envs.msg import EnvStepAction, EnvStepGoal
from willbot_envs.srv import Seed, SeedRequest
from willbot_envs.srv import Init, InitRequest
from willbot_envs.srv import Close, CloseRequest


class EnvironmentClient(gym.Env):
    """Client for ROS environment server. 

    Provides gym interface for remote environment. 
    Can be used from python 3.x. 
    """

    def __init__(self, environment_id, init_node=True, **params):
        """Connects to environment server and init standalone environment.

        Arguments:
            environment_id(string): unique environment id.
            init_node(bool): if setted register ROS node.
        """
        self._session_id = -1

        if init_node:
            rospy.myargv(argv=sys.argv)
            rospy.init_node('willbot_env_client', anonymous=False)

        self._init_client = rospy.ServiceProxy(
            '/willbot_env/init', Init)
        self._close_client = rospy.ServiceProxy(
            '/willbot_env/close', Close)
        self._seed_client = rospy.ServiceProxy(
            '/willbot_env/seed', Seed)
        self._reset_client = actionlib.SimpleActionClient(
            '/willbot_env/reset', EnvResetAction)
        self._step_client = actionlib.SimpleActionClient(
            '/willbot_env/step', EnvStepAction)

        self._init_client.wait_for_service(timeout=5.0)
        resp = self._init_client(
            environment_id, dumps(params))
        self._session_id = resp.session_id

    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state."""

        goal = EnvStepGoal(
            session_id=self._session_id,
            action=dumps(action)
        )
        self._step_client.send_goal(goal)

        finished = self._step_client.wait_for_result(rospy.Duration(30.0))
        if not finished:
            raise RuntimeError('Environment server not responding')

        state = self._step_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError(self._step_client.get_goal_status_text())

        result = self._step_client.get_result()
        observation = loads(result.observation)
        reward = result.reward
        done = result.done
        info = loads(result.info)

        return observation, reward, done, info

    def reset(self, **params):
        """Resets the state of the environment and returns an initial observation."""

        goal = EnvResetGoal(
            session_id=self._session_id,
            params=dumps(params)
        )
        self._reset_client.send_goal(goal)

        finished = self._reset_client.wait_for_result(rospy.Duration(30.0))
        if not finished:
            raise RuntimeError('Environment server not responding')

        state = self._reset_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError(self._reset_client.get_goal_status_text())

        result = self._reset_client.get_result()
        observation = loads(result.observation)

        return observation

    def seed(self, seed=None):
        """Sets the seed for this env's random number generator(s)."""

        resp = self._seed_client(self._session_id, seed)
        return resp.seeds

    def render(self, mode='human', close=False):
        """Renders the environment."""
        raise NotImplementedError

    def close(self):
        """Override _close in your subclass to perform any necessary cleanup."""

        if not rospy.core.is_shutdown() and self._session_id != -1:
            self._reset_client.cancel_all_goals()
            self._step_client.cancel_all_goals()
            self._close_client(self._session_id)
            self._session_id = -1

    def __enter__(self):
        """Just return self."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """close connection at exit."""
        self.close()
