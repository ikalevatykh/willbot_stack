import json
from gym.utils import seeding

import rospy
import actionlib

import willbot_envs.msg


class WillbotEnvClient(gym.Env):

    def __init__(self, environment_id):
        self._reset_client = actionlib.SimpleActionClient(
            '/willbot_env/reset', willbot_envs.msg.EnvResetAction)
        self._reset_client.wait_for_server()

        self._step_client = actionlib.SimpleActionClient(
            '/willbot_env/step', willbot_envs.msg.EnvStepAction)
        self._step_client.wait_for_server()

        self._environment_id = environment_id
        self._episode_id = 0
        self._np_random = None
        self._seed = 0

    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state."""

        goal = willbot_envs.msg.EnvStepGoal(
            episode_id=self._episode_id,
            action=json.dumps(action))
        self._reset_client.send_goal(goal)
        self._reset_client.wait_for_result()

        result = self._reset_client.get_result()
        observation = json.loads(result.observation)
        reward = result.reward
        done = result.done
        info = json.loads(result.info)

        return observation, reward, done, info

    def reset(self):
        """Resets the state of the environment and returns an initial observation."""

        goal = willbot_envs.msg.EnvResetGoal(
            environment_id=self._environment_id,
            seed=self._seed
        )
        self._reset_client.send_goal(goal)
        self._reset_client.wait_for_result()

        result = self._reset_client.get_result()
        self._episode_id = result.episode_id
        observation = json.loads(result.observation)

        return observation

    def render(self, mode='human', close=False):
        """Renders the environment."""
        raise NotImplementedError

    def close(self):
        """Override _close in your subclass to perform any necessary cleanup."""
        return

    def seed(self, seed=None):
        """Sets the seed for this env's random number generator(s)."""

        np_random, seed = seeding.np_random(seed)
        self._np_random = np_random
        self._seed = seed
        return [seed]
