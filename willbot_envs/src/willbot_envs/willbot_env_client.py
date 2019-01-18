import json
import gym

import rospy
import actionlib

import willbot_envs.msg


class WillbotEnv(gym.Env):

    def __init__(self):
        self._reset_client = actionlib.SimpleActionClient(
            '/willbot_env/reset', willbot_envs.msg.EnvReset)
        self._reset_client.wait_for_server()

        self._step_client = actionlib.SimpleActionClient(
            '/willbot_env/step', willbot_envs.msg.EnvStep)
        self._step_client.wait_for_server()

        self._np_random = None
        self._seed = 0
        self._episode_id = 0     

    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state.

        Accepts an action and returns a tuple (observation, reward, done, info).

        Args:
            action (object): an action provided by the environment

        Returns:
            observation (object): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (boolean): whether the episode has ended, in which case further step() calls will return undefined results
            info (dict): contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """

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
        """Resets the state of the environment and returns an initial observation.

        Returns: observation (object): the initial observation of the
            space.
        """

        goal = willbot_envs.msg.EnvResetGoal()
        self._reset_client.send_goal(goal)
        self._reset_client.wait_for_result()

        result = self._reset_client.get_result()
        self._episode_id = result.episode_id
        observation = json.loads(result.observation)

        return observation

    def render(self, mode='human', close=False):
        """Renders the environment.

        Args:
            mode (str): the mode to render with
            close (bool): close all open renderings
        """
        raise NotImplementedError

    def close(self):
        """Override _close in your subclass to perform any necessary cleanup.

        Environments will automatically close() themselves when
        garbage collected or when the program exits.
        """
        return

    def seed(self, seed=None):
        """Sets the seed for this env's random number generator(s).

        Note:
            Some environments use multiple pseudorandom number generators.
            We want to capture all such seeds used in order to ensure that
            there aren't accidental correlations between multiple generators.

        Returns:
            list<bigint>: Returns the list of seeds used in this env's random
              number generators. The first value in the list should be the
              "main" seed, or the value which a reproducer should pass to
              'seed'. Often, the main seed equals the provided 'seed', but
              this won't be true if seed=None, for example.
        """

        rospy.set_param('/willbot_env/seed', seed)
        np_random, seed = gym.utils.seeding.np_random(seed)
        self._np_random = np_random
        self._seed = seed
        return [seed]
