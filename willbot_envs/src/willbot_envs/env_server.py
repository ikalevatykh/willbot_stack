import rospy
import actionlib

from willbot_envs.msg import EnvResetAction, EnvResetResult
from willbot_envs.msg import EnvStepAction, EnvStepResult

from willbot_envs.utils import loads, dumps
from willbot_envs.envs.pick_env import PickEnv


class EnvironmentServer(object):
    def __init__(self):
        self._environment_id = 'UR5-PickEnv-v0'  # TODO: choose by name
        self._environment = PickEnv()
        self._episode_id = 0

        self._reset_server = actionlib.SimpleActionServer(
            '/willbot_env/reset', EnvResetAction,
            execute_cb=self.reset_cb, auto_start=False)

        self._step_server = actionlib.SimpleActionServer(
            '/willbot_env/step', EnvStepAction,
            execute_cb=self.step_cb, auto_start=False)

        self._reset_server.start()            
        self._step_server.start()

        rospy.loginfo('Environment ready')

    def reset_cb(self, goal):
        try:
            rospy.logdebug('reset')

            if goal.environment_id != self._environment_id:
                raise RuntimeError(
                    "Wrong environment id '{}', expected: '{}'".format(
                        goal.environment_id, self._environment_id))

            self._episode_id += 1

            if goal.seed != goal.NO_SEED:
                self._environment.seed(goal.seed)

            observation = self._environment.reset()

            result = EnvResetResult(
                episode_id=self._episode_id,
                observation=dumps(observation)
            )
            self._reset_server.set_succeeded(result)
        except Exception as e:
            rospy.logerr('Reset exception: %s', e)
            self._reset_server.set_aborted(text=e.message)

    def step_cb(self, goal):
        try:
            rospy.logdebug('step %s', goal.action)

            if self._environment is None:
                raise RuntimeError(
                    'Environment not initialized')

            if goal.episode_id != self._episode_id:
                raise RuntimeError(
                    'Wrong episode id, only one client can be connected')

            action = loads(goal.action)
            obs, reward, done, info = self._environment.step(action)

            result = EnvStepResult(
                observation=dumps(obs),
                reward=reward,
                done=done,
                info=dumps(info)
            )
            self._step_server.set_succeeded(result)
        except Exception as e:
            rospy.logerr('Step exception: %s', e)
            self._step_server.set_aborted(text=e.message)
