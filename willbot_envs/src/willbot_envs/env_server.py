import sys
import traceback
import rospy
import actionlib

from willbot_envs.msg import EnvResetAction, EnvResetResult
from willbot_envs.msg import EnvStepAction, EnvStepResult
from willbot_envs.srv import Init, InitResponse
from willbot_envs.srv import Close, CloseResponse
from willbot_envs.srv import Seed, SeedResponse

from willbot_envs.utils import loads, dumps, load_env
from willbot_envs.envs.pick_env import PickEnv


class EnvironmentServer(object):
    def __init__(self):
        self._debug = rospy.get_param('~debug', default=True)
        self._environment = None
        self._session_id = -1

        self._seed_server = rospy.Service(
            '/willbot_env/init', Init, self.init_cb)
        self._seed_server = rospy.Service(
            '/willbot_env/close', Close, self.close_cb)
        self._seed_server = rospy.Service(
            '/willbot_env/seed', Seed, self.seed_cb)
        self._reset_server = actionlib.SimpleActionServer(
            '/willbot_env/reset', EnvResetAction,
            execute_cb=self.reset_cb, auto_start=False)
        self._step_server = actionlib.SimpleActionServer(
            '/willbot_env/step', EnvStepAction,
            execute_cb=self.step_cb, auto_start=False)

        self._reset_server.start()
        self._step_server.start()

        rospy.loginfo('Environment ready')

    def init_cb(self, req):
        try:
            rospy.logdebug('init %s', req.environment_id)

            if self._environment is not None:
                self._environment.close()
                self._environment = None

            env_cls = load_env(req.environment_id)
            params = loads(req.params)
            
            self._environment = env_cls(**params)
            self._session_id += 1

            result = InitResponse(
                session_id=self._session_id
            )
            return result
        except Exception as e:
            rospy.logerr('Init exception: %s', e)
            if self._debug:
                traceback.print_exc(file=sys.stdout)
            raise e

    def close_cb(self, req):
        try:
            rospy.logdebug('close %s', req.session_id)

            if req.session_id != self._session_id:
                raise RuntimeError('Wrong session')

            self._environment.close()
            self._environment = None

            return CloseResponse()
        except Exception as e:
            rospy.logerr('Seed exception: %s', e)
            if self._debug:
                traceback.print_exc(file=sys.stdout)
            raise e

    def seed_cb(self, req):
        try:
            rospy.logdebug('seed %s', req.seed)

            if req.session_id != self._session_id:
                raise RuntimeError('Wrong session')

            seeds = self._environment.seed(req.seed)

            result = SeedResponse(
                seeds=seeds
            )
            return result
        except Exception as e:
            rospy.logerr('Seed exception: %s', e)
            if self._debug:
                traceback.print_exc(file=sys.stdout)
            raise e

    def reset_cb(self, goal):
        try:
            rospy.logdebug('reset')

            if goal.session_id != self._session_id:
                raise RuntimeError('Wrong session')

            params = loads(goal.params)
            observation = self._environment.reset(**params)

            result = EnvResetResult(
                observation=dumps(observation)
            )
            self._reset_server.set_succeeded(result)
        except Exception as e:
            rospy.logerr('Reset exception: %s', e)
            if self._debug:
                traceback.print_exc(file=sys.stdout)
            self._reset_server.set_aborted(text=e.message)

    def step_cb(self, goal):
        try:
            rospy.logdebug('step %s', goal.action)

            if goal.session_id != self._session_id:
                raise RuntimeError('Wrong session')

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
            if self._debug:
                traceback.print_exc(file=sys.stdout)
            self._step_server.set_aborted(text=e.message)
