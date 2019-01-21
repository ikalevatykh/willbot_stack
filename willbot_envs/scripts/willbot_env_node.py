#!/usr/bin/env python
from __future__ import print_function

import sys
import json
import numpy as np

import rospy
import actionlib

import willbot_envs.msg

from willbot_pick_env import WillbotPickEnv


class EnvironmentNode(object):
    def __init__(self):
        self._environment_id = 'UR5-PickEnv-v0' #TODO: choose by name
        self._environment = WillbotPickEnv()
        self._episode_id = 0

        self._reset_server = actionlib.SimpleActionServer(
            '/willbot_env/reset',  willbot_envs.msg.EnvResetAction,
            execute_cb=self.reset_cb, auto_start=False)
        self._reset_server.start()

        self._step_server = actionlib.SimpleActionServer(
            '/willbot_env/step',  willbot_envs.msg.EnvStepAction,
            execute_cb=self.step_cb, auto_start=False)
        self._step_server.start()

        rospy.loginfo('Environment ready')

    def reset_cb(self, goal):
        try:
            rospy.loginfo('reset')

            if goal.environment_id != self._environment_id:
                raise RuntimeError(
                    "Wrong environment id '{}', expected: '{}'".format(
                        goal.environment_id, self._environment_id))

            self._episode_id += 1

            if goal.seed != goal.NO_SEED:
                self._environment.seed(goal.seed)

            observation = self._environment.reset()

            result = willbot_envs.msg.EnvResetResult(
                episode_id=self._episode_id,
                observation=json.dumps(observation)
            )
            self._reset_server.set_succeeded(result)
        except Exception as e:
            print(e)
            self._reset_server.set_aborted(text=e.message)

    def step_cb(self, goal):
        try:
            rospy.loginfo('step %s', goal.action)

            if self._environment is None:
                raise RuntimeError(
                    'Environment not initialized')

            if goal.episode_id != self._episode_id:
                raise RuntimeError(
                    'Wrong episode id, only one client can be connected')

            action = json.loads(goal.action)
            obs, reward, done, info = self._environment.step(action)

            result = willbot_envs.msg.EnvStepResult(
                observation=json.dumps(obs),
                reward=reward,
                done=done,
                info=json.dumps(info)
            )
            self._reset_server.set_succeeded(result)
        except Exception as e:
            self._reset_server.set_aborted(text=e.message)


def main():
    rospy.myargv(argv=sys.argv)
    rospy.init_node('willbot_env', anonymous=False)
    try:
        node = EnvironmentNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
