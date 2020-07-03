#!/usr/bin/env python2
import sys
import sys
import traceback
import numpy as np

import rospy

from willbot_envs.env_client import EnvironmentClient
from willbot_envs.env_wrappers import CameraObserver, JointStateObserver


def run_client():
    """Test environment client.

    This node can be started from python 3.x.
    """
    # Client for standalone environment server
    with EnvironmentClient('ReachEnv') as env:

        # Wrappers for observations
        env = CameraObserver(
            env, key='depth', topic='/kinect2/sd/image_depth_rect')
        env = CameraObserver(
            env, key='rgb', topic='/kinect2/sd/image_color_rect')
        env = JointStateObserver(env)

        # Use env like an ordinary gym environment
        env.seed(777)
        obs = env.reset()
        rospy.loginfo('Reset ok')

        done = False
        while not done:
            target_xy = obs['target_position']
            act = dict(
                tool_position=target_xy + [0.20, ]
            )
            obs, reward, done, info = env.step(act)
            rospy.loginfo('Step ok')


def main():
    try:
        run_client()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
