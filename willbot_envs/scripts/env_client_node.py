#!/usr/bin/env python
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
    env = EnvironmentClient('ReachEnv')

    # Wrappers for observations what you need
    env = CameraObserver(
        env, key='depth', topic='/kinect2/sd/image_depth_rect')
    env = CameraObserver(
        env, key='rgb', topic='/kinect2/sd/image_color_rect')
    env = JointStateObserver(env)

    # Use env like an ordinary gym environment
    env.seed(0)
    obs = env.reset()
    rospy.loginfo('Reset ok')

    tool_pose = obs['target_position'] + [0.20, ]
    act = dict(
        tool_position=tool_pose
    )
    obs, reward, done, info = env.step(act)
    rospy.loginfo('Step ok. done={}', done)


def main():
    rospy.myargv(argv=sys.argv)
    rospy.init_node('willbot_env_client', anonymous=False)
    try:
        run_client()
    except Exception as e:
        rospy.logerr(e)
        traceback.print_exc(file=sys.stdout)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
