#!/usr/bin/env python
import sys
import json
import numpy as np

import rospy

from willbot_envs.env_client import EnvironmentClient
from willbot_envs.env_wrappers import CameraObserver


def run_client():
    """Test environment client.

    This node can be started from python 3.x.
    """
    print(sys.version)

    # Client for standalone environment server
    env = EnvironmentClient('UR5-PickEnv-v0')
    env = CameraObserver(env, key='depth', topic='/kinect2/sd/image_depth_rect')

    env.seed(0)
    obs = env.reset()
    print('reset ok')
    print('obs', obs.keys())

    act = dict(
        tool_position=(0.3, 0.1, 0.250)
    )
    obs, reward, done, info = env.step(act)
    print('step ok')
    print('obs', obs.keys())


def main():
    rospy.myargv(argv=sys.argv)
    rospy.init_node('willbot_env_client', anonymous=False)
    try:
        run_client()
    except Exception as e:
        rospy.logerr(e)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
