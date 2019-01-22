#!/usr/bin/env python
import sys
import json
import numpy as np

import rospy

from willbot_envs.env_client import EnvironmentClient


def run_client():
    """Test environment client.

    This node can be started from python 3.x.
    """
    print(sys.version)

    # Client for standalone environment server
    env = EnvironmentClient('UR5-PickEnv-v0')

    env.seed(0)
    obs = env.reset()
    print('reset ok')
    print('obs', obs)

    act = dict(
        tool_position=(0.3, 0.1, 0.250)
    )
    obs = env.step(act)
    print('step ok')
    print('obs', obs)


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
