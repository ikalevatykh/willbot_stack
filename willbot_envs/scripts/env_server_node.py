#!/usr/bin/env python
import sys
import rospy

from willbot_envs.env_server import EnvironmentServer


def main():
    rospy.myargv(argv=sys.argv)
    rospy.init_node('willbot_env', anonymous=False)
    try:
        node = EnvironmentServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
