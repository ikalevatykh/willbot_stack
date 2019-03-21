#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger


init_srv = rospy.ServiceProxy('/gripper/init', Trigger)

try:
    init_srv.wait_for_service(timeout=5.0)
    resp = init_srv()
    if not resp.success:
        raise RuntimeError(resp.message)
    rospy.loginfo('Gripper initialized')
except Exception as e:
    rospy.logerr('Error during gripper initialization: {}'.format(e))
