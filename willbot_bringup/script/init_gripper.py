#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

rospy.myargv(argv=sys.argv)
rospy.init_node('init_gripper', anonymous=False)

try:
    gripper = rospy.get_param('~gripper_name', 'gripper')

    # waiting for a state to be ensure the gripper is connected
    topic = '/{}/joint_states'.format(gripper)
    rospy.wait_for_message(topic, JointState, timeout=5.0)

    # initialize the gripper if it is not yet initialized
    service = '/{}/init'.format(gripper)
    rospy.wait_for_service(service, timeout=5.0)

    init_srv = rospy.ServiceProxy(service, Trigger)
    resp = init_srv()

    if not resp.success:
        raise RuntimeError(resp.message)

    rospy.loginfo('Gripper initialized')
except Exception as e:
    rospy.logerr('Error during gripper initialization: {}'.format(e))
