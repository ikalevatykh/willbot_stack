#!/usr/bin/env python2

from __future__ import print_function

import json
import numpy as np
import rospy

from geometry_msgs.msg import TwistStamped, Vector3

from willbot_utils.arm import UR5
from willbot_utils.hand import RobotiqHand
from willbot_utils.controller import Controller, ControllerManager
from willbot_utils.scene import StandardScene


class VelocityController(Controller):
    def __init__(self, cm, name='cartesian_velocity_safe_controller'):
        Controller.__init__(self, cm, name)
        topic = '/{}/{}/command'.format(cm.group, name)
        self._pub = rospy.Publisher(topic, TwistStamped, queue_size=1)

    def command(self, linear, angular):
        msg = TwistStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = rospy.Time.now() + rospy.Duration(15.0)
        msg.twist.linear = Vector3(*linear)
        msg.twist.angular = Vector3(*angular)
        self._pub.publish(msg)


class EstimationNode(object):
    def __init__(self):
        print('Setup...')

        hand = RobotiqHand()
        hand.mode = 1
        hand.target_velocity = 0
        hand.target_effort = 255

        hand.close()

        arm = UR5()
        ws = np.array([[0.3, -0.2, 0.05], [0.9, 0.2, 0.25]])
        arm.set_workspace(ws.flatten())

        cm = ControllerManager('arm')

        q = arm.get_current_quaternion()
        print(q)
        o = arm.get_current_rpy()
        print(o)
        #exit(0)

        print('Ready!')

        self.run(arm, cm)

    def run(self, arm, cm):

        statistics = []
        orn = (np.pi, 0, np.pi / 2)  # [0, 1 / np.sqrt(2), 1 / np.sqrt(2), 0]

        for epoch in range(2):
            for x in np.linspace(0.3, 0.7, 10):
                for y in np.linspace(-0.2, 0.2, 10):
                    arm.joint_move(pos=(x, y, 0.05), orn=orn)

                    with VelocityController(cm) as vc:
                        rospy.sleep(0.5)
                        vc.command((0, 0, -0.01), (0, 0, 0))
                        rospy.sleep(5)

                    pos = arm.get_current_pos_orn()
                    statistics.append(pos)

                    arm.joint_move(pos=(x, y, 0.05), orn=orn)

                with open('/home/robot/statistics.json', 'w') as f:
                    json.dump(statistics, f)


def main():
    rospy.init_node('plane_estimation', anonymous=True)
    try:
        EstimationNode()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
