#!/usr/bin/env python2

from __future__ import print_function

import numpy as np
import rospy

from geometry_msgs.msg import TwistStamped, Vector3

from willbot_utils.arm import UR5
from willbot_utils.controller import Controller, ControllerManager
from willbot_utils.joystick import Joystick
from willbot_utils.hand import RobotiqHand
from willbot_utils.scene import StandardScene


class VelocityController(Controller):
    def __init__(self, cm, name='cartesian_velocity_controller'):
        Controller.__init__(self, cm, name)
        topic = '/{}/{}/command'.format(cm.group, name)
        self._pub = rospy.Publisher(topic, TwistStamped, queue_size=10)

    def command(self, linear, angular):
        msg = TwistStamped()
        msg.header.frame_id = 'world'
        msg.twist.linear = Vector3(*linear)
        msg.twist.angular = Vector3(*angular)
        self._pub.publish(msg)


class JoyControllerNode(object):
    def __init__(self):
        print('Setup...')

        hand = RobotiqHand()
        hand.mode = 1
        hand.target_velocity = 15
        hand.target_effort = 5

        arm = UR5(hand=hand)
        ws = np.array([[0.3, -0.2, 0.05], [0.9, 0.2, 0.25]])
        arm.set_workspace(ws.flatten())

        scene = StandardScene(arm)
        joy = Joystick()

        print('Ready!')

        cm = ControllerManager('arm')
        with VelocityController(cm) as arm_ctrl:
            self.run(joy, arm_ctrl, hand)

    def run(self, joy, arm_ctrl, hand):
        max_linear_vel = rospy.get_param('max_linear_vel', 0.05)
        max_angular_vel = rospy.get_param('max_angular_vel', 0.05)

        hand.mode = 1
        grasp = False

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            joy.update()

            signal = joy.axes[[2, 3, 1]]
            if np.any(signal):
                if joy.buttons[5]:
                    # rotation
                    w = signal * [1, -1, 1] * max_angular_vel
                    v = (0, 0, 0)
                else:
                    # linear step
                    v = signal * [1, -1, 1] * max_linear_vel
                    w = (0, 0, 0)

                arm_ctrl.command(v, w)

            if np.any(joy.pushed[0:3]):
                hand.mode = np.argmax(joy.pushed[0:3])
                if grasp:
                    hand.close(wait=False)
                else:
                    hand.open(wait=False)
                hand.move(int(grasp) * 255)
                print('Changed hand mode:', hand.mode)

            if np.any(joy.pushed[4]):
                grasp = not grasp
                if grasp:
                    hand.close(wait=False)
                else:
                    hand.open(wait=False)
                print('Grasped' if grasp else 'Released')

            rate.sleep()


def main():
    rospy.init_node('joy_controller', anonymous=True)
    try:
        JoyControllerNode()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
