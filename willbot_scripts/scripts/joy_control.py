#!/usr/bin/env python

from __future__ import print_function

import sys

import moveit_commander
import numpy as np
import rospy

from willbot_utils.joystick import Joystick
from willbot_utils.scene import StandardScene
from willbot_utils.velocity_controller import VelocityController, ControllerException
from willbot_utils.hand import RobotiqHand


class JoyControllerNode(object):
    def __init__(self):
        print('Setup...')

        arm = moveit_commander.MoveGroupCommander("manipulator")
        arm.set_planner_id("RRTConnectkConfigDefault")
        arm.set_max_velocity_scaling_factor(0.5)
        scene = StandardScene()
        joy = Joystick()
        hand = RobotiqHand()
        ws = np.array([[0.3, -0.2, 0.05], [0.9, 0.2, 0.25]])
        arm.set_workspace(ws.flatten())        

        print('Ready!')

        with VelocityController(arm, ws) as arm_ctrl:
            self.run(joy, arm_ctrl, hand)

    def run(self, joy, arm_ctrl, hand):
        max_linear_vel = rospy.get_param('max_linear_vel', 0.05)
        max_angular_vel = rospy.get_param('max_angular_vel', 0.1)

        hand.mode = 1
        grasp = hand.is_object_held         

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            joy.update()

            signal = joy.axes[[2, 3, 1]]
            if np.any(signal):
                if joy.buttons[5]:
                    # rotation
                    w = signal * [1, -1, 1] * max_linear_vel
                    v = (0, 0, 0)
                else:
                    # linear step
                    v = signal * [1, -1, 1] * max_angular_vel
                    w = (0, 0, 0)
                try:
                    arm_ctrl.step(0.1, v, w, check=True)
                except ControllerException as e:
                    print(e)

            if np.any(joy.pushed[0:3]):
                hand.mode = np.argmax(joy.pushed[0:3])
                hand.move(int(grasp) * 255)
                print('Changed hand mode:', hand.mode)

            if np.any(joy.pushed[4]):
                grasp = not grasp
                hand.move(int(grasp) * 255)
                print('Grasped' if grasp else 'Released')

            print(arm_ctrl._group.get_current_pose().pose)

            rate.sleep()


def main():
    rospy.init_node('joy_controller', anonymous=True)
    try:
        JoyControllerNode()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
