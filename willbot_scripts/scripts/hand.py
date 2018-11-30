import numpy as np
import rospy
from robotiq_s_model_control.msg import _SModel_robot_input as inputMsg
from robotiq_s_model_control.msg import _SModel_robot_output as outputMsg


class RobotiqHand():
    def __init__(self):
        self._status_msg = None
        self._in_motion = False
        self._is_object_held = False
        self._position = 0

        def status_cb(status):
            self._status_msg = status
            self._in_motion = (status.gSTA == 0)
            self._is_object_held = (status.gSTA == 1 or status.gSTA == 2)
            self._position = status.gPOA

        msg_type = inputMsg.SModel_robot_input
        rospy.Subscriber("SModelRobotInput", msg_type, status_cb)
        while self._status_msg is None:
            rospy.sleep(0.1)

        msg_type = outputMsg.SModel_robot_output
        self.pub = rospy.Publisher('SModelRobotOutput', msg_type, queue_size=10)
        self.command = outputMsg.SModel_robot_output()
        self._mode = np.uint8(1)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        self._mode = value

    @property
    def position(self):
        return self._position

    @property
    def is_object_held(self):
        return self._is_object_held

    def move(self, position):
        self.command.rACT = 1
        self.command.rMOD = self._mode
        self.command.rGTO = 1
        self.command.rATR = 0
        self.command.rPRA = position
        self.command.rSPA = 1 #255
        self.command.rFRA = 1 #150
        self.pub.publish(self.command)

    def open(self, wait=False):
        self.move(0)
        if wait:
            self.wait()

    def close(self, wait=False):
        self.move(255)
        if wait:
            self.wait()

    def wait(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if not self._in_motion:
                rospy.sleep(1.0)
                break


