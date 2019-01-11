import numpy as np
import rospy

from sensor_msgs.msg import Joy


class Joystick():
    def __init__(self):

        def joy_cb(msg):
            self._message = msg
            self._counter += np.int32(msg.buttons)

        joy_topic = rospy.get_param('joy_topic', '/joy')
        self._message = rospy.wait_for_message(joy_topic, Joy)
        self._buttons = np.int32(self._message.buttons)
        self._counter = np.zeros_like(self._buttons)
        self.update()
        rospy.Subscriber(joy_topic, Joy, joy_cb, queue_size=10)

    def update(self):
        msg = self._message
        self._axes = np.float32(msg.axes)
        self._pushed = np.logical_and(
                self._counter, np.logical_not(self._buttons))
        self._counter = 0
        self._released = np.logical_and(
                self._buttons, np.logical_not(msg.buttons))
        self._buttons = np.int32(msg.buttons)

    @property
    def axes(self):
        return self._axes

    @property
    def buttons(self):
        return self._buttons

    @property
    def pushed(self):
        return self._pushed

    @property
    def released(self):
        return self._released
