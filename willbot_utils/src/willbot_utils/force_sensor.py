import numpy as np
import rospy

from geometry_msgs.msg import WrenchStamped


class ForceTorqueListener():
    """
    Helper to asynchronously receiving latest force torque data from a topic.
    """

    def __init__(self, topic='/robotiq_ft_wrench', callback=None):
        self._callback = callback
        self._topic = topic

        self._msg = None
        rospy.Subscriber(topic, WrenchStamped, self._wrench_cb, queue_size=100)

        deadline = rospy.Time.now() + rospy.Duration(1.0)
        while not rospy.core.is_shutdown() and self._msg is None:
            if rospy.Time.now() > deadline:
                rospy.logwarn_throttle(
                    1.0, 'Waiting for an wrench ({})...'.format(topic))
            rospy.rostime.wallsleep(0.01)

        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")

    def _wrench_cb(self, msg):
        self._msg = msg
        if self._callback is not None:
            self._callback(msg)

    @property
    def wrench(self):
        return self._msg


class ForceLimitTrigger():
    def __init__(self, callback, absolute_limit=0.0, relative_error=0.0):
        self._callback = callback
        self._absolute_limit = absolute_limit
        self._relative_limit = relative_error
        self._bias_force = None
        self._prev_force = None
        self._sensor = ForceTorqueListener(callback=self._wrench_cb)

    def _wrench_cb(self, msg):
        msg_force = msg.wrench.force
        force = np.array([msg_force.x, msg_force.y, msg_force.z])

        if self._absolute_limit:
            if self._bias_force is None:
                self._bias_force = force

            diff = np.linalg.norm(force - self._bias_force)
            if diff > self._absolute_limit:
                rospy.logwarn('Force absolute limit exceeded, {} > {}'.format(force, self._bias_force))
                self._callback()

        if self._relative_limit:
            if self._prev_force is None:
                self._prev_force = force

            diff = np.linalg.norm(force - self._prev_force)
            if diff > self._relative_limit:
                rospy.logwarn('Force relative limit exceeded, {} > {}'.format(force, self._prev_force))
                self._callback()

        self._prev_force = force
