import rospy
from sensor_msgs.msg import JointState


class JointStateListener:
    """Wrapper to asynchronously receiving latest JointState from a topic."""

    def __init__(self, topic='/joint_states', joint_names=None):
        """[summary]

        Args:
            topic (str, optional): topic name. Defaults to '/joint_states'.
            joint_names (list, optional): joint filter and order. Defaults to None.

        Raises:
            rospy.exceptions.ROSInterruptException: in case of ros is shutdown
        """
        self._topic = topic
        self._state_msg = None
        self._joint_names = joint_names

        def _state_cb(msg):
            self._state_msg = msg

        self._sub = rospy.Subscriber(
            topic, JointState, _state_cb, queue_size=10)

        deadline = rospy.Time.now() + rospy.Duration(1.0)
        while not rospy.core.is_shutdown() and self._state_msg is None:
            if rospy.Time.now() > deadline:
                rospy.logwarn_throttle(
                    1.0, 'Waiting for joint ststes ({})...'.format(topic))
            rospy.rostime.wallsleep(0.01)

        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")

    def latest(self):
        """Return latest received message.

        Joint filtering and reordering will be applied if joint_names parameter is set.
        """
        msg = self._state_msg
        if self._joint_names is not None:
            indices = [msg.name.index(name) for name in self._joint_names]
            msg.name = [msg.name[i] for i in indices]
            msg.position = [msg.position[i] for i in indices]
            msg.velociy = [msg.velociy[i] for i in indices]
            msg.effort = [msg.effort[i] for i in indices]
        return msg
