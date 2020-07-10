import numpy as np

import rospy
from geometry_msgs.msg import TwistStamped, Vector3
from .controller import Controller, ControllerManager


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
