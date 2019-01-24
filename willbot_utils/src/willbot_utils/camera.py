import numpy as np

import rospy
from sensor_msgs.msg import Image

from willbot_utils.img_utils import imgmsg_to_array


class ImageListener():
    """ 
    Wrapper to asynchronously receiving latest Image from a topic.

    Useful when you capture data frequently.    
    """

    def __init__(self, topic, statistics=False):
        self._topic = topic
        self._imgmsg = None
        self._delays = []

        def imgmsg_cb(msg):
            self._imgmsg = msg

            if statistics:
                delay = (rospy.Time.now() - msg.header.stamp).to_sec()
                self._delays.append(delay)
                if len(self._delays) > 100:
                    self._delays.pop(0)

        self._sub = rospy.Subscriber(
            topic, Image, imgmsg_cb, queue_size=1, buff_size=2**24)

        deadline = rospy.Time.now() + rospy.Duration(1.0)
        while not rospy.core.is_shutdown() and self._imgmsg is None:
            if rospy.Time.now() > deadline:
                rospy.logwarn_throttle(
                    1.0, 'Waiting for an image ({})...'.format(topic))
            rospy.rostime.wallsleep(0.01)

        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")

    def latest(self, encoding=None, with_time=False):
        """ 
        Return latest received message as numpy array in specified encoding.

        @param encoding: one of 'rgb8', 'bgr8', 'F32C1'. Auto if not specified.
        @param with_time: return tuple (mmesage time, data) if True.
        """
        msg = self._imgmsg
        data = imgmsg_to_array(msg, encoding)
        if with_time:
            return (msg.header.stamp, data)
        return data

    def delay_statistic(self):
        """ Return delay statistic for debug purposes. """
        return self._delays.copy()


class ImageSnapshot():
    """  
    Wrapper to synchronously receiving Image from a topic.

    Useful when you capture data rarely.
    """

    def __init__(self, topic):
        self._topic = topic

    def wait_for_image(self, encoding=None, with_time=False, timeout=None):
        """ 
        Return next received image as numpy array in specified encoding.

        @param encoding: one of 'rgb8', 'bgr8', 'F32C1'. Auto if not specified.
        @param with_time: return tuple (mmesage time, data) if True.
        @param timeout: time in seconds
        """
        msg = rospy.wait_for_message(self._topic, Image, timeout=timeout)
        data = imgmsg_to_array(msg, encoding)
        if with_time:
            return (msg.header.stamp, data)
        return data
