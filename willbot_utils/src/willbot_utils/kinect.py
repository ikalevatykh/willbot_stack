import numpy as np

import rospy
from willbot_utils.camera import ImageListener, ImageSnapshot
from willbot_utils.img_utils import scale_depth


DEFAULT_COLOR_TOPIC = '/camera/rgb/image_rect_color'
DEFAULT_DEPTH_TOPIC = '/camera/depth_registered/image'


class KinectListener():
    """
    Wrapper to asynchronously receiving latest rgbd from kinect topics.

    Useful when you capture data frequently.
    """

    def __init__(self, color_topic='', depth_topic=''):
        # TODO: From Kinect v2.0 can receive synchronized color and depth
        self._color_listener = ImageListener(
            color_topic or DEFAULT_COLOR_TOPIC)
        self._depth_listener = ImageListener(
            depth_topic or DEFAULT_COLOR_TOPIC)

    def rgb(self):
        """ Return color image as np.uint8 array """
        return self._color_listener.latest(encoding='rgb8')

    def depth(self):
        """ Return depth data in meters as np.float32 array """
        depth = self._depth_listener.latest()
        if depth.dtype == np.uint16:
            return depth * np.float32(0.001)
        return depth

    def depth_u8(self, near, far):
        """ Scale specified depth range to uint and return as np.uint8 array """
        return scale_depth(self.depth(), near, far)


class KinectSnapshot():
    """
    Wrapper to synchronously receiving rgbd from kinect topics.

    Useful when you capture data rarely.
    """

    def __init__(self, color_topic='', depth_topic=''):
        self._color_snapshot = ImageSnapshot(
            color_topic or DEFAULT_COLOR_TOPIC)
        self._depth_snapshot = ImageSnapshot(
            depth_topic or DEFAULT_DEPTH_TOPIC)

    def rgb(self):
        """ Return color image as np.uint8 array """
        return self._color_snapshot.wait_for_image(encoding='rgb8', timeout=5)

    def depth(self):
        """ Return depth data in meters as np.float32 array """
        depth = self._depth_snapshot.wait_for_image(timeout=5)
        if depth.dtype == np.uint16:
            return depth * np.float32(0.001)
        return depth

    def depth_u8(self, near, far):
        """ Scale specified depth range to uint and return as np.uint8 array """
        return scale_depth(self.depth(), near, far)
