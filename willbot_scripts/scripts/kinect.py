import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from PIL import Image as PIL_Image

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import tf
import tf2_ros

from convert_functions import transform_point_cloud


class Kinect():
    def __init__(self, depth=False, rgb=False, point_cloud=False, depth_conv=0):    
        if depth:    
            image_topic = rospy.get_param('depth_registered_topic',
                                          '/camera/depth_registered/image')
            print('Waiting for depth data ({})'.format(image_topic))
            self._depth_msg = None
            self._depth_conv_version = depth_conv

            def depth_cb(msg):
                self._depth_msg = msg

            s = rospy.Subscriber(image_topic, Image, depth_cb, queue_size=1)
            while self._depth_msg is None:
                rospy.sleep(0.1)
                
        if point_cloud:    
            points_topic = '/camera/depth_registered/points'
            print('Waiting for point cloud data ({})'.format(points_topic))                                         
            self._points_msg = None

            def points_cb(msg):
                self._points_msg = msg

            s = rospy.Subscriber(points_topic, PointCloud2, points_cb, queue_size=1)
            while self._points_msg is None:
                rospy.sleep(0.1)        

        self._bridge = CvBridge()
        #tf_buffer = tf2_ros.Buffer()
        #self._listener = tf2_ros.TransformListener(tf_buffer)  
        self._listener = tf.TransformListener()
        #self._listener.waitForTransform("/camera_rgb_optical_frame", "/base_link", rospy.Time(0),rospy.Duration(2.0)) 

    @property
    def depth_image(self):
        if self._depth_msg is None:
            return None, None
        msg = self._depth_msg
        depth_image = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
        return depth_image, msg.header.stamp
        
    @property
    def depth_uint8(self):
        depth_image, time_stamp = self.depth_image
        if depth_image is None:
            return None    
        im_depth_scaled = depth_image.astype(np.float32)
        max_depth_real = 1.8
        
        if self._depth_conv_version == 0:
            im_depth_scaled[np.isnan(im_depth_scaled)] = 0
            im_depth_scaled[im_depth_scaled > max_depth_real] = 0
            im = im_depth_scaled / max_depth_real
            im_pil = PIL_Image.fromarray((im / max_depth_real * 255).astype(np.uint8))
        elif self._depth_conv_version == 1:
            im_depth_scaled[np.isnan(im_depth_scaled)] = 0
            im_depth_scaled[im_depth_scaled > max_depth_real] = 0
            im = im_depth_scaled / max_depth_real
            im_pil = PIL_Image.fromarray((im * 255).astype(np.uint8))
        elif self._depth_conv_version == 2:
            im_depth_scaled[np.isnan(im_depth_scaled)] = max_depth_real
            im_depth_scaled[im_depth_scaled > max_depth_real] = max_depth_real
            im = im_depth_scaled / max_depth_real
            im_pil = PIL_Image.fromarray((im * 255).astype(np.uint8))

        im_pil = im_pil.resize((224, 224), PIL_Image.BILINEAR)
        im_data = np.array(im_pil)
        return im_data, time_stamp
        
    @property
    def point_cloud(self):
        if self._points_msg is None:
            return None
        points, transform = transform_point_cloud(self._listener, self._points_msg, '/world')
        return points
        #pts = self._listener.transformPointCloud2("/base_link", self._points_msg)
        #gen = pc2.read_points(pts, skip_nans=True, field_names=("x", "y", "z"))
        #return gen

        

