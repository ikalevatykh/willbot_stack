import rospy

from cv_bridge import CvBridge
import sensor_msgs
from sensor_msgs.msg import Image, PointCloud2
from PIL import Image as PIL_Image

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import tf
import rosbag

from convert_functions import transform_point_cloud
import ros_numpy
import pcl


class KinectBag():
    def __init__(self, bag_file, depth=False, rgb=False, point_cloud=False):        
        if depth:    
            image_topic = rospy.get_param('depth_registered_topic',
                                          '/camera/depth_registered/image')

            for topic, msg, t in rosbag.Bag(bag_file).read_messages(topics=[image_topic]):
                self._depth_msg = msg
                break
                
        if point_cloud:    
            points_topic = '/camera/depth_registered/points'

            for topic, msg, t in rosbag.Bag(bag_file).read_messages(topics=[points_topic]):
                self._points_msg = msg
                print(dir(msg))

                #msg.message.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
                break      

        self._bridge = CvBridge()  
        self._listener = tf.TransformListener()

    @property
    def depth_image(self):
        if self._depth_msg is None:
            return None
        depth_image = self._bridge.imgmsg_to_cv2(self._depth_msg, 'passthrough')
        return depth_image
        
    @property
    def depth_uint8(self):
        depth_image = self.depth_image
        if depth_image is None:
            return None    
        im_depth_scaled = depth_image.astype(np.float32)
        max_depth_real = 1.8
        
        im_depth_scaled[np.isnan(im_depth_scaled)] = 0   
        im_depth_scaled[im_depth_scaled > max_depth_real] = 0
        im = im_depth_scaled / max_depth_real
        #im_pil = PIL_Image.fromarray((im / max_depth_real * 255).astype(np.uint8))
        im_pil = PIL_Image.fromarray((im * 255).astype(np.uint8))
        im_pil = im_pil.resize((224, 224), PIL_Image.BILINEAR)
        return np.array(im_pil)
        
    @property
    def point_cloud(self):
        if self._points_msg is None:
            return None
        gen = pc2.read_points(self._points_msg, field_names = 'xyz', skip_nans=True)
        return gen

        #self._points_msg.header.stamp = rospy.Time.now()
        #points, transform = transform_point_cloud(self._listener, self._points_msg, '/world')
        #return points
        # pc = ros_numpy.numpify(self._points_msg)
        # points=np.zeros((pc.shape[0],3))
        # points[:,0]=pc['x']
        # points[:,1]=pc['y']
        # points[:,2]=pc['z']
        # p = pcl.PointCloud(np.array(points, dtype=np.float32))
        
        