#!/usr/bin/env python

import rospy
import rosbag

from sensor_msgs.msg import PointCloud2


def main():   
    pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        points_topic = '/camera/depth_registered/points'
        cloud_msg = rospy.wait_for_message(points_topic, PointCloud2)
        pub.publish(cloud_msg)        
        rate.sleep()
        
	
if __name__ == '__main__':
    try:
        rospy.init_node('recorder', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass




