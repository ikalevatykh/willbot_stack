#!/usr/bin/env python2
from __future__ import print_function

import rospy

import numpy as np
import pickle
import tf2_ros
import tf2_kdl

if __name__ == '__main__':
    rospy.init_node('vive_calibration')

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    frames = []

    while not rospy.is_shutdown():
        try:
            key = raw_input('Press Enter to read next frame ')
            if key == 'q':
                break

            trans_true = buffer.lookup_transform(
                'tool_tracker', 'world', rospy.Time())
            trans_vive = buffer.lookup_transform(
                'world_vive', 'tracker_LHR_0DBB917B', rospy.Time())

            frame_true = tf2_kdl.transform_to_kdl(trans_true)
            frame_vive = tf2_kdl.transform_to_kdl(trans_vive)

            world_vive = frame_vive * frame_true
            print('Current:', world_vive.p, world_vive.M.GetRPY())

            frames.append(world_vive)
            xyz = [(f.p.x(), f.p.y(), f.p.z()) for f in frames]
            rpy = [f.M.GetRPY() for f in frames]
            print('Mean:', np.mean(xyz, axis=0).tolist(), np.mean(rpy, axis=0).tolist())
            print('Std:', np.std(xyz, axis=0).tolist(), np.std(rpy, axis=0).tolist())

            print()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Failed')
            continue

    with open('/home/robot/vive_calibration.pkl', 'wb') as f:
        pickle.dump(frames, f)