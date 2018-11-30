#!/usr/bin/env python
import rospy
import tf
import numpy as np
from pynput.keyboard import Key, KeyCode, Listener
import PyKDL as kdl
import tf_conversions.posemath as pm


position = [1.378, 0.033, 0.686]
orientation = [-2.036, 0.024, 1.578]
shift = False

def on_press(key):
    if key == Key.shift:
        shift = True
    print(type(key))
    if key == KeyCode(char='q'):
        print('YES')
        position[0] += 0.001
    if key == KeyCode(char='a'):
        position[0] -= 0.001
    if key == KeyCode(char='w'):
        position[1] += 0.001
    if key == KeyCode(char='s'):
        position[1] -= 0.001
    if key == KeyCode(char='e'):
        position[2] += 0.001
    if key == KeyCode(char='d'):
        position[2] -= 0.001
        
    if key == KeyCode(char='r'):
        orientation[0] += 0.001
    if key == KeyCode(char='f'):
        orientation[0] -= 0.001
    if key == KeyCode(char='t'):
        orientation[1] += 0.001
    if key == KeyCode(char='g'):
        orientation[1] -= 0.001
    if key == KeyCode(char='y'):
        orientation[2] += 0.001
    if key == KeyCode(char='h'):
        orientation[2] -= 0.001
    
    print(position)
    print(orientation)

def on_release(key):
    if key == Key.shift:
        shift = False

    if key == Key.esc:
        return False


def main():
    rospy.init_node('send_transform', anonymous=True)
    br = tf.TransformBroadcaster()
    
    with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
        
        rate = rospy.Rate(50)
        while listener.running:
            p = kdl.Vector(*position)
            q = kdl.Rotation.RPY(*orientation)
            f = kdl.Frame(q, p)
            
            p1 = kdl.Vector(-0.045, 0.000, 0.000)
            q1 = kdl.Rotation.RPY(1.571, -1.571, 0.000)
            f1 = kdl.Frame(q1, p1)           
        
            f2 = f * f1
            
            print(f2.p)
            print(f2.M.GetQuaternion())
            
            br.sendTransform(
                 f2.p,
                 f2.M.GetQuaternion(),
                 rospy.Time.now(),
                 'camera_link',
                 'world')

            rate.sleep()
        listener.join()
    

if __name__ == '__main__':
    main()

