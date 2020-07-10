import threading

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from tf2_msgs.msg import TFMessage
from transforms3d.quaternions import mat2quat


class TransformBroadcaster(threading.Thread):

    def __init__(self, parent_frame_id, child_frame_ids):
        self._parent_frame_id = parent_frame_id
        self._child_frame_id = child_frame_ids
        self._pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
        self._lock = threading.Lock()
        self._transform = None
        self._stop_flag = False
        super(TransformBroadcaster, self).__init__()
        self.start()

    def publish(self, pos, quat=None):
        t = TransformStamped()
        t.header.frame_id = self._parent_frame_id
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self._child_frame_id

        if len(pos) == 4:  # homogenous matrix
            t.transform.translation = Vector3(*pos[:3, 3])
            w, x, y, z = mat2quat(pos[:3, :3])
            t.transform.rotation = Quaternion(x, y, z, w)
        else:
            t.transform.translation = Vector3(*pos)
            if quat is not None:
                t.transform.rotation = Quaternion(*quat)
            else:
                t.transform.rotation.w = 1.0

        with self._lock:
            self._transform = t

    def run(self):
        while not self._stop_flag:
            rospy.sleep(0.01)
            with self._lock:
                if self._transform is None:
                    continue
                tfm = TFMessage([self._transform, ])
            self._pub_tf.publish(tfm)

    def stop(self):
        self._stop_flag = True
        self.join()
