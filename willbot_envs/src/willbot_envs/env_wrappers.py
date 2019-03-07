import gym
from gym import ObservationWrapper

import rospy
from sensor_msgs.msg import JointState

from willbot_utils.camera import ImageListener


class CameraObserver(ObservationWrapper):
    def __init__(self, env, key, topic, encoding=None, timeout=rospy.Duration(0.5)):
        super(CameraObserver, self).__init__(env)
        self._key = key
        self._stream = ImageListener(topic)
        self._encoding = encoding
        self._timeout = timeout

    def observation(self, observation):
        stamp, img = self._stream.latest(self._encoding, with_time=True)
        if rospy.Time.now() - stamp > self._timeout:
            raise RuntimeError(
                'Image from camera "{}" expired'.format(self._key))
        observation[self._key] = img
        return observation


class JointStateObserver(ObservationWrapper):
    ArmNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    GrpName = 'hand_finger_middle_joint_1'

    def __init__(self, env):
        super(JointStateObserver, self).__init__(env)
        self._msg = None

        def msg_cb(msg):
            self._msg = msg

        self._sub = rospy.Subscriber(
            '/joint_states', JointState, msg_cb, queue_size=1)

    def observation(self, observation):
        msg = self._msg
        if msg is not None:
            # TODO: use msg.name to get joint indicies for ArmNames
            observation['joint_position'] = msg.position[:6]
            observation['joint_velocity'] = msg.velocity[:6]
            observation['grip_position'] = msg.position[6]
            observation['grip_velocity'] = msg.velocity[6]
        return observation

# TODO: ToolStateObserver
