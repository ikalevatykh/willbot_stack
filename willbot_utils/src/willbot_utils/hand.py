import rospy
import moveit_commander
import dynamic_reconfigure.client


class RobotiqHand(object):
    def __init__(self):
        self._open_position = 0.22
        self._close_position = 0.6

        self._gripper = moveit_commander.MoveGroupCommander("gripper")
        self._client = dynamic_reconfigure.client.Client("gripper", timeout=5)
        self._config = self._client.get_configuration(timeout=5)

    @property
    def mode(self):
        return self._config['mode']

    @mode.setter
    def mode(self, value):
        self._config['mode'] = value
        self._client.update_configuration(self._config)

    @property
    def target_velocity(self):
        return self._config['velocity']

    @target_velocity.setter
    def target_velocity(self, value):
        self._config['velocity'] = value
        self._client.update_configuration(self._config)

    @property
    def target_effort(self):
        return self._config['force']

    @target_effort.setter
    def target_effort(self, value):
        self._config['force'] = value
        self._client.update_configuration(self._config)

    @property
    def position(self):
        joints = self._gripper.get_current_joint_values()
        return joints[0]

    @property
    def is_object_held(self):
        return self.position < 0.6

    def move(self, position, wait=True):
        self._gripper.set_start_state_to_current_state()
        self._gripper.set_joint_value_target({
            'hand_finger_middle_joint_1':
            position,
            'hand_finger_1_joint_1':
            position,
            'hand_finger_2_joint_1':
            position
        })
        return self._gripper.go(wait=wait)

    def open(self, wait=True):
        return self.move(self._open_position, wait)

    def close(self, wait=True):
        return self.move(self._close_position, wait)

    def stop(self):
        self._gripper.stop()
