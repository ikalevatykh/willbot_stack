import math
import numpy as np
import rospy
import moveit_commander
import dynamic_reconfigure.client


class _ParrallelGraspKinematics(object):
    def __init__(self):
        self._zero_width = 0.07
        self._zero_angle = 0.52
        self._finger_len = 0.095

    def angle_to_width(self, angle):
        return self._zero_width + math.sin(self._zero_angle - angle) * self._finger_len * 2

    def width_to_angle(self, width):
        width = np.clip(width, 0.0, 0.16)
        return self._zero_angle + math.asin((self._zero_width - width) / self._finger_len / 2)


class RobotiqHand(object):
    def __init__(self, group_name="gripper"):
        self._open_position = 0.05
        self._close_position = 0.95

        self._gripper = moveit_commander.MoveGroupCommander(group_name)
        self._kinematics = _ParrallelGraspKinematics()
        self._client = dynamic_reconfigure.client.Client(group_name, timeout=5)
        self._config = self._client.get_configuration(timeout=5)

        links = ['hand_palm']
        for f in ['1', '2', 'middle']:
            for l in ['0', '1', '2', '3']:
                links.append('hand_finger_{}_link_{}'.format(f, l))
        self._links = links

    @property
    def links(self):
        return self._links

    @property
    def mode(self):
        return self._config['mode']

    @mode.setter
    def mode(self, value):
        self._config['mode'] = value
        self._client.update_configuration(self._config)

    @property
    def open_position(self):
        return self._open_position

    @open_position.setter
    def open_position(self, value):
        self._open_position = value

    @property
    def open_width(self):
        return self._kinematics.angle_to_width(self._open_position)

    @open_width.setter
    def open_width(self, value):
        self._open_position = self._kinematics.width_to_angle(value)

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
        return joints[1]

    @property
    def width(self):
        '''Distance between finger tips assuming parallel grasp.

        Returns:
            float -- width
        '''

        width = self._kinematics.angle_to_width(self.position)
        return width

    def move(self, position, velocity=0, force=0, wait=True):
        if velocity:
            self.target_velocity = velocity
        if force:
            self.target_force = force
        position = np.clip(position, 0.05, 6.0)
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

    def parallel_move(self, width, wait=True):
        '''Move fingers assuming parallel grasp.

        Arguments:
            width {float} -- a width between fingers

        Keyword Arguments:
            wait {bool} -- wait motion finished (default: {True})

        Returns:
            bool -- True if finger stopped at target width
        '''
        angle = self._kinematics.width_to_angle(width)
        return self.move(angle, wait)

    def stop(self):
        self._gripper.stop()
