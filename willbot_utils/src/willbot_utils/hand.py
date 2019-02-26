import math
import rospy
import moveit_commander
import dynamic_reconfigure.client


class RobotiqHand(object):
    def __init__(self, group_name="gripper"):
        self._open_position = 0.05
        self._close_position = 0.95

        self._gripper = moveit_commander.MoveGroupCommander(group_name)
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
    def is_object_held(self):
        return self.position < 0.6

    @property
    def width(self):
        '''Distance between finger tips assuming parallel grasp.

        Returns:
            float -- width
        '''

        angle = self.position
        return 0.09 - math.sin(angle - 0.52) * 2 * 0.1

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

    def parallel_grasp(self, width, wait=True):
        '''Grasp an object assuming parallel grasp.

        Arguments:
            width {float} -- a grasping object width

        Keyword Arguments:
            wait {bool} -- wait motion finished (default: {True})

        Returns:
            bool -- True if finger stopped at target width
        '''

        angle = 0.52 + math.asin((0.09 - width) / 2 / 0.1)
        return self.move(angle, wait)

    def stop(self):
        self._gripper.stop()
