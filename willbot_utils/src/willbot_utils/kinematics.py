import ur5_kinematics
from pyquaternion import Quaternion
import numpy as np


class UR5Kinematics():
    """
    These kinematics find the tranfrom from the base link to the end effector.
    """
    def __init__(self):
        def H(link_name):
            # TODO: Read URDF correctly instead of doing this (measured in pybullet)
            if link_name == 'base_link':
                T = [[1., 0., 0., 0.],
                     [0., 1., 0., 0.],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1.]]
            elif link_name == 'ee_link':
                T = [[4.10301202e-17,  1.00000000e+00, -8.74227797e-08,  8.17250013e-01],
                     [1.00000000e+00, -1.44810311e-17,  5.23771700e-25,  1.91450000e-01],
                     [5.23771700e-25, -8.74227797e-08, -1.00000000e+00, -5.49103925e-03],
                     [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
            elif link_name == 'tool':
                T = [[-7.07106818e-01, -7.07106745e-01,  3.22577968e-08,  8.17250013e-01],
                     [-3.22577968e-08,  7.78772161e-08,  1.00000000e+00,  4.16449994e-01],
                     [-7.07106745e-01,  7.07106818e-01, -7.78772161e-08, -5.49102947e-03],
                     [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
            return T

        Tw0 = H('base_link')
        T0w = np.linalg.inv(Tw0)
        Tw6 = H('ee_link')
        Twt = H('tool')
        Tt6 = np.dot(np.linalg.inv(Twt), Tw6)
        T6t = np.linalg.inv(Tt6)

        self.Tw0 = Tw0
        self.T0w = T0w
        self.Tt6 = Tt6
        self.T6t = T6t
        self.lower = [-6.2831855, -3.1415927, -3.1415927, -3.1415927, -3.1415927, -3.1415927]
        self.upper = [0., 3.1415927, 3.1415927, 3.1415927, 3.1415927, 3.1415927]
        self.kin_indicies = np.array([0, 0, 0])

    def forward(self, q):
        """Find the tranfrom from the base link to the tool link.

        Arguments:
            q {list(6)} -- joint angles.

        Returns:
            (list(3), list(4)) -- position and orientation of the tool link.
        """
        q = np.float64(q)
        T06 = np.zeros((4, 4), dtype=np.float64)
        ur5_kinematics.forward(q, T06)
        T0t = np.dot(T06, self.T6t)
        Twt = np.dot(self.Tw0, T0t)
        return _pos_orn(Twt)

    def set_configuration(self, desc):
        """Specify target kinematic configuration.

        Arguments:
            desc {str} -- configuraton description like 'right up forward'.
                            Posiible options: left/right shoulder,
                            up/down elbow, forward/backward wrist
        """

        config = dict(
            right=(0, 1), left=(0, -1),
            up=(1, 1), down=(1, -1),
            forward=(2, 1), backward=(2, -1))

        indicies = np.array([0, 0, 0])
        for name in desc.split(' '):
            assert name in config, 'Unknown kinematic index: {}'.format(name)
            i, val = config[name]
            indicies[i] = val

        self.kin_indicies = indicies

    def _get_configuration(self, q):
        # TODO: update for all cases
        right = q[0] < -np.pi / 2
        up = -np.pi <= q[1] < 0
        forward = -np.pi <= q[3] < 0.1
        return [0, 1 if up else -1, 1 if forward else -1]

    def inverse_all(self, pos, orn, q_init):
        """Find all posiible solutions for inverse kinematics.

        Arguments:
            pos {list(3)} -- target tool position.
            orn {list(4)} -- target tool orientation. wxyz

        Keyword Arguments:
            q6_des {float} -- An optional parameter which designates what the q6 value should take
                in case of an infinite solution on that joint. (default: {0.0})

        Returns:
            list(N,6) -- solutions.
        """

        Twt = _homogenous(pos, orn)
        T0t = np.dot(self.T0w, Twt)
        # TODO: apply transform in case base != world
        T06 = np.float64(np.dot(T0t, self.Tt6))

        q6_des = q_init[5] if q_init is not None else 0.0
        q_sol = np.zeros((8, 6), dtype=np.float64)
        n = ur5_kinematics.inverse(T06, q_sol, q6_des)
        if n == 0:
            return []

        q_sol = q_sol[:n]

        q_sol[q_sol > self.upper] -= 2 * np.pi
        q_sol[q_sol < self.lower] += 2 * np.pi

        mask = np.any((self.lower <= q_sol) & (
            q_sol <= self.upper), axis=1)

        # TODO: replace by kin indexes
        mask &= q_sol[:, 1] < 0
        mask &= q_sol[:, 0] <= - np.pi / 2
        mask &= q_sol[:, 0] >= - 3 * np.pi / 2
        q_sol = q_sol[mask]

        mask = np.array([all(self._get_configuration(q) * self.kin_indicies >= 0) for q in q_sol])
        q_sol = q_sol[mask]

        if np.any(q_sol) and q_init is not None:
            weights = [1, 1, 1, 2, 1, 0.5]
            dist = np.apply_along_axis(np.linalg.norm, 1, (q_sol-q_init) * weights)
            q_sol = q_sol[dist.argsort()]

        return q_sol

    def inverse(self, pos, orn, q_init=None):
        """Find inverse kin solution nearest to q_init.

        Arguments:
            pos {list(3)} -- target tool position.
            orn {list(4)} -- target tool orientation.

        Keyword Arguments:
            q_init {list(6)} -- initial solution (default: zeros(6))

        Returns:
            list(6) / None -- joint positions or None if solution not found
        """

        if q_init is None:
            q_init = np.zeros(6, dtype=np.float64)

        q_sol = self.inverse_all(pos, orn, q_init)

        if np.any(q_sol):
            return q_sol[0]

    # def forward_vel(self, q, dq, dt):
    #     pos0, orn0 = self.forward(q)
    #     pos1, orn1 = self.forward(q + np.array(dq) * dt)

    #     diff = _quat(orn1) * _quat(orn0).inverse
    #     axis, angle = diff.get_axis(undefined=[0, 0, 0]), diff.angle

    #     v = np.subtract(pos1, pos0) / dt
    #     w = np.array(axis) * angle / dt
    #     return v, w

    # def inverse_vel(self, q, v, w, dt):
    #     pos0, orn0 = self.forward(q)
    #     pos1 = pos0 + v * dt
    #     orn1 = _orn(_quat([*(0.5 * w * dt), 1.0]) * _quat(orn0))

    #     q1 = self.inverse(pos1, orn1, q)
    #     if q1 is not None:
    #         dq = np.subtract(q1, q) / dt
    #         return dq
    #     return np.zeros(6)


def _quat(orn):
    # return Quaternion(orn[3], *orn[:3])
    return Quaternion(*orn)


def _orn(q):
    # Convention is wxyz on real and xyzw in sim
    return [q[0], q[1], q[2], q[3]]


def _homogenous(pos, orn):
    mat = _quat(orn).transformation_matrix
    mat[:3, 3] = pos
    return mat


def _pos_orn(mat):
    pos = mat[:3, 3]
    q = Quaternion(matrix=mat)
    return pos, _orn(q)
