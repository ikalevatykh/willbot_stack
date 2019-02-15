import numpy as np

from transforms3d import euler, quaternions


class Frame():
    '''Simple abstraction for 3d transformation'''

    def __init__(self, pos, orn):
        '''Constructor

        Arguments:
            pos {list} -- position xyz
            orn {list} -- orientation: quat wxyz or euler xyz
        '''

        self.p = pos
        if orn is None:
            self.q = [1, 0, 0, 0]
        elif len(orn) == 3:
            self.q = euler.euler2quat(*orn)
        else:
            self.q = orn

    def __mul__(self, other):
        '''Frame multiplication

        Arguments:
            other {Frame} -- transformation

        Returns:
            Frame -- combined transformation
        '''

        p = np.add(self.p, quaternions.rotate_vector(other.p, self.q))
        q = quaternions.qmult(self.q, other.q)
        return Frame(p, q)

    def lin_diff(self, other):
        '''Compute linear distance btw frames

        Arguments:
            other {Frame} -- target frame

        Returns:
            np.array -- distance vector xyz
        '''

        dist = np.subtract(other.p, self.p)
        return dist

    def rot_diff(self, other):
        '''Compute rotation btw frames

        Arguments:
            other {Frame} -- target frame

        Returns:
            np.array, float -- axis and angle
        '''

        dq = quaternions.qmult(other.q, quaternions.qinverse(self.q))
        axis, angle = quaternions.quat2axangle(dq)
        return axis, angle


class TrapVelocityProfile():
    '''Trapezoid velocity profile'''

    def __init__(self, t_acc, *dist_vel_pairs):
        '''Constructor

        Arguments:
            dist_vel_pairs {list} -- distance and max velocity pairs

        Keyword Arguments:
            t_acc {float} -- time to accelerate and decelerate (default: {1.0})
        '''

        t_max = [np.linalg.norm(d / v) for d, v in dist_vel_pairs]
        t_dec = np.max(t_max)
        t_acc = np.min([t_acc, t_dec])
        t_end = t_dec + t_acc

        self._t_acc = t_acc  # time to end accelerate
        self._t_dec = t_dec  # time to start decelerate
        self._t_end = t_end  # time to stop
        self._vels = np.array([d / t_dec for d, _ in dist_vel_pairs])

    @property
    def time_length(self):
        return self._t_end

    def __call__(self, t):
        '''Call

        Arguments:
            t {[type]} -- time at what get profile value

        Returns:
            float -- profile level at time t (0..1)
        '''

        if t > self._t_end:
            return self._vels * 0.0
        if t < self._t_acc:
            return self._vels * (t / self._t_acc)
        if t > self._t_dec:
            return self._vels * (1.0 - (t - self._t_dec) / self._t_acc)
        return self._vels


class RectVelocityProfile():
    '''Rectangular velocity profile'''

    def __init__(self, *dist_vel_pairs):
        '''Constructor

        Arguments:
            dist_vel_pairs {list} -- distance and max velocity pairs
        '''

        t_end = [np.linalg.norm(d / v) for d, v in dist_vel_pairs]
        self._t_end = t_end  # time to stop

    @property
    def time_length(self):
        return self._t_end

    def __call__(self, t):
        '''Call

        Arguments:
            t {[type]} -- time at what get profile value

        Returns:
            float -- profile level at time t (0..1)
        '''

        if t > self._t_end:
            return 0.0
        return 1.0
