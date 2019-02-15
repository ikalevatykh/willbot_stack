from functools import partial

from transforms3d import euler, quaternions

import numpy as np

from utils import Frame, TrapVelocityProfile


class ScriptAgent():
    '''Simple motion script.

    Arguments:
        state {dict} -- environment state
    '''

    def __init__(self, state):
        self._max_tool_vels = 0.1, 0.01
        self._max_grip_vel = 1.0

        # get current tool position
        self._frame = Frame(
            state.get('tool_position'),
            state.get('tool_orientation', None)
        )
        # script can have many stages: move to a, grasp, move to b, ...
        self._stages = []

    def move(self, pos=None, orn=None):
        '''Move arm tool to an absolute postion'''
        p = pos or self._frame.p
        q = orn or self._frame.q
        frame = Frame(p, q)
        self._stages.append(
            LinearMove(self._dt, self._frame, frame, self._max_tool_vels))
        self._frame = frame
        return self  # just for chain operations

    def step_base(self, pos=(0, 0, 0), orn=(0, 0, 0)):
        '''Shift in base frame'''
        frame = Frame(pos, orn) * self._frame
        return self.move(frame.p, frame.q)

    def step_tool(self, pos=(0, 0, 0), orn=(0, 0, 0)):
        '''Shift in tool frame'''
        frame = self._frame * Frame(pos, orn)
        return self.move(frame.p, frame.q)

    def aproach(self, distance):
        '''Approach movement in the tool's z-axis direction'''
        return self.step_tool(pos=(0, 0, distance))

    def retreat(self, distance):
        '''Retreat movement in the tool's z-axis direction'''
        return self.step_tool(pos=(0, 0, -distance))

    def grip(self, operation):
        '''Move gripper'''
        self._stages.append(
            GripperMove(operation, self._max_grip_vel))
        return self

    def grip_close(self):
        '''Close gripper'''
        return self.grip('close')

    def grip_open(self):
        '''Open gripper'''
        return self.grip('open')

    def get_action(self, state):
        '''Compute next script action

        Arguments:
            state {dict} -- an environment state

        Returns:
            bool, dict -- done flag and next action
        '''

        stage = self._stages[0]
        done, act = stage.get_action(state)
        if done:
            self._stages.pop(0)

        done = not self._stages
        act = dict(
            linear_velocity=[0, 0, 0],
            angular_velocity=[0, 0, 0],
            grip_velocity=0.0
        ).update(act)

        return done, act


class LinearMove():
    def __init__(self, dt, frame0, frame1, max_vels):
        step = frame0.lin_diff(frame1)
        axis, angle = frame0.rot_diff(frame1)

        self._profile = TrapVelocityProfile(
            1.0, (step, max_vels[0]), (angle*angle, max_vels[1]))

        self._dt = dt
        self._t = 0.0

    def get_action(self, state):
        self._t += self._dt
        v, w = self._profile(self._t)

        done = self._t > self._profile.time_length
        act = dict(linear_velocity=v, angular_velocity=w)
        return done, act


class GripperMove():
    def __init__(self, dt, operaton, max_vel):
        if operaton == 'open':
            self._v = max_vel
        else:
            self._v = -max_vel

    def get_action(self, t, state):
        done = state['grip_velocity'] == 0
        act = dict(grip_velocity=0.0 if done else self._v)
        return done, act
