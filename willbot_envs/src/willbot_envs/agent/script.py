from .utils import Frame, TrapVelocityProfile


class ScriptAgent():
    '''Simple motion script.

    Arguments:
        rate {float} -- script update rate
        state {dict} -- environment state
    '''

    def __init__(self, rate, state):
        self._max_tool_vels = 0.05, 0.25
        self._max_grip_vel = 1.0
        self._dt = 1.0 / rate

        # get current tool position
        self._frame = Frame(
            state.get('tool_position'),
            state.get('tool_orientation', None))
        # accumulate action signals
        self._action = dict(
            linear_velocity=[0, 0, 0],
            angular_velocity=[0, 0, 0],
            grip_velocity=0.0)
        # script can have many stages: move to a, grasp, move to b, ...
        self._stages = []

    def move(self, pos=None, orn=None):
        '''Move arm tool to an absolute postion'''
        if pos is None:
            pos = self._frame.p
        if orn is None:
            orn = self._frame.q
        frame = Frame(pos, orn)
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

        if self._stages:
            done, act = \
                self._stages[0].get_action(state)
            if done:
                self._stages.pop(0)
            self._action.update(act)

        done = not self._stages
        return done, self._action


class LinearMove():
    def __init__(self, dt, frame0, frame1, max_vels):
        step = frame0.lin_diff(frame1)
        axis, angle = frame0.rot_diff(frame1)

        self._profile = TrapVelocityProfile(
            1.0, (step, max_vels[0]), (axis*angle, max_vels[1]))

        self._dt = dt
        self._t = 0.0

    def get_action(self, state):
        self._t += self._dt
        v, w = self._profile(self._t)

        done = self._t > self._profile.time_length
        act = dict(linear_velocity=v, angular_velocity=w)
        return done, act


class GripperMove():
    def __init__(self, operaton, max_vel):
        if operaton == 'open':
            self._v = max_vel
        else:
            self._v = -max_vel

    def get_action(self, state):
        if 'object_grasped' in state:
            done = state['object_grasped']
        else:
            done = state['grip_velocity'] == 0
        act = dict(grip_velocity=self._v)
        return done, act
