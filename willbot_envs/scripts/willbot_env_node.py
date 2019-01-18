#!/usr/bin/env python
from __future__ import print_function

import json
import numpy as np

import rospy
import actionlib
import moveit_commander
import message_filters
import PyKDL as kdl
import tf_conversions.posemath as pm

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image

from agent import AgentClient
from sampler import Sampler
from robotiq_hand import RobotiqHand, RobotiqHandMock
from kinect_camera import KinectCamera, KinectCameraMock
from simple_scene import Scene

import willbot_envs.msg


class EnvironmentNode(object):
    def __init__(self):
        rospy.init_node('willbot_env', anonymous=False)

        print('Setup...')

        print('Connecting arm ...')
        arm = moveit_commander.MoveGroupCommander("manipulator")
        arm.set_planner_id("RRTConnectkConfigDefault")
        # arm.set_max_velocity_scaling_factor(0.5)
        self._arm = arm

        workspace = np.array([[0.3, -0.20, 0.050], [0.7, 0.20, 0.250]])
        arm.set_workspace(workspace.flatten())
        self._workspace = workspace
        print('- Arm OK')

        print('Connecting hand ...')
        hand = RobotiqHand()
        hand.mode = 1
        hand.target_velocity = 5
        hand.target_effort = 5
        self._hand = hand
        print('- Hand OK')

        self._scene = Scene(arm)

        self._reset_server = actionlib.SimpleActionServer(
            'reset',  willbot_envs.msg.EnvResetAction,
            execute_cb=self.reset_cb, auto_start=False)
        self._reset_server.start()

        self._step_server = actionlib.SimpleActionServer(
            'step',  willbot_envs.msg.EnvStepAction,
            execute_cb=self.step_cb, auto_start=False)
        self._step_server.start()

        self._episode_id = 0
        print('Ready!')

    def reset_cb(self, goal):
        try:
            rospy.loginfo('reset')

            self._episode_id += 1
            seed = rospy.get_param('seed')
            self.seed(seed)
            observation = self.reset()

            result = willbot_envs.msg.EnvResetResult(
                episode_id=self._episode_id,
                observation=json.dumps(observation)
            )
            self._reset_server.set_succeeded(result)
        except Exception as e:
            self._reset_server.set_aborted(text=e.message)

    def step_cb(self, goal):
        try:
            rospy.loginfo('step %s', goal.action)

            if goal.episode_id != self._episode_id:
                raise RuntimeError(
                    'Wrong episode id, only one client can be connected')

            action = json.loads(goal.action)
            obs, reward, done, info = self.step(action)

            result = willbot_envs.msg.EnvStepResult(
                observation=json.dumps(obs),
                reward=reward,
                done=done,
                info=json.dumps(info)
            )
            self._reset_server.set_succeeded(result)
        except Exception as e:
            self._reset_server.set_aborted(text=e.message)

    def step(self, action):
        """Run one timestep of the environment's dynamics."""

        if 'tool_position' in action:
            pos = action.get('tool_position')
            self.arm_move([pos])

        # tool_position, tool_orientation
        # linear_velocity, angular_velocity
        # joint_position
        # joint_velocity

        raise NotImplementedError

    def reset(self):
        """Resets the state of the environment and returns an initial observation."""
        raise NotImplementedError

    def seed(self, seed):
        """Sets the seed for this env's random number generator(s)."""
        raise NotImplementedError      

    def arm_move(self, waypoints, wait=True):
        frame = pm.fromMsg(self._arm.get_current_pose().pose)
        _waypoints = [self._arm.get_current_pose().pose]
        for wp in waypoints:
            if len(wp) == 2:
                p = kdl.Vector(*wp[0])
                q = kdl.Rotation.Quaternion(*wp[1])
                frame = kdl.Frame(q, p)
            else:
                p = kdl.Vector(*wp)
                q = frame.M
            frame = kdl.Frame(q, p)
            _waypoints.append(pm.toMsg(frame))

        (plan, frac) = self._arm.compute_cartesian_path(_waypoints, 0.005, 5.0)
        if frac < 1.0:
            raise Exception()

        # waypoints has to strictly increasing in time
        t = rospy.Duration()
        points = []
        for p in plan.joint_trajectory.points:
            if p.time_from_start != t:
                points.append(p)
                t = p.time_from_start
        if len(points) < 2:
            return
        plan.joint_trajectory.points = points

        success = self._arm.execute(plan, wait)
        rospy.sleep(1)
        if not success:
            raise Exception()

    def hand_open(self, wait=True):
        self._hand.open(wait)

    def hand_close(self, wait=True):
        self._hand.close(wait)


def main():
    rospy.myargv(argv=sys.argv)
    try:
        node = EnvironmentNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
