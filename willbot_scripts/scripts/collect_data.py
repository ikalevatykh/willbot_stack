#!/usr/bin/env python
from __future__ import print_function

import sys

import moveit_commander
import numpy as np
import rospy

from agent_client import AgentClient
from hand import RobotiqHand
from kinect import Kinect
from planner import CartesianPlanner
#from remote_agent import RemoteAgent
from scene import StandardScene
from record import Recorder


class AgentControllerNode(object):
    def __init__(self):
        print('Setup...')

        print('Connecting arm...')
        arm = moveit_commander.MoveGroupCommander("manipulator")
        arm.set_planner_id("RRTConnectkConfigDefault")
        arm.set_max_velocity_scaling_factor(0.5)

        x0, x1 = 0.25, 0.65
        y0, y1 = -0.2, 0.2
        z0, z1 = 0.050, 0.250
        workspace = np.array([
            [x0, y0, z0], [x1, y1, z1]
        ])
        arm.set_workspace(workspace.flatten())
        self._workspace = workspace
        self._arm = arm
        print('OK')

        print('Connecting hand...')
        hand = RobotiqHand()
        hand.mode = 1
        self._hand = hand
        print('OK')

        print('Preparing scene...')
        scene = StandardScene()
        self._scene = scene

        self._object_name = None
        self._object_attached = False
        
        print('Preparing recorder...')
        dataset_path = rospy.get_param('~dataset_dir')
        self._recorder = Recorder(dataset_path)

        print('Ready!')

    def run(self):
        arm = self._arm
        hand = self._hand
        ws = self._workspace
        x0, x1 = 0.3, 0.65
        y0, y1 = -0.2, 0.2

        plan = CartesianPlanner(arm)
        cube_x = 0.28
        cube_y = 0.0


        succeed = 0
        executed = 0

        seed = rospy.get_param('~seed', 0)
        count = rospy.get_param('~count', 1000)

        print('Start count={}, seed={}'.format(count, seed))

        for seed in range(seed, seed + count):
            print('Experiment {}'.format(seed))
            try:
		    if not hand.is_object_held:
			self.pick_cube(plan, cube_x, cube_y)

		    # Place cube
		    np.random.seed(seed)
		    i, j = np.random.random_sample(2)
		    x = x0 + i * (x1 - x0)
		    y = y0 + j * (y1 - y0)
		    self.place_cube(plan, x, y)
		    cube_x = x
		    cube_y = y 

		    self.go_to_start(plan)
		    
		    with self._recorder.record('{}.bag'.format(seed)):
			# Pick cube
			self.pick_cube(plan, cube_x, cube_y)

            except Exception as e:
               print(e)

    def go_to_start(self, arm):
        self._hand.open(wait=True)
        #arm.step(pos=(0.0, 0.0, 0.175))
        arm.move(pos=(0.28, 0.0, 0.225), orn=(-0.7, 0.7, 0.0, 0.0))
        arm.execute(wait=True)

    def pick_cube(self, arm, cube_x, cube_y):
        arm.move(pos=(cube_x, cube_y, 0.2))
        arm.execute(wait=True)

        hand = self._hand
        while not hand.is_object_held:
            hand.open(wait=True)
            arm.move(pos=(cube_x, cube_y, 0.05))
            arm.execute(wait=True)
            rospy.sleep(2)
            hand.close(wait=True)

	self.attach_box()
        arm.move(pos=(cube_x, cube_y, 0.2))
        arm.execute(wait=True)
 
    def place_cube(self, arm, cube_x, cube_y, cube_z=0.052):
        print(' place cube {}:{}'.format(cube_x, cube_y))
        arm.move(pos=(cube_x, cube_y, 0.2))
        arm.move(pos=(cube_x, cube_y, cube_z))
        arm.execute(wait=True)
        self._hand.open(wait=True)
        self.detach_box()
        arm.move(pos=(cube_x, cube_y, 0.2))
        #arm.move(pos=(0.28, 0.0, 0.225))
        arm.execute(wait=True)

    def attach_box(self):
        if self._object_name is not None and not self._object_attached:
            arm = self._arm
            links = ['hand_palm']
            for f in ['1', '2', 'middle']:
                for l in ['0', '1', '2', '3']:
                    links.append('hand_finger_{}_link_{}'.format(f, l))
            print(links)
            arm.attach_object(self._object_name, 'hand_palm', links)
            self._object_attached = True

    def detach_box(self):
        if self._object_name is not None and self._object_attached:
            arm = self._arm
            arm.detach_object(self._object_name)
            self._object_attached = False


def main():
    rospy.myargv(argv=sys.argv)
    rospy.init_node('agent_controller', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    try:
        ctrl = AgentControllerNode()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print('error: ', e)
        raise e
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == '__main__':
    main()
