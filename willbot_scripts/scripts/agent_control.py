#!/usr/bin/env python
from __future__ import print_function

import os
import sys
import pickle
import smtplib

import moveit_commander
import numpy as np
import rospy

from willbot_utils.hand import RobotiqHand
from willbot_utils.kinect import Kinect
from willbot_utils.planner import CartesianPlanner
from willbot_utils.scene import StandardScene
from willbot_utils.velocity_controller import VelocityController, ControllerException

from agent_client import AgentClient


class AgentControllerNode(object):
    def __init__(self):
        print('Setup...')
        #agent = RemoteAgent(0)

        self._notify = lambda msg: None
        email = rospy.get_param('~email', '')
        if email:
            server = smtplib.SMTP('smtp.gmail.com:587')
            server.starttls()
            server.login('willow.robot', 'Willow_2018')
            self._notify = lambda msg: server.sendmail("willow.robot@gmail.com", email, '\n' + msg)

        print('Connecting arm...')
        arm = moveit_commander.MoveGroupCommander("manipulator")
        arm.set_planner_id("RRTConnectkConfigDefault")
        arm.set_max_velocity_scaling_factor(0.5)

        x0, x1 = 0.25, 0.65
        y0, y1 = -0.2, 0.2
        z0, z1 = 0.050, 0.250
        workspace = np.array([[x0, y0, z0], [x1, y1, z1]])
        arm.set_workspace(workspace.flatten())
        self._workspace = workspace
        self._arm = arm
        print('OK')

        print('Connecting hand...')
        hand = RobotiqHand()
        hand.mode = 1
        #hand.open(wait=True)
        self._hand = hand
        print('OK')

        print('Connecting kinect...')
        depth_conv = rospy.get_param('~depth_conversion', 0)
        cam = Kinect(depth=True, depth_conv=depth_conv)
        self._cam = cam
        print('OK')

        print('Preparing scene...')
        scene = StandardScene()
        self._scene = scene

        if not hand.is_object_held:
            hand.open(wait=True)
            arm.detach_object('box')
            scene.add_box(
                'box', (0.06, 0.06, 0.06), (0.28, 0.00, 0.041),
                color=(1, 0, 0))
            self._object_name = 'box'
            self._object_attached = False
        else:
            self._object_name = 'box'
            self._object_attached = True

        env_name = rospy.get_param('~env', 'UR5-PickEnv')
        agent_type = rospy.get_param('~agent', 'network')
        settings = dict(env_name=env_name, agent_type=agent_type)
        if agent_type == 'network':
            settings.update(
                arch=rospy.get_param('~arch', 'resnet18'),
                epoch=rospy.get_param('~epoch'),
                net_path=rospy.get_param('~net_path'),
                timesteps=rospy.get_param('~timesteps', 5),
                max_steps=rospy.get_param('~max_steps', 200),
                dim_action=rospy.get_param('~dim_action', 4),
                steps_action=rospy.get_param('~steps_action', 4),
            )
        elif agent_type == 'dataset':
            settings.update(
                dataset_path=rospy.get_param('~path'),
                lmdb=rospy.get_param('~lmdb', True),
            )
        self._settings = settings

        print('Ready!')

    def run(self):
        self._step = 0
        try:
            self._run()
        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            self._notify(
                'Agent control failed on step {} / {} with error: {}'.format(
                    self._step, self._count, e.message))
            raise e

    def _run(self):
        arm = self._arm
        hand = self._hand
        cam = self._cam
        ws = self._workspace
        x0, x1 = 0.3, 0.65
        y0, y1 = -0.2, 0.2

        plan = CartesianPlanner(arm)
        cube_x = 0.28
        cube_y = 0.0

        succeed = 0
        executed = 0

        auto_repeat = rospy.get_param('~auto', False)

        experiment_path = rospy.get_param('~experiment_path', '')
        seed = rospy.get_param('~seed', 1000)
        count = rospy.get_param('~count', 1)
        self._count = count

        plan.movez(0.15)
        plan.execute(wait=True)
        print('Start auto={}, count={}, seed={}'.format(
            auto_repeat, count, seed))

        for i, seed in enumerate(range(seed, seed + count)):
            print('Experiment {} / {}, seed {}'.format(i + 1, count, seed))
            self._step = i + 1

            if not auto_repeat:
                self.go_to_start(plan)
            else:
                # Pick cube
                if not hand.is_object_held:
                    self.pick_cube(plan, cube_x, cube_y)

                # Place cube
                np.random.seed(seed)
                i, j = np.random.random_sample(2)
                cube_x = x0 + i * (x1 - x0)
                cube_y = y0 + j * (y1 - y0)
                self.place_cube(plan, cube_x, cube_y)
                self.go_to_start(plan)

                #plan.move(pos=(0.28, 0.0, 0.15))
                #plan.move(pos=(0.28, 0.0, 0.225))
                #plan.execute(wait=True)
                #print('At start point')
                #rospy.sleep(4)

            grasped_pos = None

            # Run agent
            print('Agent running ...')
            vel_scale = rospy.get_param('~velocity_scale', 1.0)
            agent_port = rospy.get_param('~agent_port', 9999)

            agent = AgentClient('localhost', agent_port)
            settings = self._settings
            settings.update(seed=seed)
            agent.send_settings(settings)

            states = []
            actions = []
            with VelocityController(arm, ws) as ctrl:
                success = False
                timing = []
                timing_rate = []
                t0 = rospy.get_rostime()
                rate = rospy.Rate(10)
                for i in range(100):
                    depth, depth_t = cam.depth_uint8
                    obs = {
                        'depth': depth,
                        'timestamp': depth_t.to_sec()
                    }  #cam.depth_image}
                    act = agent.get_action(obs)
                    ta = rospy.get_rostime()
                    #print(act)
                    if act is None:
                        break
                    states.append(obs)
                    actions.append(act)

                    v = act['linear_velocity']
                    v = np.array(v) * vel_scale
                    try:
                        ctrl.step(v, 0.1)  #, base_time=depth_t)
                    except ControllerException as e:
                        print(e)
                        #break

                    g = act['grip_velocity']
                    if g < -0.01:
                        hand.close(wait=False)
                    else:
                        hand.open(wait=False)

                    if hand.is_object_held:
                        if not self._object_attached:
                            self.attach_box()
                            pos = arm.get_current_pose().pose.position
                            grasped_pos = (pos.x, pos.y, pos.z)
                    else:
                        self.detach_box()

                    z = arm.get_current_pose().pose.position.z
                    if hand.is_object_held and z > 0.1:
                        success = True
                        break

                    rate.sleep()

                    t = rospy.get_rostime()
                    d = t - t0
                    da = t - ta
                    timing.append(d.to_sec())
                    timing_rate.append(da.to_sec() / d.to_sec())
                    t0 = t

            hand.stop()
            plan.reset()

            print('Position end: ', arm.get_current_pose().pose)

            print('Timing: {:.2f} / min {:.2f} / max {:.2f}'.format(
                np.mean(timing), np.min(timing), np.max(timing)))
            #print(timing)
            print('Timing rate: {:.2f} / min {:.2f} / max {:.2f}'.format(
                np.mean(timing_rate), np.min(timing_rate),
                np.max(timing_rate)))

            print('Success: {}'.format(success))

            executed += 1
            if success:
                succeed += 1
            print('Rate {} / {}'.format(succeed, executed))

            if grasped_pos is not None:
                self._scene.remove_world_object('box')
                self.place_cube(plan, *grasped_pos)
                self._scene.add_box(
                    'box', (0.06, 0.06, 0.06), (cube_x, cube_y, 0.041),
                    color=(1, 0, 0))
            else:
                self._scene.remove_world_object('box')
                rospy.sleep(1)
                hand.open(wait=True)
                #plan.step_tool(pos=(0, 0, -0.1))
                plan.movez(0.15)
                plan.execute(wait=True)
                self._scene.add_box(
                    'box', (0.06, 0.06, 0.06), (cube_x, cube_y, 0.041),
                    color=(1, 0, 0))

            if experiment_path:
                with open(
                        os.path.join(experiment_path, '{}_{}.pkl'.format(
                            seed, success)), 'wb') as output:
                    pickle.dump(
                        dict(
                            seed=seed,
                            success=success,
                            states=states,
                            actions=actions), output)

        self._notify('Agent control done, success rate {} / {}'.format(
            succeed, executed))

    def go_to_start(self, arm):
        self._hand.open(wait=True)
        #arm.step(pos=(0.0, 0.0, 0.175))
        arm.move(pos=(0.28, 0.0, 0.225), orn=(-0.7, 0.7, 0.0, 0.0))
        arm.execute(wait=True)

    def pick_cube(self, arm, cube_x, cube_y):
        #print('Hey, give me a cube!')
        print(' picking a cube {}:{}'.format(cube_x, cube_y))
        arm.move(pos=(cube_x, cube_y, 0.15))
        arm.execute(wait=True)

        hand = self._hand
        while not hand.is_object_held:
            hand.open(wait=True)
            arm.move(pos=(cube_x, cube_y, 0.05), orn=(-1.0, 0, 0, 0))
            arm.execute(wait=True)
            rospy.sleep(2)
            hand.close(wait=True)
            hand.open(wait=True)
            arm.move(pos=(cube_x, cube_y, 0.15), orn=(-1.0, 0, 0, 0))
            arm.execute(wait=True)
            arm.move(pos=(cube_x, cube_y, 0.05), orn=(-0.7, 0.7, 0.0, 0.0))
            arm.execute(wait=True)
            rospy.sleep(2)
            hand.close(wait=True)
            '''
            hand.open(wait=True)
            arm.move(pos=(cube_x, cube_y, 0.05), orn=(-0.7, 0.7, 0.0, 0.0))
            arm.execute(wait=True)
            rospy.sleep(2)
            hand.close(wait=True)
            hand.open(wait=True)
            arm.movez(0.15)
            arm.execute(wait=True)
            arm.move(pos=(cube_x, cube_y, 0.05), orn=(-1.0, 0, 0, 0))
            arm.execute(wait=True)
            rospy.sleep(2)
            hand.close(wait=True)
            '''

        self.attach_box()
        arm.move(pos=(cube_x, cube_y, 0.15))
        arm.execute(wait=True)
        #print('Merci!')
        print(' picked')

    def place_cube(self, arm, cube_x, cube_y, cube_z=0.052):
        print(' placing a cube {}:{}'.format(cube_x, cube_y))
        arm.move(pos=(cube_x, cube_y, 0.15))
        arm.move(pos=(cube_x, cube_y, cube_z))
        arm.execute(wait=True)
        self._hand.open(wait=True)
        self.detach_box()
        arm.move(pos=(cube_x, cube_y, 0.15))
        #arm.move(pos=(0.28, 0.0, 0.225))
        arm.execute(wait=True)
        print(' placed')

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
        print('Error: ', e.message)
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == '__main__':
    main()
