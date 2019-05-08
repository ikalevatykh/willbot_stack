import PyKDL as kdl
import numpy as np
import rospy
import tf_conversions.posemath as pm
from pykdl_utils.kdl_kinematics import KDLKinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
from willbot_utils.kinematics import UR5Kinematics, _homogenous


class ControllerException(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class VelocityController(object):
    def __init__(self, group, workspace):
        self._group = group
        self._workspace = workspace

        base_link = group.get_planning_frame()
        end_link = group.get_end_effector_link()
        robot = URDF.from_parameter_server()

        self._kin = UR5Kinematics()
        self._kdl_kin = KDLKinematics(robot, base_link, end_link)
        self._kin.set_configuration('right up forward')
        self._joint_names = self._kdl_kin.get_joint_names()

        cmd_topic = '/arm/pos_based_pos_traj_controller/command'
        self._publisher = rospy.Publisher(cmd_topic, JointTrajectory, queue_size=10)
        self.update_current()

    def update_current(self):
        self._q = self._group.get_current_joint_values()
        pos, orn = self._kin.forward(self._q)
        self._x = pm.fromMatrix(_homogenous(pos, orn))

    def send_joint_position(self, q, dt):
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = q
        point.time_from_start = rospy.Duration(dt)
        traj.points.append(point)
        self._publisher.publish(traj)

    def step(self, dt, lin_vel=(0, 0, 0), rot_vel=(0, 0, 0), check=False, base_time=None):
        q0 = self._q
        X0 = self._x

        dist = np.linalg.norm(lin_vel)
        if dist > 0:
            lin_step = kdl.Vector(*(np.array(lin_vel) * dt))
        else:
            lin_step = kdl.Vector()

        angle = np.linalg.norm(rot_vel)
        if angle > 0:
            axis = np.array(rot_vel) / angle
            axis_vec = kdl.Vector(*axis)
            axis_vec = X0.M.Inverse() * axis_vec
            # axis_vec = kdl.Vector(0, 0, axis_vec.z())
            rot_step = kdl.Rotation.Rot(axis_vec, angle * dt)
        else:
            rot_step = kdl.Rotation()

        base_tf = kdl.Frame(kdl.Rotation(), lin_step)
        tool_tf = kdl.Frame(rot_step, kdl.Vector())

        X1 = base_tf * X0 * tool_tf
        X2 = base_tf * X1 * tool_tf

        p = np.array([X2.p.x(), X2.p.y(), X2.p.z()])
        if np.any(p < self._workspace[0, :]-0.01) or np.any(p > self._workspace[1, :]+0.01):
            raise ControllerException(
               'Target outside workspace bounds: {} / {}'.format(p, self._workspace))
            return

        R, p = X1.M.GetQuaternion(), X1.p
        R = [R[3], R[0], R[1], R[2]]
        orn, pos = R, [p.x(), p.y(), p.z()]
        q1 = self._kin.inverse(pos, orn, q_init=q0)
        if q1 is None:
            raise ControllerException('IK solution not found')

        R, p = X2.M.GetQuaternion(), X2.p
        R = [R[3], R[0], R[1], R[2]]
        orn, pos = R, [p.x(), p.y(), p.z()]
        q2 = self._kin.inverse(pos, orn, q_init=q1)
        if q2 is None:
            raise ControllerException('IK solution not found')

        jerk = np.max(np.abs(np.subtract(q2, q0)))
        if jerk > 0.2:
            raise ControllerException('IK solution not continuous')

        if check:
            points = [pm.toMsg(w) for w in [X1, X2]]
            (_, frac) = self._group.compute_cartesian_path(points, 0.01, 5.0)
            if frac < 1.0:
                raise ControllerException('Moveit planner failed')

        self._q = q1
        self._x = X1

        traj = JointTrajectory()
        if base_time is not None:
            traj.header.stamp = base_time
        traj.joint_names = self._joint_names
        # target position
        point = JointTrajectoryPoint()
        point.positions = q1
        point.time_from_start = rospy.Duration(dt)
        traj.points.append(point)
        point = JointTrajectoryPoint()
        point.positions = q2
        point.time_from_start = rospy.Duration(dt * 2)
        traj.points.append(point)
        self._publisher.publish(traj)

    def stop(self):
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        self._publisher.publish(traj)

        rospy.sleep(0.5)
        # self._x = pm.fromMsg(self._group.get_current_pose().pose)
        # self._q = self._group.get_current_joint_values()
        self.update_current()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
