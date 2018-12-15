import rospy
import PyKDL
import tf_conversions.posemath as pm
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics


class PlannerException(Exception):
    def __init__(self, message):
        super(PlannerException, self).__init__(message)


class CartesianPlanner(object):
    KIN_CACHE = None

    def __init__(self, group):
        self._group = group
        self._points = []
        self.reset()
        self._max_tool_velocity = (0.05, 0.25)

    def reset(self):
        w1 = pm.fromMsg(self._group.get_current_pose().pose)
        print(self._group.get_current_pose().pose)
        self._points = [w1, ]

    def execute(self, wait=False):
        plan = self.compute()
        if len(plan.joint_trajectory.points) < 2:
            return
        success = self._group.execute(plan, wait)
        if not success:
            raise PlannerException('Cannot execute trajectory')

    def compute(self):
        w1 = pm.fromMsg(self._group.get_current_pose().pose)
        self._points[0] = w1
        points = [pm.toMsg(w) for w in self._points]
        (plan, frac) = self._group.compute_cartesian_path(points, 0.005, 5)
        if frac == 1.0:
            self._points = [self._points[-1], ]
            # Filter waypoints strictly increasing in time
            '''
            t = rospy.Duration()
            for p in plan.joint_trajectory.points:
                if p.time_from_start <= t:
                    p.time_from_start = t + rospy.Duration(nsecs=100)
                    t = p.time_from_start
            '''
            t = rospy.Duration()
            points = []
            for p in plan.joint_trajectory.points:
                if p.time_from_start != t:
                    points.append(p)
                    t = p.time_from_start
            plan.joint_trajectory.points = points
            return plan
        self._points = [self._points[0], ]
        raise PlannerException('Cannot plan cartesian path')

    def move(self, pos=None, orn=None):
        w = self._points[-1]
        w = self._update(w, pos, orn)
        self._points.append(w)

    def movez(self, z):
        w = self._points[-1]
        w = self._update(w, (w.p.x(), w.p.y(), z), None)
        self._points.append(w)

    def step_base(self, pos=None, orn=None):
        dw = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector())
        dw = self._update(dw, pos, orn)
        w = dw * self._points[-1]
        self._points.append(w)

    def step_tool(self, pos=None, orn=None):
        dw = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector())
        dw = self._update(dw, pos, orn)
        w = self._points[-1] * dw

        print(w.M)

        self._points.append(w)

    def move_origin(self, origin, pos=None, orn=None):
        x, y, z = origin

        w = self._points[-1]
        w = self._update(w, pos, orn)

        dw = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(-x, -y, -z))
        w = w * dw

        print(w.p)

        self._points.append(w)

    def rotate_about(self, origin, axis, angle):
        x, y, z = origin
        axis = PyKDL.Vector(*axis)

        dw1 = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(x, y, z))
        dw2 = PyKDL.Frame(PyKDL.Rotation.Rot(axis, angle), PyKDL.Vector())
        dw3 = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(-x, -y, -z))
        dw = dw1 * dw2 * dw3

        w = self._points[-1] * dw
        self._points.append(w)

    def duplicate(self):
        self._points.append(self._points[-1])

    def _update(self, frame, pos, orn):
        p = frame.p
        q = frame.M
        if pos is not None:
            p = PyKDL.Vector(*pos)
        if orn is not None:
            if len(orn) == 4:
                q = PyKDL.Rotation.Quaternion(*orn)
            else:
                q = PyKDL.Rotation.RPY(*orn)
        return PyKDL.Frame(q, p)

    def _compute_manual(self):
        if CartesianPlanner.KIN_CACHE is None:
            base_link = group.get_planning_frame()
            end_link = group.get_end_effector_link()
            robot = URDF.from_parameter_server()
            kin = KDLKinematics(robot, base_link, end_link)
            CartesianPlanner.KIN_CACHE = kin
        self._kin = CartesianPlanner.KIN_CACHE
        self._joint_names = self._kin.get_joint_names()

    def _interpolate(self, f0, f1, dt=0.1, t_acc=1.0):
        max_v, max_w = self._max_tool_velocity
        dist = np.subtract(f0.p, f1.p)

        t_dec = np.linalg.norm(dist) / max_v
        t_acc = np.min([t_acc, t_dec])
        t_end = t_dec + t_acc
        v_max = dist / t_dec

        for t in np.arange(0.0, t_end, dt) + dt:
            k = 1.0
            if t > t_end:
                k = 0.0
            elif t <= t_acc:
                k = t / t_acc
            elif t >= t_dec:
                k = 1 - (t - t_dec) / t_acc
            yield v_max * k
