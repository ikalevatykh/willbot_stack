import numpy as np
import rospy
import PyKDL as kdl
import tf_conversions.posemath as pm


class CartesianPlan(object):
    """ Helper for cartesian path planning """

    def __init__(self, arm):
        self._arm = arm
        self._frames = [pm.fromMsg(arm.get_current_pose().pose), ]

    def move(self, pos=None, orn=None):
        """ Move arm tool to an absolute postion """
        p, q = _from_pos_orn(pos, orn)
        p = p or self._frames[-1].p
        q = q or self._frames[-1].M
        frame = kdl.Frame(q, p)
        self._frames.append(frame)

    def step_base(self, pos=(0, 0, 0), orn=(0, 0, 0)):
        """ Shift in base frame """
        p, q = _from_pos_orn(pos, orn)
        frame = kdl.Frame(p, q) * self._frames[-1]
        self._frames.append(frame)

    def step_tool(self, pos=(0, 0, 0), orn=(0, 0, 0)):
        """ Shift in tool frame """
        p, q = _from_pos_orn(pos, orn)
        frame = self._frames[-1] * kdl.Frame(p, q)
        self._frames.append(frame)

    def aproach(self, distance):
        """ Approach movement in the tool's z-axis direction """
        self.step_tool(pos=(0, 0, distance))

    def retreat(self, distance):
        """ Retreat movement in the tool's z-axis direction """
        self.step_tool(pos=(0, 0, -distance))

    def execute(self, wait=True, plan_only=False):
        """ Plan and execute cartesian path thru specified waypoints """
        waypoints = [pm.toMsg(f) for f in self._frames]
        (plan, frac) = self._arm.compute_cartesian_path(
            waypoints, eef_step=0.005, jump_threshold=5.0)
        if frac < 1.0:
            return False
        if plan_only:
            return True

        t = rospy.Duration()
        points = []
        for p in plan.joint_trajectory.points:
            if p.time_from_start != t:
                points.append(p)
                t = p.time_from_start
        plan.joint_trajectory.points = points

        if not self._arm.execute(plan, wait):
            current = self._arm.get_current_joint_values
            desired = plan.joint_trajectory.points[-1].positions
            if not np.allclose(current, desired, atol=0.01):
                return False

        return True


def _from_pos_orn(pos, orn):
    if pos is not None:
        pos = kdl.Vector(*pos)

    if orn is not None:
        if len(orn) == 4:
            orn = kdl.Rotation.Quaternion(*orn)
        else:
            orn = kdl.Rotation.RPY(*orn)

    return pos, orn
