import rospy
import moveit_commander

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

from willbot_utils.cartesian import CartesianPlan
from willbot_utils.pick_place import TopPickPlacePlan


class UR5(moveit_commander.MoveGroupCommander):
    """A thin wrapper for MoveIt move group for willbot arm control."""

    def __init__(self, group_name="manipulator", hand=None):
        super(UR5, self).__init__(group_name)
        # set default path planner
        self.set_planner_id("RRTConnectkConfigDefault")
        self._hand = hand

    @property
    def hand(self):
        """Gripper attached to the arm."""
        return self._hand

    def get_current_position(self):
        """Returns a list of 3 elements defining the [x, y, z] of the end-effector."""
        pose = self.get_current_pose()
        vec = pose.pose.position
        return [vec.x, vec.y, vec.z]

    def joint_move(self, pos, orn=None, wait=True):
        """Plan and execute motion in joint space."""
        if orn is None:
            orn = self.get_current_rpy()
        self.set_pose_target(list(pos) + list(orn))
        plan = self.plan()
        if not plan.joint_trajectory.points:
            return False
        return self.execute(plan, wait)

    def linear_move(self, pos=None, orn=None, wait=True):
        """Plan and execute motion in cartesian space."""
        plan = CartesianPlan(self)
        plan.move(pos, orn)
        return plan.execute(wait)

    def cartesian(self):
        """Returns interface for cartesian path planning."""
        return CartesianPlan(self)

    def pick_place(self, target_object):
        """Returns interface for simple pick&place planning."""
        return TopPickPlacePlan(self, self._hand, target_object)
