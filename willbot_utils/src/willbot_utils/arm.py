import rospy
import moveit_commander

from willbot_utils.cartesian import CartesianPlan
from willbot_utils.pick_place import TopPickPlacePlan


class UR5(moveit_commander.MoveGroupCommander):
    """A thin wrapper for MoveIt move group for willbot arm control."""

    def __init__(self, group_name="manipulator"):
        super(UR5, self).__init__(group_name)
        # set default path planner
        self.set_planner_id("RRTConnectkConfigDefault")

    def cartesian(self):
        """Returns interface for cartesian path planning."""
        return CartesianPlan(self)

    def pick_place(self, hand, object_name):
        """Returns interface for simple pick&place planning."""        
        return TopPickPlacePlan(self, hand, object_name)