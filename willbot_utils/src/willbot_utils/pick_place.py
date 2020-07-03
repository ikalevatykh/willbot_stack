import numpy as np
import rospy


class TopPickPlacePlan(object):
    """ Helper for planning a simple pick and place action """

    def __init__(self, arm, hand, target_object=None):
        self._arm = arm
        self._hand = hand
        self._object = target_object
        self._pick_width = 0
        self._pick_open_width = 0
        self._place_open_width = 0
        self._grasp_tolerance = 0.04
        self._pick_position = None
        self._place_position = None

    def pick(self, pick_pos, pick_orn=None, pick_width=0, open_width=0):
        self._pick_position = (pick_pos, pick_orn)
        self._pick_width = pick_width
        self._pick_open_width = open_width
        self._place_open_width = open_width

    def place(self, place_pos, place_orn=None, open_width=0):
        self._place_position = (place_pos, place_orn)
        if open_width:
            self._place_open_width = open_width

    def execute(self):
        if self._pick_position is not None:
            if not self._pick(*self._pick_position):
                return False
        if self._place_position is not None:
            if not self._place(*self._place_position):
                return False
        return True

    def _pick(self, pick_pos, pick_orn=None):
        upper_pos = np.add(pick_pos, [0, 0, 0.1])
        grasp_pos = np.add(pick_pos, [0, 0, 0.0])

        if not self._hand.open(width=self._pick_open_width, wait=True):
            rospy.logerr('hand open failed')
            # return False

        if not self._arm.joint_move(upper_pos, pick_orn):
            rospy.logerr('joint move failed')
            return False

        plan = self._arm.cartesian()
        # plan.move(upper_pos, pick_orn)
        plan.move(grasp_pos, pick_orn)
        if not plan.execute():
            rospy.logerr('cartesian path failed')
            return False

        if self._object is not None:
            self._object.attach(
                link=self._arm.get_end_effector_link(),
                touch_links=self._hand.links)
            rospy.sleep(1.0)

        grasped = self._hand.parallel_move(width=self._pick_width, wait=True)
        # if not grasped and self._pick_width:
        #     err = self._hand.width - self._pick_width
        #     if np.abs(err) > self._grasp_tolerance:
        #         print(err, self._grasp_tolerance)
        #         return False

        plan = self._arm.cartesian()
        plan.move(upper_pos, pick_orn)
        return plan.execute()

    def _place(self, place_pos, place_orn=None):
        upper_pos = np.add(place_pos, [0, 0, 0.1])
        release_pos = np.add(place_pos, [0, 0, 0.001])

        if not self._arm.joint_move(upper_pos, place_orn):
            rospy.logerr('joint move failed')
            return False

        plan = self._arm.cartesian()
        # plan.move(upper_pos, place_orn)
        plan.move(release_pos, place_orn)
        if not plan.execute():
            rospy.logerr('cartesian path failed')
            return False

        if not self._hand.open(width=self._place_open_width, wait=True):
            rospy.logerr('hand open failed')
            # return False

        if self._object is not None:
            self._object.detach()
            self._object.shift((0, 0, -0.002))

        plan = self._arm.cartesian()
        plan.move(upper_pos, place_orn)
        return plan.execute()
