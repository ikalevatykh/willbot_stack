import numpy as np
import rospy


class TopPickPlacePlan(object):
    """ Helper for planning a simple pick and place action """

    def __init__(self, arm, hand, target_object=None):
        self._arm = arm
        self._hand = hand
        self._object = target_object
        self._object_width = 0
        self._grasp_tolerance = 0.01
        self._pick_position = None
        self._place_position = None

    def pick(self, pick_pos, pick_orn=None, object_width=0):
        self._pick_position = (pick_pos, pick_orn)
        self._object_width = object_width

    def place(self, place_pos, place_orn=None):
        self._place_position = (place_pos, place_orn)

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

        if not self._hand.open():
            return False

        plan = self._arm.cartesian()
        plan.move(upper_pos, pick_orn)
        plan.move(grasp_pos, pick_orn)
        if not plan.execute():
            return False

        if self._object is not None:
            self._object.attach(
                link=self._arm.get_end_effector_link(),
                touch_links=self._hand.links)

        grasped = self._hand.parallel_grasp(self._object_width)
        if not grasped and self._object_width:
            err = self._hand.width - self._object_width
            if np.abs(err) > self._grasp_tolerance:
                return False

        plan = self._arm.cartesian()
        plan.move(upper_pos, pick_orn)
        return plan.execute()

    def _place(self, place_pos, place_orn=None):
        upper_pos = np.add(place_pos, [0, 0, 0.1])
        release_pos = np.add(place_pos, [0, 0, 0.002])

        plan = self._arm.cartesian()
        plan.move(upper_pos, place_orn)
        plan.move(release_pos, place_orn)
        if not plan.execute():
            return False

        if not self._hand.open():
            return False

        if self._object is not None:
            self._object.detach()
            self._object.shift((0, 0, -0.002))

        plan = self._arm.cartesian()
        plan.move(upper_pos, place_orn)
        return plan.execute()
