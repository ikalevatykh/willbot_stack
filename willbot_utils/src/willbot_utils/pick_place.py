import rospy


class TopPickPlacePlan(object):
    """ Helper for planning a simple pick and place action """

    def __init__(self, arm, hand, target_object=None):
        self._arm = arm
        self._hand = hand
        self._object = target_object
        self._object_width = 0
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
        upper_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + 0.1]

        if not self._hand.open():
            return False

        plan = self._arm.cartesian()
        plan.move(upper_pos, pick_orn)
        plan.move(pick_pos, pick_orn)
        if not plan.execute():
            return False

        if self._object is not None:
            self._object.attach(
                link=self._arm.get_end_effector_link(),
                touch_links=self._hand.links)

        if not self._hand.parallel_grasp(self._object_width):
            if self._object_width > 0:
                return False

        plan = self._arm.cartesian()
        plan.move(upper_pos, pick_orn)
        return plan.execute()

    def _place(self, place_pos, place_orn=None):
        upper_pos = [place_pos[0], place_pos[1], place_pos[2] + 0.1]
        above_pos = [place_pos[0], place_pos[1], place_pos[2] + 0.002]
        plan = self._arm.cartesian()
        plan.move(upper_pos, place_orn)
        plan.move(above_pos, place_orn)
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
