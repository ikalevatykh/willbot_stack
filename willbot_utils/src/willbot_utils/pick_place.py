class TopPickPlacePlan(object):
    """ Helper for planning a simple pick and place action """

    def __init__(self, arm, hand, object_name):
        self._arm = arm
        self._hand = hand
        self._object_name = object_name
        self._pick_position = None
        self._place_position = None

    def pick(self, pick_pos, pick_orn=None):
        self._pick_position = (pick_pos, pick_orn)

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
        self._hand.open()
        plan = self._arm.cartesian()
        plan.move(upper_pos, pick_orn)
        plan.move(pick_pos, pick_orn)
        if not plan.execute():
            return False

        self._arm.attach_object(
            self._object_name, touch_links=self._hand.links)
        self._hand.close()

        plan = self._arm.cartesian()
        plan.move(upper_pos, pick_orn)
        return plan.execute()

    def _place(self, place_pos, place_orn=None):
        upper_pos = [place_pos[0], place_pos[1], place_pos[2] + 0.1]
        plan = self._arm.cartesian()
        plan.move(upper_pos, place_orn)
        plan.move(place_pos, place_orn)
        if not plan.execute():
            return False

        self._hand.open()
        self._arm.detach_object()

        plan = self._arm.cartesian()
        plan.move(upper_pos, place_orn)
        return plan.execute()
