import rospy
import moveit_commander

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor


class StandardScene(object):
    def __init__(self):
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        self._planning_frame = robot.get_planning_frame()
        self._scene = scene
        self._apply_service = rospy.ServiceProxy(
            'apply_planning_scene', ApplyPlanningScene)
        self._colors = {}

        self.clear()
        rospy.rostime.wallsleep(0.1)

        setup = rospy.get_param('/willbot_setup')
        rospy.logdebug('Scene setup: {}', setup)

        if setup == 'paris':
            self.add_box("table",
                         (1.2, 0.8, 0.1), (0.36, 0.0, -0.05))
            self.add_box("rubber",
                         (1.2, 0.625, 0.01), (0.36, 0.0, 0.005), color=(1, 1, 1))
            self.add_box("wall",
                         (1.2, 0.04, 0.7), (0.36, -0.42, 0.25))
            rospy.rostime.wallsleep(0.1)

            robot.manipulator.set_support_surface_name('rubber')

        if setup == 'grenoble':
            self.add_box("table",
                         (1.0, 1.0, 0.1), (0.16, 0.0, -0.05))
            self.add_box("wall_right",
                         (1.05, 0.01, 0.55), (0.15, 0.48, 0.275))
            self.add_box("wall_left",
                         (1.05, 0.01, 0.55), (0.15, -0.48, 0.275))
            self.add_box("wall_back",
                         (1.0, 0.01, 0.62), (-0.34, 0.0, 0.31))
            self.add_box("pole_left",
                         (0.04, 0.04, 1.2), (-0.21, -0.4, 0.55))
            rospy.rostime.wallsleep(0.1)

            robot.manipulator.set_support_surface_name('table')

    def set_color(self, name, rgba):
        color = ObjectColor()
        color.id = name
        color.color.r = rgba[0]
        color.color.g = rgba[1]
        color.color.b = rgba[2]
        color.color.a = rgba[3] if len(rgba) > 3 else 1.0
        self._colors[name] = color

    def send_colors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self._colors.values():
            p.object_colors.append(color)
        resp = self._apply_service.call(p)

    def add_box(self, name, size, pos, orn=(0, 0, 0, 1), color=(0, 1, 0, 1)):
        p = PoseStamped()
        p.header.frame_id = self._planning_frame
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        p.pose.orientation.x = orn[0]
        p.pose.orientation.y = orn[1]
        p.pose.orientation.z = orn[2]
        p.pose.orientation.w = orn[3]
        self._scene.add_box(name, p, size)
        self.set_color(name, color)
        self.send_colors()

    def remove_world_object(self, name):
        self._scene.remove_world_object(name)

    def clear(self):
        self._scene.remove_attached_object('tool')
        self._scene.remove_world_object()


def main():
    import sys

    rospy.init_node('scene', anonymous=True)
    try:
        scene = StandardScene()
        rospy.sleep(5)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
