import rospy

try:
    from pyassimp import pyassimp
    use_pyassimp = True
except:
    # In 16.04, pyassimp is busted
    # https://bugs.launchpad.net/ubuntu/+source/assimp/+bug/1589949
    use_pyassimp = False

from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane


class SceneObject(object):
    """Collision object on the planning scene."""

    def __init__(self, scene, collision_object):
        self._scene = scene
        self._co = collision_object

    @property
    def name(self):
        """Unique object name."""
        return self._co.id

    def attach(self, link, touch_links):
        """Attach the object to an arm link."""
        self._scene.attach_object(self.name, link, touch_links)

    def detach(self):
        """Detach the object."""
        self._scene.detach_object(self.name)

    def move(self, pos):
        """Move the object to a new position."""
        self._scene.move_object(self.name, pos)

    def shift(self, dpos):
        """Shift the object."""
        self._scene.shift_object(self.name, dpos)

    def remove(self):
        """Remove the object from the scene."""
        self._scene.remove_object(self.name)


class Scene(object):
    """Interface for the planning scene."""

    def __init__(self, planning_frame):
        self._planning_frame = planning_frame

        self._get_planning_scene = rospy.ServiceProxy(
            '/get_planning_scene', GetPlanningScene)
        self._get_planning_scene.wait_for_service(timeout=5.0)

        self._apply_planning_scene = rospy.ServiceProxy(
            '/apply_planning_scene', ApplyPlanningScene)
        self._apply_planning_scene.wait_for_service(timeout=5.0)

    def add_box(self, name, size, pos):
        """Add a box primitive to the planning scene.

        Args:
            @name (str): collision object name
            @size (x,y,z): object size
            @pos (x,y,z): position
        Returns:
            CollsionObject interface for the new object
        """
        co = self._make_box(name, size, pos)
        self._apply_scene_diff([co])
        return SceneObject(self, co)

    def add_mesh(self, name, path, scale, pos):
        """Add the mesh to the planning scene.

        Args:
            @name (str): collision object name
            @path (str): path to a mesh model
            @scale (x,y,z): scale
            @pos (x,y,z): position
        Returns:
            CollsionObject interface for the new object
        """
        if type(scale) not in [list, tuple]:
            scale = (scale, ) * 3
        co = self._make_mesh(name, path, scale, pos)
        self._apply_scene_diff([co])
        return SceneObject(self, co)

    def attach_object(self, name, link='', touch_links=[]):
        """Given the name of an object existing in the planning scene,
        attach it to a robot link.

        Args:
            @name (str): collision object name
            @link (str): link to attach to
            @touch_links (list): links that can touch an object
        """
        aoc = AttachedCollisionObject()
        aoc.object.operation = CollisionObject.ADD
        aoc.object.id = name
        aoc.touch_links = touch_links
        aoc.link_name = link
        self._apply_scene_diff([aoc])

    def detach_object(self, name='', link=''):
        """Given the name of a link, detach the object(s) from that link.

        Args:
            @name (str): object name (detach all attached objects if avoided)
            @link (str): link to detach from
        """
        aoc = AttachedCollisionObject()
        aoc.object.operation = CollisionObject.REMOVE
        if name:
            aoc.object.id = name
        if link:
            aoc.link_name = link
        self._apply_scene_diff([aoc])

    def move_object(self, name, pos):
        """Move a collision object to a new position.
        Args:
            @name (str): object name
            @pos (x,y,z): new position
        """
        pose = self._make_pose(pos)
        co = self._move_command(name, pose)
        self._apply_scene_diff([co])

    def shift_object(self, name, dpos):
        """Shift a collision object to a new position.
        Args:
            @name (str): object name
            @spos (x,y,z): shift
        """
        scene = self._get_scene()
        obj = next((obj for obj in scene.world.collision_objects
                    if obj.id == name), None)
        if obj is not None:
            for pose in obj.primitive_poses:
                pose.position.x += dpos[0]
                pose.position.y += dpos[1]
                pose.position.z += dpos[2]
            for pose in obj.mesh_poses:
                pose.position.x += dpos[0]
                pose.position.y += dpos[1]
                pose.position.z += dpos[2]
            co = self._move_command(
                name,
                prim_poses=obj.primitive_poses,
                mesh_poses=obj.mesh_poses)
            self._apply_scene_diff([co])

    def remove_object(self, name):
        """Remove a collision object from the planning scene.
        Args:
            @name (str): object name
        """
        co = self._remove_command(name)
        self._apply_scene_diff([co])

    def clear(self):
        """Clear the planning scene."""
        # self.detach_object()
        # self._scene.clear()
        scene = self._get_scene()
        for obj in scene.robot_state.attached_collision_objects:
            self.detach_object(obj.object.id)
        for obj in scene.world.collision_objects:
            self.remove_object(obj.id)

    def _make_box(self, name, size, pos):
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = size
        return self._add_command(
            name, self._make_pose(pos), solid=sp)

    def _make_mesh(self, name, path, scale, pos):
        if not use_pyassimp:
            raise RuntimeError('pyassimp is broken, cannot load meshes')

        scene = pyassimp.load(path)
        if not scene.meshes:
            pyassimp.release(scene)
            raise RuntimeError('Unable to load mesh "{}"'.format(path))
        mesh = Mesh()
        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if len(face.indices) == 3:
                triangle.vertex_indices = list(face.indices)
            mesh.triangles.append(triangle)
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        pyassimp.release(scene)

        return self._add_command(
            name, self._make_pose(pos), mesh=mesh)

    def _add_command(self, name, pose, solid=None, mesh=None):
        co = CollisionObject()
        co.header.stamp = rospy.Time.now()
        co.header.frame_id = self._planning_frame
        co.id = name
        if solid is not None:
            co.primitives.append(solid)
            co.primitive_poses.append(pose)
        if mesh is not None:
            co.meshes.append(mesh)
            co.mesh_poses.append(pose)
        co.operation = CollisionObject.ADD
        return co

    def _move_command(self, name, prim_poses=[], mesh_poses=[]):
        co = CollisionObject()
        co.header.stamp = rospy.Time.now()
        co.header.frame_id = self._planning_frame
        co.id = name
        co.primitive_poses = prim_poses
        co.mesh_poses = mesh_poses
        co.operation = CollisionObject.MOVE
        return co

    def _remove_command(self, name):
        co = CollisionObject()
        co.header.stamp = rospy.Time.now()
        co.header.frame_id = self._planning_frame
        co.id = name
        co.operation = CollisionObject.REMOVE
        return co

    def _make_color(self, name, rgba):
        color = ObjectColor()
        color.id = name
        color.color.r = rgba[0]
        color.color.g = rgba[1]
        color.color.b = rgba[2]
        color.color.a = rgba[3] if len(rgba) > 3 else 1.0
        return color

    def _make_pose(self, pos):
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = 1.0
        return pose

    def _apply_scene_diff(self, items):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        for item in items:
            if isinstance(item, CollisionObject):
                scene.world.collision_objects.append(item)
            elif isinstance(item, AttachedCollisionObject):
                scene.robot_state.attached_collision_objects.append(item)
            elif isinstance(item, ObjectColor):
                scene.object_colors.append(item)

        resp = self._apply_planning_scene(scene)
        if not resp.success:
            raise RuntimeError("Could not apply planning scene diff.")

    def _get_scene(self):
        req = PlanningSceneComponents()
        req.components = sum([
            PlanningSceneComponents.WORLD_OBJECT_NAMES,
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS])
        try:
            resp = self._get_planning_scene(req)
            return resp.scene
        except rospy.ServiceException as e:
            raise RuntimeError("Failed to get initial planning scene.")


class StandardScene(Scene):
    """Interface for the planning scene with a standart initialization."""

    def __init__(self, arm):
        super(StandardScene, self).__init__(arm.get_planning_frame())

        setup = rospy.get_param('/willbot_setup')
        rospy.logdebug('Scene setup: {}', setup)

        self.clear()

        if setup == 'paris':
            self._apply_scene_diff([
                self._make_box(
                    "wall", (1.2, 0.04, 0.7), (0.36, -0.42, 0.25)),
                self._make_box(
                    "table", (1.2, 0.8, 0.1), (0.36, 0.0, -0.05)),
            ])

        elif setup == 'grenoble':
            self._apply_scene_diff([
                self._make_box(
                    "wall_right", (1.05, 0.01, 0.55), (0.15, 0.48, 0.275)),
                self._make_box(
                    "wall_left", (1.05, 0.01, 0.55), (0.15, -0.48, 0.275)),
                self._make_box(
                    "wall_back", (0.01, 1.0, 0.62), (-0.34, 0.0, 0.31)),
                self._make_box(
                    "pole_left", (0.04, 0.04, 1.2), (-0.21, -0.4, 0.55)),
                self._make_box(
                    "table", (1.0, 1.0, 0.1), (0.16, 0.0, -0.05)),
                self._make_color(
                    "table", (1, 1, 1))
            ])

        arm.set_support_surface_name('table')


def main():
    import sys
    import moveit_commander

    rospy.init_node('scene', anonymous=True)
    try:
        arm = moveit_commander.MoveGroupCommander('manipulator')
        scene = StandardScene(arm)
        rospy.sleep(5)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
