import os
import rospy

from controller_manager_msgs.srv import *


class ControllerManager():
    """
    Utility to simplify managing hw controllers.
    """

    def __init__(self, group='', persistent=True):
        """Initializer

        Keyword Arguments:
            group {str} -- controller manager group (default: {''})
            persistent {bool} -- use persistent connection (default: {True})
        """
        self._group = group

        if group:
            namespace = '/{}/controller_manager/'.format(group)
        else:
            namespace = '/controller_manager/'

        self._list_srv = rospy.ServiceProxy(
            namespace + 'list_controllers', ListControllers, persistent=persistent)
        self._load_srv = rospy.ServiceProxy(
            namespace + 'load_controller', LoadController, persistent=persistent)
        self._unload_srv = rospy.ServiceProxy(
            namespace + 'unload_controller', UnloadController, persistent=persistent)
        self._switch_srv = rospy.ServiceProxy(
            namespace + 'switch_controller', SwitchController, persistent=persistent)

    @property
    def group(self):
        return self._group

    def list_controllers(self):
        """ Returns a list of controller names/states/types of the
            controllers that are loaded inside the controller_manager.

        Returns:
            dict -- dict of controller name,state pairs
        """
        res = self._list_srv.call()
        return {c.name: c for c in res.controller}

    def load_controller(self, name):
        """Loads a single controller inside controller_manager

        Arguments:
            name {str} -- controller name

        Returns:
            bool -- indicates if the controller was successfully
                    constructed and initialized or not
        """
        try:
            req = LoadControllerRequest(name=name)
            res = self._load_srv.call(req)
            return res.ok
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

    def unload_controller(self, name):
        """Unloads a single controller from controller_manager

        Arguments:
            name {str} -- controller name

        Returns:
            bool -- indicates if the controller was successfully unloaded or not
        """
        try:
            req = UnloadControllerRequest(name=name)
            res = self._unload_srv.call(req)
            return res.ok
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

    def switch_controllers(self, start_controllers=[], stop_controllers=[], strict=True):
        """Stops a number of controllers and start a number of controllers,
            all in one single timestep of the controller_manager control loop.

        Keyword Arguments:
            start_controllers {list} -- the list of controller names to start (default: {[]})
            stop_controllers {list} -- the list of controller names to stop (default: {[]})
            strict {bool} -- if False even when something goes wrong with on controller,
                the service will still try to start/stop the remaining controllers (default: {True})

        Returns:
            [type] -- indicates if the controllers were switched successfully or not
        """
        strictness = SwitchControllerRequest.BEST_EFFORT
        if strict:
            strictness = SwitchControllerRequest.STRICT
        try:
            req = SwitchControllerRequest(
                start_controllers=start_controllers,
                stop_controllers=stop_controllers,
                strictness=strictness)
            res = self._switch_srv.call(req)
            return res.ok
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

    def start_controller(self, start_name):
        """Starts specified controller.
        If controller not loaded it will be.
        If other running controller claimed the same resources it will be stopped.

        Arguments:
            start_name {str} -- controller name

        Returns:
            bool -- indicates if the controllers were started successfully or not
            list -- stopped controllers names
        """
        controllers = self.list_controllers()

        if start_name in controllers:
            # do nothing if the controller already started
            if controllers[start_name].state == 'running':
                return True, []
        else:
            # load controller if not loaded
            if not self.load_controller(start_name):
                return False, []
            # update list
            controllers = self.list_controllers()

        # find controllers claiming resources required
        # for the starting controller
        resources = {name: set(r for cr in c.claimed_resources for r in cr.resources)
                     for name, c in controllers.items()
                     if c.state == 'running' or name == start_name}
        start_resources = resources.pop(start_name)

        stop_names = [name for name, res in resources.items()
                      if start_resources.intersection(res)]

        started = self.switch_controllers(
            start_controllers=[start_name],
            stop_controllers=stop_names)

        return started, stop_names

    def stop_controller(self, name):
        """Stop controller

        Arguments:
            name {str} -- controller name

        Returns:
            bool -- indicates if the controllers were stopped successfully or not
        """
        return self.switch_controllers(stop_controllers=[name])


class Controller():
    """
    Utility to simplify managing hw controllers.
    """

    def __init__(self, controller_manager, name):
        if not isinstance(controller_manager, ControllerManager):
            namespace = controller_manager
            controller_manager = ControllerManager(namespace)
        self._cm = controller_manager
        self._name = name
        self._started = False
        self._restore = []

    @property
    def namespace(self):
        return '/{}/{}/'.format(self._cm.group, self._name)

    @property
    def started(self):
        return self._started

    def start(self):
        """Start controller."""
        started, restore = self._cm.start_controller(self._name)
        self._started = started
        self._restore = restore

    def stop(self):
        """Stop controller and restart stopped controllers."""
        if not self._started:
            return
        self._cm.switch_controllers(
            self._restore, [self._name])

    def __enter__(self):
        """Start at enter."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Stop at exit."""
        self.stop()
