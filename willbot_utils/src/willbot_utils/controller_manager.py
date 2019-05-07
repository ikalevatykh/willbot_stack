import os
import rospy

from controller_manager_msgs.srv import *


class ControllerManager():
    """ 
    Utility to simplify managing hw controllers.
    """

    def __init__(self, namespace='/', persistent=True):
        """Initializer
        
        Keyword Arguments:
            namespace {str} -- controller manager namespace (default: {'/'})
            persistent {bool} -- use persistent connection (default: {False})
        """
        self._list_srv = rospy.ServiceProxy(
            os.path.join(namespace, 'list_controllers'),
            ListControllers,
            persistent=persistent)
        self._load_srv = rospy.ServiceProxy(
            os.path.join(namespace, 'load_controller'),
            LoadController,
            persistent=persistent)
        self._unload_srv = rospy.ServiceProxy(
            os.path.join(namespace, 'unload_controller'),
            UnloadController,
            persistent=persistent)
        self._switch_srv = rospy.ServiceProxy(
            os.path.join(namespace, 'switch_controller'),
            SwitchController,
            persistent=persistent)

    def list_controllers(self):
        """ Returns a list of controller names/states/types of the
            controllers that are loaded inside the controller_manager.

        Returns:
            list -- list of controller states
        """
        res = self._list_srv.call()
        return res.controller

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

    def start_controller(self, name):
        """Starts specified controller.
        If controller not loaded it will be.
        If other running controller claimed the same resources it will be stopped.

        Arguments:
            name {str} -- controller name

        Returns:
            bool -- indicates if the controllers were started successfully or not
        """
        controllers = self.list_controllers()

        # do nothing if the controller already started, load if not loaded
        ctr = next((c for c in controllers if c.name == name), None)
        if ctr is None:
            if not self.load_controller(name):
                return False
            controllers = self.list_controllers()
        elif ctr.state == 'running':
            return True

        # find controllers claiming resources required
        # for the starting controller
        require_resources = ()
        claimed_resources = {}
        for c in controllers:
            resources = [r for cr in c.claimed_resources for r in cr.resources]
            if c.name == name:
                require_resources = set(resources)
            else:
                claimed_resources[c.name] = resources

        stop_controllers = [c for c, rs in claimed_resources.items()
                            for r in rs if r in require_resources]

        return self.switch_controllers(
            start_controllers=[name],
            stop_controllers=list(set(stop_controllers)))

    def stop_controller(self, name):
        """Stop controller

        Arguments:
            name {str} -- controller name

        Returns:
            bool -- indicates if the controllers were stopped successfully or not
        """
        return self.switch_controllers(stop_controllers=[name])
