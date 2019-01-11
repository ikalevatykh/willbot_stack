import rospy
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from recorder.srv import *


class Recorder(object):

    def __init__(self, path):
        self._path = Path(path)
        rospy.wait_for_service('record_command')
        self._service = rospy.ServiceProxy('record_command', Command)

    def record(self, bag_name):
        self._service(enable_record=True, bag_file_name=str(self._path / bag_name))
        return self

    def __enter__(self):
        pass

    def __exit__(self, a, b, c):
        self._service(enable_record=False)


