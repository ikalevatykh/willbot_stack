import json
import numpy as np


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def loads(s):
    return json.loads(s)


def dumps(obj):
    return json.dumps(obj, cls=NumpyEncoder)


def load_env(class_name):
    """ Loads environment by class name. 

    Implementation from gym.
    """
    import pkg_resources  # takes ~400ms to load, so we import it lazily
    if not ':' in class_name:
        class_name = 'willbot_envs.envs:' + class_name
    entry_point = pkg_resources.EntryPoint.parse('x={}'.format(class_name))
    result = entry_point.resolve()
    return result
