import json
import numpy as np
import sys


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def loads(s):
    return json.loads(s)


def dumps(obj):
    return json.dumps(obj, cls=NumpyEncoder)


def load_env(class_name, reload_module=False):
    '''Loads environment class by entry point name.

    Implementation from gym.

    Arguments:
        class_name {str} -- some.module:some_class [extra1,extra2]

    Keyword Arguments:
        reload_module {bool} -- reload modified env code each time (default: {False})

    Returns:
        class -- environment class
    '''
    import pkg_resources  # takes ~400ms to load, so we import it lazily
    if not ':' in class_name:
        class_name = 'willbot_envs.envs:' + class_name
    entry_point = pkg_resources.EntryPoint.parse('x={}'.format(class_name))

    if reload_module and entry_point.module_name in sys.modules:
        module = sys.modules[entry_point.module_name]
        # this check helps if the class imported from a sub module
        attrib = getattr(module, entry_point.attrs[0])
        if attrib.__module__ is not None:
            module = sys.modules[attrib.__module__]
        import imp  # assume python 2.x
        imp.reload(module)
        result = getattr(module, entry_point.attrs[0])
        return result

    result = entry_point.resolve()
    return result
