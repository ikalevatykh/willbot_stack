from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['willbot_utils'],
    scripts=[],
    package_dir={'': 'src'}
)

setup(**d)