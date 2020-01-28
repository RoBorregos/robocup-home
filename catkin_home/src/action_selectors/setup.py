## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# https://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html
# https://answers.ros.org/question/283140/how-to-import-python-modules-to-my-node/
# https://roboticsbackend.com/ros-import-python-module-from-another-package/

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    # Because how is structured the directories and to preserve the
    # package name in the imports, we have to make this trick that
    # doesn't seem to hurt.
    packages=['action_selectors'],
    package_dir={'': '../'}
)

setup(**setup_args)
