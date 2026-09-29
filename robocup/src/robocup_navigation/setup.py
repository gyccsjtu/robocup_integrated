#!/usr/bin/env python3
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    # coordination is the Codex core (pure Python, schema v1).
    # coordination_adapter is WorkBuddy's ROS-side plumbing for it.
    # Sub-packages must be listed explicitly or `pip/py setup.py install`
    # (install mode) will not ship them.
    packages=[
        "robocup_navigation",
        "robocup_navigation.coordination",
        "robocup_navigation.coordination_adapter",
    ],
    package_dir={"": "src"},
))
