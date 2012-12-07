#!/usr/bin/env python
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
with open("/opt/ros/groovy/share/catkin/cmake/templates/_setup_util.py", 'r') as fh:
    exec(fh.read())
