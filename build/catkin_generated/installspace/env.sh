#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/dave/ros/clam/install', type 'exit' to leave"
  . "/home/dave/ros/clam/install/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/dave/ros/clam/install'"
else
  . "/home/dave/ros/clam/install/setup.sh"
  exec "$@"
fi
