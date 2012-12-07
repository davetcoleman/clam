#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/dave/ros/clam/devel', type 'exit' to leave"
  . "/home/dave/ros/clam/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/dave/ros/clam/devel'"
else
  . "/home/dave/ros/clam/devel/setup.sh"
  exec "$@"
fi
