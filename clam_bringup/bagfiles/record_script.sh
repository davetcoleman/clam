rm boulder_demo.bag
#rosbag record -O boulder_demo.bag /clam_marker_server/feedback
rosbag record -O boulder_demo.bag -e "/(.*)_controller/command"
