# be careful - this will overwrite changes
#rosrun moveit_ikfast_converter create_ikfast_moveit_plugin.py clam arm clam_moveit_ikfast_plugin ~/ros/clam/src/clam_ik/clam_openrave/output_ikfast61.cpp 
rosrun moveit_ikfast create_ikfast_moveit_plugin.py clam arm clam_moveit_ikfast_plugin clam_moveit_ikfast_plugin/src/clam_arm_ikfast_solver.cpp