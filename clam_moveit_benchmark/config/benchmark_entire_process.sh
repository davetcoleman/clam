# Old documentation
# http://moveit.ros.org/doxygen/moveit_benchmarking.html#visualizing_benchmark_results

# Pre-req: 
#roslaunch clam_bringup clam_benchmark.launch 

# Edit clam_moveit_config/config/ompl_planning.yaml
# Add
#  projection_evaluator: joints(shoulder_pan_joint,shoulder_pitch_joint)
#  longest_valid_segment_fraction: 0.05
# to the 'arm' group

# First 
~/ros/moveit/devel/lib/moveit_ros_benchmarks/moveit_call_benchmark --planners-only config1.cfg 
#rosrun moveit_ros_benchmarks moveit_call_benchmark config1.cfg 

cd ~/.ros

# Create pdf
/home/dave/ros/moveit/src/moveit_ros/benchmarks/benchmarks/scripts/moveit_benchmark_statistics.py test_scene_1.log.1.log -p benchmark_results.pdf

# View pdf
gopen benchmark_results.pdf