# Convert xarco to URDF
rosrun xacro xacro.py clam.xacro > clam.urdf;
roslaunch clam_bringup clam_simulation.launch;
