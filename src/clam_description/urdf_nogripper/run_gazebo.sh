# Convert xarco to URDF
rosrun xacro xacro.py clam.xacro > clam.urdf;
rosrun gazebo spawn_model -file clam.urdf -urdf -x 1 -y 1 -z 1 -model clam

