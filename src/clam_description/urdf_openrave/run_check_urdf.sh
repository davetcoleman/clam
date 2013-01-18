# Convert to urdf and run through URDF verifier
rosrun xacro xacro.py clam.xacro > clam.urdf;
rosrun urdf_parser check_urdf clam.urdf;
