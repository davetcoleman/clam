# Convert XACRO to URDF
rosrun xacro xacro.py clam.xacro > clam.urdf

# Convert URDF to collada format
rosrun collada_urdf urdf_to_collada clam.urdf clam.dae

# Install openrave here: http://openrave.org/docs/latest_stable/install/#install

# View list of links in model:
/usr/bin/openrave-robot.py clam.dae --info links

# Launch openrave
read -p "Press any key to launch openrave";
openrave clam.dae

