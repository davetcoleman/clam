# Convert XACRO to URDF
rosrun xacro xacro.py clam.xacro > clam.urdf

# Convert URDF to collada format
rosrun collada_urdf urdf_to_collada clam.urdf clam.dae

# Convert Collada to a rounded version
python round_collada_numbers.py clam.dae clam.rounded.dae 2

# Copy rounded to regular and remove rounded
cp clam.rounded.dae clam.dae
rm clam.rounded.dae

# Install openrave here: http://openrave.org/docs/latest_stable/install/#install

# View list of links in model:
/usr/bin/openrave-robot.py clam.dae --info links

# Launch openrave
read -p "Press any key to launch openrave";
openrave clam.dae

