# Generate IKFast module
# Assume clam.dae file already exists in clam_description/urdf/clam.dae
cp ~/ros/clam/src/clam_description/urdf/clam.dae .

# View list of links in model:
read -p "Press any key to see link info";
/usr/bin/openrave-robot.py clam.dae --info links

read -p "Press any key to continue";
# Now create IK. Make sure urdf/dae file location path is correct
#python /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/ikfast.py --robot=clam.dae --iktype=transform6d --baselink=0 --eelink=7 --savefile=output_ikfast61.cpp #--freeindex=2
#python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=clam.robot.xml --iktype=transform6d --baselink=0 --eelink=7 --savefile=output_ikfast61.cpp #--freeindex=2

#python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=clam.robot.xml --iktype=transform6d --baselink=0 --eelink=7 --savefile=output_ikfast61.cpp --freeindex=3
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=clam.robot.xml --iktype=transform6d --baselink=0 --eelink=7 --savefile=output_ikfast61.cpp --freeindex=5