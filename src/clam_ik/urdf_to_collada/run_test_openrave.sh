cp ~/ros/clam/src/clam_description/urdf/clam.dae .
#you can use the following to test success rate: 
openrave.py --database inversekinematics --robot=clam.robot.xml --iktests=1000