# Create a visual PDF file from URDF
#rosrun urdf_parser urdf_to_graphiz clam.urdf
rosrun urdfdom urdf_to_graphiz clam.urdf
rm clam.gv # this is a mid-step file, not needed
gvfs-open clam.pdf # open file in ubuntu