For information regarding the general simulation environment, please refer to Sam Pottinger's openRave documentation in readme_openRave.html

This readme assumes that ROS is already installed on this computer.  

###############################################################################
Design:
The language.py demo first asks for a text command.  This text is fed into an NLTK chart parser that is setup with an arm-specific grammar.  The grammar is a context free grammar, and does not need to be in Chomsky Normal Form to be valid.  After the tree is parsed, an interpreter function looks for certain keywords in the tree that appear based on expected arguments.  Sam Pottinger's openRave enviroment is used to pass messages to the simulation robot.  The messages can be used to get the status of the blocks/robot, as well as to manipulate the arm.

References:
http://www.nltk.org/
http://nltk.googlecode.com/svn/trunk/doc/api/index.html
http://www.shankarambady.com/nltk.pdf
https://github.com/shanbady/NLTK-Boston-Python-Meetup
http://code.google.com/p/ua-ros-pkg/

###############################################################################
Setup:

The following files need to be modified with the absolute path for the demo to run:

(modify the directory locations containing "shenas" to the related path on your system)
rave_experimental/cmake_install.cmake
rave_experimental/Makefile
rave_experimental/manifest.xml

(delete this file and recompile the rave_experimental directory)
rave_experimental/CMakeCache.txt  

don't forget to add the rave_experimental directory into the setup.sh file in your ROS install

For example, and several files the directories are hard coded for account "shenas":
"/home/shenas/rave_experimental/${CMAKE_INSTALL_MANIFEST}"
replace /home/shenas with the appropriate directory location


###############################################################################
To run the text command demo, run the following commands in order (in separate terminals):
roscore
rosrun rave_experimental listener.py
rosrun rave_experimental language.py

You may need to rerun the language node if the demo blocks do not appear in the simulation.  Also, it is normal that all the blocks look red, since they are all based on the same xml file in:
rave_experimental/src/rave_experimental/arm/test_box.xml


###############################################################################
Files specific to natural language processing:

rave_experimental/nodes/language.py
Demo of the text processing command demo.
Example commands:
point to the green block              (<--- very buggy!)
pick up the red block
put the red block on the blue block   (<--- somewhat buggy)


rave_experimental/nodes/alternate_grammar.py
An example of a more traditional grammar.  Resulting tree is more complicated and harder to interpret for simulation commands

###############################################################################
Future work:
Currently, all commands need to be hard coded into the grammar and the interpreter sections.  In the future, it would be beneficial to implement some sort of gradual learning, either through classifying labeled sentences, or developing a database of objects and actions.
The simulation is blind in the sense that messages need to be passed to the listener node to get the location of objects.  It may be beneficial to add some sort of knowledge model to the system.

Bugs: The "replace" action abstraction tends to fail, as openRave requires a specific roll, pitch and yaw for the arm position in order to move without reference to an object.  Oftentimes, the planning fails and the system hangs.
