Kille
======

A followup on [the Wall-e project](https://github.com/mmehdig/wall-e), a project for the course Embodied and Situated Language Processing. Kille was written for my Master's thesis for the Master in Language Technology at the University of Gothenburg, Sweden. 
The code is a bit messy, but should be readable. If you need help, feel free to contact me. 

The system has only been tested on Ubuntu 12.04, because of the requirement of drivers for the camera
and compatibility for those with ROS. Please use Ubuntu 12.04 if you would like to try this software. It is
possible to run the software in a virtual machine, yet this is not recommended because some host systems
do not manage to properly forward all three virtual USB devices Kinect offers (looking at you, Mac OS X). Non-kinect RGB-d cameras
should also work, as long as they provide /camera/depth/image raw and /camera/RGB/image color through
ROS. This has not been tested.



Installation
======
Instructions can be found in the appendice of the thesis.
