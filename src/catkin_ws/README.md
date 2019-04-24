To change the location of the block you need to edit the world file. The world file can be found in:

/src/tester/worlds/test.world

<pose frame=''>0.6 0 0.5 0 0 0</pose>


To change the size of the block change the size parameters in:

/src/tester/worlds/test.world

<size>0.03 0.03 1</size>

You can also use the world we created by running:

roslaunch tester test.launch world_name:=blind.world



To change the laser scanner you need to edit
Uncomment the robot you want to use.
src/catkin_ws/src/LMS1xx/urdf/sick_lms1xx.urdf.xacro