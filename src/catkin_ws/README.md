# Husky Simulation

This folder contains the three sets of code.

* Husky source code
* LMS1xx ROS laser scanner driver code
* Tester code

This is what we used to run the Husky in Simultion. Change the laser scanner properties and test the Husky using the different configurations.

## Prerequisits

You need to have ROS installed as well as a number of ROS nodes installed in order for this code to work. For detailed instructions please read the InstallingHusky document in the docs folder.

## Running the code

### World files

If you want to create a world with a new block location. Simply go to the `tester/worlds/` and copy any of the world files. Inside the world file change the following lines, replacing x and y with the position

```
<model name='unit_box_0'>
    <pose frame=''>x y 0.5 0 0 0</pose>
    ...
```

To change the size of the block you edit the following line in the world block. Replace x_size and y_size with the required dimensions. Note that you need to change both the collision box's size as well as a visualization size.

```
<model name='unit_box_0'>
    ...
    <size>x_size y_size 1</size>
    ...
```

### Laser Scanner Properties

To change the laser scanner properties you need to edit `LMS1xx/urdf/sick_lms1xx.urdf.xacro` file. You will find a line which declares all the sensors attributes:

```
<xacro:macro name="sick_lms1xx" params="frame:=laser topic:=scan sample_size:=720 update_rate:=50 min_angle:=-1.57 max_angle:=1.57 min_range:=0.1 max_range:=10.0 robot_namespace:=/" >
```

You can change any of the lines to meet your laser scanner specification needs.

### Running the tester

Before running make sure everything is compiled you can do this by running the following code inside the `catkin_ws` directory.

```
catkin build
```

To run the tester which includes collision detection, world visualization, sensor visualization, frontier exploration, SLAM, and goal publisher you can run the following in your terminal

```
source devel/setup.bash
roslaunch tester test.launch world_name:=blind.world
```

You can replace the world name with any of the worlds included in the `tester/worlds/`.

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt-carl](https://github.com/hildebrandt-carl)
* **Melony Bennis** - *Initial work* - [mmb4vu](https://github.com/mmb4vu)
