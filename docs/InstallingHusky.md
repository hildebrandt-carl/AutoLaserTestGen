# Husky

[Husky](https://www.clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) is a medium sized research unmanned ground vehicle. It has a large array of payloads and sensor configurations. Husky is fully supported in [ROS](https://wiki.ros.org).

The original instructions are found at the [ROS Husky](https://wiki.ros.org/Robots/Husky) page. These notes are modified from the original instructions.

--------

## Prerequisites

Husky supports ROS up to Melodic. This tutorial installs Husky on ROS Indigo.

* [Ubuntu 16.04](https://www.ubuntu.com/download/alternative-downloads)
* [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu)

**Note:** Although ROS is technically available for MacOS I would not recommend that.

### Installing Ubuntu

The [Ubuntu 16.04](https://www.ubuntu.com/download/alternative-downloads) link will take you to a site which allows you to download [Ubuntu 16.04](https://www.ubuntu.com/download/alternative-downloads) ISO file. You can use this to flash your PC or install it on a virtual machine.

### Installing ROS

Please check the [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu) for the latest installation guide however here are the basic commands.

Setup your sources.list
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
```

Installation
```
$ sudo apt-get install ros-kinetic-desktop-full
```

Initialize rosdep
```
$ sudo rosdep init
$ rosdep update
```

Environment Setup
```
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

We will need the [Catkin Build tools](https://catkin-tools.readthedocs.io/en/latest/installing.html). To install these run the following commands: 

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

We will also  need a set of custom ROS packages. You can install these by running: 
```
$ sudo apt-get install 'ros-kinetic-controller-*'
$ sudo apt-get install ros-kinetic-lms1xx
$ sudo apt-get install ros-kinetic-robot-localization
$ sudo apt-get install 'ros-kinetic-geographic-*'
$ sudo apt-get install 'ros-kinetic-interactive-marker-*'
$ sudo apt-get install 'ros-kinetic-twist-*'
$ sudo apt-get install ros-kinetic-joint-state-controller
$ sudo apt-get install ros-kinetic-diff-drive-controller
$ sudo apt-get install ros-kinetic-moveit-visual-tools
$ sudo apt-get install ros-kinetic-rviz-imu-plugin
$ sudo apt-get install ros-kinetic-move-base
$ sudo apt-get install 'ros-kinetic-dwa-*'
```

### Creating Workspace
```
mkdir -p catkin_ws/src
```

Clone the Husky [source code](https://github.com/husky/husky)
```
$ cd catkin_ws/src/
$ git clone https://github.com/husky/husky.git
```

Build and source the workspace:
```
$ catkin build
$ source devel/setup.bash
```

--------

## Simulation

### Testing the Simulation

Run the simulation
```
$ roslaunch husky_gazebo husky_empty_world.launch
```

You should start with the husky on a flat surface as shown below:

![A robot in an empty simulation](images/empty.png)

**Note If you get the error:**
```
Warning [ModelDatabase.cc:334] Getting models from[http://gazebosim.org/models/]. This may take a few seconds.
Warning [gazebo.cc:215] Waited 1seconds for namespaces.
[0.000000] script_server is running
Warning [gazebo.cc:215] Waited 1seconds for namespaces.
Warning [gazebo.cc:215] Waited 1seconds for namespaces.
Warning [gazebo.cc:215] Waited 1seconds for namespaces.
...
```

This means that your Gazebo cant connect to the model database. Give it some time, and maybe try restarting it. However if it does not go away you can circumvent this by downloading the models yourself. Do using the solution found [here](https://answers.ros.org/question/199401/problem-with-indigo-and-gazebo-22/):
```
$ cd ~/.gazebo
$ mkdir models
```

Then download the models
```
$ cd ~/.gazebo/models
$ wget -r -R "index\.html*" http://models.gazebosim.org/
```


### Testing the Control Software

You will need to open three terminals to do this. Make sure you source the catkin_ws in each of the terminals using:

```
$ cd catkin_ws
$ source devel/setup.bash
```

In the first terminal run open the physics simulator Gazebo. You can do that using:

```
$ roslaunch husky_gazebo husky_playpen.launch
```

Then start the visualization tool rviz using the line below. Make sure you check the navigation checkbox in rviz.

```
$ roslaunch husky_gazebo husky_playpen.launch
```

Finally launch the Husky's mapping and controller code using:

```
$ roslaunch husky_navigation move_base_mapless_demo.launch
```

Use the 2D Nav button to generate new navigation points for the Husky.



















