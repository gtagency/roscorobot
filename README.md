roscorobot
==========

Corobot ROS nodes.

Adopted from http://sourceforge.net/projects/roscorobot/ and modified to run on ROS Groovy.

These instructions work with the ROS Groovy virtual machine, available at www.ros.org.
They will likely work on any Ubuntu install with ROS, and may work on other operating systems.

These instructions assume you have a catkin workspace called 'catkin_ws' in your home folder.

## Dependencies

The Corobot ROS nodes are dependent on the following OS libraries:
* joystick
* libgps-dev
* libopenni-dev
* libqt4-dev
* libspnav-dev
* libusb-dev
* Phidgets Library

The Corobot ROS nodes are also dependent on the following ROS packages:
* joy (part of joystick_drivers, http://www.ros.org/wiki/joystick_drivers)
* openni_camera (http://www.ros.org/wiki/openni_camera)
* qt_build (part of qt_ros, http://www.ros.org/wiki/qt_ros)


Follow this guide to install the the Phidgets library:
http://www.phidgets.com/docs/OS_-_Linux#Installing

Install the other required OS libraries with apt-get
sudo apt-get install joystick libgps-dev libopenni-dev libqt4-dev libspnav-dev libusb-dev

Clone the ROS package code into 'catkin_ws/src':
```
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/joystick_drivers.git
git clone https://github.com/ros-drivers/openni_camera.git
git clone https://github.com/stonier/qt_ros.git
```

Add your workspace to the current bash session:
```
cd ~/catkin_ws
source devel/setup.bash
```

Clean and build your workspace:
```
cd ~/catkin_ws
rm -rf build
catkin_make
```
Note that you have to manually clear the build folder for changes to be picked up.

## Building Corobot

Clone this project into '~/catkin_ws/src' and build using rosmake:
```
cd ~/catkin_ws/src
git clone https://github.com/gtagency/roscorobot.git
cd roscorobot
rosmake Corobot
```

This will take several minutes to complete, after which you will be able to run the nodes.

## Launching Corobot

To run the default configuration, use the launch file contained in the corobot_teleop package:
```
roslaunch corobot_teleop corobot_ros_gui.launch 
```
This assumes that the workspace was previously added to the current bash session.  This will launch the
GUI and nodes to manage the various sensors, and enable keyboard teleop.

## Developing New Applications

To develop applications on the Corobot, you must work within a ROS workspace.  This is because the Corobot
code is built using "dry" packages (e.g. rosbuild packages).  Luckily, the ROS workspace can be overlayed on
top of the catkin workspace built above.

These instructions assume that you will use a ROS workspace called 'rosbuild_ws' in your home directory.

Create the workspace and overlay it on top of the existing catkin workspace:
```
mkdir ~/rosbuild_ws
cd ~/rosbuild_ws
rosws init . ~/catkin_ws/devel
source setup.bash
```

Create the sandbox folder that will contain your packages:
```
cd ~/rosbuild_ws
mkdir sandbox
rosws set sandbox
source setup.bash
```

Now we can create a package called my_package.  This package
will depend on rospy, std_msgs, and corobot_msgs (to publish
to Corobot notes):
```
cd ~/rosbuild_ws
source setup.bash
cd sandbox
roscreate-pkg my_package std_msgs rospy corobot_msgs
rosmake my_package
```

This should complete successfully.  
  
There is one last thing you need to do once you've created a python node.  At the top of your python file, underneath
the environment definition (e.g. "#!/usr/bin/env python"), add the following:
```
import roslib; roslib.load_manifest('my_package')
```

where my_package is the name of the package you created.

## Change Log

The following changes were made to support Groovy:

Replace
```
rosbuild_include(qt_build qt-ros)
```
with
```
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
include_directories(${catkin_INCLUDE_DIRS})
```
in corobot_teleop/CMakeLists.txt, to properly include qt_build with the Groovy build tools.

Replace the following Qt keywords
```
foreach
emit
signals
slots
```
with the corresponding macros
```
Q_FOREACH
Q_EMIT
Q_SIGNALS
Q_SLOTS
```
in corobot_teleop/src.  This was required for corobot_teleop to compile against Qt4 and ROS Groovy.
