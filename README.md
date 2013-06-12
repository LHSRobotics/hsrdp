HSRDP
=====

HackSpace Robotics Development Platform
http://wiki.london.hackspace.org.uk/view/Project:HSRDP

Current status: Can play around with visualisation of the robot model though the URDF

A robotics project aiming to build a robotic platform for hackers to play around with higher level functions and make the robot do cool things.

Based on ROS (groovy), please reffer to http://www.ros.org/wiki/ROS/Installation for installation instructions.

Once installed pull the repository into your catkin workspace (e.g. ```~/catkin_ws/src/umi_rtx_100/src```) then 
run with
```roslaunch display.launch model:=umi_rtx_100.urdf gui:=true```
