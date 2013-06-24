HSRDP
=====

HackSpace Robotics Development Platform
http://wiki.london.hackspace.org.uk/view/Project:HSRDP

Current status: Can play around with visualisation of the robot model though the URDF

A robotics project aiming to build a robotic platform for hackers to play around with higher level functions and make the robot do cool things.

Based on ROS (groovy), please reffer to http://www.ros.org/wiki/ROS/Installation for installation instructions.

URDF visualisation
------------------

Once installed pull the repository into your catkin workspace (e.g. ```~/catkin_ws/src/umi_rtx_100/src```) then 
run with
```roslaunch display.launch model:=umi_rtx_100.urdf gui:=true```

ROS - Arduino bridge
--------------------
Run ```ros_core``` on your machine
Then connect the arduino to the USB port and run
```rosrun rosserial_python serial_node.py /dev/ttyACM0```
It should show something like:

	[INFO] [WallTime: 1371999847.133541] ROS Serial Python Node
	[INFO] [WallTime: 1371999847.149722] Connecting to /dev/ttyACM0 at 57600 baud
	[INFO] [WallTime: 1371999850.217666] Note: publish buffer size is 280 bytes
	[INFO] [WallTime: 1371999850.218283] Setup publisher on arm_encoders [std_msgs/Int16MultiArray]

At this point the raw encoder readings should be published into ROS
You can view them by running rqt_plot with:

	rosrun rqt_plot rqt_plot /arm_encoders/data[0],/arm_encoders/data[1],/arm_encoders/data[2],/arm_encoders/data[3],/arm_encoders/data[4],/arm_encoders/data[5],/arm_encoders/data[6]
This will plot the raw encoder values onto a plot at 5Hz by default.


To tell the arduino the target positons of encoders on the commandline you can use the following:

	rostopic pub arm_encoders_targets std_msgs/Int16MultiArray '{layout: {dim: [], data_offset:0}, data: [0, 0, 0, 0, 0, 0, -375]}
