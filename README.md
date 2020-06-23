# Robot_Planning_A-star_VFH

The project involves planning and navigation of a differential drive robot from arbitrary start point to arbitrary destination point on a map with obstacles. 
Implemented A-star for global planning of the robot and Vector Field Histogram (VFH) for local planning of the robot
Planning is done based on observations from a LIDAR sensor

# How to run

1. Install ROS (if not already done)
2. Create a ROS catkin workspace if not already done: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
3. Create catkin ros package in the ROS catkin workspace (cmd: catkin_create_pkg ros_pa2 std_msgs rospy roscpp)
4. Download and unzip the repository in the ROS workspace
5. update the workspace (for python3 use command: $ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3)
6. Source the bash file of the workspace
7. make the files executable if required (cmd: chmod +x "file name")
8. Run "pa2.launch" file
