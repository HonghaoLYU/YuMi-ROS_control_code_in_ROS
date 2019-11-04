# YuMi-ROS_control_code_in_ROS
The code for controling yumi simulated in RobotStudio (RS) through ROS.
If you don't have a real yumi robot, now you can sim it in the RobotStudio, this repo provied the corresponding code of ROS in Ubuntu. The Rapid code sim of yumi in RS is in this repo https://github.com/HonghaoLYU/YuMi-ROS_control_sim_in_RobotStudio/edit/master/README.md

#### Author: Honghao Lv
#### Email:lvhonghao@zju.edu.cn
#### Date-Version: 2019.11.14 V1.0

## Requirements:
OS: Ubuntu16.04
ROS: Kinetic

## Demo:

![img](https://honghaolyu.github.io/assets/images/posts/4-1.gif)

## You can config my ws in your own ubuntu follow the below steps:
1. Clone this repo in your Ubuntu or your workspace of ROS.  
2. before you catkin_make my code, you need to confirm the below Binary packages have been installed:  
''''''
   * Industrial core: `apt-get install ros-kinetic-industrial-core`
   * ABB stack: `apt-get install ros-kinetic-abb`
''''''
3. open a new in a teminal cd the upper-level path of src and run the below  
''
catkin_make
''
this may cost a few minutes.  
4. remember to modified the yumi_support/robot_interface_nogrippers.launch, change the ip to 10.11.122.38 or the ip of your windows for RS sim.  
5. start and run the sim yumi in RS, run the launch file in ubuntu:   
''''''
roslaunch yumi_support robot_interface_nogrippers.launch  
roslaunch yumi_moveit_config moveit_planning_execution_nogrippers.launch rviz:=true
''''''
6. enjoy it.

