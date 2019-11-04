#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# LAST MODIFIED: 2016-05-26
# PURPOSE: Create workspace variables for YuMi during initial setup

# Get the directory location of the YuMi folder
if [ -z "$1" ]; then 
	echo "Please input the path of where the yumi directory is located"
	exit
fi

echo "Adding command line workspace variables... " # notify user the process has started

# Add YuMi alias for running scriptsecho "" >> ~/.bashrc # add in blank line before addition
echo "" >> ~/.bashrc # add in a blank ine before addition
echo "# From: YuMi Github Repo" >> ~/.bashrc # add header for added section
echo "# Purpose: Alias for YuMi commands" >> ~/.bashrc # describe purpose for added section
echo "alias yumi='bash ${1}/setup_ws/yumi.bash'" >> ~/.bashrc # allow for YuMi to be run from command line for the real controller
echo "alias yumi_demo='bash ${1}/setup_ws/yumi_demo.bash'" >> ~/.bashrc # allow for YuMi to be run from command line for the fake controller
echo "alias yumi_server='bash ${1}/setup_ws/yumi_server.bash'" >> ~/.bashrc # run the YuMi server from command line to send path commands to the real controller
echo "alias yumi_lead='rosrun yumi_scripts lead_through'" >> ~/.bashrc # run the lead through script from command line for generating RAPID modules 
echo "alias yumi_moveit='rosrun yumi_scripts moveit_interface'" >> ~/.bashrc # run the MoveIt! interface script from command line for interfacing MoveIt! with YuMi through command line
echo "alias yumi_node='roslaunch yumi_scripts yumi_node.launch'" >> ~/.bashrc # execute the node used to manipulate YuMi
echo "alias yumi_interface='rosrun yumi_scripts yumi_interface'" >> ~/.bashrc # run the script that interfaces with the YuMi node
echo "alias yumi_leap='roslaunch yumi_scripts leap_interface.launch'" >> ~/.bashrc # execute the node that interfaces the Leap Motion sensor with YuMi
echo "" >> ~/.bashrc # add in blank line underneath addition

source ~/.bashrc # source bashrc to finalize changes for current terminal window

echo "Finished." # notify user that the process is finished


