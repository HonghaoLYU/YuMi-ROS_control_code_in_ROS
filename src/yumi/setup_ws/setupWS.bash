#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-24
# PURPOSE: Setup workspace to be able to run YuMi files

# Please read the wiki on setting up the workspace before running this file: github.com/ethz-asl/yumi/wiki/YuMi-Leap-Motion-Interface

# How to set up the workspace:
#  - Clone this repo into the home directory: cd && git clone https://www.github.com/ethz-asl/yumi
#      (or move the file to the home directory: mv <path>/yumi ~/yumi)
#  - Run the following command: bash ~/yumi/setup_ws/setupWS.bash

#=========================================
#---------- PRE-SETUP CHECKING -----------
#=========================================
cd ~ # go to the home directory

#----- Check if yumi_ws already exists in the home directory -----
# REFERENCE: http://stackoverflow.com/questions/59838/check-if-a-directory-exists-in-a-shell-script
if [ -d "yumi_ws" ]; then
    echo "[ERROR]: Setup failed. YuMi workspace directory already exists (~/yumi_ws). Cannot create workspace."
    echo "  - Possible fixes: Delete existing directory or fix existing directory."
    exit
fi

echo "Please read the wiki before continuing in order to ensure the workspace is properly setup. The Wiki and more information about this repo can be found at:" # tell user to visit Wiki
echo "  https://www.github.com/ethz-asl/yumi/wiki" # webpage for the wiki of this repo
echo "" # add in a space after comments

#----- Ask the user if they are sure they would like to continue the setup -----
# REFERENCE: http://stackoverflow.com/questions/1885525/how-do-i-prompt-a-user-for-confirmation-in-bash-script
read -p "Are you sure you would like to continue with the setup (y/n)? " response # get response from user
case "$response" in 
	y|Y ) echo "Continuing setup... ";; # continue with the setup
	n|N ) echo "Exiting setup... " && echo "Exited setup." && exit;; # exit the setup
	* ) echo "invalid response. Please use 'y' for Yes and 'n' for No" && exit;; # invalid input
esac


#======================================
#---------- VERIFY INSTALLS -----------
#======================================
clear # clear the terminal window
echo "[Part 1/3] Verifying installs... " # notify the user that installs will potentially be made

#----- Ask user if ROS Kinetic is installed -----
# REFERENCE: http://stackoverflow.com/questions/1885525/how-do-i-prompt-a-user-for-confirmation-in-bash-script
read -p "Is ROS Kinetic installed on this machine (y/n)? " response # get response from user
case "$response" in 
	y|Y ) existROS=1;; # ROS has already been setup
	n|N ) existROS=0;; # Set flag to install ROS
	* ) echo "[ERROR] Invalid response. Please use 'y' for Yes and 'n' for No" && exit;; # invalid input
esac

#----- If ROS Kinetic has not been setup yet -----
if [ $existROS -eq 0 ]; then
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' # update sources list for ROS files
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116 # setup keys for ROS

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' # update sources list for catkin tools
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add - # setup keys for catkin tools

	sudo apt-get update # update potential install list

	sudo apt-get install ros-Kinetic-desktop-full -y  # install Kinetic desktop
	if [[ $? > 0 ]]; then # if ROS Kinetic did not install properly
		echo "[ERROR] Failed to install ROS Kinetic." # notify the user of the error
		echo "Please execute the following commands below then re-execute this script." # notify the user of a possible fix
		echo "	sudo apt-get remove gazeboX (where X should be a number, use tab-complete to figure out which is installed." # notify the user to remove gazebo
		echo "  sudo apt-get install libsdformat1" # notify the user to install libsdformat1
		exit # exit from the script
	fi
	sudo apt-get install python-catkin-tools -y # install catkin tools

	sudo rosdep init # initialize ROS
	rosdep update # update ROS dependencies
fi

#----- Ensure that MoveIt! is Downloaded -----
flag_moveitInstall=true; # create a flag to indicate whether MoveIt! installed properly or not
sudo apt-get install ros-Kinetic-moveit-full -y # install MoveIt!
sudo dpkg --install Leap-*-x64.deb
if [[ $? > 0 ]]; then # if ROS Kinetic MoveIt! did not install properly
	flag_moveitInstall=false; # set the flag to indicate MoveIt! was not installed properly
fi
source /opt/ros/Kinetic/setup.bash # source ROS kinetic

#----- Install Leap Motion Packages -----
sudo apt-get install ros-Kinetic-leap-motion -y # install leap motion package
MACHINE_TYPE=`uname -m` # get whether the machine is 64 or 32 bit
if [ ${MACHINE_TYPE} == 'x86_64' ]; then # if running a 64 bit machine
	echo "export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64" >> ~/.bashrc # configure python path with leap motion SDK
else # if running a 32 bit machine
	echo "export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x86" >> ~/.bashrc # configure python path with leap motion SDK
fi
source ~/.bashrc

echo "Verified installs." # notify user that the installs have been verified


#======================================
#---------- SETUP WORKSPACE -----------
#======================================
clear # clear the terminal window
echo "[Part 2/3] Setting up workspace... " # notify user the workspace setup has started

#----- Create Workspace -----
mkdir yumi_ws # create workspace folder
mkdir yumi_ws/src # create folder to contain all files in workspace

#----- Add GitHub repo's neccessary to run YuMi files -----
mv yumi yumi_ws/src # move already cloned YuMi repo into workspace
git clone -b Kinetic-devel https://github.com/ros-industrial/abb ~/yumi_ws/src/abb_driver # clone the GitHub repo for the ABB driver using the kinetic-devel branch
git clone -b Kinetic-devel  https://github.com/ros-industrial/industrial_core ~/yumi_ws/src/industrial_driver # clone the GitHub repo for the ROS-Industrial driver using the kinetic-devel branch
git clone https://github.com/ethz-asl/rotors_simulator ~/yumi_ws/src/rotors_simulator # add in robot simulator for access to vi sensor xacro file
git clone -b feature/init_yaw https://github.com/ethz-asl/rovio ~/yumi_ws/src/rovio # add in rovio for running the VI sensor scripts
git clone https://github.com/ethz-asl/libvisensor_devel ~/yumi_ws/src/libvisensor_devel # add in the libraries for the VI sensor
git clone https://github.com/ethz-asl/visensor_node_devel ~/yumi_ws/src/visensor_node_devel # add in the VI node package
git clone https://github.com/ethz-asl/catkin_simple ~/yumi_ws/src/catkin_simple # add catkin simple package
git clone https://github.com/ethz-asl/yaml_cpp_catkin ~/yumi_ws/src/yaml_cpp_catkin # add YAML C++ package

shopt -s extglob # allow for !() command
cd ~/yumi_ws/src/abb_driver && rm -R !(abb|abb_driver|README.md) # remove unneccessary files from the ABB driver to allow for faster catkin builds
cd ~/yumi_ws/src/rotors_simulator && rm -R !(rotors_description|README.md) # remove unneccessary files from rotor simulator repository to allow for faster catkin builds
cd ~ # go back to the home directory

#----- Build Workspace -----
cd ~/yumi_ws # go to the YuMi workspace
catkin build # build the workspace

echo "Finished setting up the workspace." # notify user that the workspace has been setup


#======================================
#-------- FINALIZE INSTALLS -----------
#======================================
clear # clear the terminal window
echo "[Part 3/3] Finalizing installs... " # notify user installations are being finalized

#----- Variables -----
addYuMiCommands=0 # 0) Don't add YuMi command line tools | 1) Add YuMi commands line tools (yumi, yumi_demo, yumi_server)
addYuMiSource=0 # 0) Don't add YuMi workspace source command to bashrc | 1) Add YuMi workspace source command to bashrc
addBashrcHeader=1 # 0) Header for additions to bashrc file has already been added | 1) Header for additions to bashrc file has not been added yet
addBashrcFooter=0 # 0) No additions have been made to bashrc file, footer is not needed | 1) Additions have been made to bashrc file, add footer to additions

#----- Add Workspace Variables to Allow Command Line Capabilities -----
# YuMi command line tools
read -p "Add YuMi quick commands to command line? (recommended) (y/N)? " response # get response from user
case "$response" in 
	y|Y ) addYuMiCommands=1;; # add in command line alias to run YuMi easier 
	n|N ) echo "Not adding command line tools for YuMi... ";; # don't add in command line alias
	* ) echo "[ WARN] Invalid input. Not adding command line tools for YuMi... ";; # don't add in comamnd line alias
esac
if [ $addYuMiCommands -eq 1 ]; then # if the commands should be added
	echo "Adding command line tools for YuMi... " # notify user that command line tools for YuMi are being added

	bash ~/yumi_ws/src/yumi/setup_ws/setupWSVariables.bash ~/yumi_ws/src/yumi # run file to add quick commands for YuMi
	addBashrcHeader=0 # set flag to inficate a header has already been added to the bashrc file
	addBashrcFooter=1 # set flag to add in footer to bashrc file
fi 

#----- ROS Sourcing -----
if [ $existROS -eq 0 ]; then # if this is the first time for ROS install
	echo "Adding in ROS source... " # notify user that sourcing of ROS is being added to bashrc

	# If a header hasnt been added to the bashrc file yet
	if [ $addBashrcHeader -eq 1 ]; then
		echo "" >> ~/.bashrc # add in a blank ine before addition
		echo "# From: YuMi Github Repo" >> ~/.bashrc # add in a header for added section
		echo "# Purpose: Sourcing for ROS and/or YuMi workspace" >> ~/.bashrc # adding comment to describe purpose of addition
		addBashrcHeader=0 # set flag to inficate a header has already been added to the bashrc file
	fi

	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc # add sourcing of ROS
	addBashrcFooter=1 # set flag to add in footer to bashrc file
fi

#----- YuMi Workspace Sourcing -----
read -p "Automatically add workspace source to bashrc? (recommended) (y/N)? " response # get response from user
case "$response" in 
	y|Y ) addYuMiSource=1;; # add in command line alias to run YuMi easier 
	* ) echo "Not adding source to bashrc file... ";; # don't add in comamnd line alias
esac
if [ $addYuMiSource -eq 1 ]; then
	echo "Adding in YuMi workspace source... " # notify user that sourcing of the YuMi workspace is being added to bashrc

	# If a header hasnt been added to the bashrc file yet
	if [ $addBashrcHeader -eq 1 ]; then
		echo "" >> ~/.bashrc # add in a blank ine before addition
		echo "# From: YuMi Github Repo" >> ~/.bashrc # add in a header for added section
		echo "# Purpose: Sourcing for ROS and/or YuMi workspace" >> ~/.bashrc # adding comment to describe purpose of addition
		addBashrcHeader=0 # set flag to indicate a header has already been added to the bashrc file
	fi

	echo "source ~/yumi_ws/devel/setup.bash" >> ~/.bashrc # add sourcing of the YuMi workspace
	addBashrcFooter=1 # set flag to add in footer to bashrc file
fi

#----- Adding footer to added section -----
if [ $addBashrcFooter -eq 1 ]; then
	echo "# End of Addition" >> ~/.bashrc # indicate that there are no more additions for YuMi
	source ~/.bashrc # ensure all changes has been soruced
fi

echo "Finished setting up workspace." # notify user the setup has finished

#----------------------------------------------
#-- Add in checks to ensure correct install? --
#---------- Note Create: 2016-05-24 -----------
#----------------------------------------------


#======================================
#---------- USER DIRECTIONS -----------
#======================================
clear # clear the terminal window
echo "Workspace setup successful." # notify user the setup was successful

if [ $flag_moveitInstall = false ]; then # if ROS Kinetic MoveIt! did not install properly
	echo ""
	echo "Failed to install ROS Kinetic MoveIt! during installation." # notify the user
	echo "Please take a look at any errors and look online for a solution." # notify the user to look online for a solution
	echo "The MoveIt! interface script and other functionality will not work until MoveIt! is installed" # notify the user that MoveIt! needs to be installed for this repo to function properly
fi

#----- Give directions to user on how to run YuMi files -----
echo "" # add in a blank space before instructions
echo "Please use the following command before continuing:"
echo "cd && source ~/.bashrc" # go to the home directory and ensure bashrc has been source after changes
echo ""
echo "Please refer to the repo Wiki page for futher instructions"
echo "Wiki location: www.github.com/ethz-asl/yumi/wiki"
echo "" # add in a blank space after instructions


