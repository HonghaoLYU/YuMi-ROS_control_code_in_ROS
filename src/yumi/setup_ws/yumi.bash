#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# PURPOSE: Make it easier to run YuMi scripts - Run YuMi in RViz
# REFERENCES: http://stackoverflow.com/questions/13777387/check-for-ip-validity | REASON: regexp for IP address

# Initialize Variables
runCommand="roslaunch yumi_moveit_config moveit_planning_execution.launch"; # set initial command for starting robot interface
flag_argError=false; # create flag to indicate if input arguments were valid
flag_twoGrippers=false; # flag to indicate if two grippers argument has already been set
flag_rviz=false; # flag to indicate if state servers only argument has already been set
flag_ipAddress=false; # flag to inficate if the ip address argument has already been set
flag_robotInterface=false; # flag to indicate if the robot interface argument has already been set
flag_stateServersOnly=false; # flag to indicate if state servers only argument has already been set
total_arguments=0; # indicate the total arguments passed

echo "" # add a blank line to make displayed information more visible

# Iterate Through Provided Arguments
for argument in "$@"; do # for all provided arguments
	if [ "$argument" == "rviz" ]; then # if the user would like to only run the state servers
		((total_arguments++)); # increment the passed argument count
		if [ $flag_rviz = false ]; then # if this argument has not been set yet
			echo "Loading RViz on execution." # notify the user that the argument has been received
			runCommand="$runCommand rviz:=true"; # add argument to the run command
			flag_rviz=true; # indicate that the argument has been set
		else # if the argument has already been set
			echo "Already set the argument for state_servers_only." # notify the user that this argument has already been set
		fi
	elif [ "$argument" == "two_grippers" ]; then # if the user is using two gripperson YuMi
		((total_arguments++)); # increment the passed argument count
		if [ $flag_twoGrippers = false ]; then # if this argument has not been set yet
			echo "Assuming using two grippers." # notify the user that the argument has been received
			runCommand="$runCommand two_grippers:=true"; # add argument to the run command
			flag_twoGrippers=true; # indicate that the argument has been set
		else # if the argument has already been set
			echo "Already set the argument for two_grippers." # notify the user that this argument has already been set
		fi
	elif [ "$argument" == "robot_interface" ]; then # if the user is using two gripperson YuMi
		((total_arguments++)); # increment the passed argument count
		if [ $flag_robotInterface = false ]; then # if this argument has not been set yet
			echo "Running robot interface concurrently." # notify the user that the argument has been received
			runCommand="$runCommand sim:=false"; # add argument to the run command
			flag_robotInterface=true; # indicate that the argument has been set
		else # if the argument has already been set
			echo "Already set the argument for robot_interface." # notify the user that this argument has already been set
		fi
	elif [ "$argument" == "state_servers_only" ]; then # if the user would like to only run the state servers
		((total_arguments++)); # increment the passed argument count
		if [ $flag_stateServersOnly = false ] && [ flag_robotInterface = true ]; then # if this argument has not been set yet and the robot_interface argument has been set
			echo "Running only the state servers." # notify the user that the argument has been received
			runCommand="$runCommand state_servers_only:=true"; # add argument to the run command
			flag_stateServersOnly=true; # indicate that the argument has been set
		else # if the argument has already been set
			if [ flag_robotInterface = false ]; then # if the robot_interface argument was not set before this argument
				echo "Need to set argument robot_interface before setting state servers only argument" # notify user to set robot_interface argument before this argument
			else # otherwise
				echo "Already set the argument for state_servers_only." # notify the user that this argument has already been set
			fi
		fi
	elif [[ "$argument" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then # if the user would like to specify the IP of YuMi
		((total_arguments++)); # increment the passed argument count
		if [ $flag_ipAddress = false ]; then  # if this argument has not been set yet
			echo "Setting robot interface node IP address to $argument." # notify the user that the argument has been received
			runCommand="$runCommand robot_ip:=$argument"; # add argument to the run command
			flag_ipAddress=true; # indicate that the argument has been set
		else # if the argument has already been set
			echo "Already set the argument for robot_ip." # notify the user that this argument has already been set
		fi
	else # if the argument is not recognized
		echo "Argument $currentArgument is not recognized." # notify the user that the current argument is not recognized
		flag_argError=true; # indicate that an argument error has occurred
		((total_arguments++)); # increment the passed argument count
		break; # break from the loop
	fi
done

# Check if no arguments were passed
if [ $total_arguments -eq 0 ]; then # if no arguments were passed
	echo "No arguments were passed. Not loading RViz and loading camera assembly and one gripper." # notfiy the user of the effects of not passing any arguments
	echo "To load RViz, command: rviz" # notify the user the command for loading rviz on execution
	echo "To load two grippers, command: two_grippers" # notify the user the command for loading two grippers on execution
fi

if [[ $flag_robotInterface = false && ( $flag_ipAddress = true || $flag_stateServersOnly = true ) ]]; then # if the user supplied robot interface arguments but is not loading the robot interface
	echo "" # add a blank line above error
	echo "[ERROR] Provided robot interface commands, but not loading the robot interface." # notify the user of the error
	echo "Please add 'robot_interface' as an additional command as well to execute the robot interface with the additional parameters." # notify the user that they should add the robot_interface command on execution
	flag_argError=true; # indicate that there was an argument error
fi

# Check if All Arguments Were Valid
if [ $flag_argError = false ]; then # if all arguments were valid
	echo "" # add a blank line to make displayed information more visible
	sleep 2; # sleep to allow the user to see the terminal echo's
	$runCommand # run the robot interface node initializer command with user desired arguments
else # if one or more arguments were not valid
	echo "[ WARN] Error occurred. Not loading YuMi into ROS due to error." # notify user that the robot interface will not be executed
fi


