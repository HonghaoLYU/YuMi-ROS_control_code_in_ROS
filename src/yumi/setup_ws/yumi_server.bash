#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# PURPOSE: Make it easier to run YuMi scripts - Run robot interface nodes to send trajectories to YuMi and receive joint states
# REFERENCES: http://stackoverflow.com/questions/13777387/check-for-ip-validity | REASON: regexp for IP address

# Initialize Variables
runCommand="roslaunch yumi_support robot_interface.launch"; # set initial command for starting robot interface
flag_argError=false; # create flag to indicate if input arguments were valid
flag_twoGrippers=false; # flag to indicate if two grippers argument has already been set
flag_stateServersOnly=false; # flag to indicate if state servers only argument has already been set
flag_ipAddress=false; # flag to inficate if the ip address argument has already been set
total_arguments=0; # indicate the total arguments passed

echo "" # add a blank line to make displayed information more visible

# Iterate Through Provided Arguments
for argument in "$@"; do # for all provided arguments
	if [ "$argument" == "state_servers_only" ]; then # if the user would like to only run the state servers
		((total_arguments++)); # increment the passed argument count
		if [ $flag_stateServersOnly = false ]; then # if this argument has not been set yet
			echo "Running only the state servers." # notify the user that the argument has been received
			runCommand="$runCommand state_servers_only:=true"; # add argument to the run command
			flag_stateServersOnly=true; # indicate that the argument has been set
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
	echo "No arguments were passed. Assuming YuMi has camera assembly and one gripper, and running both state and motion servers." # notfiy the user of the effects of not passing any arguments
	echo "To run only the state server, command: state_servers_only" # notify the user the command for loading rviz on execution
	echo "To load two grippers, command: two_grippers" # notify the user the command for loading two grippers on execution
fi

# Check if All Arguments Were Valid
if [ $flag_argError = false ]; then # if all arguments were valid
	echo "" # add a blank line to make displayed information more visible
	sleep 2; # sleep to allow the user to see the terminal echo's
	$runCommand # run the robot interface node initializer command with user desired arguments
else # if one or more arguments were not valid
	echo "Error occurred. Not executing YuMi state/motion servers due to error." # notify user that the robot interface will not be executed
fi


