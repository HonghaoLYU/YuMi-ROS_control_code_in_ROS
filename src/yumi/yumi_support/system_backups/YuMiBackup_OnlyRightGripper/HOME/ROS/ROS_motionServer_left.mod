MODULE ROS_motionServer_left

! Software License Agreement (BSD License)
!
! Copyright (c) 2012, Edward Venator, Case Western Reserve University
! Copyright (c) 2012, Jeremy Zoss, Southwest Research Institute
! All rights reserved.
!
! Redistribution and use in source and binary forms, with or without modification,
! are permitted provided that the following conditions are met:
!
!   Redistributions of source code must retain the above copyright notice, this
!       list of conditions and the following disclaimer.
!   Redistributions in binary form must reproduce the above copyright notice, this
!       list of conditions and the following disclaimer in the documentation
!       and/or other materials provided with the distribution.
!   Neither the name of the Case Western Reserve University nor the names of its contributors
!       may be used to endorse or promote products derived from this software without
!       specific prior written permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
! EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
! OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
! SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
! TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
! BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
! WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

! ----------------------------------------------
! ----------------- CONSTANTS ------------------
! ----------------------------------------------
LOCAL CONST num server_port := 11000;

! ----------------------------------------------
! ----------------- VARIABLES ------------------
! ----------------------------------------------
! Socket Variables
LOCAL VAR socketdev server_socket;
LOCAL VAR socketdev client_socket;

! Trajectory Variables
LOCAL VAR ROS_joint_trajectory_pt jointTrajectory{MAX_TRAJ_LENGTH};
LOCAL VAR num trajectory_size;

! Task Name
LOCAL VAR string task_name := "MS_Left";

PROC main()
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Connecting with motion server from ROS and storing joint values
! NOTES: A gripper is not attached to this arm (left arm)
! FUTURE WORK: Use the velociy values

    ! Initialize Variables
    VAR ROS_msg_joint_traj_pt jointMessage;

    ! Wait For Connections With ROS Motion Service
    WaitTime 0.5; ! stagger server connection attempts
    TPWrite task_name + ": Waiting for connection.";
    ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket, task_name;

    ! Recieve Joint Trajectory Point
    WHILE ( true ) DO
        ROS_receive_msg_joint_traj_pt client_socket, jointMessage;
        trajectory_pt_callback jointMessage;
    ENDWHILE

ERROR (ERR_SOCK_TIMEOUT, ERR_SOCK_CLOSED, ERR_WAITSYNCTASK)
    IF (ERRNO=ERR_SOCK_TIMEOUT) OR (ERRNO=ERR_SOCK_CLOSED) THEN
        SkipWarn;  ! TBD: include this error data in the message logged below?
        ErrWrite \W, "ROS " + task_name + " disconnect", "Connection lost. Resetting socket.";
        ExitCycle;  ! restart program
    ELSEIF (ERRNO=ERR_WAITSYNCTASK) THEN
        ErrWrite \W, "WaitSync timeout", "Waited too long for hand calibration";
        TPWrite "Wait sync error";
    ELSE
        TRYNEXT;
    ENDIF
UNDO
    IF (SocketGetStatus(client_socket) <> SOCKET_CLOSED) SocketClose client_socket;
    IF (SocketGetStatus(server_socket) <> SOCKET_CLOSED) SocketClose server_socket;
ENDPROC

LOCAL PROC trajectory_pt_callback(ROS_msg_joint_traj_pt jointMessage)
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Creating trajectory from data received from ROS
! NOTES: A gripper is not attached to this arm (left arm)
! FUTURE WORK: Use the velociy values

    ! Initialize Variables
    VAR ROS_joint_trajectory_pt joints;
    VAR ROS_msg reply_msg;

    ! Store Joint Values Locally
    joints := [jointMessage.joints, jointMessage.duration];
    
    ! Use sequence_id To Signal Start/End of Trajectory Download
    TEST jointMessage.sequence_id
        CASE ROS_TRAJECTORY_START_DOWNLOAD:
            TPWrite task_name + ": Traj START received";
            trajectory_size := 0;  ! Reset trajectory size
            add_traj_pt joints; ! Add this point to the trajectory
        CASE ROS_TRAJECTORY_END:
            TPWrite task_name + ": Traj END received";
            add_traj_pt joints; ! Add this point to the trajectory
            activate_trajectory;
        CASE ROS_TRAJECTORY_STOP:
            TPWrite task_name + ": Traj STOP received";
            ! trajectory_size := 0;  ! empty trajectory
            ! activate_trajectory;
            ! StopMove; ClearPath; StartMove;  ! redundant, but re-issue stop command just to be safe
        DEFAULT:
            add_traj_pt joints; ! Add this point to the trajectory
    ENDTEST

    ! Send Reply, If Requested
    IF (jointMessage.header.comm_type = ROS_COM_TYPE_SRV_REQ) THEN
        reply_msg.header := [ROS_MSG_TYPE_JOINT_TRAJ_PT, ROS_COM_TYPE_SRV_REPLY, ROS_REPLY_TYPE_SUCCESS];
        ROS_send_msg client_socket, reply_msg;
    ENDIF

ERROR
    RAISE;  ! raise errors to calling code
ENDPROC

LOCAL PROC add_traj_pt(ROS_joint_trajectory_pt joints)
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Add joint values to trajectory arrays
! NOTES: A gripper is not attached to this arm (left arm)
! FUTURE WORK: Use the velociy values

    IF (trajectory_size = MAX_TRAJ_LENGTH) THEN
        ErrWrite \W, "Too Many Trajectory Points", "Trajectory has already reached its maximum size",
            \RL2:="max_size = " + ValToStr(MAX_TRAJ_LENGTH);
    ELSE
        Incr trajectory_size; ! increment trajectory size
        jointTrajectory{trajectory_size} := joints; ! add this point to the joint trajectory
    ENDIF

ENDPROC

LOCAL PROC activate_trajectory()
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Set flags to indicate that a trajectory has been received and fully built
! NOTES: A gripper is attached to this arm (left arm)
! FUTURE WORK: Use the velociy values

    ! Aquire Data Lock
    WaitTestAndSet ROS_trajectory_lock_left; ! acquire data-lock

    ! Store Joint Trajectory and Joint Trajectory Variables Into System Variables
    ROS_trajectory_left := jointTrajectory;
    ROS_trajectory_size_left := trajectory_size;

    ! Release Locks and Notify User of Sent Trajectory
    ROS_new_trajectory_left := TRUE;
    ROS_trajectory_lock_left := FALSE; ! release data-lock
    TPWrite "Sending " + ValToStr(trajectory_size) + " points to left MOTION task";

ENDPROC
    
ENDMODULE


