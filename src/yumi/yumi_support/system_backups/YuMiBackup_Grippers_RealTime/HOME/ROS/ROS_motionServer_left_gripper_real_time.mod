MODULE ROS_motionServer_l_gripper_rt

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

! Flag Variables
LOCAL VAR bool first_trajectory := TRUE;

! Task Name
LOCAL VAR string task_name := "MS_Left";

PROC main()
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Connecting with motion server from ROS and storing joint values
! NOTES: A gripper is attached to this arm (left arm)
! FUTURE WORK: Use the velociy values

    ! Initialize Variables
    VAR ROS_msg_traj_pt message;

    ! Wait For Connections With ROS Motion Service
    WaitTime 0.5; ! stagger server connection attempts
    TPWrite task_name + ": Waiting for connection.";
    ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket, task_name;

    ! Recieve Joint Trajectory Point and Gripper Position
    WHILE ( true ) DO
        ROS_receive_msg_gripper_traj_pt client_socket, message;
        trajectory_pt_callback message;
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

LOCAL PROC trajectory_pt_callback(ROS_msg_traj_pt message)
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Creating trajectory from data received from ROS
! NOTES: A gripper is attached to this arm (left arm)
! FUTURE WORK: Use the velociy values

    ! Initialize Variables
    VAR ROS_joint_trajectory_pt joints;
    VAR ROS_gripper_trajectory_pt gripper;
    VAR ROS_msg reply_msg;

    ! Store Joint and Gripper Values Locally
    joints := [message.joints, message.duration];
    gripper.gripper_pos := message.gripperTarget;
    
    ! Store Trajectoy to Global Variable
    IF (NOT (message.sequence_id = ROS_TRAJECTORY_STOP)) THEN 

        IF (first_trajectory = TRUE) THEN
            ! Store Joint Trajectory and Joint Trajectory Variables Into System Variables
            ROS_trajectory_left{1}      := joints;
            ROS_trajectory_gripper_l{1} := gripper;

            first_trajectory := FALSE;
        ELSE
            ! Shift Second Trajectory to First
            ROS_trajectory_left{1}      := ROS_trajectory_left{2};
            ROS_trajectory_gripper_l{1} := ROS_trajectory_gripper_l{2};
        ENDIF

        ! Store Joint Trajectory and Joint Trajectory Variables Into System Variables
        ROS_trajectory_left{2}      := joints;
        ROS_trajectory_gripper_l{2} := gripper;
        
        ! Release Data Lock
        ROS_new_trajectory_left  := TRUE;

    ENDIF

    ! Send Reply, If Requested
    IF (message.header.comm_type = ROS_COM_TYPE_SRV_REQ) THEN
        reply_msg.header := [ROS_MSG_TYPE_JOINT_TRAJ_PT, ROS_COM_TYPE_SRV_REPLY, ROS_REPLY_TYPE_SUCCESS];
        ROS_send_msg client_socket, reply_msg;
    ENDIF

ERROR
    RAISE;  ! raise errors to calling code
ENDPROC

ENDMODULE


