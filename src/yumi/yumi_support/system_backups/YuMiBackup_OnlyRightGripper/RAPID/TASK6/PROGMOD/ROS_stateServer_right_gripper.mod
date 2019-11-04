MODULE ROS_stateServer_right_gripper

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
LOCAL CONST num server_port := 12002;
LOCAL CONST num update_rate := 0.05;  ! broadcast rate (sec)

! ----------------------------------------------
! ----------------- VARIABLES ------------------
! ----------------------------------------------
! Socket Variables
LOCAL VAR socketdev server_socket;
LOCAL VAR socketdev client_socket;

! Task Name
LOCAL VAR string task_name := "SS_Right";

PROC main()
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Connecting with state server from ROS and sending joint values
! NOTES: A gripper is attached to this arm (right arm)
! FUTURE WORK: Use the velociy values

    ! Wait For Connections With ROS State Service
    TPWrite task_name + ": Waiting for connection.";
    ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket, task_name;
    
    ! Send Joint and Gripper Positions
    WHILE (TRUE) DO
        send_joints;
        WaitTime update_rate;
    ENDWHILE

ERROR (ERR_SOCK_TIMEOUT, ERR_SOCK_CLOSED, ERR_WAITSYNCTASK)
    IF (ERRNO=ERR_SOCK_TIMEOUT) OR (ERRNO=ERR_SOCK_CLOSED) THEN
        SkipWarn;  ! TBD: include this error data in the message logged below?
        ErrWrite \W, "ROS " + task_name + " disconnect", "Connection lost. Waiting for new connection.";
        ExitCycle;  ! restart program
    ELSE
        TRYNEXT;
    ENDIF
UNDO
ENDPROC

LOCAL PROC send_joints()
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Sends joints and gripper positions to ROS
! NOTES: A gripper is attached to this arm (right arm)
! FUTURE WORK: Use the velociy values

    VAR ROS_msg_joint_data jointMessage;
    VAR ROS_msg_gripper_data gripperMessage;
    
    ! Create Messages
    jointMessage.header := [ROS_MSG_TYPE_JOINT, ROS_COM_TYPE_TOPIC, ROS_REPLY_TYPE_INVALID];
    jointMessage.sequence_id := 0;
    jointMessage.joints := CJointT();

    gripperMessage.position := Hand_GetActualPos();
    
    ! send message to client
    ROS_send_msg_gripper_data client_socket, jointMessage, gripperMessage;

ERROR
    RAISE;  ! raise errors to calling code
ENDPROC

ENDMODULE


