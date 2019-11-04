MODULE ROS_motion_r_gripper_rt

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
LOCAL CONST zonedata DEFAULT_CORNER_DIST := z1;
LOCAL CONST num GRIPPER_POS_TOL := 2;

! ----------------------------------------------
! ----------------- VARIABLES ------------------
! ----------------------------------------------
! Trajectory Variables
LOCAL VAR ROS_joint_trajectory_pt jointTrajectory;
LOCAL VAR num newGripperPos; ! store new gripper position locally
LOCAL VAR num previousGripperPos; ! store old gripper position locally

! Flag Variables
LOCAL VAR bool flag_handCalibrated := FALSE;
LOCAL VAR bool flag_programStarted := FALSE;
LOCAL VAR bool flag_firstPoint := FALSE;

! Task Name
LOCAL VAR string task_name := "M_Right";

! Syncronize Motion Variables
PERS tasks task_list{2} := [["T_ROB_R"],["T_ROB_L"]];
VAR syncident ready;
VAR syncident handCalibrated;

PROC main()
! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
! FIRST MODIFIED: 2016-06-14
! PURPOSE: Move the robot arm and gripper to specified trajectory
! NOTES: A gripper is attached to this arm (right arm)
! FUTURE WORK: Use the velociy values, need to find a way to send grip command to run Hand_GripInward function properly

    ! Initialize Variables
    VAR num current_index;
    VAR jointtarget target;
    VAR num gripperTarget;

    ! Syncronize Tasks
    IF (flag_programStarted = FALSE) THEN
        WaitSyncTask ready, task_list \TimeOut:=20;
        flag_programStarted := TRUE;
    ELSE
        TPWrite task_name + ": Program restarted.";
        flag_firstPoint := TRUE;
    ENDIF
    
    IF (flag_handCalibrated = FALSE) THEN
        calibrate_hand; ! Calibrate the hand
        WaitSyncTask handCalibrated, task_list \TimeOut:=20; ! Hand has been calibrated
    ENDIF

    ! Make Sure YuMi Wont Start Running Previous Stored Trajectory
    ROS_new_trajectory_right := FALSE;
    ClearPath;

    ! Wait For First Trajectory To Be Sent
    WHILE (NOT (flag_firstPoint)) DO
        IF (ROS_new_trajectory_right) THEN
            flag_firstPoint := TRUE;
        ENDIF
    ENDWHILE

    ! Get Trajectories and Move Arm
    WHILE true DO
        
        ! Move Gripper
        newGripperPos := ROS_trajectory_gripper_r{1}.gripper_pos;
        IF (NOT (newGripperPos = previousGripperPos)) THEN
            IF ((newGripperPos < GRIPPER_CLOSE_TOL) AND (previousGripperPos > (newGripperPos + GRIPPER_CLOSE_TOL))) THEN ! if gripping an object
                Hand_GripInward \holdForce:= 10, \NoWait; ! grip object without waiting for movement to complete
            ELSEIF (newGripperPos > GRIPPER_CLOSE_TOL) THEN ! if not gripping an object
                Hand_MoveTo newGripperPos, \NoWait; ! go to the gripper position without waiting for movement to complete
            ENDIF
            previousGripperPos := newGripperPos;
        ENDIF

        ! Move Arm
        jointTrajectory := ROS_trajectory_right{1};
        target.robax := jointTrajectory.joint_pos.robax;
        target.extax.eax_a := jointTrajectory.joint_pos.extax.eax_a;
        
        MoveAbsJ target, v100, \T:=0.01, DEFAULT_CORNER_DIST, tool0; ! move arm

    ENDWHILE
ERROR (ERR_WAITSYNCTASK)
    IF (ERRNO=ERR_WAITSYNCTASK) THEN
        ErrWrite \W, "WaitSync timeout", "Waited too long for hand calibration";
        TPWrite "Wait sync error";
    ELSE
        ErrWrite \W, "Motion Error", "Error executing motion.  Aborting trajectory.";
        TPWrite("Error Number: " + ValToStr(ERRNO) + " | Error executing motion");
        abort_trajectory;
    ENDIF
ENDPROC

LOCAL PROC calibrate_hand()
! PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
! DATE CREATED: 2016-06-14
! PURPOSE: Calibrate the gripper
! NOTES: A gripper is attached to this arm (right arm)
! FUTURE WORK: Fix issues with miscalibration, need to ensure hand has proper IP somehow

    ! Calibrate Hand
    Hand_JogInward; ! Right gripper has issues closing completely for caliration, this was added to ensure gripper closes all the way
    Hand_Initialize \maxSpd:=20, \holdForce:=10, \Calibrate;
    flag_handCalibrated := TRUE;
    
    ! Notify User Hand is Calibrated
    Hand_MoveTo(10);
    Hand_MoveTo(0);
    Hand_WaitMovingCompleted; ! ensure the hand is at the correct location
    TPWrite "Right hand Calibrated.";

    previousGripperPos := Hand_GetActualPos();

ENDPROC

LOCAL PROC abort_trajectory()
    clear_path;
    ExitCycle;  ! restart program
ENDPROC

LOCAL PROC clear_path()
    IF ( NOT (IsStopMoveAct(\FromMoveTask) OR IsStopMoveAct(\FromNonMoveTask)) ) THEN
        StopMove; ! stop any active motions
    ENDIF
    ClearPath; ! clear queued motion commands
    StartMove; ! re-enable motions
ENDPROC

ENDMODULE


