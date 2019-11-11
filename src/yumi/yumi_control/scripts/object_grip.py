#!/usr/bin/env python
# coding=UTF-8
# Author: Honghao Lv limit the worksapce for small motion step control

import sys
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Header
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import JointConstraint, OrientationConstraint, PositionConstraint, JointLimits, Constraints
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose2D

# 全局变量定义以及赋值
obj_x = 0
obj_y = 0

# 定义回调函数,订阅接受到的消息传给data
def Objpose_callback(data):
    global obj_x, obj_y
    obj_x = data.x
    obj_y = data.y

class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        # 初始化 `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',anonymous=True)

        # 订阅话题
        rospy.Subscriber('object_coordinate', Pose2D, Objpose_callback)

        # 实例化 a `RobotCommander`_ 对象.
        robot = moveit_commander.RobotCommander()

        # 实例化 a `MoveGroupCommander`_ 对象.
        right_arm = moveit_commander.MoveGroupCommander("right_arm")
        left_arm = moveit_commander.MoveGroupCommander("left_arm")
        both_arms = moveit_commander.MoveGroupCommander("both_arms")

        # 创建 `DisplayTrajectory`_ publisher,稍后用于发布RViz可视化的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        # 获取机器人的参考坐标系并输出
        planning_frame = both_arms.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # 获取机器人中所有groups的名称并打印:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # 输出当前机器人的全部状态便于调试:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # 各种变量
        self.box_name = ''
        self.robot = robot
        self.right_arm = right_arm
        self.left_arm = left_arm
        self.both_arms = both_arms
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.group_names = group_names

    def right_arm_go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        right_arm = self.right_arm

        # 获取当前末端执行器位置姿态
        pose_goal = right_arm.get_current_pose().pose

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = -0.12957
        pose_goal.orientation.y = 0.99157
        pose_goal.orientation.z = -0.0019431
        pose_goal.orientation.w = 0.0012715
        pose_goal.position.x = 0.26832 + obj_x/1000
        pose_goal.position.y = -0.37558 + obj_y/1000
        pose_goal.position.z = 0.1
        right_arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # 规划和输出动作
        traj = right_arm.plan()
        right_arm.execute(traj, wait=False)
        # 动作完成后清除目标信息
        right_arm.clear_pose_targets()
        # 清除路径约束
        right_arm.clear_path_constraints()
        # 确保没有剩余未完成动作在执行
        right_arm.stop()

    def left_arm_go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        left_arm = self.left_arm

        # 获取当前末端执行器位置姿态
        pose_goal = left_arm.get_current_pose().pose

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = Left_Qux
        pose_goal.orientation.y = Left_Quy
        pose_goal.orientation.z = Left_Quz
        pose_goal.orientation.w = Left_Quw
        pose_goal.position.x = 0.26832 + obj_x/1000
        pose_goal.position.y = -0.37558 + obj_y/1000
        pose_goal.position.z = 0
        left_arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # 规划和输出动作
        traj = left_arm.plan()
        left_arm.execute(traj, wait=False)
        # 动作完成后清除目标信息
        left_arm.clear_pose_targets()     
        # 清除路径约束
        left_arm.clear_path_constraints()
        # 确保没有剩余未完成动作在执行
        left_arm.stop()

    def right_gripper_go_to_close_goal(self):
        # 设置动作对象变量,此处为right_arm
        right_arm = self.right_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        # right_arm.set_goal_joint_tolerance(0.015)
        # 设置末端关节目标值
        right_joint_goal[7] = 0
        # 规划并执行路径动作
        # traj = right_arm.plan(right_joint_goal)
        # right_arm.execute(traj, wait=False)
        right_arm.go(right_joint_goal, wait=False)
        right_arm.clear_pose_targets()
        right_arm.stop()
        # right_arm.set_goal_joint_tolerance(0.0001)

    def right_gripper_go_to_open_goal(self):
        # 设置动作对象变量,此处为right_arm
        right_arm = self.right_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        # 设置末端关节目标值
        right_joint_goal[7] = 0.024
        # 规划并执行路径动作
        # traj = right_arm.plan(right_joint_goal)
        # right_arm.execute(traj, wait=False)
        right_arm.go(right_joint_goal, wait=False)
        right_arm.clear_pose_targets()
        right_arm.stop()

    def left_gripper_go_to_close_goal(self):
        # 设置动作对象变量,此处为left_arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        left_joint_goal = left_arm.get_current_joint_values()
        # 设置末端关节目标值
        left_joint_goal[7] = 0
        # 规划并执行路径动作
        left_arm.go(left_joint_goal, wait=False)
        left_arm.clear_pose_targets()
        left_arm.stop()

    def left_gripper_go_to_open_goal(self):
        # 设置动作对象变量,此处为left_arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        left_joint_goal = left_arm.get_current_joint_values()
        # 设置末端关节目标值
        left_joint_goal[7] = 0.024
        # 规划并执行路径动作
        left_arm.go(left_joint_goal, wait=False)
        left_arm.clear_pose_targets()
        left_arm.stop()

    def right_arm_get_current_joint_state(self):
        # 设置动作对象变量,此处为right_arm
        right_arm = self.right_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        return right_joint_goal

    def left_arm_get_current_joint_state(self):
        # 设置动作对象变量,此处为left_arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        left_joint_goal = left_arm.get_current_joint_values()
        return left_joint_goal


def main():
    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()
    # 循环等待,执行动作程序
    print "============ Press `Enter` to begin the teleoperation ..."
    raw_input()

    while 1:
        raw_input()
        yumi.right_arm_go_to_pose_goal()
        # time.sleep(3)


            

if __name__ == '__main__':
    main()
