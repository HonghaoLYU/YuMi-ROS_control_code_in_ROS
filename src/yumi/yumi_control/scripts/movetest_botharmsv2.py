#!/usr/bin/env python
# coding=UTF-8
# Author: Honghao Lv

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Header
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import JointConstraint, OrientationConstraint, JointLimits, Constraints
from datamessage.msg import bend
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 全局变量定义以及赋值
Neurondata = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
Rightfinger = 0
Leftfinger = 0
RightfingerT = 0
LeftfingerT = 0

# 定义回调函数,订阅接受到的消息传给data
def Neuron_callback(data):
    # 对全局变量进行赋值
    global Neurondata
    global Right_Eux, Right_Euy, Right_Euz, Left_Eux, Left_Euy, Left_Euz
    global Right_Qux, Right_Quy, Right_Quz, Right_Quw, Left_Qux, Left_Quy, Left_Quz, Left_Quw
    Neurondata = data.data
    Right_Euy = Neurondata[0]
    Right_Eux = Neurondata[1]
    Right_Euz = Neurondata[2]
    Left_Euy = Neurondata[6]
    Left_Eux = Neurondata[7]
    Left_Euz = Neurondata[8]
    (Right_Qux, Right_Quy, Right_Quz, Right_Quw) = quaternion_from_euler(Right_Euz, -Right_Eux, Right_Euy)
    (Left_Qux, Left_Quy, Left_Quz, Left_Quw) = quaternion_from_euler(Left_Euz, -Left_Eux, Left_Euy)

# 定义回调函数,订阅接受到的消息传给data
def Glove_callback(data):
    # 对全局变量进行赋值
    global Rightfinger, Leftfinger, RightfingerT, LeftfingerT
    Rightfinger = round(data.RI,3)
    Leftfinger = round(data.LI,3)
    RightfingerT = round(data.RT,3)
    LeftfingerT = round(data.LT,3)


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        # 初始化 `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',anonymous=True)

        # 订阅话题
        rospy.Subscriber('yumiaction', Float64MultiArray, Neuron_callback)
        rospy.Subscriber('dataglove', bend, Glove_callback)

        # 实例化 a `RobotCommander`_ 对象.
        robot = moveit_commander.RobotCommander()

        # 实例化 a `PlanningSceneInterface`_ 对象.
        scene = moveit_commander.PlanningSceneInterface()

        # 实例化 a `MoveGroupCommander`_ 对象.
        right_arm = moveit_commander.MoveGroupCommander("right_arm")
        left_arm = moveit_commander.MoveGroupCommander("left_arm")
        both_arms = moveit_commander.MoveGroupCommander("both_arms")

        # 创建 `DisplayTrajectory`_ publisher,稍后用于发布RViz可视化的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        # # 获取机器人的参考坐标系并输出
        # planning_frame = arm.get_planning_frame()
        # print "============ Reference frame: %s" % planning_frame

        # # 获取当前末端执行器并输出
        # eef_link = right_arm.get_end_effector_link()
        # print "============ End effector: %s" % eef_link

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
        self.scene = scene
        self.right_arm = right_arm
        self.left_arm = left_arm
        self.both_arms = both_arms
        self.display_trajectory_publisher = display_trajectory_publisher
        # self.planning_frame = planning_frame
        # self.eef_link = eef_link
        self.group_names = group_names


    def right_arm_go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        right_arm = self.right_arm
        # 限制末端夹具运动
        right_joint_const = JointConstraint()
        right_joint_const.joint_name = "gripper_r_joint_r"
        if Rightfinger >= 0.025 :
            right_joint_const.position = 0.024
        else:
            right_joint_const.position = 0
        consts = Constraints()
        consts.joint_constraints = [right_joint_const]
        right_arm.set_path_constraints(consts)
        # 获取当前末端执行器位置姿态
        pose_goal = right_arm.get_current_pose().pose

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = Right_Qux
        pose_goal.orientation.y = Right_Quy
        pose_goal.orientation.z = Right_Quz
        pose_goal.orientation.w = Right_Quw
        pose_goal.position.x = (Neurondata[5]-0.05)*1.48+0.053
        pose_goal.position.y = (Neurondata[3]+0.18)*1.48-0.12
        pose_goal.position.z = (Neurondata[4]-0.53)*1.48+0.47
        right_arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # # 设置动作对象目标位置姿态
        # pose_goal.orientation.x = pose_goal.orientation.x
        # pose_goal.orientation.y = pose_goal.orientation.y
        # pose_goal.orientation.z = pose_goal.orientation.z
        # pose_goal.orientation.w = pose_goal.orientation.w
        # pose_goal.position.x = pose_goal.position.x
        # pose_goal.position.y = pose_goal.position.y - 0.01
        # pose_goal.position.z = pose_goal.position.z
        # right_arm.set_pose_target(pose_goal)
        # print "End effector pose %s" % pose_goal

        # 规划和输出动作
        traj = right_arm.plan()
        right_arm.execute(traj, wait=False)
        # 动作完成后清除目标信息
        right_arm.clear_pose_targets()
        # 确保没有剩余未完成动作在执行
        right_arm.stop()

    def left_arm_go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        left_arm = self.left_arm
        # 限制末端夹具运动
        left_joint_const = JointConstraint()
        left_joint_const.joint_name = "gripper_l_joint_r"
        if Leftfinger >= 0.025 :
            left_joint_const.position = 0.024
        else:
            left_joint_const.position = 0
        consts = Constraints()
        consts.joint_constraints = [left_joint_const]
        left_arm.set_path_constraints(consts)

        # 获取当前末端执行器位置姿态
        pose_goal = left_arm.get_current_pose().pose

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = Left_Qux
        pose_goal.orientation.y = Left_Quy
        pose_goal.orientation.z = Left_Quz
        pose_goal.orientation.w = Left_Quw
        pose_goal.position.x = (Neurondata[11]-0.05)*1.48+0.053
        pose_goal.position.y = (Neurondata[9]-0.18)*1.48+0.12
        pose_goal.position.z = (Neurondata[10]-0.53)*1.48+0.47
        left_arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # # 设置动作对象目标位置姿态
        # pose_goal.orientation.x = pose_goal.orientation.x
        # pose_goal.orientation.y = pose_goal.orientation.y
        # pose_goal.orientation.z = pose_goal.orientation.z
        # pose_goal.orientation.w = pose_goal.orientation.w
        # pose_goal.position.x = pose_goal.position.x
        # pose_goal.position.y = pose_goal.position.y - 0.01
        # pose_goal.position.z = pose_goal.position.z
        # left_arm.set_pose_target(pose_goal)
        # print "End effector pose %s" % pose_goal

        # 规划和输出动作
        traj = left_arm.plan()
        left_arm.execute(traj, wait=False)
        # 动作完成后清除目标信息
        left_arm.clear_pose_targets()
        # 确保没有剩余未完成动作在执行
        left_arm.stop()

    def both_arms_go_to_pose_goal(self):
        # 设置动作对象变量,此处为both_arms
        both_arms = self.both_arms
        # 获取当前各轴转角
        axis_angle = both_arms.get_current_joint_values()
        # print axis_angle
        # 获取当前末端执行器位置姿态
        right_pose_goal = both_arms.get_current_pose(end_effector_link="gripper_r_finger_r")
        left_pose_goal = both_arms.get_current_pose(end_effector_link="gripper_l_finger_r")
        print right_pose_goal
        # 限制末端夹具运动
        right_joint_const = JointConstraint()
        right_joint_const.joint_name = "gripper_r_joint_r"
        if Rightfinger >= 0.025 :
            right_joint_const.position = 0.024
        else:
            right_joint_const.position = 0
        left_joint_const = JointConstraint()
        left_joint_const.joint_name = "gripper_l_joint_r"
        if Leftfinger >= 0.025 :
            left_joint_const.position = 0.024
        else:
            left_joint_const.position = 0

        # 添加末端姿态约束:
        right_orientation_const = OrientationConstraint()
        right_orientation_const.header = Header()
        right_orientation_const.orientation = right_pose_goal.pose.orientation
        right_orientation_const.link_name = "gripper_r_joint_r"
        right_orientation_const.absolute_x_axis_tolerance = 0.6
        right_orientation_const.absolute_y_axis_tolerance = 0.6
        right_orientation_const.absolute_z_axis_tolerance = 0.6
        right_orientation_const.weight = 1
        
        left_orientation_const = OrientationConstraint()
        left_orientation_const.header = Header()
        left_orientation_const.orientation = left_pose_goal.pose.orientation
        left_orientation_const.link_name = "gripper_l_joint_r"
        left_orientation_const.absolute_x_axis_tolerance = 0.6
        left_orientation_const.absolute_y_axis_tolerance = 0.6
        left_orientation_const.absolute_z_axis_tolerance = 0.6
        left_orientation_const.weight = 1

        # 施加全约束 
        consts = Constraints()
        consts.joint_constraints = [right_joint_const, left_joint_const]
        # consts.orientation_constraints = [right_orientation_const, left_orientation_const]
        both_arms.set_path_constraints(consts)

        # # 设置动作对象目标位置姿态
        # # 右臂
        # right_pose_goal.pose.orientation.x = Right_Qux
        # right_pose_goal.pose.orientation.y = Right_Quy
        # right_pose_goal.pose.orientation.z = Right_Quz 
        # right_pose_goal.pose.orientation.w = Right_Quw
        # right_pose_goal.pose.position.x = Neurondata[5]
        # right_pose_goal.pose.position.y = Neurondata[3]
        # right_pose_goal.pose.position.z = Neurondata[4]
        # # 左臂
        # left_pose_goal.pose.orientation.x = Left_Qux
        # left_pose_goal.pose.orientation.y = Left_Quy
        # left_pose_goal.pose.orientation.z = Left_Quz 
        # left_pose_goal.pose.orientation.w = Left_Quw
        # left_pose_goal.pose.position.x = Neurondata[11]
        # left_pose_goal.pose.position.y = Neurondata[9]
        # left_pose_goal.pose.position.z = Neurondata[10]

        # # 右臂
        # right_pose_goal.pose.orientation.x = Right_Qux
        # right_pose_goal.pose.orientation.y = Right_Quy
        # right_pose_goal.pose.orientation.z = Right_Quz 
        # right_pose_goal.pose.orientation.w = Right_Quw
        # right_pose_goal.pose.position.x = (1266/740)*(Neurondata[5]+0.28)-0.495
        # right_pose_goal.pose.position.y = (1295/780)*(Neurondata[3]+0.56)-0.754
        # right_pose_goal.pose.position.z = (1355/776)*(Neurondata[4]-0.054)-0.184
        # # 左臂
        # left_pose_goal.pose.orientation.x = Left_Qux
        # left_pose_goal.pose.orientation.y = Left_Quy
        # left_pose_goal.pose.orientation.z = Left_Quz 
        # left_pose_goal.pose.orientation.w = Left_Quw
        # left_pose_goal.pose.position.x = (1266/850)*(Neurondata[11]+0.33)-0.495
        # left_pose_goal.pose.position.y = (1295/720)*(Neurondata[9]+0.19)-0.541
        # left_pose_goal.pose.position.z = (1355/745)*(Neurondata[10]-0.055)-0.184

        # 右臂
        right_pose_goal.pose.orientation.x = Right_Qux
        right_pose_goal.pose.orientation.y = Right_Quy
        right_pose_goal.pose.orientation.z = Right_Quz 
        right_pose_goal.pose.orientation.w = Right_Quw
        right_pose_goal.pose.position.x = (Neurondata[5]-0.05)*1.48+0.053
        right_pose_goal.pose.position.y = (Neurondata[3]+0.18)*1.48-0.12
        right_pose_goal.pose.position.z = (Neurondata[4]-0.53)*1.48+0.47
        # 左臂
        left_pose_goal.pose.orientation.x = Left_Qux
        left_pose_goal.pose.orientation.y = Left_Quy
        left_pose_goal.pose.orientation.z = Left_Quz
        left_pose_goal.pose.orientation.w = Left_Quw
        left_pose_goal.pose.position.x = (Neurondata[11]-0.05)*1.48+0.053
        left_pose_goal.pose.position.y = (Neurondata[9]-0.18)*1.48+0.12
        left_pose_goal.pose.position.z = (Neurondata[10]-0.53)*1.48+0.47

        # # 右臂
        # right_pose_goal.pose.orientation.x = right_pose_goal.pose.orientation.x
        # right_pose_goal.pose.orientation.y = right_pose_goal.pose.orientation.y
        # right_pose_goal.pose.orientation.z = right_pose_goal.pose.orientation.z
        # right_pose_goal.pose.orientation.w = right_pose_goal.pose.orientation.w
        # right_pose_goal.pose.position.x = right_pose_goal.pose.position.x
        # right_pose_goal.pose.position.y = right_pose_goal.pose.position.y
        # right_pose_goal.pose.position.z = right_pose_goal.pose.position.z
        # # 左臂
        # left_pose_goal.pose.orientation.x = left_pose_goal.pose.orientation.x
        # left_pose_goal.pose.orientation.y = left_pose_goal.pose.orientation.y
        # left_pose_goal.pose.orientation.z = left_pose_goal.pose.orientation.z
        # left_pose_goal.pose.orientation.w = left_pose_goal.pose.orientation.w
        # left_pose_goal.pose.position.x = left_pose_goal.pose.position.x
        # left_pose_goal.pose.position.y = left_pose_goal.pose.position.y
        # left_pose_goal.pose.position.z = left_pose_goal.pose.position.z

        # 设置动作组的两个目标点
        both_arms.set_pose_target(right_pose_goal, end_effector_link="gripper_r_finger_r")
        both_arms.set_pose_target(left_pose_goal, end_effector_link="gripper_l_finger_r")
        # 规划和输出动作
        traj = both_arms.plan()
        both_arms.execute(traj, wait=False)
        # # 清除路径约束
        both_arms.clear_path_constraints()
        # 确保输出停止
        both_arms.stop()

    def right_gripper_go_to_pose_goal(self):
        # 设置动作对象变量,此处为both_arms
        right_arm = self.right_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        # 设置末端关节目标值
        if Rightfinger > 0.025 :
            right_joint_goal[7] = 0.024
        else:
            right_joint_goal[7] = 0
        # 规划并执行路径动作
        right_arm.go(right_joint_goal, wait=False)
        right_arm.stop()

    def left_gripper_go_to_pose_goal(self):
        # 设置动作对象变量,此处为both_arms
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        left_joint_goal = left_arm.get_current_joint_values()
        # 设置末端关节目标值
        if Leftfinger > 0.025 :
            left_joint_goal[7] = 0.024
        else:
            left_joint_goal[7] = 0
        # 规划并执行路径动作
        left_arm.go(left_joint_goal, wait=False)
        left_arm.stop()


def main():

    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()

    # 循环等待,执行动作程序
    while 1:
        # 执行arm目标点动作
        # print "============ Press `Enter` to execute a right arm movement using a pose goal ..."
        # raw_input()
        # yumi.right_arm_go_to_pose_goal()
        
        # print "============ Press `Enter` to execute a left arm movement using a pose goal ..."
        # raw_input()
        # yumi.left_arm_go_to_pose_goal()

        print "============ Press `Enter` to execute both arms movement using a pose goal ..."
        raw_input()
        yumi.both_arms_go_to_pose_goal()

        # print "============ Press `Enter` to execute right gripper movement using a pose goal ..."
        # raw_input()
        # yumi.right_gripper_go_to_pose_goal()
        # print Rightfinger
        # print RightfingerT

        # print "============ Press `Enter` to execute left gripper movement using a pose goal ..."
        # raw_input()
        # yumi.left_gripper_go_to_pose_goal()
        # print Leftfinger
        # print LeftfingerT[]

if __name__ == '__main__':
    main()