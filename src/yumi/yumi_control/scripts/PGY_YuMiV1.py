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
from std_msgs.msg import String, Float32
from std_msgs.msg import Float64MultiArray
from datamessage.msg import bend
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

a = 0

# 定义回调函数,订阅接受到的消息传给data
def callback(data):
    # 对全局变量a进行赋值
    global a
    a = data.data
    print a
    

class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        # 初始化 `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',anonymous=True)

        # 订阅话题
        rospy.Subscriber('grip', Float32, callback)

        # 实例化 a `RobotCommander`_ 对象.
        robot = moveit_commander.RobotCommander()

        # 实例化 a `PlanningSceneInterface`_ 对象.
        scene = moveit_commander.PlanningSceneInterface()

        # 实例化 a `MoveGroupCommander`_ 对象.
        right_arm = moveit_commander.MoveGroupCommander("right_arm")
        left_arm = moveit_commander.MoveGroupCommander("left_arm")

        # 创建 `DisplayTrajectory`_ publisher,稍后用于发布RViz可视化的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        # 获取机器人的参考坐标系并输出
        planning_frame = right_arm.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # 获取当前末端执行器并输出
        eef_link = right_arm.get_end_effector_link()
        print "============ End effector: %s" % eef_link

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
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # 设置动作对象变量
        right_arm = self.right_arm
        # 获取当前目标点关节状态
        joint_goal = right_arm.get_current_joint_values()
        if count == 1:
            joint_goal[0] = 0
            joint_goal[1] = -pi / 4
            joint_goal[2] = 0
            joint_goal[3] = -pi / 2
            joint_goal[4] = 0
            joint_goal[5] = pi / 3
            joint_goal[6] = 0
        else:
            joint_goal[0] = 0
            joint_goal[1] = -pi / 4
            joint_goal[2] = 0
            joint_goal[3] = -pi / 2
            joint_goal[4] = 0
            joint_goal[5] = pi / 3
            joint_goal[6] = 0

        # 规划并执行路径动作
        right_arm.go(joint_goal, wait=True)

        # 调用 stop() 命令，确保动作停止
        right_arm.stop()

    def right_arm_go_to_pose_goal(self):
        # 设置动作对象变量
        right_arm = self.right_arm
        eef_link = right_arm.get_end_effector_link()
        print "============ End effector: %s" % eef_link
        right_arm.set_end_effector_link("yumi_link_7_r")
        eef_link = right_arm.get_end_effector_link()
        print "============ End effector: %s" % eef_link
        # 获取当前末端执行器位置姿态
        pose_goal = self.right_arm.get_current_pose().pose

        print (pose_goal)

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = pose_goal.orientation.x
        pose_goal.orientation.y = pose_goal.orientation.y
        pose_goal.orientation.z = pose_goal.orientation.z
        pose_goal.orientation.w = pose_goal.orientation.w
        pose_goal.position.x = pose_goal.position.x
        pose_goal.position.y = pose_goal.position.y
        pose_goal.position.z = pose_goal.position.z
        right_arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # 规划和输出动作
        right_arm.go(wait=True)
        # 确保没有剩余未完成动作在执行
        right_arm.stop()
        # 动作完成后清除目标信息
        right_arm.clear_pose_targets()

    #### 夹具张开到达/闭合到指定目标goal
    def right_gripper_go_to_goal(self):
        # 设置动作对象变量
        right_arm = self.right_arm
        print (a)
        # 获取当前目标点关节状态
        joint_goal = right_arm.get_current_joint_values()
        joint_goal[7] = joint_goal[7] - 0.0005
        # 规划并执行路径动作
        right_arm.go(joint_goal, wait=False)
        # 动作完成后清除目标信息
        right_arm.clear_pose_targets()
        # 调用 stop() 命令，确保动作停止
        right_arm.stop()

    def left_gripper_go_to_goal(self):
        # 设置动作对象变量
        left_arm = self.left_arm
        print (b)
        # 获取当前目标点关节状态
        joint_goal = left_arm.get_current_joint_values()
        if b > 0.025 :
            joint_goal[7] = 0.024
        else:
            joint_goal[7] = 0
        # 规划并执行路径动作
        left_arm.go(joint_goal, wait=False)
        # 动作完成后清除目标信息
        left_arm.clear_pose_targets()
        # 调用 stop() 命令，确保动作停止
        left_arm.stop()

    #### 几个预先定义的测试及校准位置
    def right_arm_go_to_home_goal(self):
        # 控制机械臂回到初始化位置
        right_arm = self.right_arm
        right_arm.set_named_target('home')
        right_arm.go(wait=False)

    def right_arm_go_to_cal_goal(self):
        # 控制机械臂回到校准位置
        right_arm = self.right_arm
        right_arm.set_named_target('calc')
        right_arm.go(wait=False)

    def right_arm_go_to_ready_goal(self):
        # 控制机械臂回到校准位置
        right_arm = self.right_arm
        right_arm.set_named_target('ready')
        right_arm.go(wait=False)

    def left_arm_go_to_home_goal(self):
        # 控制机械臂回到初始化位置
        left_arm = self.left_arm
        left_arm.set_named_target('home')
        left_arm.go(wait=False)

    def left_arm_go_to_cal_goal(self):
        # 控制机械臂回到校准位置
        left_arm = self.left_arm
        left_arm.set_named_target('calc')
        left_arm.go(wait=False)

    def left_arm_go_to_ready_goal(self):
        # 控制机械臂回到校准位置
        left_arm = self.left_arm
        left_arm.set_named_target('ready')
        left_arm.go(wait=False)

    def right_arm_go_to_joint_goal(self, jointnum):
        # 设置动作对象变量,此处为right_arm
        right_arm = self.right_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        armnum = 'right_arm'
        # 设置末端关节目标值
        right_joint_goal[0] = return_joint_state(armnum,jointnum)[0]
        right_joint_goal[1] = return_joint_state(armnum,jointnum)[1]
        right_joint_goal[2] = return_joint_state(armnum,jointnum)[2]
        right_joint_goal[3] = return_joint_state(armnum,jointnum)[3]
        right_joint_goal[4] = return_joint_state(armnum,jointnum)[4]
        right_joint_goal[5] = return_joint_state(armnum,jointnum)[5]
        right_joint_goal[6] = return_joint_state(armnum,jointnum)[6]
        print "End effector joint goal %s" % right_joint_goal
        # 规划并执行路径动作
        right_arm.go(right_joint_goal, wait=False)
        right_arm.clear_pose_targets()
        right_arm.stop()

    def left_arm_go_to_joint_goal1(self):
        # 设置动作对象变量,此处为right_arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        left_joint_goal = left_arm.get_current_joint_values()
        armnum = 'right_arm'
        # 设置末端关节目标值
        left_joint_goal[0] = left_joint_goal[0] - 0.15
        print "End effector joint goal %s" % left_joint_goal
        # 规划并执行路径动作
        left_arm.go(left_joint_goal, wait=False)
        left_arm.clear_pose_targets()
        left_arm.stop()


def main():
    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()
    # 循环等待,执行动作程序
    while 1:
        # if a == 1 :
            print "============ Press `Enter` to  open right gripper 0.001m..."
            raw_input()
            yumi.left_arm_go_to_joint_goal1()
            time.sleep(0.5)
        # if a == 2 :
            print "============ Press `Enter` to  open right gripper 0.001m..."
            raw_input()
            yumi.left_arm_go_to_joint_goal1()
            time.sleep(0.5)
        # if a == 3 :
            print "============ Press `Enter` to  open right gripper 0.001m..."
            raw_input()
            yumi.left_arm_go_to_joint_goal1()
            time.sleep(0.5)
            # yumi.left_gripper_go_to_goal()
            # time.sleep(0.15)
            # print "============ Press `Enter` to  right arm go to pose goal..."
            # raw_input()
            # yumi.right_arm_go_to_pose_goal()


if __name__ == '__main__':
    main()
