#!/usr/bin/env python
# coding=UTF-8
# Author: Honghao Lv
# Note: 记录机器人末端位姿及接收到的neuron端数据,相比V0要删掉许多冗杂的东西，最高采样频率稍微提高

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import xlwt
import time
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
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
        # 初始化 `moveit_commander`_ and a `rospy`_ 节点.
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
        # 创建 `DisplayTrajectory`_ publisher,稍后用于发布RViz可视化的轨迹.
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        # # 获取机器人的参考坐标系并输出.
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

    def echo_robot_pose(self):
        # 设置动作对象变量,此处为both_arms
        both_arms = self.both_arms
        # 获取当前末端执行器位置姿态和四元数
        right_pose_goal = both_arms.get_current_pose(end_effector_link="gripper_r_finger_r")
        left_pose_goal = both_arms.get_current_pose(end_effector_link="gripper_l_finger_r")
        # 获取当前末端执行器位置欧拉角
        right_rpy = both_arms.get_current_rpy(end_effector_link="gripper_r_finger_r")
        left_rpy = both_arms.get_current_rpy(end_effector_link="gripper_l_finger_r")
        # 打印输出
        print "YuMi末端执行器四元数 %s"
        print right_pose_goal
        print left_pose_goal
        print "YuMi末端执行器欧拉角(度) %s"
        print right_rpy
        print left_rpy
        # 清除目标信息
        both_arms.clear_pose_targets()

    def echo_robot_joint(self):
        # 设置动作对象变量,此处为both_arms
        right_arm = self.right_arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        left_joint_goal = left_arm.get_current_joint_values()
        # 打印输出
        print "YuMi右臂关节角 %s"
        print right_joint_goal
        print "YuMi左臂关节角 %s"
        print left_joint_goal

    def echo_neuron_pose(self):
        # 显示从可穿戴设备得到的位姿
        print "可穿戴设备原始数据 %s"
        print (Neurondata)
        print "可穿戴设备右手欧拉角转换后 %s"
        print (Neurondata[1] * 180 / 3.14, Neurondata[0] * 180 / 3.14, Neurondata[2] * 180 / 3.14)
        print "可穿戴设备左手欧拉角转换后 %s"
        print (Neurondata[7] * 180 / 3.14, Neurondata[6] * 180 / 3.14, Neurondata[8] * 180 / 3.14)

    def echo_glove_pose(self):
        # 显示从可穿戴设备得到的位姿
        print "手套原始数据 右手食指 %s"
        print (Rightfinger)
        print "手套原始数据 左手食指 %s"
        print (Leftfinger)
        print "手套原始数据 右手大拇指 %s"
        print (RightfingerT)
        print "手套原始数据 左手大拇指 %s"
        print (LeftfingerT)

    def log_robot_pose(self):
        # 设置动作对象变量,此处为both_arms
        both_arms = self.both_arms
        # 获取当前末端执行器位置姿态和四元数
        right_pose_goal = both_arms.get_current_pose(end_effector_link="gripper_r_finger_r").pose
        left_pose_goal = both_arms.get_current_pose(end_effector_link="gripper_l_finger_r").pose
        # 获取当前末端执行器位置欧拉角
        right_rpy = both_arms.get_current_rpy(end_effector_link="gripper_r_finger_r")
        left_rpy = both_arms.get_current_rpy(end_effector_link="gripper_l_finger_r")
        # 记录右臂数据
        sheet1.write(count, 0, count)
        sheet1.write(count, 1, right_pose_goal.position.x)
        sheet1.write(count, 2, right_pose_goal.position.y)
        sheet1.write(count, 3, right_pose_goal.position.z)
        sheet1.write(count, 4, right_rpy[0])
        sheet1.write(count, 5, right_rpy[1])
        sheet1.write(count, 6, right_rpy[2])
        # sheet1.write(count, 7, right_pose_goal.orientation.x)
        # sheet1.write(count, 8, right_pose_goal.orientation.y)
        # sheet1.write(count, 9, right_pose_goal.orientation.z)
        # sheet1.write(count, 10, right_pose_goal.orientation.w)
        # 记录左臂数据
        sheet1.write(count, 11, left_pose_goal.position.x)
        sheet1.write(count, 12, left_pose_goal.position.y)
        sheet1.write(count, 13, left_pose_goal.position.z)
        sheet1.write(count, 14, left_rpy[0])
        sheet1.write(count, 15, left_rpy[1])
        sheet1.write(count, 16, left_rpy[2])
        # sheet1.write(count, 17, left_pose_goal.orientation.x)
        # sheet1.write(count, 18, left_pose_goal.orientation.y)
        # sheet1.write(count, 19, left_pose_goal.orientation.z)
        # sheet1.write(count, 20, left_pose_goal.orientation.w)
        sheet1.write(count, 21, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

    def log_robot_joint(self):
        # 设置动作对象变量,此处为both_arms
        right_arm = self.right_arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        left_joint_goal = left_arm.get_current_joint_values()
        # 记录数据
        sheet4.write(count, 1, right_joint_goal[0])
        sheet4.write(count, 2, right_joint_goal[1])
        sheet4.write(count, 3, right_joint_goal[2])
        sheet4.write(count, 4, right_joint_goal[3])
        sheet4.write(count, 5, right_joint_goal[4])
        sheet4.write(count, 6, right_joint_goal[5])
        sheet4.write(count, 7, right_joint_goal[6])
        sheet4.write(count, 8, right_joint_goal[7])
        sheet4.write(count, 9, left_joint_goal[0])
        sheet4.write(count, 10, left_joint_goal[1])
        sheet4.write(count, 11, left_joint_goal[2])
        sheet4.write(count, 12, left_joint_goal[3])
        sheet4.write(count, 13, left_joint_goal[4])
        sheet4.write(count, 14, left_joint_goal[5])
        sheet4.write(count, 15, left_joint_goal[6])
        sheet4.write(count, 16, left_joint_goal[7])
        sheet4.write(count, 17, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

    def log_neuron_pose(self):
        # 记录数据
        sheet2.write(count, 0, count)
        sheet2.write(count, 1, Neurondata[5])
        sheet2.write(count, 2, Neurondata[3])
        sheet2.write(count, 3, Neurondata[4])
        sheet2.write(count, 4, Right_Euz)
        sheet2.write(count, 5, -Right_Eux)
        sheet2.write(count, 6, Right_Euy)
        # sheet2.write(count, 7, 'none')
        # sheet2.write(count, 8, 'none')
        # sheet2.write(count, 9, 'none')
        # sheet2.write(count, 10, 'none')
        sheet2.write(count, 11, Neurondata[11])
        sheet2.write(count, 12, Neurondata[9])
        sheet2.write(count, 13, Neurondata[10])
        sheet2.write(count, 14, Left_Euz)
        sheet2.write(count, 15, -Left_Eux)
        sheet2.write(count, 16, Left_Euy)
        # sheet2.write(count, 17, 'none')
        # sheet2.write(count, 18, 'none')
        # sheet2.write(count, 19, 'none')
        # sheet2.write(count, 20, 'none')
        sheet2.write(count, 21, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

    def log_glove_pose(self):
        # 记录数据
        sheet3.write(count, 0, count)
        sheet3.write(count, 1, Rightfinger)
        sheet3.write(count, 2, RightfingerT)
        sheet3.write(count, 3, Leftfinger)
        sheet3.write(count, 4, LeftfingerT)


def main():
    # 定义全局变量count,判断程序执行次数
    global count
    count = 1
    # 初始化记录数据表
    global wbk, sheet1, sheet2, sheet3, sheet4
    wbk = xlwt.Workbook()
    # 表1用于记录机器人状态
    sheet1 = wbk.add_sheet('Robot Pose')
    # 表2用于记录Neuron数据状态
    sheet2 = wbk.add_sheet('Neuron')
    # 表3用于记录Glove数据状态
    sheet3 = wbk.add_sheet('Glove')
    # 表4用于记录机器人关节数据状态
    sheet4 = wbk.add_sheet('Robot Joint')
    # 填写表头
    # sheet1 第一行
    sheet1.write_merge(0, 1, 0, 0, 'Num')
    sheet1.write_merge(0, 0, 1, 10, 'Right')
    sheet1.write_merge(0, 0, 11, 20, 'Left')
    # sheet1 第二行
    sheet1.write(1, 1, 'x')
    sheet1.write(1, 2, 'y')
    sheet1.write(1, 3, 'z')
    sheet1.write(1, 4, 'Eur')
    sheet1.write(1, 5, 'Eup')
    sheet1.write(1, 6, 'Euy')
    sheet1.write(1, 7, 'Qux')
    sheet1.write(1, 8, 'Quy')
    sheet1.write(1, 9, 'Quz')
    sheet1.write(1, 10, 'Quw')
    sheet1.write(1, 11, 'x')
    sheet1.write(1, 12, 'y')
    sheet1.write(1, 13, 'z')
    sheet1.write(1, 14, 'Eur')
    sheet1.write(1, 15, 'Eup')
    sheet1.write(1, 16, 'Euy')
    sheet1.write(1, 17, 'Qux')
    sheet1.write(1, 18, 'Quy')
    sheet1.write(1, 19, 'Quz')
    sheet1.write(1, 20, 'Quw')
    # sheet2 第一行
    sheet2.write_merge(0, 1, 0, 0, 'Num')
    sheet2.write_merge(0, 0, 1, 10, 'Right')
    sheet2.write_merge(0, 0, 11, 20, 'Left')
    # sheet2 第二行
    sheet2.write(1, 1, 'x')
    sheet2.write(1, 2, 'y')
    sheet2.write(1, 3, 'z')
    sheet2.write(1, 4, 'Eur')
    sheet2.write(1, 5, 'Eup')
    sheet2.write(1, 6, 'Euy')
    sheet2.write(1, 7, 'Qux')
    sheet2.write(1, 8, 'Quy')
    sheet2.write(1, 9, 'Quz')
    sheet2.write(1, 10, 'Quw')
    sheet2.write(1, 11, 'x')
    sheet2.write(1, 12, 'y')
    sheet2.write(1, 13, 'z')
    sheet2.write(1, 14, 'Eur')
    sheet2.write(1, 15, 'Eup')
    sheet2.write(1, 16, 'Euy')
    sheet2.write(1, 17, 'Qux')
    sheet2.write(1, 18, 'Quy')
    sheet2.write(1, 19, 'Quz')
    sheet2.write(1, 20, 'Quw')
    # sheet3 第一行
    sheet3.write_merge(0, 1, 0, 0, 'Num')
    sheet3.write_merge(0, 0, 1, 2, 'Right')
    sheet3.write_merge(0, 0, 3, 4, 'Left')
    # sheet3 第二行
    sheet3.write(1, 1, 'I')
    sheet3.write(1, 2, 'T')
    sheet3.write(1, 3, 'I')
    sheet3.write(1, 4, 'T')
    # sheet4 第一行
    sheet4.write_merge(0, 1, 0, 0, 'Num')
    sheet4.write_merge(0, 0, 1, 8, 'Right')
    sheet4.write_merge(0, 0, 9, 16, 'Left')
    # sheet4 第二行
    sheet4.write(1, 1, '1')
    sheet4.write(1, 2, '2')
    sheet4.write(1, 3, '3(7)')
    sheet4.write(1, 4, '4(3)')
    sheet4.write(1, 5, '5(4)')
    sheet4.write(1, 6, '6(5)')
    sheet4.write(1, 7, '7(6)')
    sheet4.write(1, 8, 'gripper')
    sheet4.write(1, 9, '1')
    sheet4.write(1, 10, '2')
    sheet4.write(1, 11, '3(7)')
    sheet4.write(1, 12, '4(3)')
    sheet4.write(1, 13, '5(4)')
    sheet4.write(1, 14, '6(5)')
    sheet4.write(1, 15, '7(6)')
    sheet4.write(1, 16, 'gripper')
    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()
    # 循环输出
    while 1:
        count = count + 1
        # 显示及记录
        print "============ Press `Enter` to execute a arm movement using a pose goal ..."
        raw_input()
        # yumi.echo_robot_pose()
        yumi.echo_robot_joint()
        # yumi.echo_neuron_pose()
        # yumi.log_robot_pose()
        # yumi.log_neuron_pose()
        # yumi.log_robot_joint()
        # wbk.save('test2.xls')


if __name__ == '__main__':
    main()