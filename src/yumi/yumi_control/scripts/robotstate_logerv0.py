#!/usr/bin/env python
# coding=UTF-8
# Author: Honghao Lv
# Note: 记录机器人末端位姿及接收到的neuron端数据
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
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 全局变量定义以及赋值
a = [0, 0, 0, 0, 0, 0]


# 定义回调函数,订阅接受到的消息传给data
def callback(data):
    # 对全局变量a进行赋值
    global a
    global W_Eux, W_Euy, W_Euz
    global Qux, Quy, Quz, Quw
    a = data.data
    W_Euy = a[0]
    W_Eux = a[1]
    W_Euz = a[2]
    (Qux, Quy, Quz, Quw) = quaternion_from_euler(W_Euz, -W_Eux, W_Euy)


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        # 初始化 `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',anonymous=True)

        # 订阅话题
        rospy.Subscriber('yumiaction', Float64MultiArray, callback)

        # 实例化 a `RobotCommander`_ 对象.
        robot = moveit_commander.RobotCommander()

        # 实例化 a `PlanningSceneInterface`_ 对象.
        scene = moveit_commander.PlanningSceneInterface()

        # 实例化 a `MoveGroupCommander`_ 对象.
        group_name = "left_arm"
        arm = moveit_commander.MoveGroupCommander(group_name)

        # 创建 `DisplayTrajectory`_ publisher,稍后用于发布RViz可视化的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        # 获取机器人的参考坐标系并输出
        planning_frame = arm.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # 获取当前末端执行器并输出
        eef_link = arm.get_end_effector_link()
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
        self.arm = arm
        # self.gripper = gripper
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        # 获取当前目标点关节状态
        joint_goal = arm.get_current_joint_values()
        if count == 1:
            joint_goal[0] = 0
            joint_goal[1] = -pi/4
            joint_goal[2] = 0
            joint_goal[3] = -pi/2
            joint_goal[4] = 0
            joint_goal[5] = pi/3
            joint_goal[6] = 0
        else:
            joint_goal[0] = 0
            joint_goal[1] = -pi/4
            joint_goal[2] = 0
            joint_goal[3] = -pi/2
            joint_goal[4] = 0
            joint_goal[5] = pi/3
            joint_goal[6] = 0

        # 规划并执行路径动作
        arm.go(joint_goal, wait=True)

        # 调用 stop() 命令，确保动作停止
        arm.stop()

    def echo_robot_pose(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        # arm.set_end_effector_link("yumi_link_7_r")
        # 获取当前末端执行器位置姿态和四元数
        pose_goal = arm.get_current_pose().pose
        rpy = arm.get_current_rpy()
        ender = arm.get_end_effector_link()
        framer = arm.get_planning_frame()
        reframer = arm.get_pose_reference_frame()
        (Eux, Euy, Euz) = euler_from_quaternion(
            [-pose_goal.orientation.x, -pose_goal.orientation.y, -pose_goal.orientation.z, -pose_goal.orientation.w])

        print ender
        print framer
        print reframer
        print "YuMi末端执行器四元数 %s"
        print (-pose_goal.orientation.x, -pose_goal.orientation.y, -pose_goal.orientation.z, -pose_goal.orientation.w)
        print (Eux, Euy, Euz)
        print "YuMi末端执行器欧拉角(度) %s"
        print rpy
        print (rpy[0] * 180 / 3.14, rpy[1] * 180 / 3.14, rpy[2] * 180 / 3.14)
        print (pose_goal)

        # 清除目标信息
        arm.clear_pose_targets()

    def echo_neuron_pose(self):
        # 显示从可穿戴设备得到的位姿
        print "可穿戴设备原始数据 %s"
        print (a)
        print "可穿戴设备欧拉角转换后 %s"
        print (a[0] * 180 / 3.14, a[1] * 180 / 3.14, a[2] * 180 / 3.14)
        print (a[2] * 180 / 3.14, a[0] * 180 / 3.14, a[1] * 180 / 3.14)

    def log_robot_pose(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        pose_goal = arm.get_current_pose().pose
        rpy = arm.get_current_rpy()

        # 记录数据
        sheet1.write(count, 0, count)
        sheet1.write(count, 1, pose_goal.position.x)
        sheet1.write(count, 2, pose_goal.position.y)
        sheet1.write(count, 3, pose_goal.position.z)
        sheet1.write(count, 4, rpy[0])
        sheet1.write(count, 5, rpy[1])
        sheet1.write(count, 6, rpy[2])
        sheet1.write(count, 7, pose_goal.orientation.x)
        sheet1.write(count, 8, pose_goal.orientation.y)
        sheet1.write(count, 9, pose_goal.orientation.z)
        sheet1.write(count, 10, pose_goal.orientation.w)
        sheet1.write(count, 11, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

        wbk.save('test4.xls')

    def log_neuron_pose(self):
        # 记录数据
        sheet2.write(count, 0, count)
        sheet2.write(count, 1, a[5])
        sheet2.write(count, 2, a[3])
        sheet2.write(count, 3, a[4])
        sheet2.write(count, 4, W_Euz)
        sheet2.write(count, 5, -W_Eux)
        sheet2.write(count, 6, W_Euy)
        sheet2.write(count, 7, 'none')
        sheet2.write(count, 8, 'none')
        sheet2.write(count, 9, 'none')
        sheet2.write(count, 10, 'none')
        sheet2.write(count, 11, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

        wbk.save('test4.xls')

    def go_to_gripper_goal(self):
        # 设置动作对象变量，此处为gripper
        gripper = self.gripper

        # 获取当前gripper姿态信息
        gripper_goal = self.gripper.get_joint_value_target()
        print "Gripper pose %s" % gripper_goal

        # 设置gripper目标姿态
        gripper.set_joint_value_target([0.02, 0.02])

        # 规划和输出动作
        gripper.go(wait=True)
        # 确保没有剩余未完成动作在执行
        gripper.stop()
        # 动作完成后清除目标信息
        gripper.clear_pose_targets()

    def go_to_gripper_joint_goal(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        # 获取当前目标点关节状态
        joint_goal = arm.get_current_joint_values()
        joint_goal[7] = joint_goal[7] + 0.001

        # 规划并执行路径动作
        arm.go(joint_goal, wait=True)

        # 调用 stop() 命令，确保动作停止
        arm.stop()

    def go_to_home_goal(self):
        # 控制机械臂回到初始化位置
        arm = self.arm
        arm.set_named_target('home')
        arm.go()

    def go_to_calc_goal(self):
        # 控制机械臂回到校准位置
        arm = self.arm
        arm.set_named_target('calc')
        arm.go()

    def go_to_ready_goal(self):
        # 控制机械臂回到校准位置
        arm = self.arm
        arm.set_named_target('ready')
        arm.go()


def main():
    # 定义全局变量count,判断程序执行次数
    global count
    count = 0

    # # 初始化记录数据表
    global wbk, sheet1, sheet2
    wbk = xlwt.Workbook()
    # 表1用于记录机器人状态
    sheet1 = wbk.add_sheet('Robot state')
    # 表2用于记录Neuron数据状态
    sheet2 = wbk.add_sheet('Neuron')
    # 填写表头
    sheet1.write(0, 0, 'Num')
    sheet1.write(0, 1, 'x')
    sheet1.write(0, 2, 'y')
    sheet1.write(0, 3, 'z')
    sheet1.write(0, 4, 'Eur')
    sheet1.write(0, 5, 'Eup')
    sheet1.write(0, 6, 'Euy')
    sheet1.write(0, 7, 'Qux')
    sheet1.write(0, 8, 'Quy')
    sheet1.write(0, 9, 'Quz')
    sheet1.write(0, 10, 'Quw')

    sheet2.write(0, 0, 'Num')
    sheet2.write(0, 1, 'x')
    sheet2.write(0, 2, 'y')
    sheet2.write(0, 3, 'z')
    sheet2.write(0, 4, 'Eur')
    sheet2.write(0, 5, 'Eup')
    sheet2.write(0, 6, 'Euy')
    sheet2.write(0, 7, 'Qux')
    sheet2.write(0, 8, 'Quy')
    sheet2.write(0, 9, 'Quz')
    sheet2.write(0, 10, 'Quw')

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
        yumi.echo_robot_pose()
        # yumi.echo_neuron_pose()
        # yumi.log_robot_pose()
        # yumi.log_neuron_pose()


if __name__ == '__main__':
    main()