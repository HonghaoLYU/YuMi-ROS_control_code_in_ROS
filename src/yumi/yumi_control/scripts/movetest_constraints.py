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
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import JointConstraint, JointLimits, Constraints
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 全局变量定义以及赋值
Neurondata = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


# 定义回调函数,订阅接受到的消息传给data
def callback(data):
    # 对全局变量a进行赋值
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
        right_arm = moveit_commander.MoveGroupCommander("right_arm")
        left_arm = moveit_commander.MoveGroupCommander("left_arm")

        # 创建 `DisplayTrajectory`_ publisher,稍后用于发布RViz可视化的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        # # 获取机器人的参考坐标系并输出
        # planning_frame = arm.get_planning_frame()
        # print "============ Reference frame: %s" % planning_frame

        # # 获取当前末端执行器并输出
        # eef_link = arm.get_end_effector_link()
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
        # self.gripper = gripper
        self.display_trajectory_publisher = display_trajectory_publisher
        # self.planning_frame = planning_frame
        # self.eef_link = eef_link
        self.group_names = group_names

    # def go_to_joint_state(self):
    #     # 设置动作对象变量,此处为arm
    #     arm = self.arm
    #     # 获取当前目标点关节状态
    #     joint_goal = arm.get_current_joint_values()
    #     if count == 1:
    #         joint_goal[0] = 0
    #         joint_goal[1] = -pi/4
    #         joint_goal[2] = 0
    #         joint_goal[3] = -pi/2
    #         joint_goal[4] = 0
    #         joint_goal[5] = pi/3
    #         joint_goal[6] = 0
    #     else:
    #         joint_goal[0] = 0
    #         joint_goal[1] = -pi/4
    #         joint_goal[2] = 0
    #         joint_goal[3] = -pi/2
    #         joint_goal[4] = 0
    #         joint_goal[5] = pi/3
    #         joint_goal[6] = 0

    #     # 规划并执行路径动作
    #     arm.go(joint_goal, wait=True)

    #     # 调用 stop() 命令，确保动作停止
    #     arm.stop()

    def right_arm_go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        right_arm = self.right_arm

        # Create a path constraint for the arm
        # UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
        joint_const = JointConstraint()
        joint_const.joint_name = "gripper_r_joint_r"
        joint_const.position = 0
        consts = Constraints()
        consts.joint_constraints = [joint_const]
        right_arm.set_path_constraints(consts)

        # 获取当前末端执行器位置姿态
        pose_goal = right_arm.get_current_pose().pose
        # print (a)
        # print (Qux, Quy, Quz, Quw)
        # 设置动作对象目标位置姿态
        # pose_goal.orientation.x = Right_Qux
        # pose_goal.orientation.y = Right_Quy
        # pose_goal.orientation.z = Right_Quz
        # pose_goal.orientation.w = Right_Quw
        # pose_goal.position.x = Neurondata[5]
        pose_goal.position.y = pose_goal.position.y - 0.01
        # pose_goal.position.z = Neurondata[4]
        right_arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

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

        # 获取当前末端执行器位置姿态
        pose_goal = left_arm.get_current_pose().pose

        # print (a)
        # print (Qux, Quy, Quz, Quw)

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = Left_Qux
        pose_goal.orientation.y = Left_Quy
        pose_goal.orientation.z = Left_Quz
        pose_goal.orientation.w = Left_Quw
        pose_goal.position.x = Neurondata[11]
        pose_goal.position.y = Neurondata[9]
        pose_goal.position.z = Neurondata[10]
        left_arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # 规划和输出动作
        traj = left_arm.plan()
        left_arm.execute(traj, wait=False)
        # 动作完成后清除目标信息
        left_arm.clear_pose_targets()
        # 确保没有剩余未完成动作在执行
        left_arm.stop()

    # def go_to_gripper_goal(self):
    #     # 设置动作对象变量，此处为gripper
    #     gripper = self.gripper

    #     # 获取当前gripper姿态信息
    #     gripper_goal = self.gripper.get_joint_value_target()
    #     print "Gripper pose %s" % gripper_goal

    #     # 设置gripper目标姿态
    #     gripper.set_joint_value_target([0.02, 0.02])

    #     # 规划和输出动作
    #     gripper.go(wait=True)

    #     # 确保没有剩余未完成动作在执行
    #     gripper.stop()
    #     # 动作完成后清除目标信息
    #     gripper.clear_pose_targets()

    # def go_to_gripper_joint_goal(self):
    #     # 设置动作对象变量,此处为arm
    #     arm = self.arm
    #     # 获取当前目标点关节状态
    #     joint_goal = arm.get_current_joint_values()
    #     joint_goal[7] = joint_goal[7] + 0.001

    #     # 规划并执行路径动作
    #     arm.go(joint_goal, wait=True)

    #     # 调用 stop() 命令，确保动作停止
    #     arm.stop()

    def go_to_home_goal(self):
        # 控制机械臂回到初始化位置
        arm = self.arm
        arm.set_max_acceleration_scaling_factor(0.0002)
        arm.set_max_velocity_scaling_factor(0.0002)
        arm.set_named_target('home')
        traj = arm.plan()
        arm.execute(traj)
        # arm.go()

    def go_to_calc_goal(self):
        # 控制机械臂回到校准位置
        arm = self.arm
        arm.set_named_target('calc')
        arm.go(wait=False)

    def go_to_ready_goal(self):
        # 控制机械臂回到校准位置
        arm = self.arm
        arm.set_named_target('ready')
        arm.go()


def main():

    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()

    # 循环等待,执行动作程序
    while 1:
        # 执行arm目标点动作
        print "============ Press `Enter` to execute a right arm movement using a pose goal ..."
        raw_input()
        yumi.right_arm_go_to_pose_goal()
        time.sleep(0.2)
        # print "============ Press `Enter` to execute a left arm movement using a pose goal ..."
        # raw_input()
        # yumi.left_arm_go_to_pose_goal()

if __name__ == '__main__':
    main()
