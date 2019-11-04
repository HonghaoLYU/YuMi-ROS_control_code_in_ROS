#!/usr/bin/env python
# coding=UTF-8
# Author: Honghao Lv limit the worksapce for small motion step control

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
from moveit_msgs.msg import JointConstraint, OrientationConstraint, PositionConstraint, JointLimits, Constraints
from datamessage.msg import bend
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from joint_state import return_joint_state

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

        # 获取机器人的参考坐标系并输出
        planning_frame = both_arms.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

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
        self.planning_frame = planning_frame
        # self.eef_link = eef_link
        self.group_names = group_names

    def set_scene(self):
        # 设置规划场景对象
        scene = self.scene
        planning_frame = self.planning_frame
        # 设置场景物体的名称
        top_id = 'top'
        top1_id = 'top1'
        box1_id = 'box1'
        box2_id = 'box2'
        box3_id = 'box3'
        box_side1_id = 'box_side1'
        box_side2_id = 'box_side2'
        box_side3_id = 'box_side3'
        box_side4_id = 'box_side4'
        desk_id = 'desk'
        cover1_id = 'cover1'
        cover2_id = 'cover2'
        cover3_id = 'cover3'
        # 设置约束形状的三维尺寸
        top_size = [0.05, 1.01, 0.05]
        top1_size = [0.05, 0.8, 0.05]
        box1_size = [0.05, 0.05, 0.7]
        box2_size = [0.05, 0.05, 0.7]
        box3_size = [0.05, 0.05, 0.7]
        box_side1_size = [0.01, 0.3, 0.1]
        box_side2_size = [0.3, 0.01, 0.1]
        box_side3_size = [0.01, 0.3, 0.1]
        box_side4_size = [0.3, 0.01, 0.1]
        desk_size = [1.5, 0.8, 0.01]
        cover1_size = [0.001, 0.8, 0.1]
        cover2_size = [0.3, 0.8, 0.001]
        cover3_size = [0.2, 0.55, 0.001]
        # 将竖直约束和顶部约束加入场景当中
        top_pose = PoseStamped()
        top_pose.header.frame_id = planning_frame
        top_pose.pose.position.x = 0.25
        top_pose.pose.position.y = 0.0
        top_pose.pose.position.z = 0.6
        top_pose.pose.orientation.w = 1.0
        # scene.add_box(top_id, top_pose, top_size)
        top1_pose = PoseStamped()
        top1_pose.header.frame_id = planning_frame
        top1_pose.pose.position.x = 0.55
        top1_pose.pose.position.y = 0.0
        top1_pose.pose.position.z = 0.48
        top1_pose.pose.orientation.w = 1.0
        # scene.add_box(top1_id, top1_pose, top1_size)
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = planning_frame
        box1_pose.pose.position.x = 0.25
        box1_pose.pose.position.y = 0
        box1_pose.pose.position.z = 0.25
        box1_pose.pose.orientation.w = 1.0   
        # scene.add_box(box1_id, box1_pose, box1_size)
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = planning_frame
        box2_pose.pose.position.x = 0.25
        box2_pose.pose.position.y = -0.48
        box2_pose.pose.position.z = 0.25
        box2_pose.pose.orientation.w = 1.0   
        # scene.add_box(box2_id, box2_pose, box2_size)
        box3_pose = PoseStamped()
        box3_pose.header.frame_id = planning_frame
        box3_pose.pose.position.x = 0.25
        box3_pose.pose.position.y = 0.48
        box3_pose.pose.position.z = 0.25
        box3_pose.pose.orientation.w = 1.0   
        # scene.add_box(box3_id, box3_pose, box3_size)
        # 添加桌面和基座包覆到规划场景当中
        cover1_pose = PoseStamped()
        cover1_pose.header.frame_id = planning_frame
        cover1_pose.pose.position.x = 0.142
        cover1_pose.pose.position.y = 0
        cover1_pose.pose.position.z = -0.05
        cover1_pose.pose.orientation.w = 1.0   
        scene.add_box(cover1_id, cover1_pose, cover1_size)
        cover2_pose = PoseStamped()
        cover2_pose.header.frame_id = planning_frame
        cover2_pose.pose.position.x = 0.292
        cover2_pose.pose.position.y = 0
        cover2_pose.pose.position.z = -0.101
        cover2_pose.pose.orientation.w = 1.0   
        scene.add_box(cover2_id, cover2_pose, cover2_size)
        cover3_pose = PoseStamped()
        cover3_pose.header.frame_id = planning_frame
        cover3_pose.pose.position.x = 0.55
        cover3_pose.pose.position.y = 0
        cover3_pose.pose.position.z = -0.063
        cover3_pose.pose.orientation.w = 1.0   
        scene.add_box(cover3_id, cover3_pose, cover3_size)
        # 添加盒子围栏障碍到规划场景当中
        box_side1_pose = PoseStamped()
        box_side1_pose.header.frame_id = planning_frame
        box_side1_pose.pose.position.x = 0.3
        box_side1_pose.pose.position.y = 0
        box_side1_pose.pose.position.z = -0.06
        box_side1_pose.pose.orientation.w = 1.0   
        # scene.add_box(box_side1_id, box_side1_pose, box_side1_size)
        box_side2_pose = PoseStamped()
        box_side2_pose.header.frame_id = planning_frame
        box_side2_pose.pose.position.x = 0.45
        box_side2_pose.pose.position.y = -0.15
        box_side2_pose.pose.position.z = -0.06
        box_side2_pose.pose.orientation.w = 1.0   
        # scene.add_box(box_side2_id, box_side2_pose, box_side2_size)
        box_side3_pose = PoseStamped()
        box_side3_pose.header.frame_id = planning_frame
        box_side3_pose.pose.position.x = 0.6
        box_side3_pose.pose.position.y = 0
        box_side3_pose.pose.position.z = -0.06
        box_side3_pose.pose.orientation.w = 1.0   
        # scene.add_box(box_side3_id, box_side3_pose, box_side3_size)
        box_side4_pose = PoseStamped()
        box_side4_pose.header.frame_id = planning_frame
        box_side4_pose.pose.position.x = 0.45
        box_side4_pose.pose.position.y = 0.15
        box_side4_pose.pose.position.z = -0.06
        box_side4_pose.pose.orientation.w = 1.0   
        # scene.add_box(box_side4_id, box_side4_pose, box_side4_size)
        # 将桌面约束加入场景当中
        desk_pose = PoseStamped()
        desk_pose.header.frame_id = planning_frame
        desk_pose.pose.position.x = 0
        desk_pose.pose.position.y = 0
        desk_pose.pose.position.z = -0.08
        desk_pose.pose.orientation.w = 1.0
        # scene.add_box(desk_id, desk_pose, desk_size)


    def right_arm_go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        right_arm = self.right_arm

        # 获取当前末端执行器位置姿态
        pose_goal = right_arm.get_current_pose().pose

        # 限制末端夹具运动
        right_joint_const = JointConstraint()
        right_joint_const.joint_name = "gripper_r_joint_r"
        if Rightfinger > -55 :
            right_joint_const.position = 0.024
        else:
            right_joint_const.position = 0
        right_joint_const.weight = 1.0

        # 限制1轴转动
        right_joint1_const = JointConstraint()
        right_joint1_const.joint_name = "yumi_joint_1_r"
        right_joint1_const.position = 0
        right_joint1_const.tolerance_above = 120
        right_joint1_const.tolerance_below = 0
        right_joint1_const.weight = 1.0

        # 限制2轴转动
        right_joint2_const = JointConstraint()
        right_joint2_const.joint_name = "yumi_joint_2_r"
        right_joint2_const.position = 0
        right_joint2_const.tolerance_above = 0
        right_joint2_const.tolerance_below = 150
        right_joint2_const.weight = 1.0

        # 限制3轴转动
        right_joint3_const = JointConstraint()
        right_joint3_const.joint_name = "yumi_joint_3_r"
        right_joint3_const.position = 0
        right_joint3_const.tolerance_above = 35
        right_joint3_const.tolerance_below = 55
        right_joint3_const.weight = 1.0

        # 限制4轴转动
        right_joint4_const = JointConstraint()
        right_joint4_const.joint_name = "yumi_joint_4_r"
        right_joint4_const.position = 0
        right_joint4_const.tolerance_above = 60
        right_joint4_const.tolerance_below = 75
        right_joint4_const.weight = 1.0

        # 限制5轴转动
        right_joint5_const = JointConstraint()
        right_joint5_const.joint_name = "yumi_joint_5_r"
        right_joint5_const.position = 40
        right_joint5_const.tolerance_above = 50
        right_joint5_const.tolerance_below = 20
        right_joint5_const.weight = 1.0

        # 限制6轴转动
        right_joint6_const = JointConstraint()
        right_joint6_const.joint_name = "yumi_joint_6_r"
        right_joint6_const.position = 0
        right_joint6_const.tolerance_above = 10
        right_joint6_const.tolerance_below = 35
        right_joint6_const.weight = 1.0

        # 限制7轴转动
        right_joint7_const = JointConstraint()
        right_joint7_const.joint_name = "yumi_joint_7_r"
        right_joint7_const.position = -10
        right_joint7_const.tolerance_above = 0
        right_joint7_const.tolerance_below = 160
        right_joint7_const.weight = 1.0

        # 限制末端位移
        right_position_const = PositionConstraint()
        right_position_const.header = Header()
        right_position_const.link_name = "gripper_r_joint_r"
        right_position_const.target_point_offset.x = 0.5
        right_position_const.target_point_offset.y = -0.5
        right_position_const.target_point_offset.z = 1.0
        right_position_const.weight = 1.0

        # 添加末端姿态约束:
        right_orientation_const = OrientationConstraint()
        right_orientation_const.header = Header()
        right_orientation_const.orientation = pose_goal.orientation
        right_orientation_const.link_name = "gripper_r_finger_r"
        right_orientation_const.absolute_x_axis_tolerance = 0.50
        right_orientation_const.absolute_y_axis_tolerance = 0.25
        right_orientation_const.absolute_z_axis_tolerance = 0.50
        right_orientation_const.weight = 100

        # 施加全约束
        consts = Constraints()
        consts.joint_constraints = [right_joint_const]
        # consts.orientation_constraints = [right_orientation_const]
        # consts.position_constraints = [right_position_const]
        right_arm.set_path_constraints(consts)

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
        # pose_goal.orientation.x = 0.1644
        # pose_goal.orientation.y = 0.3111
        # pose_goal.orientation.z = 0.9086
        # pose_goal.orientation.w = 0.2247
        # pose_goal.position.x = (Neurondata[5]-0.05)*1.48+0.053
        # pose_goal.position.y = (Neurondata[3]+0.18)*1.48-0.12
        # pose_goal.position.z = (Neurondata[4]-0.53)*1.48+0.47
        # right_arm.set_pose_target(pose_goal)
        # print "End effector pose %s" % pose_goal

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
        # 清除路径约束
        right_arm.clear_path_constraints()
        # 确保没有剩余未完成动作在执行
        right_arm.stop()

    def left_arm_go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        left_arm = self.left_arm

        # 获取当前末端执行器位置姿态
        pose_goal = left_arm.get_current_pose().pose

        # 限制末端夹具运动
        left_joint_const = JointConstraint()
        left_joint_const.joint_name = "gripper_l_joint_r"
        if Leftfinger > -32 :
            left_joint_const.position = 0.024
        else:
            left_joint_const.position = 0
        left_joint_const.weight = 1.0

        # 限制末端位移
        left_position_const = PositionConstraint()
        left_position_const.header = Header()
        left_position_const.link_name = "gripper_l_joint_r"
        left_position_const.target_point_offset.x = 0.5
        left_position_const.target_point_offset.y = -0.5
        left_position_const.target_point_offset.z = 1.0
        left_position_const.weight = 1.0        

        # # 限制1轴转动
        left_joint1_const = JointConstraint()
        left_joint1_const.joint_name = "yumi_joint_1_l"
        left_joint1_const.position = 0
        left_joint1_const.tolerance_above = 1.76 
        left_joint1_const.tolerance_below = 0
        left_position_const.weight = 1.0  

        # 限制2轴转动
        left_joint2_const = JointConstraint()
        left_joint2_const.joint_name = "yumi_joint_2_l"
        left_joint2_const.position = 0
        left_joint2_const.tolerance_above = 0
        left_joint2_const.tolerance_below = 150
        left_joint2_const.weight = 1.0

        # 限制3轴转动
        left_joint3_const = JointConstraint()
        left_joint3_const.joint_name = "yumi_joint_3_l"
        left_joint3_const.position = 0
        left_joint3_const.tolerance_above = 35
        left_joint3_const.tolerance_below = 55
        left_joint3_const.weight = 1.0

        # 限制4轴转动
        left_joint4_const = JointConstraint()
        left_joint4_const.joint_name = "yumi_joint_4_l"
        left_joint4_const.position = 0
        left_joint4_const.tolerance_above = 60
        left_joint4_const.tolerance_below = 75
        left_joint4_const.weight = 1.0

        # 限制5轴转动
        right_joint5_const = JointConstraint()
        right_joint5_const.joint_name = "yumi_joint_5_l"
        right_joint5_const.position = 40
        right_joint5_const.tolerance_above = 50
        right_joint5_const.tolerance_below = 20
        right_joint5_const.weight = 1.0

        # 限制6轴转动
        left_joint6_const = JointConstraint()
        left_joint6_const.joint_name = "yumi_joint_6_l"
        left_joint6_const.position = 0
        left_joint6_const.tolerance_above = 10
        left_joint6_const.tolerance_below = 35
        left_joint6_const.weight = 1.0

        # 限制7轴转动
        left_joint7_const = JointConstraint()
        left_joint7_const.joint_name = "yumi_joint_7_l"
        left_joint7_const.position = -10
        left_joint7_const.tolerance_above = 0
        left_joint7_const.tolerance_below = 160
        left_joint7_const.weight = 1.0

        # 限制末端位移
        left_position_const = PositionConstraint()
        left_position_const.header = Header()
        left_position_const.link_name = "gripper_l_joint_r"
        left_position_const.target_point_offset.x = 0.5
        left_position_const.target_point_offset.y = 0.25
        left_position_const.target_point_offset.z = 0.5
        left_position_const.weight = 1.0

        # 添加末端姿态约束:
        left_orientation_const = OrientationConstraint()
        left_orientation_const.header = Header()
        left_orientation_const.orientation = pose_goal.orientation
        left_orientation_const.link_name = "gripper_l_finger_r"
        left_orientation_const.absolute_x_axis_tolerance = 0.5
        left_orientation_const.absolute_y_axis_tolerance = 0.25
        left_orientation_const.absolute_z_axis_tolerance = 0.5
        left_orientation_const.weight = 1

        # 施加全约束
        consts = Constraints()
        consts.joint_constraints = [left_joint_const]
        # consts.orientation_constraints = [left_orientation_const]
        # consts.position_constraints = [left_position_const]
        left_arm.set_path_constraints(consts)

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
        # 清除路径约束
        left_arm.clear_path_constraints()
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
        if Rightfinger > -55 :
            right_joint_const.position = 0.024
        else:
            right_joint_const.position = 0
        left_joint_const = JointConstraint()
        left_joint_const.joint_name = "gripper_l_joint_r"
        if Leftfinger > -32 :
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
        right_joint_goal[7] = 0.0236
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

    def right_arm_go_to_joint_goal(self, jointnum):
        # 设置动作对象变量,此处为right_arm
        right_arm = self.right_arm
        # 获取当前末端执行器位置姿态
        right_joint_goal = right_arm.get_current_joint_values()
        print right_joint_goal
        # 设置末端关节目标值
        armnum = 'right_arm'
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
        right_arm.stop()

    def left_arm_go_to_joint_goal(self, jointnum):
        # 设置动作对象变量,此处为right_arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        left_joint_goal = left_arm.get_current_joint_values()
        # 设置末端关节目标值
        armnum = 'left_arm'
        left_joint_goal[0] = return_joint_state(armnum,jointnum)[0]
        left_joint_goal[1] = return_joint_state(armnum,jointnum)[1]
        left_joint_goal[2] = return_joint_state(armnum,jointnum)[2]
        left_joint_goal[3] = return_joint_state(armnum,jointnum)[3]
        left_joint_goal[4] = return_joint_state(armnum,jointnum)[4]
        left_joint_goal[5] = return_joint_state(armnum,jointnum)[5]
        left_joint_goal[6] = return_joint_state(armnum,jointnum)[6]
        print "End effector joint goal %s" % left_joint_goal
        # 规划并执行路径动作
        left_arm.go(left_joint_goal, wait=False)
        left_arm.stop()

    def right_arm_move_type_judge(self):
        # 设置动作对象变量,此处为arm
        right_arm = self.right_arm
        # 获取当前末端执行器位置姿态
        pose_goal = right_arm.get_current_pose().pose
        # 判断是否到达抓取工作空间/微调工作区间       
        if (pose_goal.position.x >= 0.4 and pose_goal.position.x <= 0.55) and (pose_goal.position.y <= 0.1 and pose_goal.position.y >= -0.30) and (pose_goal.position.z <= 0.15 and pose_goal.position.z >= -0.1) :
            right_posecon = 1          
        else:
            right_posecon = 0
        return right_posecon

    def left_arm_move_type_judge(self):
        # 设置动作对象变量,此处为arm
        left_arm = self.left_arm
        # 获取当前末端执行器位置姿态
        pose_goal = left_arm.get_current_pose().pose
        # 判断是否到达抓取工作空间/微调工作区间
        if (pose_goal.position.x >= 0.35 and pose_goal.position.x <= 0.50) and (pose_goal.position.y <= 0.40 and pose_goal.position.y >= 0) and (pose_goal.position.z <= 0.15 and pose_goal.position.z >= -0.1) :
            left_posecon = 1          
        else:
            left_posecon = 0
        return left_posecon

    def human_right_arm_pos_judge(self):
        # 获取当前操作者的末端位置姿态
        # 判断是否到达抓取工作空间/微调工作区间       
        # if (Neurondata[5] >= 0.3 and Neurondata[5] <= 0.5) and (Neurondata[3] <= 0.3 and Neurondata[3] >= 0.1) and (Neurondata[4] <= 0.5 and Neurondata[4] >= 0.3) :
        #     right_posecon = 1          
        # elif (Neurondata[5] >= 0.3 and Neurondata[5] <= 0.5) and (Neurondata[3] <= 0.1 and Neurondata[3] >= -0.1) and (Neurondata[4] <= 0.5 and Neurondata[4] >= 0.3) :
        #     right_posecon = 2
        # elif (Neurondata[5] >= 0.3 and Neurondata[5] <= 0.5) and (Neurondata[3] <= -0.1 and Neurondata[3] >= 0.1) and (Neurondata[4] <= 0.5 and Neurondata[4] >= 0.3) :
        #     right_posecon = 3
        # else:
        #     right_posecon = 0
        # return right_posecon
        print Neurondata[3]
        if Neurondata[3] >= 0.1 :
            right_posecon = 1          
        elif Neurondata[3] <= 0.1 and Neurondata[3] >= -0.1:
            right_posecon = 2
        elif Neurondata[3] <= -0.1  :
            right_posecon = 3
        else:
            right_posecon = 0
        return right_posecon
    
    def right_fingerT_type_judge(self):
        if RightfingerT < -40 :
            print "============ rightarm debug ..."
            raw_input()
            if T1 == T2 :
                pass
            else :
                T1 = T2
                right_fingerT_type = 1
        else :
            if T1 == T2 :
                T2 = not(T2)
                right_fingerT_type = 0
            pass
        return right_fingerT_type

    def right_fingerT_wait_judge(self):
        while 1:
            time.sleep(1)
            print "wait"
            print RightfingerT
            if RightfingerT < -10:
                print "ok"
                break

    def right_fingerI_wait_open_judge(self):
        while 1:
            time.sleep(1)
            print "wait"
            print Rightfinger
            if Rightfinger > -55:
                print "ok"
                break

    def right_fingerI_wait_close_judge(self):
        while 1:
            time.sleep(1)
            print "wait"
            print Rightfinger
            if Rightfinger < -54:
                print "ok"
                break

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
    # global tempa, tempb, tempc, tempd
    # tempa = 0
    # tempb = 1
    # tempc = 0
    # tempd = 1
    global T1, T2, T3, T4
    T1 = 0
    T2 = 1
    T3 = 0
    T4 = 1
    # global limit_workspace_time, right_outside_con, left_outside_con
    # right_limit_workspace_time = 0
    # left_limit_workspace_time = 0
    # right_outside_con = 1
    # left_outside_con = 1
    global right_arm_joint_state, left_arm_joint_state

    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()
    yumi.set_scene()
    # 循环等待,执行动作程序
    print "============ Press `Enter` to begin the teleoperation ..."
    raw_input()

    while 1:
        # 执行arm目标点动作
        # if RightfingerT < -10 :
            # print "============ rightarm debug ..."
            # raw_input()
            # if T1 == T2 :
            #     pass
            # else :
            #     T1 = T2
            #     right_arm_joint_state = yumi.right_arm_get_current_joint_state()
                
            # print "============ wait for con judge ..."
            # yumi.right_fingerT_wait_judge()
            # print "============ wait for con judge done..."
            # right_pose_condition = yumi.human_right_arm_pos_judge()
            # print right_pose_condition

            # ================================== 左臂归位

            # if right_pose_condition == 1:
            print "============ Press `Enter` to execute a left arm using a joint goal0 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.left_arm_go_to_joint_goal(0)
            time.sleep(3)
            print "============ Press `Enter` to exact approach ..."
            # yumi.right_fingerT_wait_judge()
            # raw_input()
            # for i in range(3):
            #     yumi.right_arm_go_to_joint_goal(0)
            #     time.sleep(1)

            # ================================================ 拔出销
            
            print "============ Press `Enter` to execute a right arm using a joint goal0 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(0)
            time.sleep(3)
            print "============ Press `Enter` to exact approach ..."
            # yumi.right_fingerT_wait_judge()
            # raw_input()
            # for i in range(3):
            #     yumi.right_arm_go_to_joint_goal(0)
            #     time.sleep(1)

            print "============ Press `Enter` to open a finger ..."
            # yumi.right_fingerI_wait_open_judge()
            raw_input()
            yumi.right_gripper_go_to_open_goal()
            time.sleep(1)
            
            # 销夹取点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(1)
            time.sleep(3)

            print "============ Press `Enter` to close a finger ..."
            # yumi.right_fingerI_wait_close_judge()
            raw_input()
            yumi.right_gripper_go_to_close_goal()
            time.sleep(1)

            # 销拔出点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(2)
            time.sleep(3)

            # 销放置孔位上方点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(3)
            time.sleep(3)

            # 销放置孔位点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(4)
            time.sleep(3)

            print "============ Press `Enter` to open a finger ..."
            # yumi.right_fingerI_wait_open_judge()
            raw_input()
            yumi.right_gripper_go_to_open_goal()
            time.sleep(1)

            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(3)
            time.sleep(3)

            print "============ Press `Enter` to close a finger ..."
            # yumi.right_fingerI_wait_close_judge()
            raw_input()
            yumi.right_gripper_go_to_close_goal()
            time.sleep(1)

            # ========================================== 拔出刀具

            # 刀具夹取点上方/拔出点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(6)
            time.sleep(3)

            print "============ Press `Enter` to open a finger ..."
            # yumi.right_fingerI_wait_open_judge()
            raw_input()
            yumi.right_gripper_go_to_open_goal()
            time.sleep(1)

            # 刀具夹取点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(5)
            time.sleep(3)

            print "============ Press `Enter` to close a finger ..."
            # yumi.right_fingerI_wait_close_judge()
            raw_input()
            yumi.right_gripper_go_to_close_goal()
            time.sleep(1)

            # 刀具拔出点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(6)
            time.sleep(3)

            # 刀具夹取上方到放置点上方的过渡点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(9)
            time.sleep(3)

            # 刀具放置上点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(7)
            time.sleep(3)

            # 刀具放置点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(8)
            time.sleep(3)

            print "============ Press `Enter` to open a finger ..."
            # yumi.right_fingerI_wait_open_judge()
            raw_input()
            yumi.right_gripper_go_to_open_goal()
            time.sleep(1)

            print "============ Press `Enter` to close a finger ..."
            # yumi.right_fingerI_wait_close_judge()
            raw_input()
            yumi.right_gripper_go_to_close_goal()
            time.sleep(1)

            # ======================================== 拿取刀具

            # 刀具拾取点位1上方
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(10)
            time.sleep(3)

            print "============ Press `Enter` to open a finger ..."
            # yumi.right_fingerI_wait_open_judge()
            raw_input()
            yumi.right_gripper_go_to_open_goal()
            time.sleep(1)

            # 刀具拾取点位1下
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(11)
            time.sleep(3)

            print "============ Press `Enter` to close a finger ..."
            # yumi.right_fingerI_wait_close_judge()
            raw_input()
            yumi.right_gripper_go_to_close_goal()
            time.sleep(1)

            # 刀具拾取点位1上方
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(10)
            time.sleep(3)

            # 刀具夹取上方到放置点上方的过渡点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(9)
            time.sleep(3)

            # 刀具拔出点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(6)
            time.sleep(3)

            # 刀具夹取点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(5)
            time.sleep(3)

            print "============ Press `Enter` to open a finger ..."
            # yumi.right_fingerI_wait_open_judge()
            raw_input()
            yumi.right_gripper_go_to_open_goal()
            time.sleep(1)

            # 刀具夹取上方到放置点上方的过渡点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(9)
            time.sleep(3)

            print "============ Press `Enter` to close a finger ..."
            # yumi.right_fingerI_wait_close_judge()
            raw_input()
            yumi.right_gripper_go_to_close_goal()
            time.sleep(1)

            # ===================================================== 插销

            # 销放置孔位上方点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(3)
            time.sleep(3)

            print "============ Press `Enter` to open a finger ..."
            # yumi.right_fingerI_wait_open_judge()
            raw_input()
            yumi.right_gripper_go_to_open_goal()
            time.sleep(1)

            # 销放置孔位点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(4)
            time.sleep(3)

            print "============ Press `Enter` to close a finger ..."
            # yumi.right_fingerI_wait_close_judge()
            raw_input()
            yumi.right_gripper_go_to_close_goal()
            time.sleep(1)

            # 销放置孔位上方点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(3)
            time.sleep(3)

            # 销拔出点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(2)
            time.sleep(3)

            # 销夹取点
            print "============ Press `Enter` to execute a right arm using a joint goal1 ..."
            # yumi.right_fingerT_wait_judge()
            raw_input()
            yumi.right_arm_go_to_joint_goal(1)
            time.sleep(3)




                

            # if right_pose_condition == 2:
            #     print "============ Press `Enter` to execute a right arm using a joint goal2 ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     yumi.right_arm_go_to_joint_goal(3)
            #     time.sleep(3)
            #     print "============ Press `Enter` to exact approach ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     for i in range(3):
            #         yumi.right_arm_go_to_joint_goal(3)
            #         time.sleep(1)
                
            #     print "============ Press `Enter` to execute a right arm using a joint goal2 ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     yumi.right_arm_go_to_joint_goal(4)
            #     time.sleep(3)
            #     print "============ Press `Enter` to exact approach ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     for i in range(3):
            #         yumi.right_arm_go_to_joint_goal(4)
            #         time.sleep(1)
                
            #     print "============ Press `Enter` to execute a right arm using a joint goal2 ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     yumi.right_arm_go_to_joint_goal(5)
            #     time.sleep(3)
            #     print "============ Press `Enter` to exact approach ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     for i in range(3):
            #         yumi.right_arm_go_to_joint_goal(5)
            #         time.sleep(1)

            # if right_pose_condition == 3:
            #     print "============ Press `Enter` to execute a right arm using a joint goal3 ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     yumi.right_arm_go_to_joint_goal(6)
            #     time.sleep(3)
            #     print "============ Press `Enter` to exact approach ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     for i in range(3):
            #         yumi.right_arm_go_to_joint_goal(6)
            #         time.sleep(1)
                
            #     print "============ Press `Enter` to execute a right arm using a joint goal3 ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     yumi.right_arm_go_to_joint_goal(7)
            #     time.sleep(3)
            #     print "============ Press `Enter` to exact approach ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     for i in range(3):
            #         yumi.right_arm_go_to_joint_goal(7)
            #         time.sleep(1)
                
            #     print "============ Press `Enter` to execute a right arm using a joint goal3 ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     yumi.right_arm_go_to_joint_goal(6)
            #     time.sleep(3)
            #     print "============ Press `Enter` to exact approach ..."
            #     # yumi.right_fingerT_wait_judge()
            #     # raw_input()
            #     for i in range(3):
            #         yumi.right_arm_go_to_joint_goal(6)
            #         time.sleep(1)
            
        # else :
        #     if T1 == T2 :
        #         T2 = not(T2)
        #     pass

        # if LeftfingerT < -20 :
        #     if T3 == T4 :
        #         pass
        #     else :
        #         T3 = T4
        #         left_pose_condition = yumi.left_arm_move_type_judge()
        #         left_arm_joint_state = yumi.left_arm_get_current_joint_state()
        #         # raw_input()
        #         if not left_pose_condition:
        #             print "============ left_con1 ============"
        #             yumi.left_arm_go_to_pose_goal()
        #         else:
        #             if left_outside_con == 1 and left_limit_workspace_time <= 6:
        #                 print "============ Press `Enter` to execute a left arm using a joint goal ..."
        #                 yumi.left_arm_go_to_joint_goal(left_limit_workspace_time)
        #                 left_limit_workspace_time = left_limit_workspace_time + 1
        #                 print left_limit_workspace_time
        #                 if (left_limit_workspace_time%3 == 0):
        #                     left_outside_con = 0
        #         if ((left_limit_workspace_time%4 == 0) and (left_arm_joint_state[7] <= 0.01)) or left_limit_workspace_time > 6:
        #             print "============== left_con2 ============"
        #             yumi.left_arm_go_to_pose_goal()
        #             left_outside_con = 1
            
                
        # else :
        #     if T3 == T4 :
        #         T4 = not(T4)
        #     pass

        # if Rightfinger > -55 :
        #     if tempa == tempb :
        #         pass
        #     else :
        #         tempa = tempb
        #         print "============ Press `Enter` to execute right gripper open using a pose goal ..."
        #         # raw_input()
        #         yumi.right_gripper_go_to_open_goal()
        # else :
        #     if tempa == tempb :
        #         tempb = not(tempb)
        #         print "============ Press `Enter` to execute right gripper close using a pose goal ..."
        #         # raw_input()
        #         yumi.right_gripper_go_to_close_goal()
        #     else:
        #         pass

        # if Leftfinger > -52 :
        #     if tempc == tempd :
        #         pass
        #     else :
        #         tempc = tempd
        #         print "============ Press `Enter` to execute left gripper open using a pose goal ..."
        #         # raw_input()
        #         yumi.left_gripper_go_to_open_goal()
        # else:
        #     if tempc == tempd :
        #         tempd = not(tempd)
        #         print "============ Press `Enter` to execute left gripper close using a pose goal ..."
        #         yumi.left_gripper_go_to_close_goal()
            

if __name__ == '__main__':
    main()
