#!/usr/bin/env python
# coding=UTF-8
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
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import JointConstraint, OrientationConstraint, PositionConstraint, JointLimits, Constraints
# Author: Honghao Lv
# Note: 通过预采样Neuron端的姿态数据,不过本节点程序将 预采样规划/路径显示/路径输出 三个过程分离输出,通过键盘依次触发.相比V2更新：去除了初次校准归位程序
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
        group_name = "right_arm"
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

    def go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        # arm.set_max_acceleration_scaling_factor(0.0002)
        # arm.set_max_velocity_scaling_factor(0.0002)
        # arm.set_goal_orientation_tolerance(0.5)

        # 获取当前末端执行器位置姿态
        pose_goal = self.arm.get_current_pose().pose

        # print (a)
        # print (Qux, Quy, Quz, Quw)

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = Qux
        pose_goal.orientation.y = Quy
        pose_goal.orientation.z = Quz
        pose_goal.orientation.w = Quw
        pose_goal.position.x = a[5] * 1.13
        pose_goal.position.y = a[3] * 1.13
        pose_goal.position.z = a[4] * 1.13
        arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # 规划和输出动作
        # arm.go(wait=False)

        traj = arm.plan()
        arm.execute(traj, wait=False)
        # 动作完成后清除目标信息
        # arm.clear_pose_targets()
        # 确保没有剩余未完成动作在执行
        arm.stop()

    def go_to_test_goal(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        # 获取当前末端执行器位置姿态
        pose_goal = self.arm.get_current_pose().pose

        print (a)
        (Qux, Quy, Quz, Quw) = quaternion_from_euler(pi/2, 0, pi/2)

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = -Qux
        pose_goal.orientation.y = -Quy
        pose_goal.orientation.z = -Quz
        pose_goal.orientation.w = -Quw
        pose_goal.position.x = pose_goal.position.x
        pose_goal.position.y = pose_goal.position.y
        pose_goal.position.z = pose_goal.position.z
        arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # 规划和输出动作
        arm.go(wait=False)
        # 确保没有剩余未完成动作在执行
        # arm.stop()
        # 动作完成后清除目标信息
        # arm.clear_pose_targets()

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
        arm.stop()

    def go_to_ready_goal(self):
        # 控制机械臂回到校准位置
        arm = self.arm
        arm.set_named_target('ready')
        arm.go()

    def add_way_points(self, waypoints):

        arm = self.arm

        # 限制末端夹具运动
        right_joint_const = JointConstraint()
        right_joint_const.joint_name = "gripper_r_joint_r"
        right_joint_const.position = 0
        right_joint_const.weight = 1.0
        # 施加全约束
        consts = Constraints()
        consts.joint_constraints = [right_joint_const]
        arm.set_path_constraints(consts)

        wpose = arm.get_current_pose().pose
        wpose.orientation.x = Qux
        wpose.orientation.y = Quy
        wpose.orientation.z = Quz
        wpose.orientation.w = Quw
        wpose.position.x = a[5]
        wpose.position.y = a[3]
        wpose.position.z = a[4]

        waypoints.append(copy.deepcopy(wpose))

        # print waypoints
        # print "----------------\n"
        return waypoints

    def plan_cartesian_path(self, waypoints):

        arm = self.arm
       
        # 限制末端夹具运动
        right_joint_const = JointConstraint()
        right_joint_const.joint_name = "gripper_r_joint_r"
        right_joint_const.position = 0
        right_joint_const.weight = 1.0
        # 施加全约束
        consts = Constraints()
        consts.joint_constraints = [right_joint_const]
        arm.set_path_constraints(consts)

        (plan, fraction) = arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        return plan, fraction

    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):

        arm = self.arm

        arm.execute(plan, wait=False)
        arm.stop()


def main():
    # 定义全局变量count,判断程序执行次数
    global count
    count = 0
    way = []

    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()

    # 循环等待,执行动作程序
    while 1:
        count = count + 1

        # 首次程序执行,回到动作原点
        # if count == 1:
        #     print "============ Press `Enter` to execute a arm movement using a joint state goal for the original state..."
        #     raw_input()
        #     yumi.go_to_calc_goal()

        # # 执行arm目标点动作
        # print "============ Press `Enter` to execute a arm movement using a pose goal ..."
        # raw_input()
        # yumi.go_to_pose_goal()
        # time.sleep(0.5)

        print "============ Press `Enter` to plan and display a Cartesian path ..."
        raw_input()
        for i in range(10):
            way = yumi.add_way_points(way)
            time.sleep(0.29)
            print(i)

        cartesian_plan, fraction = yumi.plan_cartesian_path(way)

        print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        raw_input()
        yumi.display_trajectory(cartesian_plan)

        print "============ Press `Enter` to execute a saved path ..."
        raw_input()
        yumi.execute_plan(cartesian_plan)

        # 输出结束,清空之前的路径点
        way = []


if __name__ == '__main__':
    main()
