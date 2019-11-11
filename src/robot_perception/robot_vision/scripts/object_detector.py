#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.coordinate_pub = rospy.Publisher("object_coordinate", Pose2D, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.callback)

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        
        ## 裁剪原始图片
        h, w, ch = cv_image.shape
        img_trim = cv_image[50:320, 210:520]

        ## 滤波&转换色彩空间
        img_blur1 = cv2.bilateralFilter(img_trim,9,75,75)
        img_blur2 = cv2.blur(img_blur1, (5,5))
        img_hsv = cv2.cvtColor(img_blur2, cv2.COLOR_BGR2HSV)


        ## 设定object的HSV色彩空间阈值范围
        lower_obj = np.array([81, 50, 0])
        higher_obj = np.array([154, 139, 255])

        ## 根据HSV色彩空间范围区分object和后景
        img_mask = cv2.inRange(img_hsv, lower_obj, higher_obj)

        ## 寻找轮廓
        contours,hierarchy = cv2.findContours(img_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        bg_contours = np.zeros(img_trim.shape, dtype = np.uint8)
        contours_result = cv2.drawContours(bg_contours, [cnt], 0, (0,255,0), 2)

        ## 轮廓逼近拟合
        epsilon = 0.01 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        ## 分析几何形状
        corners =len(approx)

        ## 绘制拟合轮廓
        bg_approx = np.zeros(img_trim.shape, dtype=np.uint8)
        approx_result = cv2.drawContours(bg_approx, [approx], -1, (255,0,0), 3)

        ## 求解中心位置
        # 设定初始值
        cx = 0
        cy = 0
        mm = cv2.moments(approx)
        if mm['m00'] != 0:
            cx = int(mm['m10']/mm['m00'])
            cy = int(mm['m01']/mm['m00'])
            result = cv2.circle(approx_result, (cx, cy), 3, (0,0,255), -1)
        else:
            pass

        # print "中心位置坐标", cx, cy 

        img_result = approx_result
        # print cv_image.shape

        # 显示Opencv格式的图像
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        object_pose = Pose2D()
        object_pose.x = cx
        object_pose.y = cy
        object_pose.theta = 0

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_result, "bgr8"))
            self.coordinate_pub.publish(object_pose)
        except CvBridgeError as e:
            print e

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")

        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()