#!/usr/bin/env python3

import cv2
import cv_bridge

import numpy as np
import rclpy
import time
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

def nothing(s):
    pass


class Adjust_hsv(Node):
    def __init__(self):
        super().__init__('Adjust_hsv')
        self.bridge = cv_bridge.CvBridge()
        qos = QoSProfile(depth=10)
        self.mat = None
        

        self.image_sub = self.create_subscription(Image,'/camera/color/image_raw',self.image_callback,qos)
        self.tmp=0


    def image_callback(self, msg):
        if self.tmp==0:
            cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
            cv2.createTrackbar("lowerbH",'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar("lowerbS",'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar("lowerbV",'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar("upperbH",'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar("upperbS",'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar("upperbV",'Adjust_hsv',0,255,nothing)
            self.tmp=1
        lowerbH=cv2.getTrackbarPos("lowerbH",'Adjust_hsv')
        lowerbS=cv2.getTrackbarPos("lowerbS",'Adjust_hsv')
        lowerbV=cv2.getTrackbarPos("lowerbV",'Adjust_hsv')
        upperbH=cv2.getTrackbarPos("upperbH",'Adjust_hsv')
        upperbS=cv2.getTrackbarPos("upperbS",'Adjust_hsv')
        upperbV=cv2.getTrackbarPos("upperbV",'Adjust_hsv')

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        kernel = numpy.ones((5,5),numpy.uint8)
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)


        mask=cv2.inRange(hsv_dilate,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        masked = cv2.bitwise_and(image, image, mask=mask)
        # 在图像某处绘制一个指示，因为只考虑20行宽的图像，所以使用numpy切片将以外的空间区域清空
        #cv2.imshow("img", image)
        cv2.imshow("Adjust_hsv", masked)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    print('start patrolling')
    adjust_hsv = Adjust_hsv()
    while rclpy.ok():
        rclpy.spin_once(adjust_hsv)
        time.sleep(0.1)

    adjust_hsv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

