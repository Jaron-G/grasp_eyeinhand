#!/usr/bin/env python3
# coding:utf-8

import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
import time
import pyk4a
from pyk4a import PyK4A, Config

if __name__=="__main__":
    # capture = cv2.VideoCapture(0) # 定义摄像头
    rospy.init_node('camera_node', anonymous=True) #定义节点
    image_pub=rospy.Publisher('/image_view/image_raw', Image, queue_size = 12) #定义话题

    k4a = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.RES_720P,
        color_format= 3,
        camera_fps=pyk4a.FPS.FPS_5,
        depth_mode=pyk4a.DepthMode.OFF ,
        synchronized_images_only=False,
        disable_streaming_indicator=True
        )
    )
    k4a.start()

    while not rospy.is_shutdown():    # Ctrl C正常退出，如果异常退出会报错device busy！
        # start = time.time()
        color_image = k4a.get_capture().color
        frame = np.ascontiguousarray(color_image[:,:,0:3])
        if frame is not None: # 如果有画面再执行
            # frame = cv2.flip(frame,0)   #垂直镜像操作
            # frame = cv2.flip(frame,1)   #水平镜像操作   
    
            ros_frame = Image()
            header = Header(stamp = rospy.Time.now())
            header.frame_id = "Camera"
            ros_frame.header=header
            ros_frame.width = 1280
            ros_frame.height = 720
            ros_frame.encoding = "bgr8"
            ros_frame.step = 1280
            ros_frame.data = np.array(frame).tostring() #图片格式转换
            image_pub.publish(ros_frame) #发布消息
            # end = time.time()  
            # print("cost time:", end-start ) # 看一下每一帧的执行时间，从而确定合适的rate
            rate = rospy.Rate(10) # 10hz 

    cv2.destroyAllWindows() 
    print("quit successfully!")