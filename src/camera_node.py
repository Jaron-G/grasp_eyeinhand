#!/usr/bin/env python3
# coding:utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
 
 
def callback(data):
    global bridge
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    image_raw = image.copy()
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters()
    arucodetector = cv2.aruco.ArucoDetector(dictionary= arucoDict, detectorParams=arucoParams)
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
    (corners, ids, rejected) = arucodetector.detectMarkers(image)

    # cameraMatrix = np.array([762.725, 0, 640.5,
    #                      0, 762.725, 640.5,
    #                      0, 0, 1]).reshape(3,3)
    # cameraMatrix = np.array([913.617065, 0, 960.503906,
    #                     0, 913.455261, 550.489502,
    #                     0, 0, 1]).reshape(3,3)#1080p
    cameraMatrix = np.array([609.078, 0, 640.169,
                        0,608.97 , 366.826,
                        0, 0, 1]).reshape(3,3)#720p
    dist = np.array([0.0001, -0.00872068, 0.000730499, 0.000393782, 0.0000648475])
    distCoeffs = dist[0:5].reshape(1,5)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))
        rvecs, tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners[0], 60, cameraMatrix, distCoeffs)
        for i in range(len(rvecs)):
            cv2.drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs, tvecs, 40)
        cv2.putText(image, "Id: " + str(ids), (10,40), font, 0.5, (0, 0, 255),1,cv2.LINE_AA)
        cv2.putText(image, "rvec: " + str(rvecs[i, :, :]), (10, 60), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(image, "tvec: " + str(tvecs[i, :, :]), (10,80), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    else:
        cv2.putText(image, "No Ids", (10,64), font, 1, (0,255,0),2,cv2.LINE_AA)
    cv2.imshow("frame" , image)
    key = cv2.waitKey(1)
    if key == 27:         # 按esc键退出
        print('esc break...')
        cv2.destroyWindow("frame")
    
    if key == ord(' '):   # 按空格键保存
        filename = "/catkin_ws/src/grasp_eyeinhand/images/scene.png"
        cv2.imwrite(filename, image_raw)
        print('save img...')

if __name__ == '__main__':
    rospy.init_node('img_process_node', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber('/image_view/image_raw', Image, callback)
    # rospy.Subscriber('/rgb/image_raw', Image, callback)
    # rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.spin()