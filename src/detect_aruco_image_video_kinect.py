# 用法
# python detect_aruco_image_video.py --type DICT_5X5_100
# 导入库
import argparse
import cv2
import sys
import numpy as np
import time
import pyk4a
from pyk4a import PyK4A, Config

# 构造参数解析器并解析参数
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_4X4_50", help="Tpe of ArUCo tag to detect")
args = vars(ap.parse_args())

# 定义 OpenCV 支持的每个可能的 ArUco 标签的名称
ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50, "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
              "DICT_4X4_250": cv2.aruco.DICT_4X4_250, "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
              "DICT_5X5_50": cv2.aruco.DICT_5X5_50, "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
              "DICT_5X5_250": cv2.aruco.DICT_5X5_250, "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
              "DICT_6X6_50": cv2.aruco.DICT_6X6_50, "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
              "DICT_6X6_250": cv2.aruco.DICT_6X6_250, "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
              "DICT_7X7_50": cv2.aruco.DICT_7X7_50, "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
              "DICT_7X7_250": cv2.aruco.DICT_7X7_250, "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
              "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
              "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
              "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
              "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
              "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11}

cameraMatrix = np.array([913.617065, 0, 960.503906,
                        0, 913.455261, 550.489502,
                        0, 0, 1]).reshape(3,3)
dist = np.array([0.0515398, -0.00872068, 0.000730499, 0.000393782, 0.0000648475])
distCoeffs = dist[0:5].reshape(1,5)

if ARUCO_DICT.get(args["type"], None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported!".format(args["type"]))
    sys.exit(0)
# 加载 ArUCo 字典，抓取 ArUCo 参数并检测标记
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters()
arucodetector = cv2.aruco.ArucoDetector(dictionary= arucoDict, detectorParams=arucoParams)

# cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
k4a = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.RES_1080P,
        camera_fps=pyk4a.FPS.FPS_30,
        depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
        synchronized_images_only=True,
    )
)
k4a.start()
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

while True:
    start = time.time()
    color_image = k4a.get_capture().color
    frame = np.ascontiguousarray(color_image[:,:,0:3])
    (corners, ids, rejected) = arucodetector.detectMarkers(frame)
    if ids is not None:
        rvecs, tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners[0], 50, cameraMatrix, distCoeffs)
        for i in range(len(rvecs)):
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i, :, :], tvecs[i, :, :], 40)
        cv2.putText(frame, "Id: " + str(ids), (10,40), font, 0.5, (0, 0, 255),1,cv2.LINE_AA)
        cv2.putText(frame, "rvec: " + str(rvecs[i, :, :]), (10, 60), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "tvec: " + str(tvecs[i, :, :]), (10,80), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, "No Ids", (10,64), font, 1, (0,255,0),2,cv2.LINE_AA)
    end = time.time()
    # 计算并显示帧率
    cv2.putText(frame, "rate: " + str(1 / (end-start )), (10, 120), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.imshow("frame",frame)
    
    key = cv2.waitKey(1)
    if key == 27:         # 按esc键退出
        print('esc break...')
        k4a.stop()
        cv2.destroyAllWindows()
        break
    
    if key == ord(' '):   # 按空格键保存
#        num = num + 1
#        filename = "frames_%s.jpg" % num  # 保存一张图像
        filename = str(time.time())[:10] + ".jpg"
        cv2.imwrite(filename, frame)

