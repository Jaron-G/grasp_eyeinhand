import cv2
import numpy as np

def detect_pose():
    path = "/home/aobo/catkin_ws/src/grasp_eyeinhand/images/"
    image = cv2.imread(path+ "scene.png")

    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()
    arucodetector = cv2.aruco.ArucoDetector(dictionary= arucoDict, detectorParams=arucoParams)
    (corners, ids, rejected) = arucodetector.detectMarkers(image)

    cameraMatrix = np.array([381, 0, 320.5,
                            0, 381, 240.5,
                            0, 0, 1]).reshape(3,3)
    dist = np.array([0.0515398, -0.00872068, 0.000730499, 0.000393782, 0.0000648475])
    distCoeffs = dist[0:5].reshape(1,5)
    cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))
    cv2.imshow("kk",image)

    rvecs, tvecs,kkk = cv2.aruco.estimatePoseSingleMarkers(corners[0], 100, cameraMatrix, distCoeffs)
    np.save(path+ "rvecs.npy",rvecs)
    np.save(path+ "tvecs.npy",tvecs)
    print(rvecs, tvecs)
    print(len(rvecs))
    
    # for i in range(len(rvecs)):
    #     cv2.drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs, tvecs, 40)

    # cv2.imshow("pose", image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    if ids is not None:
        return True
    else:
        return False
    
    
if __name__ == '__main__':
    detect_pose()


