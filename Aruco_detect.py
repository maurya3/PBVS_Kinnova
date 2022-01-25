'''
Sample Command:-
python detect_aruco_video.py --type DICT_5X5_100 --camera True
python detect_aruco_video.py --type DICT_5X5_100 --camera False --video test_video.mp4
'''

from math import degrees
import numpy as np
from numpy.core.fromnumeric import size
from utils import ARUCO_DICT, aruco_display
import argparse
import time
import cv2  
from cv2 import aruco
import sys
from realsense_depth import DepthCamera
from geometry_msgs.msg import Pose
import rospy
from scipy.spatial.transform import Rotation as RR


dc = DepthCamera()
arucoDict = cv2.aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters_create()
cameraMatrix = np.array([[913.9440307617188, 0.0, 644.210693359375], [ 0.0, 912.135009765625, 347.86767578125],[0.0, 0.0, 1.0]])
distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
msg = Pose()

while True:
	rospy.init_node("aruco_pose", anonymous=True)
	pub = rospy.Publisher('/Pose',Pose, queue_size=10)
	rate  = rospy.Rate(100)

	_,depth,frame = dc.get_frame()
	
	if depth is False:
		break

	corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
	print(ids)
	if ids == 13:
		detected_markers = aruco_display(corners, ids, rejected, frame)
		R_vec, T_vec, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.10, cameraMatrix, distCoeffs)
		rr = RR.from_rotvec([R_vec[0][0][0],R_vec[0][0][1],R_vec[0][0][2]])
		R_vec = rr.as_euler("zyx",degrees=True)
		print(T_vec[0][0][0])
		msg.position.x =T_vec[0][0][0]
		msg.position.y = T_vec[0][0][1]
		msg.position.z =  T_vec[0][0][2]
		msg.orientation.x = R_vec[0]
		msg.orientation.y =R_vec[1]
		msg.orientation.z =R_vec[2]
		
		pub.publish(msg)
		cv2.imshow("Image", detected_markers)
		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):
	   		 break
		
cv2.destroyAllWindows()
dc.release()
