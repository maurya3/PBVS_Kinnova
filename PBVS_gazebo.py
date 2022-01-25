#!/usr/bin/env python2
import rospy
import numpy as np
from sensor_msgs.msg import Image, JointState
from math import *
from std_msgs.msg import Float64MultiArray
from kortex_driver.srv import *
from kortex_driver.msg import *
from kortex_driver.msg import Base_JointSpeeds
import kinova_jac_func_new
import Vcam_to_Base_kinova 
import Plot_task_to_joint
from scipy.spatial.transform import Rotation as RR

import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

np.set_printoptions(suppress=True)

global Varm, Vcamera
global Vel
global th
global mom, Error
rgb_image = np.zeros([480, 640, 3], np.uint8)
th1 = th2 = th3 = th4 = th5 = th6 = 0
Varm = np.array([0, 0, 0, 0, 0, 0])
th = np.array([0, 0, 0, 0, 0, 0])
Vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

T_vec_curr = np.array([[0], [0], [0]])
R_vec_curr = np.array([[0], [0], [0]])
R_cc_m = np.array([[0], [0], [0]])
Ang_v = np.array([[0], [0], [0]])
Vcamera = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
Error=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
# --Get the current joint angles from /gen3_lite/joint_states


def JS_callback(js):
    global th1, th2, th3, th4, th5, th6
    global th
    global Varm
    global mom
    # th = js.position
    th1 = js.position[0]
    th2 = js.position[1]
    th3 = js.position[2]
    th4 = js.position[3]
    th5 = js.position[4]
    th6 = js.position[5]
    th = np.array([th1, th2, th3, th4, th5, th6])
    print("joint_angles = ", th)
    # th = np.asarray(ths)
    # print("joint_ang", th)


    
def rgb_callback(rgb_msg):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
    global key
    global Varm, Vcamera
    global cameraMatrix
    global distCoeffs
    global Lin_v
    global Ang_v
    global current_position
    global T_vec_curr
    global R_vec_curr
    global R_cc_m
    global th1, th2, th3, th4, th5, th6, th
    global Error

    # global current_position
    global desired_position
    # global w_cam

    cameraMatrix = np.array([[1386.4139404296875, 0.0, 960.0], [ 0.0, 1386.4139404296875, 540.0],[0.0, 0.0, 1.0]])
    distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = aruco.DetectorParameters_create()
    corners, ids, rejected_imgpoints = aruco.detectMarkers(gray, aruco_dict, cameraMatrix, distCoeffs, parameters=arucoParams)

    if np.all(ids is not None):
        for i in range(0, len(ids)):
            R_vec, T_vec, _objPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.050, cameraMatrix, distCoeffs)

            current_position = np.append(T_vec, R_vec)
            print("current position",current_position)
            R_vec_curr = np.array([current_position[3], current_position[4], current_position[5]])
            T_vec_curr = np.array([[current_position[0]], [current_position[1]], [current_position[2]]])
            # print("R_vec",R_vec_curr)
            # print("T_vec",T_vec_curr)
            (R_vec-T_vec).any()
            aruco.drawAxis(rgb_image, cameraMatrix,distCoeffs, R_vec, T_vec, 0.1)
            # print(R_vec)

        # 3x3 Rotation matrix from R_vec using Rodrigues formulation
        # Rotation matrix of current camera frame wrt marker
        R_cc_m = RR.from_rotvec(R_vec_curr)
        R_cc_m = R_cc_m.as_dcm()
    rgb_image = aruco.drawDetectedMarkers(rgb_image, corners)


    ##--Input Desired location of end effector-------------------------###
    R_vec_des = np.array([ 3.13997152, 0.19096219, 0.1264478 ])
    T_vec_des = np.array([[-0.23827062], [-0.08550893], [ 0.47555222]])

    ##-----Convert input rotation to Rotation matrix-----------------##
    R_cd_m = RR.from_rotvec(R_vec_des)
    R_cd_m = R_cd_m.as_dcm()

    # Rotation matrix R_cc_cd, Rotation of current frame wrt to desired frame
    R = np.matmul(np.linalg.inv(R_cd_m), R_cc_m)
    thu = RR.from_dcm(R)
    thu = thu.as_rotvec()
    thu = thu.reshape((3, 1))

    # Translation vector bewteen current and desired camera frame
    # T_cc_cd = T_vec_curr - T_vec_des
    T_cc_cd = np.matmul(np.transpose(R_cd_m), (T_vec_curr - T_vec_des))
    Error_T = T_cc_cd
    Err_tn=Error_T.reshape((3, 1))
    # print("Error_T",Error_T)
    # Error_T = np.array([[0], [0], [0]])

    Error_R = thu
    Err_Rn=Error_R.reshape((3, 1))
    # print(Error_R)
    # print(Error_T.shape)
    # Error_R = np.array([[0], [0], [0]])
    Error = np.concatenate([Error_T, Error_R], axis=0)
    # print("Error vector", Error)
    N = np.linalg.norm(Error)
    print("Norm of error", N)
    var_lambda = 0.3

    if np.linalg.norm(Error) < 0.025:
        var_lambda = 0
    w_1 = np.array([[0], [0], [0]])
    Lin_v = -(var_lambda) * np.matmul(np.transpose(R), Error_T  )
    Ang_v = -(var_lambda) * np.add(Error_R, w_1)

    Vcamera = np.concatenate([Lin_v, Ang_v], axis=0)
    # print(Vcamera)
    Vcam11 = Vcamera
    # print("Vcam",Vcam11)

    # Vel[0] = 0
    # Vel[1] = 0
    # Vel[2] = 0
    # Vel[3] = Vcam11[5]
    # Vel[4] = 0
    # Vel[5] = 0

    Vel[0] = Vcam11[2]
    Vel[1] = Vcam11[0]
    Vel[2] = -Vcam11[1]
    Vel[3] = Vcam11[5]
    Vel[4] = Vcam11[3]
    Vel[5] = -Vcam11[4]
    
    # Vcamera[0] = 0
    # Vcamera[1] = 0
    # Vcamera[2] = 0
    # Vcamera[3] = 0
    # Vcamera[4] = 0
    # Vcamera[5] = 0 


    # print("Camera_Velocity", Vel)


    # # --Give task space velocity to kiova arm
    # Vel[0] = 0.0 # front/back
    # Vel[1] = -0.01 # sideways
    # Vel[2] = -0.0 # up/down
    # Vel[3] = 0.0  #Rot about x
    # Vel[4] = -0.0    #Rot about y
    # Vel[5] = 0.0   #Rot about Z
    # # print("task space velocity =", Vel)

    ##---Convert this velocity to base frame---##
    Vbase, EE_pos, Rot_mat = Vcam_to_Base_kinova.Vc_2_Base(Vcamera, th)
    def trunc(values, decs=0):
        return np.trunc(values*10**decs)/(10**decs)
    Vbase=trunc(Vbase,3)

    # print("Vbase=", Vbase)


    ##------Converting Rotation matrix into RPY-------##
    # print(Rot_mat)
    # R_mat = np.asarray([[Rot_mat[0,0], Rot_mat[0,1], Rot_mat[0,2]],
    #          [Rot_mat[1,0], Rot_mat[1,1], Rot_mat[1,2]],
    #          [Rot_mat[2,0], Rot_mat[2,1], Rot_mat[2,2]]])
    r = RR.from_dcm(Rot_mat)
    Euler_ang = r.as_euler('zyx', degrees=False)
    # print("Euler_angles ",Euler_ang)

    ##---Calculate manipulator jacobian-----##
    jacobian = kinova_jac_func_new.calc_jack(th)
    # print("Jacobian =", jacobian)

    ##----Measure of singularity(Measure of manipulability)----##
    mom = sqrt(np.linalg.det(np.matmul(jacobian,np.transpose(jacobian))))
    # print("Measure of manipulability = ",mom)


    ##------Inverse of Jacobian--------##
    J = np.linalg.pinv(jacobian)
    # jacobian = kinova_jac_func_new.calc_jack(th1,th2,th3,th4,th5,th6)


    ##---Joint_space_velocity--------##
    Varm = np.matmul(J, Vel)
    # print("Joint_velocity = ", Varm)

    ##---PLOTS----##
    plot = Plot_task_to_joint.plot(Varm,mom,th,EE_pos,Euler_ang,N,Err_tn,Err_Rn)


    # plot = plts_kinova_pbvs.plot(Vcamera,N,th)


def main():
    global Varm, Jid, mom
    global rgb_image
    # global th
    Jid = np.array([1, 2, 3, 4, 5, 6])
    rospy.init_node('Publish_joint_vel', anonymous=True)
    # rospy.Subscriber('/my_gen3_lite/joint_states', JointState, JS_callback)
    rospy.Subscriber('/my_gen3_lite/joint_states', JointState, JS_callback)
    rospy.Subscriber('camera/color/image_raw', Image, rgb_callback, queue_size=1)  # GAZEEBO AND REAL ROBOT
    # print("joint_angles = ", th)

    # pub = rospy.Publisher('/my_gen3_lite/in/joint_velocity',Base_JointSpeeds,  queue_size=10)
    pub = rospy.Publisher('/my_gen3_lite/in/joint_velocity',Base_JointSpeeds,  queue_size=10)
    rate = rospy.Rate(75)
    jv = Base_JointSpeeds()
    while not rospy.is_shutdown():
        joint_speeds = []
        for Jid in range(6):
            msg = JointSpeed()
            msg.joint_identifier = Jid
            msg.value = 1.5*Varm[Jid]
            msg.duration = 0
            joint_speeds.append(msg)

        jv.joint_speeds = joint_speeds
        jv.duration = 0

        pub.publish(jv)
        image  = cv2.resize(rgb_image,(640,480))
        cv2.imshow('image', image)
        key = cv2.waitKey(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
