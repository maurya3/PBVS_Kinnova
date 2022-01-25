import cv2
import numpy as np
from cv2 import aruco
import rospy
from kortex_driver.msg import *
from kortex_driver.srv import *
from scipy.spatial.transform import Rotation as RT
from realsense_depth import DepthCamera


dc  = DepthCamera()
 
# define constants
cameraMatrix = np.array([[607.4385986328125, 0.0, 315.3274841308594], [0.0, 606.0167846679688, 241.0012969970703],[0.0, 0.0, 1.0]])
distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
arucoParams = aruco.DetectorParameters_create()
rgb_image = np.zeros([480, 640, 3], np.uint8)
Varm = np.array([0, 0, 0, 0, 0, 0])
th = np.array([0, 0, 0, 0, 0, 0])
current_position = np.array([0, 0, 0, 0, 0, 0])
T_vec_curr = np.array([[0], [0], [0]])
R_vec_curr = np.array([[0], [0], [0]])
R_cc_m = np.array([[0], [0], [0]])
# Lin_v = np.array([0,0,0])
Ang_v = np.array([[0], [0], [0]])
Vcamera = np.array([0,0,0,0,0,0])


# aruco Detect
def aruco_detect(color_frame):
    global T_vec_curr
    global R_vec_curr
#    ret, depth_frame,color_frame = dc.get_frame()
    gray = cv2.cvtColor(color_frame,cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_imgpoints = aruco.detectMarkers(gray, aruco_dict, cameraMatrix, distCoeffs, parameters=arucoParams)
    if np.all(ids is not None):
        for i in range(0, len(ids)):
            R_vec, T_vec, _objPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.10, cameraMatrix, distCoeffs)
            current_position = np.append(T_vec, R_vec)
            # print("current position",current_position)
            R_vec_curr = np.array([current_position[3], current_position[4], current_position[5]])
            T_vec_curr = np.array([[current_position[0]], [current_position[1]], [current_position[2]]])
            (R_vec-T_vec).any()
            aruco.drawAxis(color_frame, cameraMatrix,distCoeffs, R_vec, T_vec, 0.1)
        R_cc_m = RT.from_rotvec(R_vec_curr)
        R_cc_m = R_cc_m.as_dcm()

    rgb_image = aruco.drawDetectedMarkers(color_frame, corners)
    R_vec_des = np.array([-1.987, 0.120,  -0.043])
    T_vec_des = np.array([[-0.047], [-0.044],  [0.548]])
    R_cd_m = RT.from_rotvec(R_vec_des)
    R_cd_m = R_cd_m.as_dcm()
    R = np.identity(3)
    thu = RT.from_dcm(R)
    thu = thu.as_rotvec()
    thu = thu.reshape((3, 1))
    T_cc_cd = np.matmul(np.transpose(R_cd_m), (T_vec_curr - T_vec_des))
    Error_T = T_cc_cd
    Error_R = thu
    Error = np.concatenate([Error_T, Error_R], axis=0)
    print("Error vector", Error)
    N = np.linalg.norm(Error)
    print("Norm of error", N)
    var_lambda = 0.1
    if np.linalg.norm(Error) < 0.02:
        var_lambda = 0
    w_1 = np.array([[0], [0], [0]])
    Lin_v = -(var_lambda) * np.matmul(np.transpose(R), T_cc_cd)
    Ang_v = -(var_lambda) * np.add(Error_R, w_1)
    Vcamera = np.concatenate([Lin_v, Ang_v], axis=0)
    print("camera velocity calculated",Vcamera)
    Vcam11 = Vcamera
    Vcamera[0] = Vcam11[0]
    Vcamera[1] = -Vcam11[1]
    Vcamera[2] = Vcam11[2]
    Vcamera[3] = -Vcam11[3]
    Vcamera[4] = -Vcam11[4]
    Vcamera[5] = Vcam11[5]


    return Vcamera[0],Vcamera[1], Vcamera[2], Vcamera[3], Vcamera[4],Vcamera[5], rgb_image


def main():
    rospy.init_node('Aruco_detection', anonymous=True)
    msg = TwistCommand()
    pub = rospy.Publisher('/gen3_lite/in/joint_velocity', TwistCommand,  queue_size=10)   
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, depth_frame,frame = dc.get_frame()
        a,b,c,d,e,f,rgb_image = aruco_detect(color_frame=frame)
        cv2.imshow("Camera",rgb_image)
        msg.twist.linear_x = a
        msg.twist.linear_y = b
        msg.twist.linear_z = c
        msg.twist.angular_x = d
        msg.twist.angular_y = e
        msg.twist.angular_z = f
        pub.publish(msg)
        key = cv2.waitKey(10)
        if key== 27:
            break
        rospy.sleep(0.001)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass







