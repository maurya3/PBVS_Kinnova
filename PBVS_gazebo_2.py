from numpy.core.fromnumeric import shape
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as RR
from sensor_msgs.msg import Image, JointState
from kortex_driver.msg import Base_JointSpeeds
from kortex_driver.srv import *
from kortex_driver.msg import *
import cv2
from cv_bridge import CvBridge
import kinova_jac_func_new


bridge  = CvBridge()
##   Initilization
image_rgbi = np.zeros([640,480,3],dtype=np.uint8)
theta = 0
rvec = 0
R_vec = T_vec = 0
Vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
th = np.array([0, 0, 0, 0, 0, 0])
R_cc_m = np.array([[0], [0], [0]])
T_vec_curr = np.array([[0], [0], [0]])
R_vec_curr = np.array([[0], [0], [0]])

def Joint_states_cb(data):
    global theta
    
    th1 = data.position[0]
    th2 = data.position[1]
    th3 = data.position[2]
    th4 = data.position[3]
    th5 = data.position[4]
    th6 = data.position[5]
    print(theta)
    theta  = np.array([th1,th2,th3,th4,th5,th6]).T
    
def aruco(image):
    global R_vec_curr, T_vec_curr
    from cv2 import aruco
    #cameraMatrix = np.array([[1386.4139404296875, 0.0, 960.0], [ 0.0, 1386.4139404296875, 540.0],[0.0, 0.0, 1.0]])
    cameraMatrix = np.array([[913.9440307617188, 0.0, 644.210693359375], [ 0.0, 912.135009765625, 347.86767578125],[0.0, 0.0, 1.0]])
    distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = aruco.DetectorParameters_create()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_imgpoints = aruco.detectMarkers(gray, aruco_dict, cameraMatrix, distCoeffs, parameters=arucoParams)
   
    if np.all(ids is not None):
        global R_vec
        global T_vec
        
        for i in range(0, len(ids)):
            R_vec, T_vec, _objPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.050, cameraMatrix, distCoeffs)
            (R_vec-T_vec).any()
            current_position = np.append(T_vec, R_vec)             
            R_vec_curr = np.array([current_position[3], current_position[4], current_position[5]])
            T_vec_curr = np.array([[current_position[0]], [current_position[1]], [current_position[2]]])            
            aruco.drawAxis(image, cameraMatrix,distCoeffs, R_vec, T_vec, 0.1)
    
    R_cc_m = RR.from_rotvec(R_vec_curr)
    R_cc_m = R_cc_m.as_dcm()
    rgb_image = aruco.drawDetectedMarkers(image, corners)
    R_vec_des = np.array([ 3.13997152, 0.19096219, 0.1264478 ])
    T_vec_des = np.array([[-0.23827062], [-0.08550893], [ 0.87555222]])

    ##-----Convert input rotation to Rotation matrix-----------------##
    R_cd_m = RR.from_rotvec(R_vec_des)
    R_cd_m = R_cd_m.as_dcm()
    R = np.matmul(np.linalg.inv(R_cd_m), R_cc_m)
    thu = RR.from_dcm(R)
    thu = thu.as_rotvec()
    thu = thu.reshape((3, 1))

    T_cc_cd = np.matmul(np.transpose(R_cd_m), (T_vec_curr - T_vec_des))
    Error_T = T_cc_cd
    Err_tn=Error_T.reshape((3, 1))

    Error_R = thu
    Err_Rn=Error_R.reshape((3, 1))

    Error = np.concatenate([Error_T, Error_R], axis=0)
    N = np.linalg.norm(Error)
    var_lambda = 0.3
    if np.linalg.norm(Error) < 0.025:
        var_lambda = 0
    w_1 = np.array([[0], [0], [0]])
    Lin_v = -(var_lambda) * np.matmul(np.transpose(R), Error_T  )
    Ang_v = -(var_lambda) * np.add(Error_R, w_1)
    Vcamera = np.concatenate([Lin_v, Ang_v], axis=0)
    Vcam11 = Vcamera


    return ids, rgb_image, R_vec,T_vec,Vcam11


def image_cb(data):
    global image_rgbi
    image_rgbi = data #bridge.imgmsg_to_cv2(data,'bgr8')

#def conversion():



def main():
    
    rospy.init_node("Joint_velocity_publisher", anonymous=True)
    rospy.Subscriber("/gen3_lite/joint_states",JointState, Joint_states_cb)
    rospy.Subscriber("/camera/color/image_raw",Image,image_cb)
    pub = rospy.Publisher('/gen3_lite/in/joint_velocity',Base_JointSpeeds,  queue_size=10)

    rate  = rospy.Rate(100)

    rate.sleep()
    jv = Base_JointSpeeds()
    while not rospy.is_shutdown():
        
        ids,rgb_image,R_vec,T_vec,Vcam11 = aruco(image_rgbi)
        rgb_image  = cv2.resize(rgb_image,(640,480))
        cv2.imshow('image',rgb_image)
        joint_speeds = []
        Vel[0] = Vcam11[2]
        Vel[1] = Vcam11[0]
        Vel[2] = -Vcam11[1]
        Vel[3] = Vcam11[5]
        Vel[4] = Vcam11[3]
        Vel[5] = -Vcam11[4]

        jacobian = kinova_jac_func_new.calc_jack(theta)
        mom = np.sqrt(np.linalg.det(np.matmul(jacobian,np.transpose(jacobian))))
        J = np.linalg.pinv(jacobian)
        Varm = np.matmul(J, Vel)
        for Jid in range(6):
            msg = JointSpeed()
            msg.joint_identifier = Jid
            msg.value = Varm[Jid]
            msg.duration = 0
            joint_speeds.append(msg)
        jv.joint_speeds = joint_speeds
        jv.duration = 0
        
        
        pub.publish(jv)
         


        #R_vec = R_vec.reshape(3,1)
        print(shape(R_vec))
        print(T_vec)
        print(theta)
        key = cv2.waitKey(10)
        if key==27:
            cv2.destroyAllWindows()
            break

    


if __name__ == '__main__':
    main()
    
