from numpy.core.fromnumeric import transpose
import rospy
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as RR
from sensor_msgs.msg import Image, JointState
from kortex_driver.msg import Base_JointSpeeds
from kortex_driver.srv import *
from kortex_driver.msg import *
import kinova_jac_func_new

jv = Base_JointSpeeds()
global theta
theta = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
Rdes = [ 3.13997152, 0.19096219, 0.1264478 ]
Tdes = [-0.23827062, -0.08550893, 0.87555222]
Rc = [0,0,0] 
Tc = [0,0,0]

def cb_pose(data):
    global Rc
    global Tc
    Tc = data.position
    Rc = data.orientation


def JointStates_cb(data):
     
    th1 = data.position[0]
    th2 = data.position[1]
    th3 = data.position[2]
    th4 = data.position[3]
    th5 = data.position[4]
    th6 = data.position[5]
    theta  = np.array([th1,th2,th3,th4,th5,th6])

def main():
    rospy.init_node("Joint_velocity_publisher", anonymous=True)
    rospy.Subscriber("/gen3_lite/joint_states",JointState, JointStates_cb)
    rospy.Subscriber("/aruco_pose",Pose,cb_pose )
    pub = rospy.Publisher('/gen3_lite/in/joint_velocity',Base_JointSpeeds,  queue_size=10)
    rate  = rospy.Rate(100)
    #print(theta)
    while not rospy.is_shutdown() and theta is not None:

        RdesM = RR.from_rotvec(Rdes)
        RdesM = RdesM.as_dcm()

        RcurM = RR.from_rotvec([Rc[0],Rc[1],Rc[2]])
        RcurM = RcurM.as_dcm()
        
        R = np.matmul(np.linalg.inv(RdesM),RcurM)

        Tcur2des = np.matmul(np.transpose(RdesM), (np.asarray(Tc) - np.asarray(Tdes)))
        
        thU = RR.from_dcm(R)
        thU = thU.as_rotvec()
        
        #print(np.asarray(Tcur2des))
        lamb = 0.3
        VelLin = -(lamb)*np.matmul(np.transpose(R),Tcur2des)
        VelAng = -(lamb)*thU
        camVel  = np.concatenate([VelLin,VelAng],axis=0)
        #print(np.asarray(camVel))
        Jacob = kinova_jac_func_new.calc_jack(theta)
        JacobInv = np.linalg.inv(Jacob)
        VelArm = np.matmul(JacobInv,np.array(camVel))
        joint_speeds = []
        #print(camVel)
        for Jid in range(6):
            msg = JointSpeed()
            msg.joint_identifier = Jid
            msg.value = VelArm[Jid]
            if msg.value > 0.1:
                msg.value = 0.1
            msg.duration = 0 
            joint_speeds.append(msg)
        print(joint_speeds)
        jv.joint_speeds = joint_speeds*2
        jv.duration = 0

    pub.publish(jv)



    rospy.spin()


if __name__ == '__main__':
    main()
    





